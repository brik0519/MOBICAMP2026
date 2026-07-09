# pai_udp_telemetry.py
# PAI-Car UDP binary telemetry module
#
# 역할:
#   - Pico 2 W Wi-Fi 연결
#   - UDP socket 생성
#   - 주행 데이터를 binary packet으로 변환
#   - 20ms 주기로 PC에 전송
#   - non-blocking UDP 전송
#   - 네트워크 지연 시 오래된 telemetry 샘플 삭제
#
# 이 파일은 학생들이 자주 볼 필요가 없는 통신 관련 코드를 모아 둔다.

from time import ticks_ms, ticks_diff, sleep_ms
import network
import socket
import struct

from modules.pai_car_run_support import (
    CONTROL_MS,
    MIN_TOTAL,
    NOISE_CUTOFF,
    T_MARKER_TH,
    T_MARKER_MIN_COUNT,
)

from modules.pai_car_wifi_config import (
    WIFI_SSID,
    WIFI_PASSWORD,
    PC_IP,
    PC_PORT,
)


# ------------------------------------------------------------
# Telemetry settings
# ------------------------------------------------------------

SEND_MS = 20

WIFI_TIMEOUT_MS = 15000

MAGIC = 0x5041
VERSION = 1
RESERVED = 0

# 전송이 SEND_MS보다 과도하게 늦어진 경우 오래된 샘플로 보고 삭제한다.
MAX_SEND_LAG_MS = 45

# sendto()가 이 시간보다 오래 걸리면 네트워크가 느린 상태로 보고 cooldown에 들어간다.
MAX_SEND_COST_MS = 2

# 네트워크가 느린 것으로 감지된 뒤 이 시간 동안 telemetry 전송을 건너뛴다.
TELEMETRY_COOLDOWN_MS = 100


# ------------------------------------------------------------
# Binary packet format
# ------------------------------------------------------------
#
# <      : little-endian
# H      : magic, uint16
# B      : version, uint8
# B      : reserved, uint8
# H      : seq, uint16
# I      : t_ms, uint32
# H      : control_ms, uint16
# H      : send_ms, uint16
# h      : current_speed, int16
# 8H     : n0~n7, uint16 x 8
# H      : position, uint16
# h      : error, int16
# h      : d_error, int16
# h      : left_cmd, int16
# h      : right_cmd, int16
# B      : on_line, uint8
# B      : is_marker, uint8
#
# Total: 44 bytes

PACKET_FORMAT = "<HBBHIHHh8HHhhhhBB"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)


# ------------------------------------------------------------
# Line sensor helper
# ------------------------------------------------------------

def read_line_detail(line):
    """
    라인센서 값을 읽고 제어/전송에 필요한 값을 반환한다.

    반환값:
        error, position, norm, on_line

    error:
        position - 3500

    position:
        왼쪽 끝 0, 중앙 3500, 오른쪽 끝 7000

    norm:
        8개 라인센서 정규화값, 0~1000

    on_line:
        라인 감지 여부
    """

    error, position, norm, on_line = line.read_error(
        min_total=MIN_TOTAL,
        noise_cutoff=NOISE_CUTOFF,
    )

    return error, position, norm, on_line


def is_t_marker_area(norm, on_line):
    """
    현재 순간이 T 마커 구간인지 판단한다.

    lap_timer.check_finish()는 T 마커 '이벤트'를 세는 함수이고,
    이 함수는 현재 센서 상태가 T 마커 위인지 여부만 반환한다.
    """

    if not on_line:
        return False

    black_count = 0

    for v in norm:
        if v >= T_MARKER_TH:
            black_count += 1

    return black_count >= T_MARKER_MIN_COUNT


# ------------------------------------------------------------
# Wi-Fi / UDP telemetry class
# ------------------------------------------------------------

class PAIUdpTelemetry:
    """
    PAI-Car 주행 데이터를 PC로 전송하는 클래스.

    main 파일에서는 다음 정도만 사용하면 된다.

        telemetry = PAIUdpTelemetry(lap_timer)
        telemetry.begin()
        telemetry.reset_timer()
        telemetry.send_if_due(...)
        telemetry.send_now(...)
        telemetry.get_stats()
        telemetry.close()
    """

    def __init__(self, lap_timer=None):
        self.lap_timer = lap_timer

        self.wlan = None
        self.sock = None

        self.seq = 0
        self.run_start_ms = ticks_ms()
        self.last_send_ms = ticks_ms()

        self.enabled = False

        self.sent_count = 0
        self.drop_lag_count = 0
        self.drop_busy_count = 0
        self.slow_send_count = 0
        self.cooldown_until_ms = 0

    def begin(self):
        """
        Wi-Fi에 연결하고 UDP socket을 준비한다.

        실패해도 주행 자체는 가능하도록 enabled=False 상태로 둔다.
        """

        if not self._connect_wifi():
            self.enabled = False
            return False

        try:
            self.sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_DGRAM,
            )

            try:
                self.sock.setblocking(False)
            except Exception:
                pass

            self.enabled = True

            print("UDP telemetry ready")
            print("Target:", PC_IP, PC_PORT)
            print("Packet size:", PACKET_SIZE)

            if self.lap_timer is not None:
                self.lap_timer.show(
                    "UDP READY",
                    PC_IP[:16],
                    "port {}".format(PC_PORT),
                    "",
                )

            return True

        except OSError as e:
            print("UDP socket error:", e)

            if self.lap_timer is not None:
                self.lap_timer.show(
                    "UDP ERROR",
                    "socket failed",
                    "",
                    "",
                )

            self.enabled = False
            return False

    def reset_timer(self):
        """
        주행 시작 시점에 맞춰 전송 시간 기준을 초기화한다.
        """

        now = ticks_ms()

        self.run_start_ms = now
        self.last_send_ms = now
        self.cooldown_until_ms = 0

        self.sent_count = 0
        self.drop_lag_count = 0
        self.drop_busy_count = 0
        self.slow_send_count = 0

    def send_if_due(
        self,
        base_speed,
        norm,
        position,
        error,
        d_error,
        left_cmd,
        right_cmd,
        on_line,
        is_marker,
    ):
        """
        SEND_MS가 지났으면 주행 데이터를 한 번 전송한다.

        반환값:
            True  -> 실제 송신 성공
            False -> 아직 전송 시점이 아니거나, 지연/혼잡으로 샘플 삭제
        """

        if not self.enabled:
            return False

        now = ticks_ms()

        if ticks_diff(
            now,
            self.cooldown_until_ms,
        ) < 0:
            return False

        elapsed_ms = ticks_diff(
            now,
            self.last_send_ms,
        )

        if elapsed_ms < SEND_MS:
            return False

        # 제어 루프나 네트워크 지연으로 너무 늦어진 샘플은 오래된 데이터이므로 삭제한다.
        if elapsed_ms > MAX_SEND_LAG_MS:
            self.drop_lag_count += 1
            self.last_send_ms = now
            return False

        t_ms = ticks_diff(
            now,
            self.run_start_ms,
        )

        ok = self._send_packet(
            t_ms,
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker,
        )

        self.last_send_ms = now
        return ok

    def send_now(
        self,
        base_speed,
        norm,
        position,
        error,
        d_error,
        left_cmd,
        right_cmd,
        on_line,
        is_marker,
    ):
        """
        전송 주기와 관계없이 주행 데이터를 즉시 한 번 전송한다.
        Finish 순간의 마지막 상태를 보낼 때 사용한다.

        반환값:
            True  -> 실제 송신 성공
            False -> 비활성 또는 송신 실패
        """

        if not self.enabled:
            return False

        now = ticks_ms()

        if ticks_diff(
            now,
            self.cooldown_until_ms,
        ) < 0:
            return False

        t_ms = ticks_diff(
            now,
            self.run_start_ms,
        )

        ok = self._send_packet(
            t_ms,
            base_speed,
            norm,
            position,
            error,
            d_error,
            left_cmd,
            right_cmd,
            on_line,
            is_marker,
        )

        self.last_send_ms = now
        return ok

    def get_stats(self):
        """
        main.py final report에서 사용할 telemetry 통계를 반환한다.
        """

        return (
            self.sent_count,
            self.drop_lag_count,
            self.drop_busy_count,
            self.slow_send_count,
        )

    def close(self):
        """
        UDP socket을 닫는다.
        """

        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass

        self.sock = None
        self.enabled = False

    def _connect_wifi(self):
        """
        Pico 2 W를 Wi-Fi에 연결한다.
        """

        if WIFI_SSID == "" or WIFI_SSID == "YOUR_WIFI_SSID":
            print("Wi-Fi SSID is not set. UDP disabled.")

            if self.lap_timer is not None:
                self.lap_timer.show(
                    "UDP OFF",
                    "Set WiFi info",
                    "",
                    "",
                )

            return False

        if self.lap_timer is not None:
            self.lap_timer.show(
                "WiFi",
                "Connecting...",
                "",
                "",
            )

        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)

        if not self.wlan.isconnected():
            self.wlan.connect(
                WIFI_SSID,
                WIFI_PASSWORD,
            )

            start = ticks_ms()

            while not self.wlan.isconnected():
                if ticks_diff(
                    ticks_ms(),
                    start,
                ) >= WIFI_TIMEOUT_MS:
                    print("Wi-Fi connection failed. UDP disabled.")

                    if self.lap_timer is not None:
                        self.lap_timer.show(
                            "WiFi FAILED",
                            "UDP disabled",
                            "",
                            "",
                        )

                    return False

                sleep_ms(200)

        ip = self.wlan.ifconfig()[0]
        print("Wi-Fi connected:", ip)

        if self.lap_timer is not None:
            self.lap_timer.show(
                "WiFi OK",
                ip[:16],
                "",
                "",
            )

        return True

    def _send_packet(
        self,
        t_ms,
        base_speed,
        norm,
        position,
        error,
        d_error,
        left_cmd,
        right_cmd,
        on_line,
        is_marker,
    ):
        """
        실제 binary packet을 만들어 PC로 전송한다.

        non-blocking sendto()가 실패하면 샘플을 버리고 False를 반환한다.
        """

        if self.sock is None:
            self.enabled = False
            return False

        start_ms = ticks_ms()

        try:
            packet = struct.pack(
                PACKET_FORMAT,
                MAGIC,
                VERSION,
                RESERVED,
                self.seq,
                int(t_ms),
                CONTROL_MS,
                SEND_MS,
                int(base_speed),

                int(norm[0]),
                int(norm[1]),
                int(norm[2]),
                int(norm[3]),
                int(norm[4]),
                int(norm[5]),
                int(norm[6]),
                int(norm[7]),

                int(position),
                int(error),
                int(d_error),
                int(left_cmd),
                int(right_cmd),
                1 if on_line else 0,
                1 if is_marker else 0,
            )

            self.sock.sendto(
                packet,
                (
                    PC_IP,
                    PC_PORT,
                ),
            )

            cost_ms = ticks_diff(
                ticks_ms(),
                start_ms,
            )

            if cost_ms > MAX_SEND_COST_MS:
                self.slow_send_count += 1
                self.cooldown_until_ms = (
                    ticks_ms()
                    + TELEMETRY_COOLDOWN_MS
                )

            self.seq = (
                self.seq + 1
            ) & 0xFFFF

            self.sent_count += 1
            return True

        except OSError:
            self.drop_busy_count += 1
            self.last_send_ms = ticks_ms()
            return False