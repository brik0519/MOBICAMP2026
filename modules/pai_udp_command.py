# modules/pai_udp_command.py
# PAI-Car Pico 2 W UDP command receiver
#
# 역할:
#   - PC dashboard에서 오는 UDP command 수신
#   - Z STOP / Enter RUN / Space NEXT_SECTION 처리
#   - command ACK 회신
#   - telemetry V2가 사용할 마지막 command 적용 결과 보관
#
# 주의:
#   - CMD_NEXT_SECTION은 상대 명령이다.
#   - 같은 cmd_seq가 중복 수신되면 section을 다시 증가시키지 않는다.
#   - ACK TIMEOUT 이후 PC/Pico sync 보정은 telemetry V2에서 actual_section_id를 보고 처리한다.

import socket
import struct
import time


# ------------------------------------------------------------
# Command protocol
# ------------------------------------------------------------

COMMAND_MAGIC = 0x5043
ACK_MAGIC = 0x4341
VERSION = 1

COMMAND_FORMAT = "<HBBHBBhi"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

ACK_FORMAT = "<HBBHBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)


COMMAND_LISTEN_IP = "0.0.0.0"
COMMAND_PORT = 5006


# ------------------------------------------------------------
# Command types
# ------------------------------------------------------------

CMD_PING = 1
CMD_STOP = 2
CMD_SAFE_MODE = 3      # kept for compatibility
CMD_RUN = 4
CMD_NEXT_SECTION = 5


# ------------------------------------------------------------
# ACK / command status
# ------------------------------------------------------------
# dashboard 기존 호환을 위해 STATUS_OK = 0 유지

STATUS_OK = 0
STATUS_BAD_MAGIC = 1
STATUS_BAD_VERSION = 2
STATUS_BAD_SIZE = 3
STATUS_UNKNOWN_CMD = 4
STATUS_ERROR = 5


# ------------------------------------------------------------
# Run state
# ------------------------------------------------------------

RUN_STATE_STOP = 0
RUN_STATE_RUN = 1


# ------------------------------------------------------------
# Duplicate guard
# ------------------------------------------------------------

DUPLICATE_GUARD_MS = 2000


def ticks_ms():
    return time.ticks_ms()


def ticks_diff(a, b):
    return time.ticks_diff(a, b)


class PAIUdpCommand:
    def __init__(
        self,
        listen_ip=COMMAND_LISTEN_IP,
        listen_port=COMMAND_PORT,
        command_timeout_ms=1000,
        require_heartbeat=False,
    ):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.command_timeout_ms = command_timeout_ms
        self.require_heartbeat = require_heartbeat

        self.sock = None
        self.enabled = False

        self.run_state = RUN_STATE_RUN
        self.track_section_id = 0

        # 마지막으로 실제 적용된 command 결과.
        # telemetry V2가 이 값을 읽는다.
        self.last_cmd_seq = 0
        self.last_cmd_type = 0
        self.last_cmd_status = STATUS_OK
        self.last_cmd_ms = ticks_ms()

        self.recv_count = 0
        self.bad_count = 0
        self.ack_count = 0
        self.duplicate_count = 0

        self.last_addr = None
        self.last_error = ""

    def begin(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.listen_ip, self.listen_port))
            self.sock.setblocking(False)

            self.enabled = True
            self.last_error = ""

            print("PAIUdpCommand listening on {}:{}".format(
                self.listen_ip,
                self.listen_port,
            ))

            return True

        except Exception as exc:
            self.sock = None
            self.enabled = False
            self.last_error = "begin error: {}".format(exc)

            print(self.last_error)

            return False

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass

        self.sock = None
        self.enabled = False

    def poll(self):
        if not self.enabled or self.sock is None:
            return None

        handled = None

        while True:
            try:
                data, addr = self.sock.recvfrom(64)
            except OSError:
                break

            cmd, status = self.parse_command_packet(data)

            if cmd is None:
                self.bad_count += 1
                self.last_cmd_status = status
                continue

            self.recv_count += 1
            now_ms = ticks_ms()

            if self.is_duplicate_command(cmd, addr, now_ms):
                self.duplicate_count += 1

                # 같은 cmd_seq 재수신은 ACK만 다시 보내고 실제 적용하지 않는다.
                # 특히 NEXT_SECTION 중복 증가를 막기 위한 처리다.
                self.send_ack(
                    addr=addr,
                    cmd_seq=cmd["cmd_seq"],
                    cmd_type=cmd["cmd_type"],
                    status=self.last_cmd_status,
                )

                handled = cmd
                continue

            self.last_addr = addr
            self.last_cmd_ms = now_ms

            status = self.apply_command(cmd)

            self.last_cmd_seq = cmd["cmd_seq"]
            self.last_cmd_type = cmd["cmd_type"]
            self.last_cmd_status = status

            self.send_ack(
                addr=addr,
                cmd_seq=cmd["cmd_seq"],
                cmd_type=cmd["cmd_type"],
                status=status,
            )

            handled = cmd

        self.check_command_timeout()

        return handled

    def parse_command_packet(self, data):
        if len(data) != COMMAND_SIZE:
            return None, STATUS_BAD_SIZE

        try:
            values = struct.unpack(COMMAND_FORMAT, data)
        except Exception:
            return None, STATUS_ERROR

        magic = values[0]
        version = values[1]
        packet_size = values[2]
        cmd_seq = values[3]
        cmd_type = values[4]
        target_id = values[5]
        param_id = values[6]
        value = values[7]

        if magic != COMMAND_MAGIC:
            return None, STATUS_BAD_MAGIC

        if version != VERSION:
            return None, STATUS_BAD_VERSION

        if packet_size != COMMAND_SIZE:
            return None, STATUS_BAD_SIZE

        cmd = {
            "cmd_seq": int(cmd_seq) & 0xFFFF,
            "cmd_type": int(cmd_type) & 0xFF,
            "target_id": int(target_id) & 0xFF,
            "param_id": int(param_id),
            "value": int(value),
        }

        return cmd, STATUS_OK

    def is_duplicate_command(self, cmd, addr, now_ms):
        if self.last_addr is None:
            return False

        if addr != self.last_addr:
            return False

        if cmd["cmd_seq"] != self.last_cmd_seq:
            return False

        if cmd["cmd_type"] != self.last_cmd_type:
            return False

        if ticks_diff(now_ms, self.last_cmd_ms) > DUPLICATE_GUARD_MS:
            return False

        return True

    def apply_command(self, cmd):
        cmd_type = cmd["cmd_type"]

        if cmd_type == CMD_PING:
            return STATUS_OK

        if cmd_type == CMD_STOP:
            # Z key: emergency stop.
            self.run_state = RUN_STATE_STOP
            return STATUS_OK

        if cmd_type == CMD_SAFE_MODE:
            # Compatibility only. Treat as emergency stop.
            self.run_state = RUN_STATE_STOP
            return STATUS_OK

        if cmd_type == CMD_RUN:
            # Enter key: resume run.
            self.run_state = RUN_STATE_RUN
            return STATUS_OK

        if cmd_type == CMD_NEXT_SECTION:
            # Space key: track section marker.
            # This must not stop the car.
            self.track_section_id = (self.track_section_id + 1) & 0xFFFF
            return STATUS_OK

        return STATUS_UNKNOWN_CMD

    def send_ack(self, addr, cmd_seq, cmd_type, status):
        if not self.enabled or self.sock is None:
            return False

        try:
            packet = struct.pack(
                ACK_FORMAT,
                ACK_MAGIC,
                VERSION,
                ACK_SIZE,
                int(cmd_seq) & 0xFFFF,
                int(cmd_type) & 0xFF,
                int(status) & 0xFF,
            )

            self.sock.sendto(packet, addr)
            self.ack_count += 1

            return True

        except Exception as exc:
            self.last_error = "ack error: {}".format(exc)
            return False

    def check_command_timeout(self):
        if not self.require_heartbeat:
            return False

        now = ticks_ms()

        if ticks_diff(now, self.last_cmd_ms) > self.command_timeout_ms:
            self.run_state = RUN_STATE_STOP
            return True

        return False

    def should_force_stop(self):
        return self.run_state == RUN_STATE_STOP

    def is_running(self):
        return self.run_state == RUN_STATE_RUN

    def is_stopped(self):
        return self.run_state == RUN_STATE_STOP

    def get_run_state(self):
        return self.run_state

    def get_track_section_id(self):
        return self.track_section_id

    def get_last_cmd_seq(self):
        return self.last_cmd_seq

    def get_last_cmd_type(self):
        return self.last_cmd_type

    def get_last_cmd_status(self):
        return self.last_cmd_status

    def get_recv_count(self):
        return self.recv_count

    def get_bad_count(self):
        return self.bad_count

    def get_ack_count(self):
        return self.ack_count

    def get_duplicate_count(self):
        return self.duplicate_count

    def debug_text(self):
        return (
            "udp_cmd state={} section={} recv={} bad={} ack={} dup={} "
            "last_seq={} last_type={} last_status={} error={}"
        ).format(
            self.run_state,
            self.track_section_id,
            self.recv_count,
            self.bad_count,
            self.ack_count,
            self.duplicate_count,
            self.last_cmd_seq,
            self.last_cmd_type,
            self.last_cmd_status,
            self.last_error,
        )