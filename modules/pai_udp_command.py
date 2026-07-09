# modules/pai_udp_command.py
# PAI-Car Pico 2 W UDP command receiver

import socket
import struct
import time


COMMAND_MAGIC = 0x5043
ACK_MAGIC = 0x4341
VERSION = 1

COMMAND_FORMAT = "<HBBHBBhi"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

ACK_FORMAT = "<HBBHBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)


COMMAND_LISTEN_IP = "0.0.0.0"
COMMAND_PORT = 5006


CMD_PING = 1
CMD_STOP = 2
CMD_SAFE_MODE = 3      # kept for compatibility
CMD_RUN = 4
CMD_NEXT_SECTION = 5


STATUS_OK = 0
STATUS_BAD_MAGIC = 1
STATUS_BAD_VERSION = 2
STATUS_BAD_SIZE = 3
STATUS_UNKNOWN_CMD = 4
STATUS_ERROR = 5


RUN_STATE_STOP = 0
RUN_STATE_RUN = 1


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

        self.last_cmd_seq = 0
        self.last_cmd_type = 0
        self.last_cmd_status = STATUS_OK
        self.last_cmd_ms = ticks_ms()

        self.recv_count = 0
        self.bad_count = 0
        self.ack_count = 0

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
            self.last_addr = addr
            self.last_cmd_ms = ticks_ms()

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
            "cmd_seq": cmd_seq,
            "cmd_type": cmd_type,
            "target_id": target_id,
            "param_id": param_id,
            "value": value,
        }

        return cmd, STATUS_OK

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

    def get_last_cmd_status(self):
        return self.last_cmd_status

    def debug_text(self):
        return (
            "udp_cmd state={} section={} recv={} bad={} ack={} "
            "last_seq={} last_type={} last_status={} error={}"
        ).format(
            self.run_state,
            self.track_section_id,
            self.recv_count,
            self.bad_count,
            self.ack_count,
            self.last_cmd_seq,
            self.last_cmd_type,
            self.last_cmd_status,
            self.last_error,
        )