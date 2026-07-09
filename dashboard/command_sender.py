# command_sender.py
# PAI-Car Step 3 PC -> Pico UDP command sender
#
# 역할:
#   PC dashboard에서 Pico 2 W로 command packet을 전송한다.
#   3단계에서는 PING / STOP / SAFE_MODE / RUN만 지원한다.
#
# 주의:
#   이 파일의 packet format은 Pico 쪽 modules/pai_udp_command.py와 반드시 같아야 한다.

import socket
import struct
import time


# ------------------------------------------------------------
# UDP command protocol VERSION 1
# ------------------------------------------------------------

COMMAND_MAGIC = 0x5043     # "PC" 의미로 사용
ACK_MAGIC = 0x4341         # "CA" 의미로 사용
VERSION = 1

# Command packet:
#   magic          uint16
#   version        uint8
#   packet_size    uint8
#   cmd_seq        uint16
#   cmd_type       uint8
#   target_id      uint8
#   param_id       int16
#   value          int32
#
# 현재 3단계에서는 target_id, param_id, value는 거의 사용하지 않는다.
COMMAND_FORMAT = "<HBBHBBhi"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)

# ACK packet:
#   magic          uint16
#   version        uint8
#   packet_size    uint8
#   cmd_seq        uint16
#   cmd_type       uint8
#   status         uint8
ACK_FORMAT = "<HBBHBB"
ACK_SIZE = struct.calcsize(ACK_FORMAT)


# ------------------------------------------------------------
# Command types
# ------------------------------------------------------------

CMD_PING = 1
CMD_STOP = 2
CMD_SAFE_MODE = 3
CMD_RUN = 4


CMD_NAMES = {
    CMD_PING: "PING",
    CMD_STOP: "STOP",
    CMD_SAFE_MODE: "SAFE_MODE",
    CMD_RUN: "RUN",
}


# ------------------------------------------------------------
# Status codes
# ------------------------------------------------------------

STATUS_OK = 0
STATUS_BAD_MAGIC = 1
STATUS_BAD_VERSION = 2
STATUS_BAD_SIZE = 3
STATUS_UNKNOWN_CMD = 4
STATUS_ERROR = 5


STATUS_NAMES = {
    STATUS_OK: "OK",
    STATUS_BAD_MAGIC: "BAD_MAGIC",
    STATUS_BAD_VERSION: "BAD_VERSION",
    STATUS_BAD_SIZE: "BAD_SIZE",
    STATUS_UNKNOWN_CMD: "UNKNOWN_CMD",
    STATUS_ERROR: "ERROR",
}


def cmd_name(cmd_type):
    return CMD_NAMES.get(cmd_type, "CMD_{}".format(cmd_type))


def status_name(status):
    return STATUS_NAMES.get(status, "STATUS_{}".format(status))


class CommandSender:
    def __init__(
        self,
        target_ip=None,
        target_port=5006,
        bind_ip="0.0.0.0",
        bind_port=0,
    ):
        self.target_ip = target_ip
        self.target_port = target_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((bind_ip, bind_port))
        self.sock.setblocking(False)

        self.cmd_seq = 0
        self.pending = {}

        self.sent_count = 0
        self.ack_count = 0
        self.bad_ack_count = 0

        self.last_sent = None
        self.last_ack = None
        self.last_error = ""
        self.last_rtt_ms = None

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def set_target_ip(self, ip):
        self.target_ip = ip

    def has_target(self):
        return bool(self.target_ip)

    def next_seq(self):
        self.cmd_seq = (self.cmd_seq + 1) & 0xFFFF
        if self.cmd_seq == 0:
            self.cmd_seq = 1
        return self.cmd_seq

    def make_command_packet(
        self,
        cmd_type,
        target_id=0,
        param_id=0,
        value=0,
    ):
        seq = self.next_seq()

        packet = struct.pack(
            COMMAND_FORMAT,
            COMMAND_MAGIC,
            VERSION,
            COMMAND_SIZE,
            seq,
            int(cmd_type) & 0xFF,
            int(target_id) & 0xFF,
            int(param_id),
            int(value),
        )

        return seq, packet

    def send(
        self,
        cmd_type,
        target_id=0,
        param_id=0,
        value=0,
    ):
        if not self.target_ip:
            self.last_error = "no target ip"
            return None

        seq, packet = self.make_command_packet(
            cmd_type=cmd_type,
            target_id=target_id,
            param_id=param_id,
            value=value,
        )

        now = time.monotonic()

        try:
            self.sock.sendto(packet, (self.target_ip, self.target_port))
        except OSError as exc:
            self.last_error = "send error: {}".format(exc)
            return None

        self.sent_count += 1
        self.pending[seq] = {
            "time": now,
            "cmd_type": cmd_type,
            "target_id": target_id,
            "param_id": param_id,
            "value": value,
        }

        self.last_sent = {
            "seq": seq,
            "cmd_type": cmd_type,
            "cmd_name": cmd_name(cmd_type),
            "target_ip": self.target_ip,
            "target_port": self.target_port,
            "time": now,
        }

        return self.last_sent

    def parse_ack_packet(self, data):
        if len(data) != ACK_SIZE:
            return None, "bad_ack_size"

        try:
            values = struct.unpack(ACK_FORMAT, data)
        except struct.error:
            return None, "ack_struct_error"

        magic = values[0]
        version = values[1]
        packet_size = values[2]
        cmd_seq = values[3]
        cmd_type = values[4]
        status = values[5]

        if magic != ACK_MAGIC:
            return None, "bad_ack_magic"

        if version != VERSION:
            return None, "bad_ack_version"

        if packet_size != ACK_SIZE:
            return None, "bad_ack_packet_size"

        ack = {
            "cmd_seq": cmd_seq,
            "cmd_type": cmd_type,
            "cmd_name": cmd_name(cmd_type),
            "status": status,
            "status_name": status_name(status),
        }
        return ack, None

    def poll_acks(self):
        acks = []

        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
            except BlockingIOError:
                break
            except OSError:
                break

            ack, reason = self.parse_ack_packet(data)
            if ack is None:
                self.bad_ack_count += 1
                self.last_error = reason or "bad ack"
                continue

            now = time.monotonic()
            pending = self.pending.pop(ack["cmd_seq"], None)

            if pending is not None:
                rtt_ms = int((now - pending["time"]) * 1000)
            else:
                rtt_ms = None

            ack["from"] = addr
            ack["rtt_ms"] = rtt_ms

            self.ack_count += 1
            self.last_ack = ack
            self.last_rtt_ms = rtt_ms
            acks.append(ack)

        return acks

    def stats_text(self):
        target = "{}:{}".format(self.target_ip, self.target_port) if self.target_ip else "unknown:{}".format(self.target_port)

        if self.last_sent is None:
            last_sent = "none"
        else:
            last_sent = "#{seq} {cmd_name}".format(**self.last_sent)

        if self.last_ack is None:
            last_ack = "none"
        else:
            rtt = self.last_ack["rtt_ms"]
            if rtt is None:
                rtt_text = "rtt=?"
            else:
                rtt_text = "rtt={}ms".format(rtt)

            last_ack = "#{cmd_seq} {cmd_name} {status_name} {}".format(
                rtt_text,
                **self.last_ack
            )

        return (
            "command_target={}  sent={}  ack={}  bad_ack={}  "
            "last_sent={}  last_ack={}  error={}"
        ).format(
            target,
            self.sent_count,
            self.ack_count,
            self.bad_ack_count,
            last_sent,
            last_ack,
            self.last_error,
        )