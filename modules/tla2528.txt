# tla2528.py
# PAI-Car TLA2528 ADC driver
# MCU: Raspberry Pi Pico 2 W
# ADC: TI TLA2528, 8-channel, 12-bit, I2C
#
# Main interface:
#   adc = TLA2528(i2c_id=0, sda_pin=4, scl_pin=5, freq=400_000)
#   adc.begin(verbose=True)
#   values = adc.read_all_raw10()
#
# read_all_raw10():
#   returns 8 values in 0..1023
#
# PAI-Car expected use:
#   from tla2528 import TLA2528
#   adc = TLA2528(i2c_id=0, sda_pin=4, scl_pin=5, freq=400_000)
#   adc.begin()
#   raw = adc.read_all_raw10()

from machine import Pin, I2C
from time import sleep_ms


# ------------------------------------------------------------
# TLA2528 command opcodes
# ------------------------------------------------------------
# Datasheet command opcodes:
#   0x10 = single register read
#   0x08 = single register write
#   0x18 = set bit
#   0x20 = clear bit

CMD_WRITE_REG = 0x08
CMD_READ_REG  = 0x10
CMD_SET_BIT   = 0x18
CMD_CLEAR_BIT = 0x20


# ------------------------------------------------------------
# TLA2528 registers
# ------------------------------------------------------------

REG_GENERAL_CFG     = 0x01
REG_PIN_CFG         = 0x05
REG_SEQUENCE_CFG    = 0x10
REG_CHANNEL_SEL     = 0x11
REG_AUTO_SEQ_CH_SEL = 0x12


# ------------------------------------------------------------
# SEQUENCE_CFG bits
# ------------------------------------------------------------
# bit 4    : SEQ_START
# bit 1..0 : SEQ_MODE
#
# SEQ_MODE:
#   00b = manual mode
#   01b = auto-sequence mode
#
# Therefore:
#   0x00 = manual mode, stopped
#   0x01 = auto-sequence mode, stopped
#   0x11 = auto-sequence mode, started

SEQ_MODE_MANUAL = 0x00
SEQ_MODE_AUTO   = 0x01
SEQ_START       = 0x10

SEQ_AUTO_STOP   = SEQ_MODE_AUTO
SEQ_AUTO_START  = SEQ_START | SEQ_MODE_AUTO


class TLA2528:
    def __init__(
        self,
        i2c=None,
        i2c_id=0,
        sda_pin=4,
        scl_pin=5,
        freq=400_000,
        address=None,
        startup_delay_ms=300,
        retry_count=3,
        retry_delay_ms=5
    ):
        """
        TLA2528 driver for MicroPython.

        Parameters:
            i2c:
                Optional pre-created I2C object.
                If None, this class creates I2C(i2c_id, sda, scl, freq).

            address:
                7-bit I2C address.
                If None, scan 0x10~0x17 and use the first detected address.

            startup_delay_ms:
                Delay before first I2C access.
                Useful after soft reboot or power-up.

            retry_count:
                Number of retries for I2C reads/writes.
        """

        if i2c is None:
            self.i2c = I2C(
                i2c_id,
                sda=Pin(sda_pin),
                scl=Pin(scl_pin),
                freq=freq
            )
        else:
            self.i2c = i2c

        self.address = address
        self.startup_delay_ms = startup_delay_ms
        self.retry_count = retry_count
        self.retry_delay_ms = retry_delay_ms

        # Reusable buffers to reduce allocation inside control loop.
        self._read_buf_16 = bytearray(16)
        self._raw10 = [0] * 8
        self._raw12 = [0] * 8

    # ------------------------------------------------------------
    # Low-level I2C with retry
    # ------------------------------------------------------------

    def _writeto(self, data):
        last_err = None

        for _ in range(self.retry_count):
            try:
                self.i2c.writeto(self.address, data)
                return
            except OSError as e:
                last_err = e
                sleep_ms(self.retry_delay_ms)

        raise last_err

    def _readfrom_into(self, buf):
        last_err = None

        for _ in range(self.retry_count):
            try:
                self.i2c.readfrom_into(self.address, buf)
                return
            except OSError as e:
                last_err = e
                sleep_ms(self.retry_delay_ms)

        raise last_err

    def _readfrom(self, nbytes):
        last_err = None

        for _ in range(self.retry_count):
            try:
                return self.i2c.readfrom(self.address, nbytes)
            except OSError as e:
                last_err = e
                sleep_ms(self.retry_delay_ms)

        raise last_err

    # ------------------------------------------------------------
    # Address scan
    # ------------------------------------------------------------

    def scan(self):
        return self.i2c.scan()

    def find_address(self):
        devices = self.i2c.scan()

        for addr in range(0x10, 0x18):
            if addr in devices:
                return addr

        return None

    # ------------------------------------------------------------
    # Register access
    # ------------------------------------------------------------

    def write_reg(self, reg, value):
        self._writeto(
            bytes([
                CMD_WRITE_REG,
                reg & 0xFF,
                value & 0xFF
            ])
        )

    def read_reg(self, reg):
        self._writeto(
            bytes([
                CMD_READ_REG,
                reg & 0xFF
            ])
        )

        data = self._readfrom(1)
        return data[0]

    def set_bit(self, reg, mask):
        self._writeto(
            bytes([
                CMD_SET_BIT,
                reg & 0xFF,
                mask & 0xFF
            ])
        )

    def clear_bit(self, reg, mask):
        self._writeto(
            bytes([
                CMD_CLEAR_BIT,
                reg & 0xFF,
                mask & 0xFF
            ])
        )

    # ------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------

    def begin(self, verbose=False):
        """
        Initialize TLA2528 for 8-channel auto-sequence read.

        Configuration:
            - all AIN0~AIN7 as analog input
            - auto-sequence enabled for all 8 channels
            - sequence started
        """

        devices = self.i2c.scan()

        if verbose:
            print("I2C scan:", [hex(x) for x in devices])

        if self.address is None:
            self.address = self.find_address()

        if self.address is None:
            raise RuntimeError(
                "TLA2528 not found. Check SDA/SCL, power, pull-up, ADDR pin."
            )

        if verbose:
            print("TLA2528 address:", hex(self.address))

        # 1. Stop sequencing first.
        self.write_reg(REG_SEQUENCE_CFG, 0x00)

        # 2. Configure all pins as analog inputs.
        # PIN_CFG bit = 0 means analog input.
        self.write_reg(REG_PIN_CFG, 0x00)

        # 3. Enable AIN0~AIN7 in auto-sequence.
        self.write_reg(REG_AUTO_SEQ_CH_SEL, 0xFF)

        # 4. Select auto-sequence mode, but keep stopped first.
        self.write_reg(REG_SEQUENCE_CFG, SEQ_AUTO_STOP)

        # 5. Start sequence from CH0 upward.
        self.write_reg(REG_SEQUENCE_CFG, SEQ_AUTO_START)

        # 6. One dummy read to align/prime sequence.
        # The first frame after mode change can be discarded.
        self._dummy_read_8ch()

        if verbose:
            print("PIN_CFG        =", hex(self.read_reg(REG_PIN_CFG)))
            print("AUTO_SEQ_CHSEL =", hex(self.read_reg(REG_AUTO_SEQ_CH_SEL)))
            print("SEQUENCE_CFG   =", hex(self.read_reg(REG_SEQUENCE_CFG)))

        return True

    def _dummy_read_8ch(self):
        try:
            self._readfrom_into(self._read_buf_16)
        except OSError:
            # If a dummy read fails just once during startup,
            # let the next real read/retry handle the bus.
            pass

    # ------------------------------------------------------------
    # Mode control
    # ------------------------------------------------------------

    def start_sequence(self):
        self.write_reg(REG_SEQUENCE_CFG, SEQ_AUTO_START)

    def stop_sequence(self):
        self.write_reg(REG_SEQUENCE_CFG, SEQ_AUTO_STOP)

    def set_manual_mode(self):
        self.write_reg(REG_SEQUENCE_CFG, SEQ_MODE_MANUAL)

    # ------------------------------------------------------------
    # Fast 8-channel auto-sequence read
    # ------------------------------------------------------------

    def read_all_raw12(self):
        """
        Read all 8 channels using auto-sequence mode.

        Return:
            [CH0, CH1, ..., CH7] in 0..4095

        Note:
            This assumes AUTO_SEQ_CH_SEL = 0xFF and sequence started.
        """

        buf = self._read_buf_16
        self._readfrom_into(buf)

        out = self._raw12

        # TLA2528 12-bit data format:
        #   byte0 = D11..D4
        #   byte1 = D3..D0 xxxx
        out[0] = (buf[0]  << 4) | (buf[1]  >> 4)
        out[1] = (buf[2]  << 4) | (buf[3]  >> 4)
        out[2] = (buf[4]  << 4) | (buf[5]  >> 4)
        out[3] = (buf[6]  << 4) | (buf[7]  >> 4)
        out[4] = (buf[8]  << 4) | (buf[9]  >> 4)
        out[5] = (buf[10] << 4) | (buf[11] >> 4)
        out[6] = (buf[12] << 4) | (buf[13] >> 4)
        out[7] = (buf[14] << 4) | (buf[15] >> 4)

        return out

    def read_all_raw10(self):
        """
        Read all 8 channels using auto-sequence mode.

        Return:
            [CH0, CH1, ..., CH7] in 0..1023

        This is the method expected by pai_line_sensor.py.
        """

        buf = self._read_buf_16
        self._readfrom_into(buf)

        out = self._raw10

        # Convert 12-bit ADC result to 10-bit by shifting right 2 bits.
        out[0] = ((buf[0]  << 4) | (buf[1]  >> 4)) >> 2
        out[1] = ((buf[2]  << 4) | (buf[3]  >> 4)) >> 2
        out[2] = ((buf[4]  << 4) | (buf[5]  >> 4)) >> 2
        out[3] = ((buf[6]  << 4) | (buf[7]  >> 4)) >> 2
        out[4] = ((buf[8]  << 4) | (buf[9]  >> 4)) >> 2
        out[5] = ((buf[10] << 4) | (buf[11] >> 4)) >> 2
        out[6] = ((buf[12] << 4) | (buf[13] >> 4)) >> 2
        out[7] = ((buf[14] << 4) | (buf[15] >> 4)) >> 2

        return out

    # ------------------------------------------------------------
    # Slower manual read, for debug only
    # ------------------------------------------------------------

    def read_channel_raw12_manual(self, ch):
        """
        Slow manual channel read.
        Use only for debugging.
        """

        if ch < 0 or ch > 7:
            raise ValueError("channel must be 0~7")

        # Stop auto sequence and select manual mode.
        self.write_reg(REG_SEQUENCE_CFG, SEQ_MODE_MANUAL)

        # Select channel.
        self.write_reg(REG_CHANNEL_SEL, ch)

        # Read one conversion frame.
        data = self._readfrom(2)

        raw12 = (data[0] << 4) | (data[1] >> 4)
        return raw12

    def read_all_raw10_manual(self):
        """
        Slow 8-channel manual read.
        Use only to compare/debug auto-sequence behavior.
        """

        out = [0] * 8

        self.write_reg(REG_SEQUENCE_CFG, SEQ_MODE_MANUAL)

        for ch in range(8):
            self.write_reg(REG_CHANNEL_SEL, ch)
            data = self._readfrom(2)
            out[ch] = ((data[0] << 4) | (data[1] >> 4)) >> 2

        # Restore auto sequence after debug read.
        self.write_reg(REG_SEQUENCE_CFG, SEQ_AUTO_START)

        return out

    # ------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------

    def raw12_to_voltage(self, raw12, vref=3.3):
        return raw12 * vref / 4095

    def raw10_to_voltage(self, raw10, vref=3.3):
        return raw10 * vref / 1023