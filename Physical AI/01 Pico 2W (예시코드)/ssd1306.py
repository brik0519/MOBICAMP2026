# ssd1306.py
# MicroPython SSD1306 OLED driver
# Modified I2C version:
# - write_cmd(): uses control byte 0x00
# - write_data(): avoids writevto(), sends small chunks

from micropython import const
import framebuf


# SSD1306 command constants
SET_CONTRAST        = const(0x81)
SET_ENTIRE_ON       = const(0xA4)
SET_NORM_INV        = const(0xA6)
SET_DISP            = const(0xAE)
SET_MEM_ADDR        = const(0x20)
SET_COL_ADDR        = const(0x21)
SET_PAGE_ADDR       = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP       = const(0xA0)
SET_MUX_RATIO       = const(0xA8)
SET_COM_OUT_DIR     = const(0xC0)
SET_DISP_OFFSET     = const(0xD3)
SET_COM_PIN_CFG     = const(0xDA)
SET_DISP_CLK_DIV    = const(0xD5)
SET_PRECHARGE       = const(0xD9)
SET_VCOM_DESEL      = const(0xDB)
SET_CHARGE_PUMP     = const(0x8D)


class SSD1306:
    def __init__(self, width, height, external_vcc=False):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8

        self.buffer = bytearray(self.pages * self.width)

        self.framebuf = framebuf.FrameBuffer(
            self.buffer,
            self.width,
            self.height,
            framebuf.MONO_VLSB
        )

        self.poweron()
        self.init_display()

    def init_display(self):
        init_sequence = (
            SET_DISP | 0x00,                 # display off
            SET_MEM_ADDR, 0x00,              # horizontal addressing mode
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,            # column address 127 mapped to SEG0
            SET_MUX_RATIO, self.height - 1,
            SET_COM_OUT_DIR | 0x08,          # scan from COM[N] to COM0
            SET_DISP_OFFSET, 0x00,
            SET_COM_PIN_CFG, 0x12 if self.height == 64 else 0x02,
            SET_DISP_CLK_DIV, 0x80,
            SET_PRECHARGE, 0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL, 0x30,
            SET_CONTRAST, 0xFF,
            SET_ENTIRE_ON,                   # output follows RAM content
            SET_NORM_INV,                    # normal display
            SET_CHARGE_PUMP, 0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01                  # display on
        )

        for cmd in init_sequence:
            self.write_cmd(cmd)

        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        # I2C version does not need separate power control.
        pass

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1

        # Some narrow displays use a horizontal offset.
        if self.width == 64:
            x0 += 32
            x1 += 32

        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)

        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)

        self.write_data(self.buffer)

    # FrameBuffer wrapper methods
    def fill(self, color):
        self.framebuf.fill(color)

    def pixel(self, x, y, color=None):
        if color is None:
            return self.framebuf.pixel(x, y)
        self.framebuf.pixel(x, y, color)

    def scroll(self, dx, dy):
        self.framebuf.scroll(dx, dy)

    def text(self, string, x, y, color=1):
        self.framebuf.text(string, x, y, color)

    def line(self, x1, y1, x2, y2, color):
        self.framebuf.line(x1, y1, x2, y2, color)

    def hline(self, x, y, w, color):
        self.framebuf.hline(x, y, w, color)

    def vline(self, x, y, h, color):
        self.framebuf.vline(x, y, h, color)

    def rect(self, x, y, w, h, color):
        self.framebuf.rect(x, y, w, h, color)

    def fill_rect(self, x, y, w, h, color):
        self.framebuf.fill_rect(x, y, w, h, color)

    def blit(self, fbuf, x, y):
        self.framebuf.blit(fbuf, x, y)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        # 0x00 = command stream
        self.i2c.writeto(self.addr, bytes([0x00, cmd]))

    def write_data(self, buf):
        # 0x40 = data stream
        # Send small chunks instead of using writevto().
        # This is slower but often more stable on breadboard wiring.
        chunk_size = 16

        for start in range(0, len(buf), chunk_size):
            end = start + chunk_size
            chunk = buf[start:end]
            self.i2c.writeto(self.addr, b'\x40' + chunk)