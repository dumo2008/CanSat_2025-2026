# SH1106 driver for MicroPython
# Fully compatible with SSD1306 FrameBuffer API

from micropython import const
import framebuf

_SH1106_SET_PAGE_ADDRESS = const(0xB0)
_SH1106_SET_COLUMN_ADDRESS_LOW = const(0x00)
_SH1106_SET_COLUMN_ADDRESS_HIGH = const(0x10)

class SH1106(framebuf.FrameBuffer):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.width = width
        self.height = height
        self.i2c = i2c
        self.addr = addr
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def write_cmd(self, cmd):
        self.i2c.writeto(self.addr, bytearray([0x00, cmd]))

    def write_data(self, buf):
        self.i2c.writeto(self.addr, b"\x40" + buf)

    def init_display(self):
        for cmd in (
            0xAE,  # display off
            0x20, 0x00,  # horizontal addressing mode
            0xB0,        # page start
            0xC8,        # com scan direction
            0x00,        # low column
            0x10,        # high column
            0x40,        # start line
            0x81, 0x7F,  # contrast
            0xA1,        # segment remap
            0xA6,        # normal display
            0xA8, 0x3F,  # multiplex
            0xD3, 0x00,  # display offset
            0xD5, 0x80,  # clock divide
            0xD9, 0x22,  # precharge
            0xDA, 0x12,  # com pins
            0xDB, 0x20,  # vcom detect
            0x8D, 0x14,  # charge pump
            0xAF,        # display ON
        ):
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def show(self):
        for page in range(self.pages):
            self.write_cmd(_SH1106_SET_PAGE_ADDRESS | page)
            self.write_cmd(_SH1106_SET_COLUMN_ADDRESS_LOW | (0 & 0x0F))
            self.write_cmd(_SH1106_SET_COLUMN_ADDRESS_HIGH | (0 >> 4))
            start = self.width * page
            end = start + self.width
            self.write_data(self.buffer[start:end])
