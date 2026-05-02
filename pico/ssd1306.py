# MicroPython SSD1306 OLED driver, I2C and SPI interfaces
# Source: Official MicroPython repository

import framebuf


class SSD1306:
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        self.framebuf = framebuf.FrameBuffer(self.buffer, self.width, self.height,
                                             framebuf.MONO_VLSB)

        self.poweron()
        self.init_display()

    def init_display(self):
        for cmd in (
            0xAE,         # DISPLAY_OFF
            0xA4,         # DISPLAY_ALL_ON_RESUME
            0xD5, 0x80,   # SET_DISPLAY_CLK_DIV
            0xA8, self.height - 1,   # SET_MULTIPLEX
            0xD3, 0x00,   # SET_DISPLAY_OFFSET
            0x40,         # SET_START_LINE
            0x8D, 0x10 if self.external_vcc else 0x14,  # CHARGE_PUMP
            0x20, 0x00,   # MEMORY_ADDR_MODE
            0xA1,         # SEG_REMAP
            0xC8,         # COM_SCAN_DEC
            0xDA, 0x12,   # SET_COM_PINS
            0x81, 0xCF,   # SET_CONTRAST
            0xD9, 0xF1,   # SET_PRECHARGE
            0xDB, 0x40,   # SET_VCOM_DETECT
            0xA6,         # NORMAL_DISPLAY
            0xAF):        # DISPLAY_ON
            self.write_cmd(cmd)

        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(0xAE)

    def poweron(self):
        pass

    def contrast(self, contrast):
        self.write_cmd(0x81)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(0xA7 if invert else 0xA6)

    # FrameBuffer passt-through helpers
    def fill(self, col):
        self.framebuf.fill(col)

    def pixel(self, x, y, col):
        self.framebuf.pixel(x, y, col)

    def hline(self, x, y, w, col):
        self.framebuf.hline(x, y, w, col)

    def vline(self, x, y, h, col):
        self.framebuf.vline(x, y, h, col)

    def line(self, x1, y1, x2, y2, col):
        self.framebuf.line(x1, y1, x2, y2, col)

    def rect(self, x, y, w, h, col):
        self.framebuf.rect(x, y, w, h, col)

    def fill_rect(self, x, y, w, h, col):
        self.framebuf.fill_rect(x, y, w, h, col)

    def text(self, string, x, y, col=1):
        self.framebuf.text(string, x, y, col)

    def scroll(self, dx, dy):
        self.framebuf.scroll(dx, dy)

    def show(self):
        raise NotImplementedError


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.i2c.writeto(self.addr, b'\x40' + buf)

    def show(self):
        for page in range(self.pages):
            self.write_cmd(0xB0 | page)
            self.write_cmd(0x00)
            self.write_cmd(0x10)
            self.write_data(self.buffer)
