#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import arty
from litex.soc.cores import *
from litex.soc.integration import soc_core
from litex.soc.cores import gpio
from litex.soc.integration.builder import *


# ClassicLed ------------------------------------------------------------------------------------------

class ClassicLed(gpio.GPIOOut):
    def __init__(self, pads):
        gpio.GPIOOut.__init__(self, pads)
        
        # user led 0
        self.submodules.userled0 = ClassicLed(usb_pads.led)
        self.add_csr("userled0")

# Build --------------------------------------------------------------------------------------------

def main():
    sys_clk_freq=100e6

    platform = arty.Platform()
    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           integrated_rom_size=0x8000,
                           integrated_sram_size=0x8000)
    led_pad = platform.request("user_led", 0)
    soc.submodules.leds = gpio.GPIOOut(led_pad)
    builder = Builder(soc)
    builder.build()


if __name__ == "__main__":
    main()
