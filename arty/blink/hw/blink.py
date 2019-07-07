#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import arty
from litex.soc.cores import *
from litex.soc.integration import soc_core
from litex.soc.cores import gpio
from litex.soc.integration.builder import *


# Build --------------------------------------------------------------------------------------------

def main():
    platform = arty.Platform()
    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           integrated_rom_size=0x8000,
                           integrated_sram_size=0x8000)
    led_pad = platform.request("rgb_led", 0)
    print(led_pad)
    soc.submodules.leds = gpio.GPIOOut(led_pad.g)
    soc.add_csr("leds")
    
    builder = Builder(soc)
    for package in builder.software_packages:
        if package[0] == "bios":
            builder.software_packages.remove(package)
            break
    builder.add_software_package("bios", src_dir="../../../../sw")
    builder.build()


if __name__ == "__main__":
    main()
