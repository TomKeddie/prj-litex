#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import fomu_evt
from litex.soc.cores import *
from litex.soc.integration import soc_core
from litex.soc.cores import gpio
from litex.soc.integration.builder import *

from gateware import up5kspram

# Build --------------------------------------------------------------------------------------------

def main():
    platform = fomu_evt.Platform()
    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           cpu_variant="lite",
                           integrated_rom_size=0x2000,
                           integrated_sram_size=0)
    # SPRAM- UP5K has single port RAM, might as well use it as SRAM to
    # free up scarce block RAM.
    spram_size = 128*1024
    soc.submodules.spram = up5kspram.Up5kSPRAM(size=spram_size)
    soc.register_mem("sram", 0x10000000, soc.spram.bus, spram_size)
    
    led_pad = platform.request("user_led_n")
    soc.submodules.leds = gpio.GPIOOut(led_pad)
    builder = Builder(soc)
    for package in builder.software_packages:
        if package[0] == "bios":
            builder.software_packages.remove(package)
            break
    builder.add_software_package("bios", src_dir="../../../../sw")
    builder.build()


if __name__ == "__main__":
    main()
