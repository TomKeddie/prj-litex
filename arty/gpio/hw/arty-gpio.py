#!/usr/bin/env python3

import argparse

from migen import *

from litex.boards.platforms import arty
from litex.soc.cores import *
from litex.soc.integration import soc_core
from litex.soc.interconnect.csr import *
from litex.soc.cores import gpio
from litex.soc.cores.uart import UARTWishboneBridge
from litex.soc.integration.builder import *
from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal
from migen.genlib.cdc import MultiReg


_pmods = [
    ("pmoda", 0,
     Subsignal("p0", Pins("pmoda:0")),
     Subsignal("p1", Pins("pmoda:1")),
     Subsignal("p2", Pins("pmoda:2")),
     Subsignal("p3", Pins("pmoda:3")),
     Subsignal("p4", Pins("pmoda:4")),
     Subsignal("p5", Pins("pmoda:5")),
     Subsignal("p6", Pins("pmoda:6")),
     Subsignal("p7", Pins("pmoda:7")),
     IOStandard("LVCMOS33")
    ),
    ("pmodb", 0,
     Subsignal("p0", Pins("pmodb:0")),
     Subsignal("p1", Pins("pmodb:1")),
     Subsignal("p2", Pins("pmodb:2")),
     Subsignal("p3", Pins("pmodb:3")),
     Subsignal("p4", Pins("pmodb:4")),
     Subsignal("p5", Pins("pmodb:5")),
     Subsignal("p6", Pins("pmodb:6")),
     Subsignal("p7", Pins("pmodb:7")),
     IOStandard("LVCMOS33")
    ),
    ("pmodc", 0,
     Subsignal("p0", Pins("pmodc:0")),
     Subsignal("p1", Pins("pmodc:1")),
     Subsignal("p2", Pins("pmodc:2")),
     Subsignal("p3", Pins("pmodc:3")),
     Subsignal("p4", Pins("pmodc:4")),
     Subsignal("p5", Pins("pmodc:5")),
     Subsignal("p6", Pins("pmodc:6")),
     Subsignal("p7", Pins("pmodc:7")),
     IOStandard("LVCMOS33")
    ),
    ("pmodd", 0,
     Subsignal("p0", Pins("pmodd:0")),
     Subsignal("p1", Pins("pmodd:1")),
     Subsignal("p2", Pins("pmodd:2")),
     Subsignal("p3", Pins("pmodd:3")),
     Subsignal("p4", Pins("pmodd:4")),
     Subsignal("p5", Pins("pmodd:5")),
     Subsignal("p6", Pins("pmodd:6")),
     Subsignal("p7", Pins("pmodd:7")),
     IOStandard("LVCMOS33")
    ),
]

_serial2 = [
    ("serial2", 0,
        Subsignal("rx", Pins("N15")),
        Subsignal("tx", Pins("M16"), Misc("PULLUP")),
        IOStandard("LVCMOS33")
    ),
]

class GPIOBidirectional(Module, AutoCSR):
    def __init__(self, soc, pad_name):
        pins_pad = soc.platform.request(pad_name)
        pins_t = TSTriple(len(pins_pad))

        self.specials += pins_t.get_tristate(Cat(pins_pad.p0, pins_pad.p1, pins_pad.p2, pins_pad.p3, pins_pad.p4, pins_pad.p5, pins_pad.p6, pins_pad.p7))
        self._pins_in = CSRStatus(len(pins_t))
        self._pins_out = CSRStorage(len(pins_t))
        self._pins_oe = CSRStorage(len(pins_t))

        self.specials += MultiReg(pins_t.i, self._pins_in.status)
        pins_t.o.eq(self._pins_out.storage)
        pins_t.oe.eq(self._pins_oe.storage)

    
        
# Build --------------------------------------------------------------------------------------------

def main():
    platform = arty.Platform()
    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           cpu_variant="lite+debug",
                           integrated_rom_size=0x8000,
                           integrated_sram_size=0x8000)

    # rgb led
    led_pad = platform.request("rgb_led", 0)
    soc.submodules.leds = gpio.GPIOOut(led_pad.r)
    soc.add_csr("leds")

    # pmods
    platform.add_extension(_pmods)
    soc.submodules.pmoda = GPIOBidirectional(soc, "pmoda")
    soc.submodules.pmodb = GPIOBidirectional(soc, "pmodb")
    soc.submodules.pmodc = GPIOBidirectional(soc, "pmodc")
    soc.submodules.pmodd = GPIOBidirectional(soc, "pmodd")
    soc.add_csr("pmoda")
    soc.add_csr("pmodb")
    soc.add_csr("pmodc")
    soc.add_csr("pmodd")

    # debug
    # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
    platform.add_extension(_serial2)
    soc.submodules.uartbridge = UARTWishboneBridge(platform.request("serial2"), int(sys_clk_freq), baudrate=115200)
    soc.add_wb_master(soc.uartbridge.wishbone)
    soc.register_mem("vexriscv_debug", 0xf00f0000, soc.cpu.debug_bus, 0x10)

    builder = Builder(soc, csr_csv="csr.csv")
#     for package in builder.software_packages:
#         if package[0] == "bios":
#             builder.software_packages.remove(package)
#             break
#     builder.add_software_package("bios", src_dir="../../../../sw")

    builder.build()
 

if __name__ == "__main__":
    main()
