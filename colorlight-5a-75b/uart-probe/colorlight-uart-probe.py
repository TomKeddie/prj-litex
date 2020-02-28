#!/usr/bin/env python3

# This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

# Disclaimer: This SoC is still a Proof of Concept with large timings violations on the IP/UDP and
# Etherbone stack that need to be optimized. It was initially just used to validate the reversed
# pinout but happens to work on hardware...

import argparse
import sys

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex_boards.platforms import colorlight_5a_75b

from litex.soc.cores.clock import *
from litex.soc.cores.uart import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal

_serial = [
    ("serial", 0,
        Subsignal("rx", Pins("M1")),
        Subsignal("tx", Pins("M2"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33")
    ),
]

_test = [
    ("test", 0,
        Subsignal("tx", Pins("F3"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33")
    ),
]

#  ----------------------------------------------------------------------------------------------
class RS232TextSender(Module):
    def __init__(self, pads, clk_freq, baudrate=115200):

        tuning_word = Signal(32, reset=int((baudrate/clk_freq)*2**32))
        self.source = stream.Endpoint([("data", 8)])
        self.submodules.tx = RS232PHYTX(pads, tuning_word)

        self.comb += [
            self.tx.sink.valid.eq(1),
            self.tx.sink.data.eq(ord('@')),
        ]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # Clk / Rst
        clk25 = platform.request("clk25")
        platform.add_period_constraint(clk25, 1e9/25e6)

        # PLL
        self.submodules.pll = pll = ECP5PLL()

        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll.locked)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision, **kwargs):
        platform     = colorlight_5a_75b.Platform(revision=revision)
        sys_clk_freq = int(12.5e6)

        # SoCCore ----------------------------------------------------------------------------------
        platform.add_extension(_serial)
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs, cpu_variant="standard")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # uarts ------------------------------------------------------------------------------------
        platform.add_extension(_test)
        self.submodules.test0 = RS232TextSender(platform.request("test"), sys_clk_freq)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Colorlight 5A-75B")
    builder_args(parser)
    soc_core_args(parser)
    parser.add_argument("--revision", default="7.0", type=str, help="Board revision 7.0 (default) or 6.1")
    args = parser.parse_args()
    
    argdict = soc_core_argdict(args)
    soc = BaseSoC(args.revision, **argdict)
    argdict = builder_argdict(args)
    argdict["output_dir"]="build"
    builder = Builder(soc, **argdict)
    builder.csr_csv="csr.csv"
    builder.build()

if __name__ == "__main__":
    main()
