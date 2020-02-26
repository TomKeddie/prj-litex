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
from litex.soc.cores.uart import UARTWishboneBridge
from litex.soc.cores.gpio import GPIOTristate
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal

## _serial2 = [
##     ("serial2", 0,
##         Subsignal("rx", Pins("F12")),
##         Subsignal("tx", Pins("F3"), Misc("PULLUP=TRUE")),
##         IOStandard("LVCMOS33")
##     ),
## ]
_serial = [
    ("serial", 0,
        Subsignal("rx", Pins("M1")),
        Subsignal("tx", Pins("M2"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33")
    ),
]

_gpio = [
    ("j1_gpio", 0, Subsignal("pins", Pins("j1:0 j1:1 j1:2 j1:4 j1:5 j1:6 j1:7 j1:8 j1:9 j1:10 j1:11 j1:12 j1:13 j1:14")), IOStandard("LVCMOS33")),
    ("j2_gpio", 0, Subsignal("pins", Pins("j2:0 j2:1 j2:2 j2:4 j2:5 j2:6")), IOStandard("LVCMOS33")),
    ("j3_gpio", 0, Subsignal("pins", Pins("j3:0 j3:1 j3:2 j3:4 j3:5 j3:6")), IOStandard("LVCMOS33")),
    ("j4_gpio", 0, Subsignal("pins", Pins("j4:0 j4:1 j4:2 j4:5 j4:6")), IOStandard("LVCMOS33")),
    ("j5_gpio", 0, Subsignal("pins", Pins("j5:0 j5:1 j5:2 j5:4 j5:5 j5:6")), IOStandard("LVCMOS33")),
    ("j6_gpio", 0, Subsignal("pins", Pins("j6:0 j6:1 j6:2 j6:4 j6:5 j6:6")), IOStandard("LVCMOS33")),
    ("j7_gpio", 0, Subsignal("pins", Pins("j7:0 j7:1 j7:2 j7:4 j7:5 j7:6")), IOStandard("LVCMOS33")),
    ("j8_gpio", 0, Subsignal("pins", Pins("j8:0 j8:1 j8:2 j8:4 j8:5 j8:6")), IOStandard("LVCMOS33")),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        # Clk / Rst
        clk25 = platform.request("clk25")
        rst_n = platform.request("user_btn_n", 0)
        platform.add_period_constraint(clk25, 1e9/25e6)

        # PLL
        self.submodules.pll = pll = ECP5PLL()

        pll.register_clkin(clk25, 25e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~pll.locked | ~rst_n)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, revision, **kwargs):
        platform     = colorlight_5a_75b.Platform(revision=revision)
        sys_clk_freq = int(125e6)

        # Extensions ----------------------------------------------------------------------------------
        platform.add_extension(_serial)
        platform.add_extension(_gpio)
##        platform.add_extension(_serial2)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs, cpu_variant="standard+debug")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # GPIO --------------------------------------------------------------------------------------
        j1_gpio_pads = platform.request("j1_gpio")
        j2_gpio_pads = platform.request("j2_gpio")
        j3_gpio_pads = platform.request("j3_gpio")
        j4_gpio_pads = platform.request("j4_gpio")
        j5_gpio_pads = platform.request("j5_gpio")
        j6_gpio_pads = platform.request("j6_gpio")
        j7_gpio_pads = platform.request("j7_gpio")
        j8_gpio_pads = platform.request("j8_gpio")
        self.submodules.j1_gpio = GPIOTristate(j1_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j2_gpio = GPIOTristate(j2_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j3_gpio = GPIOTristate(j3_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j4_gpio = GPIOTristate(j4_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j5_gpio = GPIOTristate(j5_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j6_gpio = GPIOTristate(j6_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j7_gpio = GPIOTristate(j7_gpio_pads.pins, per_pad_oe=True)
        self.submodules.j8_gpio = GPIOTristate(j8_gpio_pads.pins, per_pad_oe=True)
        self.add_csr("j1_gpio")
        self.add_csr("j2_gpio")
        self.add_csr("j3_gpio")
        self.add_csr("j4_gpio")
        self.add_csr("j5_gpio")
        self.add_csr("j6_gpio")
        self.add_csr("j7_gpio")
        self.add_csr("j8_gpio")

##         # Debug bridge ----------------------------------------------------------------------------------
##         # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
##         self.submodules.uartbridge = UARTWishboneBridge(platform.request("serial2"), int(sys_clk_freq), baudrate=115200)
##         self.add_wb_master(self.uartbridge.wishbone)
##         self.register_mem("vexriscv_debug", 0xf00f0000, self.cpu.debug_bus, 0x10)

        # Led --------------------------------------------------------------------------------------
        led_counter = Signal(32)
        self.sync += led_counter.eq(led_counter + 1)
        self.comb += platform.request("user_led_n", 0).eq(led_counter[26])

# EtherboneSoC -------------------------------------------------------------------------------------

class EtherboneSoC(BaseSoC):
    def __init__(self, eth_phy=0, **kwargs):
        BaseSoC.__init__(self, **kwargs)

        # Ethernet ---------------------------------------------------------------------------------
        # phy
        self.submodules.ethphy = LiteEthPHYRGMII(
            clock_pads = self.platform.request("eth_clocks", eth_phy),
            pads       = self.platform.request("eth", eth_phy),
            tx_delay   = 0e-9, # 0ns FPGA delay (Clk delay added by PHY)
            rx_delay   = 2e-9) # 2ns FPGA delay to compensate Clk routing to IDDRX1F
        self.add_csr("ethphy")
        # core
        self.submodules.ethcore = LiteEthUDPIPCore(
            phy         = self.ethphy,
            mac_address = 0x10e2d5000000,
            ip_address  = "192.168.1.50",
            clk_freq    = self.clk_freq)
        # etherbone
        self.submodules.etherbone = LiteEthEtherbone(self.ethcore.udp, 1234)
        self.add_wb_master(self.etherbone.wishbone.bus)
        # timing constraints
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 1e9/125e6)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 1e9/125e6)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)


# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Colorlight 5A-75B")
    builder_args(parser)
    soc_core_args(parser)
    parser.add_argument("--revision", default="7.0", type=str, help="Board revision 7.0 (default) or 6.1")
    parser.add_argument("--with-etherbone", action="store_true", help="enable Etherbone support")
    parser.add_argument("--eth-phy", default=0, type=int, help="Ethernet PHY 0 or 1 (default=0)")
    args = parser.parse_args()

    if args.with_etherbone:
        soc = EtherboneSoC(eth_phy=args.eth_phy, revision=args.revision, **soc_core_argdict(args))
    else:
        soc = BaseSoC(args.revision, **soc_core_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    builder.csr_csv="csr.csv"
    builder.build()

if __name__ == "__main__":
    main()
