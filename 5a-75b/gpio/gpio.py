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
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from liteeth.core import LiteEthUDPIPCore
from liteeth.frontend.etherbone import LiteEthEtherbone

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal

_hub75_connectors = [
    ("j1", 0,
     Subsignal("R0", Pins("F3")),               
     Subsignal("G0", Pins("F1")),               
     Subsignal("B0", Pins("G3")),               
     Subsignal("R1", Pins("G2")),               
     Subsignal("G1", Pins("H3")),               
     Subsignal("B1", Pins("H5")),               
     Subsignal("E", Pins("F15")),                
     Subsignal("A", Pins("L2")),               
     Subsignal("B", Pins("K1")),               
     Subsignal("C", Pins("J5")),               
     Subsignal("D", Pins("K2")),               
     Subsignal("CLK", Pins("B16")),                
     Subsignal("STB", Pins("J14")),                
     Subsignal("OE", Pins("F12")),                
     IOStandard("LVCMOS33")
    ),
    ("j2", 0,
     Subsignal("R0", Pins("J4")),
     Subsignal("G0", Pins("K3")),
     Subsignal("B0", Pins("G1")),
     Subsignal("R1", Pins("K4")),
     Subsignal("G1", Pins("C2")),
     Subsignal("B1", Pins("E3")),
     #Subsignal("E", Pins("F15")),
     #Subsignal("A", Pins("L2")),
     #Subsignal("B", Pins("K1")),
     #Subsignal("C", Pins("J5")),
     #Subsignal("D", Pins("K2")),
     #Subsignal("CLK", Pins("B16")),  
     #Subsignal("STB", Pins("J14")),  
     #Subsignal("OE", Pins("F12")), 
    ),
    ("j3", 0,
     Subsignal("R0", Pins("H4")),      
     Subsignal("G0", Pins("K5")),      
     Subsignal("B0", Pins("P1")),      
     Subsignal("R1", Pins("R1")),      
     Subsignal("G1", Pins("L5")),      
     Subsignal("B1", Pins("F2")),      
     Subsignal("E", Pins("F15")),      
     #Subsignal("A", Pins("L2")),     
     #Subsignal("B", Pins("K1")),     
     #Subsignal("C", Pins("J5")),     
     #Subsignal("D", Pins("K2")),     
     #Subsignal("CLK", Pins("B16")),        
     #Subsignal("STB", Pins("J14")),        
     #Subsignal("OE", Pins("F12")),       
    ),
    ("j4", 0,
     Subsignal("R0", Pins("P4")),      
     Subsignal("G0", Pins("R2")),      
     Subsignal("B0", Pins("M8")),      
     #Subsignal("R1", Pins("?")),     
     Subsignal("G1", Pins("T6")),      
     Subsignal("B1", Pins("R6")),      
     #Subsignal("E", Pins("F15")),      
     #Subsignal("A", Pins("L2")),     
     #Subsignal("B", Pins("K1")),     
     #Subsignal("C", Pins("J5")),     
     #Subsignal("D", Pins("K2")),     
     #Subsignal("CLK", Pins("B16")),        
     #Subsignal("STB", Pins("J14")),        
     #Subsignal("OE", Pins("F12")),
     ),
]

_serial2 = [
    ("serial2", 0,
        Subsignal("rx", Pins("F12")),
        Subsignal("tx", Pins("F3"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33")
    ),
]
_serial = [
    ("serial", 0,
        Subsignal("rx", Pins("B16")),
        Subsignal("tx", Pins("F1"), Misc("PULLUP=TRUE")),
        IOStandard("LVCMOS33")
    ),
]

class GPIOBidirectional(Module, AutoCSR):
    def __init__(self, pads):
        pad_count = len(pads)
        self._pins_in = CSRStatus(pad_count)
        self._pins_out = CSRStorage(pad_count)
        self._pins_oe = CSRStorage(pad_count)
        gpio_pins_t = [None] * pad_count
        bit = 0
        for pin_group in pads.layout:
            for pin in getattr(pads, pin_group[0]):
                gpio_pins_t[bit] = TSTriple()
                self.specials += gpio_pins_t[bit].get_tristate(pin)
                self.comb += gpio_pins_t[bit].o.eq(self._pins_out.storage[bit])
                self.comb += gpio_pins_t[bit].oe.eq(self._pins_oe.storage[bit])
                self.comb += self._pins_in.status[bit].eq(gpio_pins_t[bit].i)
                bit=bit+1

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
        platform.add_extension(_hub75_connectors)
        platform.add_extension(_serial)
        platform.add_extension(_serial2)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq, **kwargs, cpu_variant="standard+debug")

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # GPIO --------------------------------------------------------------------------------------
        #self.submodules.j1 = GPIOBidirectional(platform.request("j1"))
        #self.add_csr("j1")
        self.submodules.j2 = GPIOBidirectional(platform.request("j2"))
        self.add_csr("j2")
        self.submodules.j2 = GPIOBidirectional(platform.request("j3"))
        self.add_csr("j3")
        self.submodules.j2 = GPIOBidirectional(platform.request("j4"))
        self.add_csr("j4")

        # Debug bridge ----------------------------------------------------------------------------------
        # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
        self.submodules.uartbridge = UARTWishboneBridge(platform.request("serial2"), int(sys_clk_freq), baudrate=115200)
        self.add_wb_master(self.uartbridge.wishbone)
        self.register_mem("vexriscv_debug", 0xf00f0000, self.cpu.debug_bus, 0x10)

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
