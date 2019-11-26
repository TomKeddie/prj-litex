#!/usr/bin/env python3

import sys

sys.path.append("ov_ftdi/software/fpga/ov3")

from migen import *

from litex.boards.platforms import arty
from litex.soc.cores import *
from litex.soc.cores.clock import *
from litex.soc.cores import gpio
from litex.soc.cores.uart import UARTWishboneBridge
from litex.soc.integration import soc_core
from litex.soc.integration.builder import *
import litex.soc.interconnect.stream as al_fifo
from litex.soc.interconnect.stream import Endpoint
from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal, Inverted

from litescope import LiteScopeAnalyzer

from ovhw.ulpi import ULPI_ctrl, ULPI_pl, ULPI_REG
from ovhw.ulpicfg import ULPICfg
from ovhw.ovf_insert import OverflowInserter
from ovhw.cfilt import RXCmdFilter
from ovhw.whacker.whacker import Whacker
from ovhw.ov_types import ULPI_DATA_D
from ovhw.ov_types import D_LAST

_ulpi_pmod = [
    ("ulpi", 0, 
        Subsignal("d", Pins("pmoda:0 pmoda:1 pmoda:2 pmoda:3 pmoda:4 pmoda:5 pmoda:6 pmoda:7")),
        Subsignal("rst", Pins("pmodb:4")),
        Subsignal("stp", Pins("pmodb:3")),
        Subsignal("dir", Pins("pmodb:6")),
        Subsignal("clk", Pins("pmodb:2"), Misc("PULLDOWN")),
        Subsignal("nxt", Pins("pmodb:7")),
        IOStandard("LVCMOS33")
    ),
    ("target", 0,
        Subsignal("dp", Pins("pmodb:0")),
        Subsignal("dm", Pins("pmodb:1")),
        IOStandard("LVCMOS33")
    ),
    ("ulpi_led", 0,
        Subsignal("led", Pins("pmodb:5"), Inverted()),
        IOStandard("LVCMOS33")
    ),
]

_serial2 = [
    ("serial2", 0,
        Subsignal("rx", Pins("ck_io:ck_io8")),
        Subsignal("tx", Pins("ck_io:ck_io9"), Misc("PULLUP")),
        IOStandard("LVCMOS33")
    ),
]

class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.cd_sys.clk.attr.add("keep")
        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys, 12e6)

class _OV3(Module):
    def __init__(self, soc):
        # ULPI Interfce

        # Diagnostics/Testing signals
        ulpi_cd_rst = Signal()
        ulpi_stp_ovr = Signal(1)
        
        # ULPI physical layer
        soc.submodules.ulpi_pl = ULPI_pl(
            soc.platform.request("ulpi"), ulpi_cd_rst, ulpi_stp_ovr)
        soc.clock_domains.cd_ulpi = soc.ulpi_pl.cd_ulpi
        
        # ULPI controller
        ulpi_reg = Record(ULPI_REG)
        soc.submodules.ulpi = ClockDomainsRenamer({"sys": "ulpi"}) (
          ULPI_ctrl(soc.ulpi_pl.ulpi_bus, ulpi_reg),
        )

        # ULPI register R/W CSR interface
        soc.submodules.ucfg = ULPICfg(
            soc.cd_ulpi.clk, ulpi_cd_rst, soc.ulpi_pl.ulpi_bus.rst,
            ulpi_stp_ovr, ulpi_reg)
        soc.add_csr("ucfg")

        # Litescope Analyzer
        analyzer_groups = {}
        analyzer_groups[0] = [
            ulpi_reg,
        ]
        soc.submodules.analyzer = LiteScopeAnalyzer(analyzer_groups, 8192, csr_csv="analyzer.csv")
        soc.add_csr("analyzer")

        # Receive Path
        soc.submodules.ovf_insert = ClockDomainsRenamer(
            {"sys": "ulpi"}
        )(OverflowInserter())
        soc.add_csr("ovf_insert")

        soc.submodules.udata_fifo = ClockDomainsRenamer(
            {"write":"ulpi", "read":"sys"})(
                al_fifo.AsyncFIFO(ULPI_DATA_D, 1024)
            )

        soc.submodules.cfilt = RXCmdFilter()

        soc.submodules.cstream = Whacker(1024)
        soc.add_csr("cstream")
        soc.comb += [
            soc.ulpi.data_out_source.connect(soc.ovf_insert.sink),
            soc.ovf_insert.source.connect(soc.udata_fifo.sink),
            soc.udata_fifo.source.connect(soc.cfilt.sink),
            soc.cfilt.source.connect(soc.cstream.sink),
#            soc.cstream.source.connect(soc.sdram_sink.sink),
        ]

        # fake sdram controller ready
        soc.sync += [ soc.cstream.source.ready.eq(1) ]

# Build --------------------------------------------------------------------------------------------



def main():
    platform = arty.Platform()
    sys_clk_freq = 12e6
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           cpu_variant="lite+debug",
                           integrated_rom_size=0x8000,
                           integrated_sram_size=0x8000)

    # clocks
    soc.submodules.crg = _CRG(platform)
    
    # reset button
    soc.comb += soc.cpu.reset.eq(platform.request("user_btn", 3) | soc.ctrl.reset)

    # pmod
    platform.add_extension(_ulpi_pmod)

    # led
    led_pad = platform.request("ulpi_led", 0)
    soc.submodules.led = gpio.GPIOOut(led_pad.led)
    soc.add_csr("led")

    # debug
    # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
    platform.add_extension(_serial2)
    soc.submodules.uartbridge = UARTWishboneBridge(platform.request("serial2"), int(sys_clk_freq), baudrate=115200)
    soc.add_wb_master(soc.uartbridge.wishbone)
    soc.register_mem("vexriscv_debug", 0xf00f0000, soc.cpu.debug_bus, 0x10)

    soc.debug_sink_last = Endpoint(D_LAST)
    soc.debug_sink_data = Endpoint(ULPI_DATA_D)

    # ulpi
    ov3 = _OV3(soc)
    
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build()


if __name__ == "__main__":
    main()
    


