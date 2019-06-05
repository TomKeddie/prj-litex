#!/usr/bin/env python3

import argparse
import pprint

# Import lxbuildenv to integrate the deps/ directory
import lxbuildenv

from migen import *

from litex.boards.platforms import arty

from litex.soc.cores.clock import *
from litex.soc.cores import gpio
from litex.soc.cores import spi_flash
from litex.soc.integration.soc_core import mem_decoder
from litex.soc.integration.soc_sdram import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import wishbone

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal

from litedram.modules import MT41K128M16
from litedram.phy import s7ddrphy

from liteeth.phy.mii import LiteEthPHYMII
from liteeth.core.mac import LiteEthMAC
from liteeth.core import LiteEthUDPIPCore
from liteeth import common
from liteeth.frontend.etherbone import LiteEthEtherbone

from valentyusb import usbcore
from valentyusb.usbcore import io as usbio
from valentyusb.usbcore.cpu import epmem, unififo, epfifo
from valentyusb.usbcore.endpoint import EndpointType

_usb_pmod = [
    ("usb", 0,
     Subsignal("d_p", Pins("pmoda:0")),
     Subsignal("d_n", Pins("pmoda:1")),
     Subsignal("pullup", Pins("pmoda:2")),
     Subsignal("led", Pins("pmoda:3")),
     IOStandard("LVCMOS33")
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_clk200 = ClockDomain()
        self.clock_domains.cd_usb_12 = ClockDomain()
        self.clock_domains.cd_usb_48 = ClockDomain()

        # # #

        self.cd_sys.clk.attr.add("keep")
        self.cd_sys4x.clk.attr.add("keep")
        self.cd_sys4x_dqs.clk.attr.add("keep")
        self.cd_usb_12.clk.attr.add("keep")
        self.cd_usb_48.clk.attr.add("keep")

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_sys4x, 4*sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4*sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_clk200, 200e6)
        pll.create_clkout(self.cd_usb_48, 48e6)

        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_clk200)

        eth_clk = Signal()
        self.specials += [
            Instance("BUFR", p_BUFR_DIVIDE="4", i_CE=1, i_CLR=0, i_I=self.cd_sys.clk, o_O=eth_clk),
            Instance("BUFG", i_I=eth_clk, o_O=platform.request("eth_ref_clk")),
        ]

        # use BUFR to derive 12MHz to avoid significant phase shift as ValentyUSB requires these clocks to be phase aligned
        usb_12_bufr_clk = Signal()
        self.specials += [
            Instance("BUFR", p_BUFR_DIVIDE="4", i_CE=1, i_CLR=0, i_I=self.cd_usb_48.clk, o_O=usb_12_bufr_clk),
            Instance("BUFG", i_I=usb_12_bufr_clk, o_O=self.cd_usb_12.clk),
        ]

# RGBLED ------------------------------------------------------------------------------------------
class RGBLED(gpio.GPIOOut):
    def __init__(self, pads):
        gpio.GPIOOut.__init__(self, pads)

# WarmBoot ------------------------------------------------------------------------------------------
class WarmBoot(Module, AutoCSR):
    def __init__(self):
        self.ctrl = CSRStorage(size=8)
        self.addr = CSRStorage(size=32)
        do_reset = Signal()
        self.comb += [
            # "Reset Key" is 0xac (0b101011xx)
            do_reset.eq(self.ctrl.storage[2] & self.ctrl.storage[3] & ~self.ctrl.storage[4]
                        & self.ctrl.storage[5] & ~self.ctrl.storage[6] & self.ctrl.storage[7])
        ]
        do_reset.attr.add("keep")

# PicoRVSpi ----------------------------------------------------------------------------------------------
class PicoRVSpi(Module, AutoCSR):
    def __init__(self, platform, pads, size=2*1024*1024):
        self.size = size

        self.bus = bus = wishbone.Interface()

        self.reset = Signal()

        self.cfg1 = CSRStorage(size=8)
        self.cfg2 = CSRStorage(size=8)
        self.cfg3 = CSRStorage(size=8)
        self.cfg4 = CSRStorage(size=8)

        self.stat1 = CSRStatus(size=8)
        self.stat2 = CSRStatus(size=8)
        self.stat3 = CSRStatus(size=8)
        self.stat4 = CSRStatus(size=8)

        cfg = Signal(32)
        cfg_we = Signal(4)
        cfg_out = Signal(32)
        self.comb += [
            cfg.eq(Cat(self.cfg1.storage, self.cfg2.storage, self.cfg3.storage, self.cfg4.storage)),
            cfg_we.eq(Cat(self.cfg1.re, self.cfg2.re, self.cfg3.re, self.cfg4.re)),
            self.stat1.status.eq(cfg_out[0:8]),
            self.stat2.status.eq(cfg_out[8:16]),
            self.stat3.status.eq(cfg_out[16:24]),
            self.stat4.status.eq(cfg_out[24:32]),
        ]

        mosi_pad = TSTriple()
        miso_pad = TSTriple()
        cs_n_pad = TSTriple()
        clk_pad  = TSTriple()
        wp_pad   = TSTriple()
        hold_pad = TSTriple()
        self.specials += mosi_pad.get_tristate(pads.mosi)
        self.specials += miso_pad.get_tristate(pads.miso)
        self.specials += cs_n_pad.get_tristate(pads.cs_n)
        self.specials += clk_pad.get_tristate(pads.clk)
        self.specials += wp_pad.get_tristate(pads.wp)
        self.specials += hold_pad.get_tristate(pads.hold)

        reset = Signal()
        self.comb += [
            reset.eq(ResetSignal() | self.reset),
            cs_n_pad.oe.eq(~reset),
            clk_pad.oe.eq(~reset),
        ]

        flash_addr = Signal(24)
        mem_bits = bits_for(size)
        self.comb += flash_addr.eq(bus.adr[0:mem_bits-2] << 2),

        read_active = Signal()
        spi_ready = Signal()
        self.sync += [
            If(bus.stb & bus.cyc & ~read_active,
                read_active.eq(1),
                bus.ack.eq(0),
            )
            .Elif(read_active & spi_ready,
                read_active.eq(0),
                bus.ack.eq(1),
            )
            .Else(
                bus.ack.eq(0),
                read_active.eq(0),
            )
        ]

        o_rdata = Signal(32)
        self.comb += bus.dat_r.eq(o_rdata)

        self.specials += Instance("spimemio",
            o_flash_io0_oe = mosi_pad.oe,
            o_flash_io1_oe = miso_pad.oe,
            o_flash_io2_oe = wp_pad.oe,
            o_flash_io3_oe = hold_pad.oe,

            o_flash_io0_do = mosi_pad.o,
            o_flash_io1_do = miso_pad.o,
            o_flash_io2_do = wp_pad.o,
            o_flash_io3_do = hold_pad.o,
            o_flash_csb    = cs_n_pad.o,
            o_flash_clk    = clk_pad.o,

            i_flash_io0_di = mosi_pad.i,
            i_flash_io1_di = miso_pad.i,
            i_flash_io2_di = wp_pad.i,
            i_flash_io3_di = hold_pad.i,

            i_resetn = ~reset,
            i_clk = ClockSignal(),

            i_valid = bus.stb & bus.cyc,
            o_ready = spi_ready,
            i_addr  = flash_addr,
            o_rdata = o_rdata,

	        i_cfgreg_we = cfg_we,
            i_cfgreg_di = cfg,
	        o_cfgreg_do = cfg_out,
        )
        platform.add_source("../foboot/hw/rtl/spimemio.v")

# ClassicLed ------------------------------------------------------------------------------------------
class ClassicLed(gpio.GPIOOut):
    def __init__(self, pads):
        gpio.GPIOOut.__init__(self, pads)
        
# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCSDRAM):
    soc_interrupt_map = {
        "usb":    3,
    }
    soc_interrupt_map.update(SoCSDRAM.soc_interrupt_map)
    csr_map = {
        "ddrphy":       20,
        "usb":          21,
        "usbled":       22,
        "reboot":       23,
        "picorvspi":    24,
        "rgb":          25,
    }
    csr_map.update(SoCSDRAM.csr_map)
    mem_map = {
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
    }
    mem_map.update(SoCSDRAM.mem_map)

    def __init__(self, sys_clk_freq=int(100e6), **kwargs):
        platform = arty.Platform()
        SoCSDRAM.__init__(self, platform, clk_freq=sys_clk_freq,
                          integrated_rom_size=0x8000,
                          integrated_sram_size=0x8000,
                          cpu_type = "vexriscv",
                          cpu_variant = "std_debug",
                          **kwargs)

        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # sdram
        self.submodules.ddrphy = s7ddrphy.A7DDRPHY(platform.request("ddram"), sys_clk_freq=sys_clk_freq)
#        self.add_csr("ddrphy")
        sdram_module = MT41K128M16(sys_clk_freq, "1:4")
        self.register_sdram(self.ddrphy,
                            sdram_module.geom_settings,
                            sdram_module.timing_settings)
        # usb
        platform.add_extension(_usb_pmod)
        usb_pads = platform.request("usb")
        usb_iobuf = usbio.IoBuf(usb_pads.d_p, usb_pads.d_n, usb_pads.pullup)
        self.submodules.usb = epfifo.PerEndpointFifoInterface(usb_iobuf, debug=False)
#        self.add_csr("usb")
        
        # usb led
        self.submodules.usbled = ClassicLed(usb_pads.led)
#        self.add_csr("usbled")

        # spi flash
        spi_pads = platform.request("spiflash")
        self.submodules.picorvspi = PicoRVSpi(platform, spi_pads)
        self.register_mem("spiflash", self.mem_map["spiflash"],
            self.picorvspi.bus, size=self.picorvspi.size)

        # RGB leds
        self.submodules.rgb = RGBLED(platform.request("rgb_led", 0))

        # reboot
        self.submodules.reboot = WarmBoot()
        self.cpu.cpu_params.update(i_externalResetVector=self.reboot.addr.storage)
#        self.add_csr("reboot")

        # config memory confiig for xdc
        platform.add_platform_command("set_property CFGBVS VCCO [current_design]")
        platform.add_platform_command("set_property CONFIG_VOLTAGE 3.3 [current_design]")
        platform.add_platform_command("set_property BITSTREAM.GENERAL.COMPRESS True [current_design]")


# EthernetSoC --------------------------------------------------------------------------------------

class EthernetSoC(BaseSoC):
    mem_map = {
        "ethmac": 0x30000000,  # (shadow @0xb0000000)
    }
    mem_map.update(BaseSoC.mem_map)
    soc_interrupt_map = {
        "ethmac":    4,
    }
    soc_interrupt_map.update(BaseSoC.soc_interrupt_map)
    csr_map = {
        "ethphy":         30,
        "ethmac":         31,
    }
    csr_map.update(BaseSoC.csr_map)

    def __init__(self, **kwargs):
        BaseSoC.__init__(self, **kwargs)

        self.submodules.ethphy = LiteEthPHYMII(self.platform.request("eth_clocks"),
                                               self.platform.request("eth"))
#        self.add_csr("ethphy")
        self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=32,
                                            interface="wishbone", endianness=self.cpu.endianness)
        self.add_wb_slave(mem_decoder(self.mem_map["ethmac"]), self.ethmac.bus)
        self.add_memory_region("ethmac", self.mem_map["ethmac"] | self.shadow_base, 0x2000)
#        self.add_csr("ethmac")
#        self.add_interrupt("ethmac")

        self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 1e9/12.5e6)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 1e9/12.5e6)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)

# EtherboneSoC --------------------------------------------------------------------------------------
# debug (see https://github.com/timvideos/litex-buildenv/wiki/Debugging)
class EtherboneSoC(BaseSoC):
    csr_map = {
        "ethphy":         30,
    }
    csr_map.update(BaseSoC.csr_map)
    def __init__(self,
                 mac_address=0x10e2d5000000,
                 ip_address="192.168.1.50",
                 **kwargs):
        BaseSoC.__init__(self, **kwargs)

        # Ethernet PHY and UDP/IP stack
        self.submodules.ethphy = LiteEthPHYMII(self.platform.request("eth_clocks"),
                                               self.platform.request("eth"))
#        self.add_csr("ethphy")
        self.submodules.ethcore = LiteEthUDPIPCore(self.ethphy,
                                                   mac_address,
                                                   common.convert_ip(ip_address),
                                                   self.clk_freq,
                                                   with_icmp=True)

        # Etherbone bridge
        self.submodules.etherbone = LiteEthEtherbone(self.ethcore.udp, 20000)
        self.add_wb_master(self.etherbone.wishbone.bus)

        self.ethphy.crg.cd_eth_rx.clk.attr.add("keep")
        self.ethphy.crg.cd_eth_tx.clk.attr.add("keep")
        self.platform.add_period_constraint(self.crg.cd_sys.clk, 10.0)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_rx.clk, 40.0)
        self.platform.add_period_constraint(self.ethphy.crg.cd_eth_tx.clk, 40.0)

        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.ethphy.crg.cd_eth_rx.clk,
            self.ethphy.crg.cd_eth_tx.clk)

        self.register_mem("vexriscv_debug", 0xf00f0000, self.cpu.debug_bus, 0x10)
        

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Arty")
    builder_args(parser)
    soc_sdram_args(parser)
    parser.add_argument("--with-ethernet", action="store_true",
                        help="enable Ethernet support")
    parser.add_argument("--without-debug", action="store_true",
                        help="disable Ethernet debugx support")
    args = parser.parse_args()

    
    cls = EthernetSoC if args.with_ethernet else BaseSoC if args.without_debug else EtherboneSoC
    soc = cls(**soc_sdram_argdict(args))
    builder = Builder(soc, **builder_argdict(args))
    for package in builder.software_packages:
       if package[0] == "bios":
           builder.software_packages.remove(package)
           break
    builder.add_software_package("bios", src_dir="../../../../sw")
    builder.build()



if __name__ == "__main__":
    main()
