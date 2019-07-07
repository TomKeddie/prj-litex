#!/usr/bin/env python3

import argparse
import pprint
import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/../foboot/hw/deps/valentyusb")

from migen import *
from migen.genlib.cdc import MultiReg

from litex.boards.platforms import arty

from litex.soc.cores.clock import *
from litex.soc.cores import gpio
from litex.soc.cores import spi_flash
from litex.soc.integration.soc_core import mem_decoder
from litex.soc.integration.soc_core import *
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

from litescope import LiteScopeIO, LiteScopeAnalyzer

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
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_usb_12 = ClockDomain()
        self.clock_domains.cd_usb_48 = ClockDomain()
        self.clock_domains.cd_clk100 = ClockDomain()

        # # #

        self.cd_sys.clk.attr.add("keep")
        self.cd_usb_12.clk.attr.add("keep")
        self.cd_usb_48.clk.attr.add("keep")

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(~platform.request("cpu_reset"))
        pll.register_clkin(platform.request("clk100"), 100e6)
        pll.create_clkout(self.cd_clk100, 100e6)
        pll.create_clkout(self.cd_usb_48, 48e6)

        eth_clk = Signal()
        self.specials += [
            Instance("BUFR", p_BUFR_DIVIDE="4", i_CE=1, i_CLR=0, i_I=self.cd_clk100.clk, o_O=eth_clk),
            Instance("BUFG", i_I=eth_clk, o_O=platform.request("eth_ref_clk")),
        ]

        # use BUFR to derive 12MHz to avoid significant phase shift as ValentyUSB requires these clocks to be phase aligned
        clk12_bufr = Signal()
        clk12 = Signal()
        self.specials += [
            Instance("BUFR", p_BUFR_DIVIDE="4", i_CE=1, i_CLR=0, i_I=self.cd_usb_48.clk, o_O=clk12_bufr),
            Instance("BUFG", i_I=clk12_bufr, o_O=clk12),
        ]
        self.comb += self.cd_sys.clk.eq(clk12)
        self.comb += self.cd_usb_12.clk.eq(clk12)


# WarmBoot ------------------------------------------------------------------------------------------
class WarmBoot(Module, AutoCSR):
    def __init__(self, parent):
        self.ctrl = CSRStorage(size=8)
        self.addr = CSRStorage(size=32)
        boot = Signal()
        self.comb += [
            # "Reset Key" is 0xac (0b101011xx)
            boot.eq(self.ctrl.storage[2] & self.ctrl.storage[3] & ~self.ctrl.storage[4]
                        & self.ctrl.storage[5] & ~self.ctrl.storage[6] & self.ctrl.storage[7])
        ]
        addr = Signal(32)
        # mangle the address so entry 2 points to the rescue image
        # 0 : DFU bootloader  (160 on fomu)
        # 1 : ???             (157696 on fomu)
        # 2 : user fpga image (262144 on fomu)
        # 3 : ???             (266240 on fomu)
        self.comb += addr.eq((parent.config["RESCUE_IMAGE_OFFSET"] >> 1) * (self.ctrl.storage[1] * 2 + self.ctrl.storage[0]))

        c_dummy_word = Constant(0xFFFFFFFF)  # Dummy Word
        c_sync_word = Constant(0xAA995566)   # Sync Word
        c_noop_word = Constant(0x20000000)   # Type 1 NO OP
        c_wbstar_word = Constant(0x30020001) # Type 1 Write 1 Words to WBSTAR
        c_cmd_word = Constant(0x30008001)    # Type 1 Write 1 Words to CMD
        c_iprog_word = Constant(0x0000000F)  # IPROG Command

        clk = ClockSignal()
        reset = ResetSignal()
        boot_1d = Signal()
        icape2_input = Signal(32)
        icape2_input_swapped = Signal(32)
        icape2_cs = Signal()
        icape2_wr = Signal()

        self.sync += boot_1d.eq(boot)

        fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        # start on rising edge of boot
        fsm.act("IDLE",
                If(boot & ~boot_1d, NextState("DUMMY")))
        fsm.act("DUMMY",
                icape2_input.eq(c_dummy_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("SYNC"))
        fsm.act("SYNC",
                icape2_input.eq(c_sync_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("NOOP1"))
        fsm.act("NOOP1",
                icape2_input.eq(c_noop_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("WBSTAR"))
        fsm.act("WBSTAR",
                icape2_input.eq(c_wbstar_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("ADDRESS"))
        # UG470 "When using ICAPE2 to set the WBSTAR address,
        # the 24 most significant address bits should be
        # written to WBSTAR[23:0]. For SPI 32-bit
        # addressing mode, WBSTAR[23:0] are sent as
        # address bits [31:8]. The lower 8 bits of the
        # address are undefined and the value could be
        # as high as 0xFF. Any bitstream at the WBSTAR
        # address should contain 256 dummy bytes before
        # the start of the bitstream."
        fsm.act("ADDRESS",
                icape2_input.eq(Cat(0, addr[8:31])),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("CMD"))
        fsm.act("CMD",
                icape2_input.eq(c_cmd_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("IPROG"))
        fsm.act("IPROG",
                icape2_input.eq(c_iprog_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("NOOP2"))
        fsm.act("NOOP2",
                icape2_input.eq(c_noop_word),
                icape2_wr.eq(1),
                icape2_cs.eq(1),
                NextState("DONE"))
        fsm.act("DONE",
                icape2_wr.eq(0),
                icape2_cs.eq(0),
                NextState("DONE"))

        icape2_cs_1d = Signal()
        icape2_wr_1d = Signal()
        self.sync += icape2_cs_1d.eq(icape2_cs)
        self.sync += icape2_wr_1d.eq(icape2_wr)

        icape2_csib = Signal()
        icape2_rdwrb = Signal()
        self.comb += icape2_csib.eq(~icape2_cs_1d)
        self.comb += icape2_rdwrb.eq(~icape2_wr_1d)

        # UG470 : "Traditionally, in SelectMAP x8 mode, configuration data is loaded one byte per CCLK,
        # with the most-significant bit (MSB) of each byte presented to the D0 pin. Although this convention
        # (D0 = MSB, D7 = LSB) differs from many other devices, it is consistent across all Xilinx FPGAs.
        # The bit-swap rule also applies to 7 series FPGA BPI x8 modes and to the ICAPE2 interface (see
        # Bit Swapping). In 7 series devices, the bit-swap rule is extended to x16 and x32 bus widths,
        # i.e., the data is bit swapped within each byte. The bit order in 7 series FPGAs is the same as
        # in Virtex-6 FPGAs."
        for iy in range(8):
            self.sync += icape2_input_swapped[iy   ].eq(icape2_input[7-iy])
            self.sync += icape2_input_swapped[iy+ 8].eq(icape2_input[15-iy])
            self.sync += icape2_input_swapped[iy+16].eq(icape2_input[23-iy])
            self.sync += icape2_input_swapped[iy+24].eq(icape2_input[31-iy])
        
        icape2_output = Signal(32)
        self.specials += [
            Instance("ICAPE2",
                     p_ICAP_WIDTH="X32",          # Specifies the input and output data width.
                     o_O = icape2_output,         # 32-bit output: Configuration data output bus
                     i_CLK = clk,                 # 1-bit input: Clock Input
                     i_CSIB = icape2_csib,     # 1-bit input: Active-Low ICAP Enable
                     i_I = icape2_input_swapped,  # 32-bit input: Configuration data input bus
                     i_RDWRB = icape2_rdwrb,   # 1-bit input: Read/Write Select input
            )
        ]
        parent.config["BITSTREAM_SYNC_HEADER1"] = 0xAA995566
        parent.config["BITSTREAM_SYNC_HEADER2"] = 0x665599AA

# PicoRVSpi ----------------------------------------------------------------------------------------------
class PicoRVSpi(Module, AutoCSR):
    def __init__(self, platform, pads, ila_clk, size=16*1024*1024):
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
        # size/4 because data bus is 32 bits wide, -1 for base 0
        mem_bits = bits_for(int(size/4) - 1)
        print("mem_bits={}".format(mem_bits))
        pad = Signal(2)
        self.comb += flash_addr.eq(Cat(pad, bus.adr[0:mem_bits]))

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
        platform.add_source("../picorv32/picosoc/spimemio.v")

        if True:
            probe8 = Signal(4)
            probe9 = Signal(4)
            probe10 = Signal(4)

            self.comb += probe8.eq(Cat(mosi_pad.oe, miso_pad.oe, wp_pad.oe, hold_pad.oe))
            self.comb += probe9.eq(Cat(mosi_pad.o, miso_pad.o, wp_pad.o, hold_pad.o))
            self.comb += probe10.eq(Cat(mosi_pad.i, miso_pad.i, wp_pad.i, hold_pad.i))
            
            platform.add_source("ila_1/ila_1.xci")
            self.specials += [
                Instance("ila_1",
                         i_clk=ila_clk,
                         i_probe0=flash_addr,
                         i_probe1=o_rdata,
                         i_probe2=spi_ready,
                         i_probe3=bus.stb & bus.cyc,
                         i_probe4=cfg,
                         i_probe5=cfg_we,
                         i_probe6=clk_pad.o,
                         i_probe7=cs_n_pad.o,
                         i_probe8=probe8,
                         i_probe9=probe9,
                         i_probe10=probe10,
                         i_probe11=mosi_pad.o,
                         i_probe12=miso_pad.i,
                ),
            ]
            platform.toolchain.additional_commands +=  [
                "write_debug_probes -force {build_name}.ltx",
            ]
        

# TouchPad ------------------------------------------------------------------------------------------
class TouchPad(Module, AutoCSR):
    def __init__(self, signal):
        pressed = Signal()
        self.o  = CSRStorage(size=4)
        self.oe = CSRStorage(size=4)
        self.i  = CSRStatus(size=4)
        self.specials += MultiReg(signal, pressed)
        # // Set pin 2 as output, and pin 0 as input, and see if it loops back.
        self.comb += self.i.status[0].eq(self.o.storage[2] & self.oe.storage[2] & pressed)
        
# TouchPad ------------------------------------------------------------------------------------------
class SBLED(Module, AutoCSR):
    def __init__(self, pads):
        self.dat = CSRStorage(8)
        self.addr = CSRStorage(4)
        self.ctrl = CSRStorage(4)

#        self.specials += Instance("SB_RGBA_DRV",
#            i_CURREN = self.ctrl.storage[1],
#            i_RGBLEDEN = self.ctrl.storage[2],
#            i_RGB0PWM = rgba_pwm[0],
#            i_RGB1PWM = rgba_pwm[1],
#            i_RGB2PWM = rgba_pwm[2],
#            o_RGB0 = pads.rgb0,
#            o_RGB1 = pads.rgb1,
#            o_RGB2 = pads.rgb2,
#            p_CURRENT_MODE = "0b1",
#            p_RGB0_CURRENT = "0b000011",
#            p_RGB1_CURRENT = "0b000001",
#            p_RGB2_CURRENT = "0b000011",
#        )
#
#        self.specials += Instance("SB_LEDDA_IP",
#            i_LEDDCS = self.dat.re,
#            i_LEDDCLK = ClockSignal(),
#            i_LEDDDAT7 = self.dat.storage[7],
#            i_LEDDDAT6 = self.dat.storage[6],
#            i_LEDDDAT5 = self.dat.storage[5],
#            i_LEDDDAT4 = self.dat.storage[4],
#            i_LEDDDAT3 = self.dat.storage[3],
#            i_LEDDDAT2 = self.dat.storage[2],
#            i_LEDDDAT1 = self.dat.storage[1],
#            i_LEDDDAT0 = self.dat.storage[0],
#            i_LEDDADDR3 = self.addr.storage[3],
#            i_LEDDADDR2 = self.addr.storage[2],
#            i_LEDDADDR1 = self.addr.storage[1],
#            i_LEDDADDR0 = self.addr.storage[0],
#            i_LEDDDEN = self.dat.re,
#            i_LEDDEXE = self.ctrl.storage[0],
#            # o_LEDDON = led_is_on, # Indicates whether LED is on or not
#            # i_LEDDRST = ResetSignal(), # This port doesn't actually exist
#            o_PWMOUT0 = rgba_pwm[0], 
#            o_PWMOUT1 = rgba_pwm[1], 
#            o_PWMOUT2 = rgba_pwm[2],
#            o_LEDDON = Signal(), 
#        )

# Version ------------------------------------------------------------------------------------------
class Version(Module, AutoCSR):
    def __init__(self):
        def makeint(i, base=10):
            try:
                return int(i, base=base)
            except:
                return 0
        def get_gitver():
            import subprocess
            def decode_version(v):
                version = v.split(".")
                major = 0
                minor = 0
                rev = 0
                if len(version) >= 3:
                    rev = makeint(version[2])
                if len(version) >= 2:
                    minor = makeint(version[1])
                if len(version) >= 1:
                    major = makeint(version[0])
                return (major, minor, rev)
            git_rev_cmd = subprocess.Popen(["git", "describe", "--tags", "--always", "--dirty=+"],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE)
            (git_stdout, _) = git_rev_cmd.communicate()
            if git_rev_cmd.wait() != 0:
                print('unable to get git version')
                return
            raw_git_rev = git_stdout.decode().strip()

            dirty = False
            if raw_git_rev[-1] == "+":
                raw_git_rev = raw_git_rev[:-1]
                dirty = True

            parts = raw_git_rev.split("-")
            major = 0
            minor = 0
            rev = 0
            gitrev = 0
            gitextra = 0

            if len(parts) >= 3:
                if parts[0].startswith("v"):
                    version = parts[0]
                    if version.startswith("v"):
                        version = parts[0][1:]
                    (major, minor, rev) = decode_version(version)
                gitextra = makeint(parts[1])
                if parts[2].startswith("g"):
                    gitrev = makeint(parts[2][1:], base=16)
            elif len(parts) >= 2:
                if parts[1].startswith("g"):
                    gitrev = makeint(parts[1][1:], base=16)
                version = parts[0]
                if version.startswith("v"):
                    version = parts[0][1:]
                (major, minor, rev) = decode_version(version)
            elif len(parts) >= 1:
                version = parts[0]
                if version.startswith("v"):
                    version = parts[0][1:]
                (major, minor, rev) = decode_version(version)

            return (major, minor, rev, gitrev, gitextra, dirty)

        self.major = CSRStatus(8)
        self.minor = CSRStatus(8)
        self.revision = CSRStatus(8)
        self.gitrev = CSRStatus(32)
        self.gitextra = CSRStatus(10)
        self.dirty = CSRStatus(1)

        (major, minor, rev, gitrev, gitextra, dirty) = get_gitver()
        self.comb += [
            self.major.status.eq(major),
            self.minor.status.eq(minor),
            self.revision.status.eq(rev),
            self.gitrev.status.eq(gitrev),
            self.gitextra.status.eq(gitextra),
            self.dirty.status.eq(dirty),
        ]

# BaseSoC ------------------------------------------------------------------------------------------
class BaseSoC(SoCCore):
    mem_map = {
        "spiflash": 0x20000000,  # (default shadow @0xa0000000)
    }
    mem_map.update(SoCCore.mem_map)

    def __init__(self, sys_clk_freq=int(12e6), **kwargs):
        platform = arty.Platform()
        SoCCore.__init__(self, platform, clk_freq=sys_clk_freq,
                          integrated_rom_size=0x8000,
                          integrated_sram_size=0x8000,
                          cpu_type = "vexriscv",
                          cpu_variant = "std_debug",
                          **kwargs)

        self.submodules.crg = _CRG(platform)

        # -rw-r--r-- 1 tom tom  2192012 Jun 16 16:33 top.bin
        # Tell the s/w where to put the user image.  N25Q128 is 128-Mbit or
        # 16Mbyte, leave 4Mbyte for each image (could be smaller but is
        # simpler if they are all the same)
        # rescue image goes in slot 2 so use 2x image size for the offsetx
        self.config["RESCUE_IMAGE_OFFSET"] = 0x800000

        # usb
        platform.add_extension(_usb_pmod)
        usb_pads = platform.request("usb")
        usb_iobuf = usbio.IoBuf(usb_pads.d_p, usb_pads.d_n, usb_pads.pullup)
        self.submodules.usb = epfifo.PerEndpointFifoInterface(usb_iobuf, debug=True)
        self.add_wb_master(self.usb.debug_bridge.wishbone)
        self.add_csr("usb")
        self.add_interrupt("usb")

        # RGB led, ice40 emulation
        rgbled = platform.request("rgb_led", 0)
        self.submodules.rgb = SBLED(Cat(rgbled.r, rgbled.g, rgbled.b))
        self.add_csr("rgb")
        
        self.submodules.version = Version()
        self.add_csr("version")

        # spi flash
        spi_pads = platform.request("spiflash")
        self.submodules.picorvspi = PicoRVSpi(platform, spi_pads, self.crg.cd_usb_48.clk)
        self.add_csr("picorvspi")
        self.register_mem("spiflash",
                          self.mem_map["spiflash"],
                          self.picorvspi.bus,
                          size=self.picorvspi.size)

        # RGB leds direct drive
        rgbleddirect = platform.request("rgb_led", 1)
        self.submodules.rgbdirect = gpio.GPIOOut(Cat(rgbleddirect.r, rgbleddirect.g, rgbleddirect.b))
        self.add_csr("rgbdirect")

        # debug
        debugreg = Signal(16)
        self.submodules.debugreg = gpio.GPIOOut(debugreg)
        self.add_csr("debugreg")
            
        # reset button
        self.comb += self.cpu.reset.eq(platform.request("user_btn", 3) | self.ctrl.reset)

        # user button emulating touch
        self.submodules.touch = TouchPad(platform.request("user_btn", 0))
        self.add_csr("touch")

        # reboot
        self.submodules.reboot = WarmBoot(self)
        self.add_csr("reboot")

        # config memory settings for xdc
        platform.add_platform_command("set_property CFGBVS VCCO [current_design]")
        platform.add_platform_command("set_property CONFIG_VOLTAGE 3.3 [current_design]")
        platform.add_platform_command("set_property BITSTREAM.GENERAL.COMPRESS True [current_design]")
        platform.add_platform_command("set_property BITSTREAM.CONFIG.CONFIGRATE 40 [current_design]")
        platform.add_platform_command("set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]")
        platform.add_platform_command("set_property CONFIG_MODE SPIx4 [current_design]")

        # analyzer
        if False:
            analyzer_groups = {}
            miso = Signal(name_override="miso")
            mosi = Signal(name_override="mosi")
            sck = Signal(name_override="sck")
            cs_n = Signal(name_override="cs_n")
            wp   = Signal(name_override="wp")
            hold = Signal(name_override="hold")
            self.comb += miso.eq(spi_pads.miso)
            self.comb += mosi.eq(spi_pads.mosi)
            self.comb += sck.eq(spi_pads.clk)
            self.comb += cs_n.eq(spi_pads.cs_n)
            self.comb += wp.eq(spi_pads.wp)
            self.comb += hold.eq(spi_pads.hold)
            analyzer_groups[0] = [
                miso,
                mosi,
                sck,
                cs_n,
                wp,
                hold,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_groups, 512)
            self.add_csr("analyzer")

        # ila
        if False:
            platform.add_source("ila_0/ila_0.xci")
            probe0 = Signal(6)
            self.comb += probe0.eq(Cat(spi_pads.clk, spi_pads.cs_n, spi_pads.wp, spi_pads.hold, spi_pads.miso, spi_pads.mosi))
            self.specials += [
                Instance("ila_0", i_clk=ClockSignal(), i_probe0=probe0, i_probe1=debugreg),
            ]
            platform.toolchain.additional_commands +=  [
                "write_debug_probes -force {build_name}.ltx",
            ]

    def do_exit(self, vns):
        self.analyzer.export_csv(vns, "test/analyzer.csv")

# EthernetSoC --------------------------------------------------------------------------------------

class EthernetSoC(BaseSoC):
    mem_map = {
        "ethmac": 0x30000000,  # (shadow @0xb0000000)
    }
    mem_map.update(BaseSoC.mem_map)

    def __init__(self, **kwargs):
        BaseSoC.__init__(self, **kwargs)

        self.submodules.ethphy = LiteEthPHYMII(self.platform.request("eth_clocks"),
                                               self.platform.request("eth"))
        self.add_csr("ethphy")
        self.submodules.ethmac = LiteEthMAC(phy=self.ethphy, dw=32,
                                            interface="wishbone", endianness=self.cpu.endianness)
        self.add_wb_slave(mem_decoder(self.mem_map["ethmac"]), self.ethmac.bus)
        self.add_memory_region("ethmac", self.mem_map["ethmac"] | self.shadow_base, 0x2000)
        self.add_csr("ethmac")
        self.add_interrupt("ethmac")

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
    parser = argparse.ArgumentParser(description="LiteX SoC on Arty", conflict_handler="resolve")
    builder_args(parser)
    soc_core_args(parser)
    parser.add_argument("--with-ethernet", action="store_true",
                        help="enable Ethernet support")
    parser.add_argument("--with-ethernet-debug", action="store_true",
                        help="enable Ethernet debug support")
    parser.add_argument("--csr-csv", action="store_true", default="csr.csv",
                        help="csv filename")
    parser.add_argument("--output-dir", action="store_true", default="build",
                        help="build directory")
    args = parser.parse_args()
    
    cls = EthernetSoC if args.with_ethernet else EtherboneSoC if args.with_ethernet_debug else BaseSoC
    soc = cls(**soc_core_argdict(args))
    builder = Builder(soc, **builder_argdict(args))

    # replace bios with our software
    for package in builder.software_packages:
       if package[0] == "bios":
           builder.software_packages.remove(package)
           break
    builder.add_software_package("bios", src_dir="../../../../sw")
    
    builder.build()



if __name__ == "__main__":
    main()
