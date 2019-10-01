#!/usr/bin/env python3

from migen import *

from platforms import icebreaker

from litex.soc.cores import *
from litex.soc.cores import gpio
from litex.soc.cores import uart
from litex.soc.cores.uart import UARTWishboneBridge

from litex.soc.integration import soc_core
from litex.soc.integration.builder import *

from litex.soc.interconnect.csr import *

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal


from gateware import ice40

# Build --------------------------------------------------------------------------------------------

_sao_pmod = [
    ("sao", 0,
     Subsignal("eye_right", Pins("PMOD1A:0")),
     Subsignal("eye_left", Pins("PMOD1A:1")),
     Subsignal("osh_lower", Pins("PMOD1A:2")),
     Subsignal("osh_upper", Pins("PMOD1A:3")),
     IOStandard("LVCMOS33")
    ),
]

_serial2_pmod = [
    ("serial2", 0,
        Subsignal("rx", Pins("PMOD1B:0")),
        Subsignal("tx", Pins("PMOD1B:1"), Misc("PULLUP")),
        IOStandard("LVCMOS33")
    ),
]


class PWM(Module, AutoCSR):
    def __init__(self, pwm_pin, width=32):
        self._pwm_enable = CSRStorage()
        self._pwm_divider = CSRStorage(16)
        self._pwm_width  = CSRStorage(width)
        self._pwm_period = CSRStorage(width)
        self._pwm_cnt    = CSRStatus(width)

        # # #

        pwm_count = Signal(width)
        pwm_clock_divide = Signal(16)

        self.sync += self._pwm_cnt.status.eq(pwm_count)
        self.sync += \
            If(self._pwm_enable.storage,
                If(pwm_clock_divide == self._pwm_divider.storage,
                   pwm_clock_divide.eq(0),
                   If(pwm_count < self._pwm_width.storage,
                      pwm_pin.eq(1)
                   ).Else(
                      pwm_pin.eq(0)
                   ),
                   If(pwm_count == self._pwm_period.storage-1,
                      pwm_count.eq(0)
                   ).Else(
                       pwm_count.eq(pwm_count+1)
                   )
                ).Else(
                    pwm_clock_divide.eq(pwm_clock_divide+1)
                )
            ).Else(
                pwm_count.eq(0),
                pwm_pin.eq(0),
                pwm_clock_divide.eq(0)
            )

def main():
    platform = icebreaker.Platform()
    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           cpu_variant="lite+debug",
                           with_uart=False,
                           uart_stub=True,
                           with_ctrl=False,
                           integrated_rom_size=0x2000,
                           integrated_sram_size=0)
    # SPRAM- UP5K has single port RAM, might as well use it as SRAM to
    # free up scarce block RAM.
    spram_size = 128*1024
    soc.submodules.spram = ice40.SPRAM(size=spram_size)
    soc.register_mem("sram", 0x10000000, soc.spram.bus, spram_size)
    
    platform.add_extension(_sao_pmod)
    sao_pads = platform.request("sao")

    soc.submodules.pwm_upper = PWM(sao_pads.osh_upper, 8)
    soc.add_csr("pwm_upper")
    soc.submodules.pwm_lower = PWM(sao_pads.osh_lower, 8)
    soc.add_csr("pwm_lower")
    soc.submodules.pwm_right = PWM(sao_pads.eye_right, 8)
    soc.add_csr("pwm_right")
    soc.submodules.pwm_left = PWM(sao_pads.eye_left, 8)
    soc.add_csr("pwm_left")

    # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
    platform.add_extension(_serial2_pmod)
    soc.submodules.uartbridge = UARTWishboneBridge(platform.request("serial2"), int(sys_clk_freq), baudrate=115200)
    soc.add_wb_master(soc.uartbridge.wishbone)

    builder = Builder(soc, csr_csv="csr.csv")
    
    for package in builder.software_packages:
        if package[0] == "bios":
            builder.software_packages.remove(package)
            break
    builder.add_software_package("bios", src_dir="../../../../sw")
    builder.build()


if __name__ == "__main__":
    main()
