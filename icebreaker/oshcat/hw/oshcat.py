#!/usr/bin/env python3

from migen import *

from platforms import icebreaker

from litex.soc.cores import *
from litex.soc.cores import gpio
from litex.soc.cores import uart
from litex.soc.cores.uart import UARTWishboneBridge

from litex.soc.integration import soc_core
from litex.soc.integration.builder import *

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal


from gateware import ice40
from gateware import pwm

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
        Subsignal("tx", Pins("PMOD1B:1")),
        IOStandard("LVCMOS33")
    ),
]


def main():
    platform = icebreaker.Platform()
    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           with_uart=True,
                           integrated_rom_size=0x2000,
                           integrated_sram_size=0)
    # SPRAM- UP5K has single port RAM, might as well use it as SRAM to
    # free up scarce block RAM.
    spram_size = 128*1024
    soc.submodules.spram = ice40.SPRAM(size=spram_size)
    soc.register_mem("sram", 0x10000000, soc.spram.bus, spram_size)
    
#    platform.add_extension(_serial2_pmod)
    platform.add_extension(_sao_pmod)
    sao_pads = platform.request("sao")
    soc.submodules.sao = gpio.GPIOOut(Cat(sao_pads.eye_right, sao_pads.osh_lower))
    soc.add_csr("sao")

    soc.submodules.pwm = pwm.PWM(sao_pads.osh_upper)
    soc.add_csr("pwm")

    # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
    # soc.submodules.uartbridge = UARTWishboneBridge(platform.request("serial2"), int(sys_clk_freq), baudrate=115200)
    # soc.add_wb_master(soc.uartbridge.wishbone)

    led_pad = platform.request("user_led_n", 1)
    soc.submodules.leds = gpio.GPIOOut(led_pad)
    soc.add_csr("leds")

    builder = Builder(soc) # csr_csv="csr.csv")
    
    for package in builder.software_packages:
        if package[0] == "bios":
            builder.software_packages.remove(package)
            break
    builder.add_software_package("bios", src_dir="../../../../sw")
    builder.build()


if __name__ == "__main__":
    main()
