#!/usr/bin/env python3
import sys

from migen import *

from litex.soc.cores import *
from litex.soc.cores import gpio
from litex.soc.cores import uart
from litex.soc.cores.uart import UARTWishboneBridge

from litex.soc.integration import soc_core
from litex.soc.integration.builder import *

from litex.soc.interconnect.csr import *

from litex.build.generic_platform import Pins, IOStandard, Misc, Subsignal

sys.path.insert(0, "litex-buildenv")
from gateware import ice40
from platforms import icebreaker


# Build --------------------------------------------------------------------------------------------

_charlieplexed_pmod7 = [
    ("pmod7", 0,
     Subsignal("led1_a", Pins("PMOD1A:0")),
     Subsignal("led2_a", Pins("PMOD1A:1")),
     Subsignal("led3_a", Pins("PMOD1A:2")),
     Subsignal("led4_a", Pins("PMOD1A:3")),
     Subsignal("led5_a", Pins("PMOD1A:4")),
     Subsignal("led6_a", Pins("PMOD1A:5")),
     Subsignal("led7_a", Pins("PMOD1A:6")),
     Subsignal("led8_a", Pins("PMOD1A:7")),
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

class Charlieplex(Module, AutoCSR):
    def __init__(self, pads):
        pad_count = len(pads)
        self._pins_out = CSRStorage(size=pad_count)
        self._pins_oe = CSRStorage(size=pad_count)
        self._pins_gpio = CSRStorage(size=pad_count, reset=0xff)
        self._value = CSRStorage(size=64)
        local_out = Signal(pad_count)
        local_oe = Signal(pad_count)
        gpio_pins_t = [None] * pad_count
        bit = 0
        # per pin control
        for pin_group in pads.layout:
            for pin in getattr(pads, pin_group[0]):
                gpio_pins_t[bit] = TSTriple()
                self.specials += gpio_pins_t[bit].get_tristate(pin)
                self.comb += gpio_pins_t[bit].o.eq((self._pins_gpio.storage[bit] & self._pins_out.storage[bit]) | (~self._pins_gpio.storage[bit] & local_out[bit]))
                self.comb += gpio_pins_t[bit].oe.eq((self._pins_gpio.storage[bit] & self._pins_oe.storage[bit]) | (~self._pins_gpio.storage[bit] & local_oe[bit]))
                bit=bit+1
        # charlieplex scanning all digits and segments, one at a time. refresh rate is clk/(segments*digits)
        digit_index = Signal(3)
        segment_index = Signal(3)
        next_digit_value = Signal(4)
        self.sync += If(segment_index == 6, segment_index.eq(0),
                        If(digit_index == 7, digit_index.eq(0))
                        .Else(digit_index.eq(digit_index+1))).\
                     Else(segment_index.eq(segment_index+1))
        # we always pull the digit anode high and pull all segment
        # cathodes low in the output register, what goes to the pins
        # is controlled by the output enable register
        self.sync += local_out.eq(1 << digit_index)
        self.sync += local_oe.eq((1 << digit_index) | 1)
        
def main():
    platform = icebreaker.Platform()
    platform.add_extension(_serial2_pmod)
    platform.add_extension(_charlieplexed_pmod7)

    sys_clk_freq = 1/platform.default_clk_period*1e9
    soc = soc_core.SoCCore(platform,
                           sys_clk_freq,
                           cpu_type=None,
                           cpu_variant="minimal+debug",
                           uart_name="serial2",
                           with_uart=True,
                           with_ctrl=False,
                           integrated_rom_size=0x2000,
                           integrated_sram_size=0)
    # SPRAM- UP5K has single port RAM, might as well use it as SRAM to
    # free up scarce block RAM.
    spram_size = 128*1024
    soc.submodules.spram = ice40.SPRAM(size=spram_size)
    soc.register_mem("sram", 0x10000000, soc.spram.bus, spram_size)
    
    # https://github.com/timvideos/litex-buildenv/wiki/LiteX-for-Hardware-Engineers#litescope-bridge
    soc.submodules.uartbridge = UARTWishboneBridge(platform.request("serial"), int(sys_clk_freq), baudrate=115200)
    soc.add_wb_master(soc.uartbridge.wishbone)

    soc.submodules.pmod7 = Charlieplex(platform.request("pmod7"))
    soc.add_csr("pmod7")
    
    led_pad = platform.request("user_led_n", 1)
    soc.submodules.leds = gpio.GPIOOut(led_pad)
    soc.add_csr("leds")

    builder = Builder(soc, csr_csv="csr.csv")
    
#     for package in builder.software_packages:
#         if package[0] == "bios":
#             builder.software_packages.remove(package)
#             break
#     builder.add_software_package("bios", src_dir="../../../../sw")
    builder.build()


if __name__ == "__main__":
    main()
