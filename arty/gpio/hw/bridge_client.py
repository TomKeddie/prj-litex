#!/usr/bin/env python3
from litex.tools.litex_client import RemoteClient

wb = RemoteClient(debug=True)
wb.open()

wb.regs.leds_out.write(1)

wb.regs.pmoda_pins_oe.write(0xff)
wb.regs.pmoda_pins_out.write(0xff)
wb.regs.pmodb_pins_oe.write(0xff)
wb.regs.pmodb_pins_out.write(0xff)
wb.regs.pmodc_pins_oe.write(0xff)
wb.regs.pmodc_pins_out.write(0xff)
wb.regs.pmodd_pins_oe.write(0xff)
wb.regs.pmodd_pins_out.write(0xff)

print(wb.regs.pmoda_pins_oe.read())

wb.close()

