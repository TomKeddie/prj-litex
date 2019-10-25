#!/usr/bin/env python3
from litex.tools.litex_client import RemoteClient

wb = RemoteClient(debug=True)
wb.open()

wb.regs.leds_out.write(1)

wb.regs.pmoda_pmod_oe.write(0xff)
wb.regs.pmoda_pmod_out.write(0xff)

wb.close()

