#!/usr/bin/env python3
from litex.tools.litex_client import RemoteClient
import sys

wb = RemoteClient(debug=True)
wb.open()


wb.regs.j2_pins_oe.write(int(sys.argv[1], 0))
print("0x{:08x}".format(wb.regs.j2_pins_oe.read()))
wb.regs.j2_pins_out.write(int(sys.argv[2], 0))
print("0x{:08x}".format(wb.regs.j2_pins_out.read()))
wb.close()

