#!/usr/bin/env python3
from litex.tools.litex_client import RemoteClient

wb = RemoteClient()
wb.open()

val = wb.read(0x82003000)
print("{}\n".format(val))

wb.close()

