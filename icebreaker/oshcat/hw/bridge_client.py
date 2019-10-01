#!/usr/bin/env python3
from litex.tools.litex_client import RemoteClient

wb = RemoteClient()
wb.open()


print("\nPWM")
#wb.write(0x82002800, 0xff)
#wb.write(0x82002810, 0xff)
#wb.write(0x8200280C, 0x40)

print("Enable  0x{:08X}".format(wb.read(0x82002800)))
print("Divider 0x{:08X}".format(wb.read(0x82002804) << 8 | wb.read(0x82002808)))
print("Period  0x{:08X}".format(wb.read(0x82002810)))
print("Width   0x{:08X}".format(wb.read(0x8200280C)))
print("Count   0x{:08X}".format(wb.read(0x82002814)))

print("Enable  0x{:08X}".format(wb.read(0x82003000)))
print("Divider 0x{:08X}".format(wb.read(0x82003004) << 8 | wb.read(0x82002808)))
print("Period  0x{:08X}".format(wb.read(0x82003010)))
print("Width   0x{:08X}".format(wb.read(0x8200300C)))
print("Count   0x{:08X}".format(wb.read(0x82003014)))

wb.close()

