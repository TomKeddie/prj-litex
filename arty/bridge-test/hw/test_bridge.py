#!/usr/bin/env python3
import sys
from litex.tools.litex_client import RemoteClient

wb = RemoteClient()
wb.open()

expected = [ 0x12345678, 0x9abcdef0, 0xdeadbeef, 0x55aaaa55 ]

value = wb.regs.scratch_scratch0.read()
expect = expected[0]
if value != expect:
    print("expected 0x{:08x} got 0x{:08x}".format(expect, value))
value = wb.regs.scratch_scratch1.read()
expect = expected[1]
if value != expect:
    print("expected 0x{:08x} got 0x{:08x}".format(expect, value))
value = wb.regs.scratch_scratch2.read()
expect = expected[2]
if value != expect:
    print("expected 0x{:08x} got 0x{:08x}".format(expect, value))
value = wb.regs.scratch_scratch3.read()
expect = expected[3]
if value != expect:
    print("expected 0x{:08x} got 0x{:08x}".format(expect, value))

value = wb.read(wb.regs.scratch_scratch0.addr, length=4)
for ix in range(len(value)):
    if value[ix] != expected[ix]:
        print("{}:expected 0x{:08x} got 0x{:08x}".format(ix, expected[ix], value[ix]))

    
wb.close()

