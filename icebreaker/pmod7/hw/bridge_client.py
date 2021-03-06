#!/usr/bin/env python3
import sys
import time
from litex.tools.litex_client import RemoteClient

wb = RemoteClient()
wb.open()

wb.regs.pmod7_pins_gpio.write(0xff)


## Hexadecimal encodings for displaying the digits 0 to F[11][12]
## DigitDisplay	gfedcba	abcdefg	a	b	c	d	e	f	g
## 0	0	0x3F	0x7E	on	on	on	on	on	on	
## 1	1	0x06	0x30		on	on				
## 2	2	0x5B	0x6D	on	on		on	on		on
## 3	3	0x4F	0x79	on	on	on	on			on
## 4	4	0x66	0x33		on	on			on	on
## 5	5	0x6D	0x5B	on		on	on		on	on
## 6	6	0x7D	0x5F	on		on	on	on	on	on
## 7	7	0x07	0x70	on	on	on				
## 8	8	0x7F	0x7F	on	on	on	on	on	on	on
## 9	9	0x6F	0x7B	on	on	on	on		on	on
## A	A	0x77	0x77	on	on	on		on	on	on
## b	b	0x7C	0x1F			on	on	on	on	on
## C	C	0x39	0x4E	on			on	on	on	
## d	d	0x5E	0x3D		on	on	on	on		on
## E	E	0x79	0x4F	on			on	on	on	on
## F	F	0x71	0x47	on				on	on	on

# lookup
# gfedcba = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71]

# led1 = [0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]
# led2 = [0x01, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80]
# led3 = [0x02, 0x01, 0x08, 0x10, 0x20, 0x40, 0x80]
# led4 = [0x02, 0x04, 0x01, 0x10, 0x20, 0x40, 0x80]
# led5 = [0x02, 0x04, 0x08, 0x01, 0x20, 0x40, 0x80]
# led6 = [0x02, 0x04, 0x08, 0x10, 0x01, 0x40, 0x80]
# led7 = [0x02, 0x04, 0x08, 0x10, 0x20, 0x01, 0x80]
# led8 = [0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x01]

# led1_values = [None] * 16
# led2_values = [None] * 16
# led3_values = [None] * 16
# led4_values = [None] * 16
# led5_values = [None] * 16
# led6_values = [None] * 16
# led7_values = [None] * 16
# led8_values = [None] * 16

# for ix in range(len(gfedcba)):
#     value = gfedcba[ix]
#     led1_value = 0x01
#     led2_value = 0x02
#     led3_value = 0x04
#     led4_value = 0x08
#     led5_value = 0x10
#     led6_value = 0x20
#     led7_value = 0x40
#     led8_value = 0x80
#     for iy in range(0, 7):
#         if ((1 << iy) & value) != 0:
#             led1_value |= led1[iy]
#             led2_value |= led2[iy]
#             led3_value |= led3[iy]
#             led4_value |= led4[iy]
#             led5_value |= led5[iy]
#             led6_value |= led6[iy]
#             led7_value |= led7[iy]
#             led8_value |= led8[iy]
#     led1_values[ix] = led1_value
#     led2_values[ix] = led2_value
#     led3_values[ix] = led3_value
#     led4_values[ix] = led4_value
#     led5_values[ix] = led5_value
#     led6_values[ix] = led6_value
#     led7_values[ix] = led7_value
#     led8_values[ix] = led8_value
# #    wb.regs.pmod7_pins_out.write(0x10)
# #    wb.regs.pmod7_pins_oe.write(led5_value)
# #    time.sleep(1)

# print('led1_lookup = [{}]'.format(', '.join(hex(x) for x in led1_values)))
# print('led2_lookup = [{}]'.format(', '.join(hex(x) for x in led2_values)))
# print('led3_lookup = [{}]'.format(', '.join(hex(x) for x in led3_values)))
# print('led4_lookup = [{}]'.format(', '.join(hex(x) for x in led4_values)))
# print('led5_lookup = [{}]'.format(', '.join(hex(x) for x in led5_values)))
# print('led6_lookup = [{}]'.format(', '.join(hex(x) for x in led6_values)))
# print('led7_lookup = [{}]'.format(', '.join(hex(x) for x in led7_values)))
# print('led8_lookup = [{}]'.format(', '.join(hex(x) for x in led8_values)))

led1_lookup = [0x7f, 0x0d, 0xb7, 0x9f, 0xcd, 0xdb, 0xfb, 0x0f, 0xff, 0xdf, 0xef, 0xf9, 0x73, 0xbd, 0xf3, 0xe3]
led2_lookup = [0x7f, 0x0e, 0xb7, 0x9f, 0xce, 0xdb, 0xfb, 0x0f, 0xff, 0xdf, 0xef, 0xfa, 0x73, 0xbe, 0xf3, 0xe3]
led3_lookup = [0x7f, 0x0d, 0xb7, 0x9f, 0xcd, 0xde, 0xfe, 0x0f, 0xff, 0xdf, 0xef, 0xfc, 0x76, 0xbd, 0xf6, 0xe6]
led4_lookup = [0x7f, 0x0d, 0xbe, 0x9f, 0xcd, 0xdb, 0xfb, 0x0f, 0xff, 0xdf, 0xef, 0xf9, 0x7a, 0xbd, 0xfa, 0xea]
led5_lookup = [0x7f, 0x1c, 0xb7, 0x9f, 0xdc, 0xdb, 0xfb, 0x1e, 0xff, 0xdf, 0xfe, 0xf9, 0x73, 0xbd, 0xf3, 0xf2]
led6_lookup = [0x7f, 0x2c, 0xb7, 0xbe, 0xec, 0xfa, 0xfb, 0x2e, 0xff, 0xfe, 0xef, 0xf9, 0x73, 0xbd, 0xf3, 0xe3]
led7_lookup = [0x7f, 0x4c, 0xf6, 0xde, 0xcd, 0xdb, 0xfb, 0x4e, 0xff, 0xdf, 0xef, 0xf9, 0x73, 0xfc, 0xf3, 0xe3]
led8_lookup = [0xfe, 0x8c, 0xb7, 0x9f, 0xcd, 0xdb, 0xfb, 0x8e, 0xff, 0xdf, 0xef, 0xf9, 0xf2, 0xbd, 0xf3, 0xe3]

test=[led1_lookup[0xd],
      led2_lookup[0xe],
      led3_lookup[0xa],
      led4_lookup[0xd],
      led5_lookup[0xb],
      led6_lookup[0xe],
      led7_lookup[0xe],
      led8_lookup[0xf]]

if len(sys.argv) == 3:
    wb.regs.pmod7_pins_out.write(int(sys.argv[1],0))
    wb.regs.pmod7_pins_oe.write(int(sys.argv[2], 0))
else:
    while True:
        for ix in range(0,8):
            wb.regs.pmod7_pins_oe.write(0)
            wb.regs.pmod7_pins_out.write(1 << ix)
            wb.regs.pmod7_pins_oe.write((1 << ix) | test[ix])
            time.sleep(0.003)
            

    
wb.close()

