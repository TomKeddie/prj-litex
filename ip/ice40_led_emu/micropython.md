```
import fomu
rgb = fomu.rgb()

# enable, 250Hz, Active Low, 
rgb.write_raw(0b1000, 0b11100011)

# disable breathe
rgb.write_raw(0b0101, 0b00000000)
rgb.write_raw(0b0110, 0b00000000)

# 1s blink
rgb.write_raw(0b1010, 0b00100000)
rgb.write_raw(0b1011, 0b00100000)

# all red (scope is on red pin)
rgb.write_raw(0b0001, 0)
rgb.write_raw(0b0010, 255)
rgb.write_raw(0b0011, 0)

import machine
machine.mem32[0xe0006808] = 0
machine.mem32[0xe0006808] = 7

```