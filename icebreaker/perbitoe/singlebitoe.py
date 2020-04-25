import sys

if len(sys.argv) != 3:
    print("Usage:   {} <migenpath> <builddir>".format(sys.argv[0]))
    sys.exit(-1)

sys.path.append(sys.argv[1])
build_dir=sys.argv[2]

from migen import *
from migen.build.generic_platform import Pins
from migen.build.platforms import icebreaker, arty_a7, papilio_pro, versaecp55g

_icebreaker_connector = [
    ("io", 0, Pins("PMOD1A:0", "PMOD1A:1")),
]

_papilio_pro_connector = [
    ("io", 0, Pins("A:0", "A:1")),
]

_arty_a7_connector = [
    ("io", 0, Pins("pmoda:0", "pmoda:1")),
]

_versaecp55g_connector = [
    ("io", 0, Pins("X3:4", "X3:5")),
]
    
class module(Module):
    def __init__(self, plat):
        pads = plat.request("io")
        counter = Signal(26)
        t = TSTriple(len(pads))
        self.comb += t.o.eq(counter[25:23:-1])
        self.comb += t.oe.eq(counter[25:23:-1])
        self.specials += t.get_tristate(pads)
        self.sync += If(t.i[0] | t.i[1], counter.eq(0)).Else(counter.eq(counter + 1))

plat_icebreaker = icebreaker.Platform()
plat_icebreaker.add_extension(_icebreaker_connector)
plat_icebreaker.build(module(plat_icebreaker), run=False, build_dir=build_dir + "/icebreaker_build")

plat_papilio_pro = papilio_pro.Platform()
plat_papilio_pro.add_extension(_papilio_pro_connector)
plat_papilio_pro.build(module(plat_papilio_pro), run=False, build_dir=build_dir + "/papilio_pro_build")

plat_versaecp55g = versaecp55g.Platform()
plat_versaecp55g.add_extension(_versaecp55g_connector)
plat_versaecp55g.build(module(plat_versaecp55g), run=False, build_dir=build_dir + "/versaecp55g_build")

plat_arty_a7 = arty_a7.Platform()
plat_arty_a7.add_extension(_arty_a7_connector)
plat_arty_a7.build(module(plat_arty_a7), run=False, build_dir=build_dir + "/arty_a7_build")






