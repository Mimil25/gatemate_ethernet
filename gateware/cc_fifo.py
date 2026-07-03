from migen.fhdl.structure import *
from migen.fhdl.module import Module
from migen.fhdl.bitcontainer import log2_int
from migen.fhdl.decorators import ClockDomainsRenamer
from migen.genlib.fifo import _FIFOInterface
from migen import *

__all__ = ["CC_AsyncFIFO", "CC_AsyncFIFOBuffered"]


class CC_AsyncFIFO(Module, _FIFOInterface):
    """Asynchronous FIFO (first in, first out)

    Read and write interfaces are accessed from different clock domains,
    named `read` and `write`. Use `ClockDomainsRenamer` to rename to
    other names.

    {interface}
    """
    __doc__ = __doc__.format(interface=_FIFOInterface.__doc__)

    impl_widths = [1, 2, 5, 10, 20, 40] # TODO add 80

    def __init__(self, width, depth):

        impl_width = min(w for w in self.impl_widths if w >= width)

        assert depth == 1024 # TODO

        _FIFOInterface.__init__(self, width, depth)

        ###

        full = Signal()
        empty = Signal()

        self.fifo = Instance("CC_FIFO_40K",
            p_A_RD_WIDTH = impl_width,
            p_A_DO_REG = True,
            p_B_RW_WIDTH = impl_width,

            p_FIFO_MODE = "ASYNC",

            i_A_CLK = ClockSignal("read"),
            i_A_EN = self.re,
            o_A_DO = self.dout,
            
            i_B_CLK = ClockSignal("write"),
            i_B_EN = self.we,
            i_B_WE = self.we,
            o_B_DI = self.din,

            o_F_FULL = full,
            o_F_EMPTY = empty,

            i_F_RST_N = ~(ResetSignal("write") | ResetSignal("read")),
        )

        self.comb += [
            self.writable.eq(~full),
            self.readable.eq(~empty),
        ]

class CC_AsyncFIFOBuffered(Module, _FIFOInterface):
    """Improves timing when it breaks due to sluggish clock-to-output
    delay in e.g. Xilinx block RAMs. Increases latency by one cycle."""
    def __init__(self, width, depth):
        _FIFOInterface.__init__(self, width, depth)
        self.submodules.fifo = fifo = CC_AsyncFIFO(width, depth)

        self.writable = fifo.writable
        self.din = fifo.din
        self.we = fifo.we

        self.sync.read += \
            If(self.re | ~self.readable,
                self.dout.eq(fifo.dout),
                self.readable.eq(fifo.readable)
            )
        self.comb += fifo.re.eq(self.re | ~self.readable)
