#
# This file is part of LiteEth.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 LumiGuide Fietsdetectie B.V. <goemansrowan@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen.genlib.fifo import _FIFOInterface, AsyncFIFO
from migen.genlib.cdc  import PulseSynchronizer
from litex.gen import *

from liteeth.common import *
from liteeth.mac.common import *

# MAC Packet CDCs ---------------------------------------------------------------------------------

class MacPacketSlowToFastCDC(LiteXModule):
    def __init__(self, dw=32, fifo_impl=AsyncFIFO):
        self.sink      = stream.Endpoint(eth_phy_description(dw))
        self.source    = stream.Endpoint(eth_phy_description(dw))
        
        self.submodules.fifo = fifo = fifo_impl(width=len(self.sink.payload), depth=1024)

        self.packet_rdy = PulseSynchronizer("write", "read")
        self.packet_ack = PulseSynchronizer("read", "write")

        self.comb += [
            fifo.din.eq(self.sink.payload.raw_bits()),
            self.source.payload.raw_bits().eq(fifo.dout)
        ]

        self.slow_fsm = ClockDomainsRenamer("write")(FSM())
        self.slow_fsm.act("IDLE",
            self.sink.ready.eq(fifo.writable),
            fifo.we.eq(self.sink.valid),
            If(self.sink.valid,
               NextState("LOADING-FIFO"),
            ),
        )
        self.slow_fsm.act("LOADING-FIFO",
            self.sink.ready.eq(fifo.writable),
            fifo.we.eq(self.sink.valid),
            If(self.sink.valid & (self.sink.last_be | self.sink.last),
               NextState("SEND-PACKET"),
            ),
        )
        self.slow_fsm.act("SEND-PACKET",
            self.sink.ready.eq(0),
            fifo.we.eq(0),
            self.packet_rdy.i.eq(1),
            If(self.packet_ack.o,
               NextState("IDLE"),
            ),
        )

        self.fast_fsm = ClockDomainsRenamer("read")(FSM())
        self.fast_fsm.act('IDLE',
            self.source.valid.eq(0),
            fifo.re.eq(0),
            If(self.packet_rdy.o,
               self.packet_ack.i.eq(1),
               NextState("SENDING-PACKET"),
            ),
        )
        self.fast_fsm.act("SENDING-PACKET",
            fifo.re.eq(1),
            If(self.source.last | self.source.last_be,
                NextState("IDLE"),
            ),
        )
