#
# This file is part of MiSoC and has been adapted/modified for LiteEth.
#
# Copyright (c) 2018-2020 Sebastien Bourdeauducq <sb@m-labs.hk>
# Copyright (c) 2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.fsm import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer
from litex.gen.genlib.cdc import BusSynchronizer

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.cores.code_8b10b import K, D, Encoder, Decoder

from liteeth.common import *

_K = K
_D = D
K = lambda x, y: C(_K(x, y), 8)
D = lambda x, y: C(_D(x, y), 8)


# PCS SGMII Timer ----------------------------------------------------------------------------------

SGMII_1000MBPS_SPEED = 0b10
SGMII_100MBPS_SPEED  = 0b01
SGMII_10MBPS_SPEED   = 0b00

class PCSSGMIITimer(LiteXModule):
    def __init__(self, speed):
        self.enable = Signal()
        self.done   = Signal()

        # # #

        count = Signal(max=100)
        self.comb += self.done.eq(count == 0)
        self.sync += [
            count.eq(count - 1),
            If(~self.enable | self.done,
                Case(speed, {
                    SGMII_10MBPS_SPEED   : count.eq(99),
                    SGMII_100MBPS_SPEED  : count.eq(9),
                    SGMII_1000MBPS_SPEED : count.eq(0),
                })
            )
        ]

# PCS TX -------------------------------------------------------------------------------------------

class PCSTX32(LiteXModule):
    def __init__(self):
        self.config_valid = Signal()                               # Config valid.
        self.config_reg   = Signal(16)                             # Config register (16-bit).
        self.sink         = sink = stream.Endpoint(eth_phy_description(32))  # Data input.

        # PHY ports output
        self.char_is_k = Signal(4) # is K output
        self.data = Signal(32) # Data output
        
        self.data.attr.add('keep')
        self.char_is_k.attr.add('keep')

        # Signals.
        # --------
        ctype  = Signal() # Toggles config type.

        # FSM.
        # ----
        self.fsm = fsm = FSM()
        fsm.act("START",
            sink.ready.eq(1),
            NextValue(self.char_is_k, 0b0001),
            NextValue(self.data[0:8], K(28, 5)),
            # Wait for valid Config.
            If(self.config_valid,
                NextValue(ctype, ~ctype),
                NextValue(self.data[8:16], Mux(ctype,
                    D(21, 5), # /C1/.
                    D( 2, 2), # /C2/.
                )),
                NextValue(self.data[16:32], self.config_reg),
            # Wait for valid Data.
            ).Else(
                If(sink.valid,
                    NextValue(self.data, Cat(
                        K(27, 7), # Start-of-packet /S/.
                        sink.data[8:32]
                    )),
                    NextState("DATA")
                ).Else(
                    NextValue(self.data, Cat( # IDLE, disparity handeled by CC_SERDES
                        K(28, 5), # D(5, 6), 
                        D(5, 6), 
                        D(5, 6), 
                        D(5, 6), 
                    )),
                )
            )
        )
        fsm.act("DATA",
            # Send Data.
            sink.ready.eq(1),
            If(sink.valid,
                If(sink.last,
                    If(sink.be == 0b1111,
                        NextValue(self.char_is_k, 0b0000),
                        NextValue(self.data, sink.data),
                    ),
                    If(sink.be == 0b0111,
                        NextValue(self.char_is_k, 0b1000),
                        NextValue(self.data, Cat(
                            sink.data[0:24],
                            K(29, 7), # End-of-frame /T/.
                            )),
                        NextState("START")
                    ),
                    If(sink.be == 0b0011,
                        NextValue(self.char_is_k, 0b1100),
                        NextValue(self.data, Cat(
                            sink.data[0:16],
                            K(29, 7), # End-of-frame /T/.
                            K(23, 7), # Carrier Extend /R/.
                            )),
                        NextState("START")
                    ),
                    If(sink.be == 0b0001,
                        NextValue(self.char_is_k, 0b1110),
                        NextValue(self.data, Cat(
                            sink.data[0:8],
                            K(29, 7), # End-of-frame /T/.
                            K(23, 7), # Carrier Extend /R/.
                            K(23, 7),
                            )),
                        NextState("START")
                    ),
                ).Else(
                    NextValue(self.char_is_k, 0b0000),
                    NextValue(self.data, sink.data),
                ),
            ).Else(
                NextValue(self.char_is_k, 0b1111),
                NextValue(self.data, Cat(
                    K(29, 7), # End-of-frame /T/.
                    K(23, 7), # Carrier Extend /R/.
                    K(23, 7), 
                    K(23, 7), 
                )),
                NextState("START")
            )
        )
        #self.sync += parity.eq(~parity) # Toggle parity for /R/ extension.

# PCS RX -------------------------------------------------------------------------------------------

class PCSRX32(LiteXModule):
    def __init__(self):
        self.seen_valid_ci   = Signal()   # CI seen.
        self.seen_config_reg = Signal()   # Config seen.
        self.config_reg      = Signal(16) # Config register (16-bit).
        self.error_count     = Signal(8, reset=0xff)
        self.error_data      = Signal(32) # data capture in last unexpected state
        self.error_char_is_k = Signal(4)  # char_is_k capture in last unexpected state
        self.source          = source = stream.Endpoint(eth_phy_description(32)) # Data output.

        # PHY ports inputs
        self.data = Signal(32)
        self.char_is_k = Signal(4)
        self.disparity_err = Signal(4)
        self.table_err = Signal(4)

        # # #

        # Signals.
        # --------
        invalid = Signal()

        eth_invalid = Signal()


        # Pre Buffer.
        # To ease routing around the CC_SERDES this buffer allow the logic to be placed further away
        # -----------
        data          = Signal.like(self.data)
        char_is_k     = Signal.like(self.char_is_k)
        disparity_err = Signal.like(self.disparity_err)
        table_err     = Signal.like(self.table_err)
        data.attr.add('keep')
        char_is_k.attr.add('keep')
        disparity_err.attr.add('keep')
        table_err.attr.add('keep')
        self.sync += [
            data.eq(self.data),
            char_is_k.eq(self.char_is_k),
            disparity_err.eq(self.disparity_err),
            table_err.eq(self.table_err),
        ]
        
        # Invalid char counter.
        # ---------------------
        self.comb += invalid.eq(table_err)# | disparity_err)
        self.sync += [
            If((invalid | eth_invalid) & (self.error_count < 0xfe),
                self.error_count.eq(self.error_count + 1),
                self.error_data.eq(data),
                self.error_char_is_k.eq(char_is_k),
            ),
            If((self.error_count == 0xff) & self.seen_config_reg, # start error count once rx is aligned
                self.error_count.eq(0),
            ),
        ]

        # Buffer.
        # -------
        self.buffer = buffer = stream.Buffer(eth_phy_description(32), pipe_valid=True, pipe_ready=False)
        self.comb += [
            buffer.source.connect(source),
            source.last.eq(source.valid & ~buffer.sink.valid),
        ]

        # FSM.
        # ----
        self.fsm = fsm = FSM()
        fsm.act("START",
            NextValue(buffer.sink.valid, 0),
            eth_invalid.eq(1),
            # Wait for a K-character.
            If(char_is_k[:2] == 0x0001,
                # K-character is Config or Idle K28.5.
                If(data[0:8] == K(28, 5),
                    If((data[8:16] == D(21, 5)) | # /C1/.
                       (data[8:16] == D( 2, 2)),  # /C2/.
                        eth_invalid.eq(0),
                        self.seen_valid_ci.eq(1),
                        self.seen_config_reg.eq(1),
                        self.config_reg.eq(data[16:32]),
                    ),
                    # Check for Idle Word.
                    If((data[8:16] == D( 5, 6)) | # /I1/.
                       (data[8:16] == D(16, 2)),  # /I2/.
                        eth_invalid.eq(0),
                        self.seen_valid_ci.eq(1),
                    ),
                ),
                # K-character is Start-of-packet /S/.
                If(data[0:8] == K(27, 7),
                    eth_invalid.eq(0),
                    NextValue(buffer.sink.valid, 1),
                    NextValue(buffer.sink.be, 0b1111),
                    NextValue(buffer.sink.data, Cat(
                        C(0x55, 8), # First Preamble Byte.
                        data[8:32]
                    )),
                    NextState("DATA")
                )
            ),

        )
        fsm.act("DATA",
            eth_invalid.eq(1),
            self.seen_valid_ci.eq(1),
            If((char_is_k == 0) & ~invalid,
                # Receive Data.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.be, 0b1111),
                NextValue(buffer.sink.data, data),
            ),
            If((char_is_k[0] == 1) & (data[0:8] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 0),
                NextValue(buffer.sink.be, 0b0000),
                NextState("START"),
                eth_invalid.eq(1),
            ),
            If((char_is_k[:2] == 0b0010) & (data[8:16] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.be, 0b0001),
                NextValue(buffer.sink.data, data),
                NextState("START"),
            ),
            If((char_is_k[:3] == 0b0100) & (data[16:24] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.be, 0b0011),
                NextValue(buffer.sink.data, data),
                NextState("START"),
            ),
            If((char_is_k == 0b1000) & (data[24:32] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.be, 0b0111),
                NextValue(buffer.sink.data, data),
                NextState("START"),
            ),
        )


# PCS ----------------------------------------------------------------------------------------------

class PCS32(LiteXModule):
    autocsr_exclude = {"ev"}
    def __init__(self, lsb_first=False,
        check_period    = 6e-3,
        breaklink_time  = 10e-3,
        more_ack_time   = 10e-3,
        sgmii_ack_time  = 1.6e-3,
        eth_tx_clk_freq = 125e6,
        with_csr        = False,
        with_autoneg    = True,
        sys_clk_freq    = None,
    ):
        sgmii           = False
        self.sys_clk_freq = sys_clk_freq
        self.tx = ClockDomainsRenamer("eth_tx")(PCSTX32())
        self.rx = ClockDomainsRenamer("eth_rx")(PCSRX32())

        self.sink      = stream.Endpoint(eth_phy_description(32))
        self.source    = stream.Endpoint(eth_phy_description(32))

        self.link_up = Signal()
        self.restart = Signal()
        self.align   = Signal()

        self.lp_abi = BusSynchronizer(16, "eth_rx", "eth_tx")

        # # #

        # Signals.
        # --------
        self.config_empty = config_empty = Signal()
        self.is_sgmii     = is_sgmii     = Signal()
        self.linkdown     = linkdown     = Signal()
        self.autoneg_ack  = autoneg_ack  = Signal()

        # Sink -> TX / RX -> Source.
        self.comb += [
            self.sink.connect(self.tx.sink,     omit={"error"}),
            self.rx.source.connect(self.source),
        ]

        # Pulse Synchronizers.
        # --------------------
        self.seen_valid_ci      = seen_valid_ci      = PulseSynchronizer("eth_rx", "eth_tx")
        self.rx_config_reg_abi  = rx_config_reg_abi  = PulseSynchronizer("eth_rx", "eth_tx")
        self.rx_config_reg_ack  = rx_config_reg_ack  = PulseSynchronizer("eth_rx", "eth_tx")
        self.rx_config_reg_zero = rx_config_reg_zero = PulseSynchronizer("eth_rx", "eth_tx")
        self.comb += seen_valid_ci.i.eq(self.rx.seen_valid_ci)

        # Timers.
        # -------
        self.breaklink_timer = breaklink_timer = ClockDomainsRenamer("eth_tx")(WaitTimer(breaklink_time * eth_tx_clk_freq))
        self.more_ack_timer  = more_ack_timer  = ClockDomainsRenamer("eth_tx")(WaitTimer(more_ack_time  * eth_tx_clk_freq))
        self.sgmii_ack_timer = sgmii_ack_timer = ClockDomainsRenamer("eth_tx")(WaitTimer(sgmii_ack_time * eth_tx_clk_freq))

        # Checker.
        # --------
        checker_max   = int(check_period*eth_tx_clk_freq)
        checker_count = Signal(max=checker_max + 1)
        checker_tick  = Signal()
        checker_error = Signal()
        self.sync.eth_tx += [
            checker_tick.eq(0),
            If(checker_count == 0,
                checker_tick.eq(1),
                checker_count.eq(checker_max)
            ).Else(
                checker_count.eq(checker_count - 1)
            ),
            If(seen_valid_ci.o, checker_error.eq(0)),
            If(checker_tick,    checker_error.eq(1))
        ]

        # Linkdown/Speed Detection.
        # -------------------------
        if sgmii is None and not with_autoneg:
            # If autoneg FSM is disabled, but sgmii mode is not specified, start in SGMII mode.
            sgmii = True
        sgmii_speed_valid = Signal()
        sgmii_tx_speed    = Signal(2)
        sgmii_rx_speed    = Signal(2)
        if sgmii is None:
            self.comb += is_sgmii.eq(self.lp_abi.o[0])
        else:
            self.comb += is_sgmii.eq(int(sgmii))
        # self.comb += [
        #     # Speed 0b11 is reserved and an empty register means nothing received yet: 1000 Mbps.
        #     sgmii_speed_valid.eq((self.lp_abi.o[10:12] != 0b11) & (self.lp_abi.o != 0)),
        #     sgmii_tx_speed.eq(Mux(sgmii_speed_valid, self.lp_abi.o[10:12], SGMII_1000MBPS_SPEED)),
        #     sgmii_rx_speed.eq(Mux((self.lp_abi.i[10:12] != 0b11) & (self.lp_abi.i != 0),
        #         self.lp_abi.i[10:12], SGMII_1000MBPS_SPEED)),
        #     # Detect that link is down:
        #     # - 1000BASE-X : linkup can be inferred by non-empty reg.
        #     # - SGMII      : linkup is indicated with bit 15.
        #     If(~is_sgmii,
        #         linkdown.eq(self.lp_abi.o == 0),
        #         self.tx.sgmii_speed.eq(0b10),
        #         self.rx.sgmii_speed.eq(0b10),
        #     ).Else(
        #         linkdown.eq(~self.lp_abi.o[15] | ~sgmii_speed_valid),
        #         self.tx.sgmii_speed.eq(sgmii_tx_speed),
        #         self.rx.sgmii_speed.eq(sgmii_rx_speed),
        #     )
        # ]

        # TX Config.
        # ----------
        self.comb += [
            If(~config_empty,
                self.tx.config_reg[0].eq(is_sgmii),                     # SGMII: SGMII in-use.
                self.tx.config_reg[5].eq(~is_sgmii),                    # 1000BASE-X: Full-duplex.
                If(is_sgmii,
                    self.tx.config_reg[10:12].eq(sgmii_tx_speed),       # SGMII: Speed.
                    self.tx.config_reg[12].eq(1),                       # SGMII: Full-duplex.
                    self.tx.config_reg[15].eq(self.link_up),            # SGMII: Link-up.
                ),
                self.tx.config_reg[14].eq(autoneg_ack),                 # SGMII/1000BASE-X: Acknowledge Bit.
            )
        ]

        if with_autoneg:
            # FSM.
            # ----
            self.fsm = fsm = ClockDomainsRenamer("eth_tx")(FSM())
            # AN_ENABLE.
            fsm.act("AUTONEG-BREAKLINK",
                self.tx.config_valid.eq(1),
                config_empty.eq(1),
                breaklink_timer.wait.eq(1),
                If(breaklink_timer.done,
                    NextState("AUTONEG-WAIT-ABI")
                )
            )
            # ABILITY_DETECT.
            fsm.act("AUTONEG-WAIT-ABI",
                self.align.eq(1),
                self.tx.config_valid.eq(1),
                If(rx_config_reg_abi.o,
                    NextState("AUTONEG-WAIT-ACK")
                ),
                If(checker_tick & checker_error,
                    self.restart.eq(1),
                    NextState("AUTONEG-BREAKLINK")
                )
            )
            # ACKNOWLEDGE_DETECT.
            fsm.act("AUTONEG-WAIT-ACK",
                self.tx.config_valid.eq(1),
                autoneg_ack.eq(1),
                If(rx_config_reg_ack.o,
                    NextState("AUTONEG-SEND-MORE-ACK")
                ),
                # Link partner restarted Auto-Negotiation: restart too.
                If(rx_config_reg_zero.o,
                    NextState("AUTONEG-BREAKLINK")
                ),
                If(checker_tick & checker_error,
                    self.restart.eq(1),
                    NextState("AUTONEG-BREAKLINK")
                )
            )
            # COMPLETE_ACKNOWLEDGE.
            fsm.act("AUTONEG-SEND-MORE-ACK",
                self.tx.config_valid.eq(1),
                autoneg_ack.eq(1),
                more_ack_timer.wait.eq(~is_sgmii),
                sgmii_ack_timer.wait.eq(is_sgmii),
                If((is_sgmii & sgmii_ack_timer.done) |
                    (~is_sgmii & more_ack_timer.done),
                    NextState("RUNNING")
                ),
                # Link partner restarted Auto-Negotiation: restart too.
                If(rx_config_reg_zero.o,
                    NextState("AUTONEG-BREAKLINK")
                ),
                If(checker_tick & checker_error,
                    self.restart.eq(1),
                    NextState("AUTONEG-BREAKLINK")
                )
            )
            # LINK_OK.
            fsm.act("RUNNING",
                self.link_up.eq(~linkdown),
                If((checker_tick & checker_error) | linkdown,
                    self.restart.eq(1),
                    NextState("AUTONEG-BREAKLINK")
                )
            )

            # RX Config (and consistency check).
            # ----------------------------------
            rx_config_reg_count  = Signal(4)
            rx_config_reg_last   = Signal(16)
            self.sync.eth_rx += [
                # Single-cycle pulses (to the PulseSynchronizers).
                rx_config_reg_abi.i.eq(0),
                rx_config_reg_ack.i.eq(0),
                rx_config_reg_zero.i.eq(0),
                If(self.rx.seen_config_reg,
                    # Consistency Count/Check.
                    rx_config_reg_last.eq(self.rx.config_reg),
                    If(self.rx.config_reg != rx_config_reg_last,
                        rx_config_reg_count.eq(8 - 1)
                    ).Else(
                        If(rx_config_reg_count != 0,
                            rx_config_reg_count.eq(rx_config_reg_count - 1),
                        ).Else(
                            # When RX Config is consistent (Clause 37).
                            If(self.rx.config_reg == 0,
                                # Link partner restarting Auto-Negotiation (AN_ENABLE/breaklink).
                                rx_config_reg_zero.i.eq(1),
                            ).Else(
                                # Ability match, regardless of the Acknowledge bit: the link
                                # partner can already be acknowledging.
                                rx_config_reg_abi.i.eq(1),
                                # Acknowledge match.
                                rx_config_reg_ack.i.eq(self.rx.config_reg[14]),
                            )
                        )
                    ),
                    self.lp_abi.i.eq(self.rx.config_reg)
                )
            ]
        else:
            # Fixed-rate link, no Clause 37 autonegotiation.
            self.comb += self.align.eq(1)
            self.sync.eth_tx += [
                # Release RX reset between failed checks so valid code groups can return.
                self.restart.eq(checker_tick & checker_error),
                If(checker_tick,
                    self.link_up.eq(~checker_error),
                ),
            ]

        if with_csr:
            self.add_csr()

    def add_csr(self, sys_clk_freq=None):
        if sys_clk_freq is None:
            sys_clk_freq = self.sys_clk_freq
        if sys_clk_freq is None:
            sys_clk_freq = getattr(LiteXContext.top, "sys_clk_freq", None)
        if sys_clk_freq is None or int(sys_clk_freq) <= 0:
            raise ValueError("A positive sys_clk_freq is required for PCS CSRs.")

        self.status = CSRStatus(fields=[
            CSRField("link_up",    size=1,  offset=0,  description="Link is up."),
            CSRField("is_sgmii",   size=1,  offset=1,  description="SGMII in-use."),
            CSRField("config_reg", size=16, offset=16, description="Link partner ability register."),
        ])

        self.lp_abi_csr = BusSynchronizer(16, "eth_rx", "sys")

        self.ev      = EventManager()
        self.ev.link = EventSourceProcess(edge="any")
        self.ev.finalize()

        self.comb += [
            self.lp_abi_csr.i.eq(self.lp_abi.i),
            self.status.fields.config_reg.eq(self.lp_abi_csr.o)
        ]

        # The PCS status originates in eth_tx. Both software and the event FSM consume
        # synchronized system-domain copies; the link-partner word uses its bus handshake.
        link_up_tx  = Signal()
        is_sgmii_tx = Signal()
        self.sync.eth_tx += [
            link_up_tx.eq(self.link_up),
            is_sgmii_tx.eq(self.is_sgmii),
        ]
        self.specials += [
            MultiReg(link_up_tx,  self.status.fields.link_up),
            MultiReg(is_sgmii_tx, self.status.fields.is_sgmii),
        ]

        self.link_up_timer = link_up_timer = WaitTimer(int(sys_clk_freq))

        self.csr_fsm = fsm = FSM()
        fsm.act("DOWN",
            If(self.status.fields.link_up,
                NextState("UP")
            )
        )
        fsm.act("UP",
            link_up_timer.wait.eq(1),
            self.ev.link.trigger.eq(link_up_timer.done),
            If(~self.status.fields.link_up,
                NextState("DOWN"),
            )
        )
