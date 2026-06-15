#
# This file is part of MiSoC and has been adapted/modified for LiteEth.
#
# Copyright (c) 2018-2020 Sebastien Bourdeauducq <sb@m-labs.hk>
# Copyright (c) 2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.fsm import *
from migen.genlib.cdc import PulseSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer
from litex.gen.genlib.cdc import BusSynchronizer

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.cores.code_8b10b import K, D

from liteeth.common import *


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
        self.sgmii_speed  = Signal(2)                              # SGMII speed.
        self.sink         = sink = stream.Endpoint([("data", 32)])  # Data input.

        # PHY ports output
        self.char_is_k = Signal(4) # is K output
        self.data = Signal(32) # Data output

        # Signals.
        # --------
        count  = Signal() # Byte counter for config register.
        parity = Signal() # Parity for /R/ extension.
        ctype  = Signal() # Toggles config type.

        # SGMII Timer.
        # ------------
        self.timer = timer = PCSSGMIITimer(speed=self.sgmii_speed)

        # FSM.
        # ----
        self.fsm = fsm = FSM()
        fsm.act("START",
            self.char_is_k.eq(0b0001),
            self.data[0:8].eq(K(28, 5)),
            # Wait for valid Config.
            If(self.config_valid,
                NextValue(ctype, ~ctype),
                Case(ctype, {
                    0b0 : self.data[8:16].eq(D(21, 5)), # /C1/.
                    0b1 : self.data[8:16].eq(D( 2, 2)), # /C2/.
                }),
                self.data[16:32].eq(self.config_reg),
            # Wait for valid Data.
            ).Else(
                If(sink.valid,
                    sink.ready.eq(timer.done),
                    self.data.eq(Cat(
                        K(27, 7), # Start-of-packet /S/.
                        sink.data[8:32]
                    )),
                    NextState("DATA")
                ).Else(
                    self.data.eq(Cat( # IDLE TODO handle disparity
                        K(28, 5), # D(5, 6), 
                        D(5, 6), 
                        D(5, 6), 
                        D(5, 6), 
                    )),
                    # NextState("IDLE")
                )
            )
        )
        fsm.act("DATA",
            # Send Data.
            timer.enable.eq(1),
            sink.ready.eq(timer.done),
            If(sink.valid,
                self.data.eq(sink.data),
            ).Else(
                self.char_is_k.eq(0b1111),
                self.data.eq(Cat(
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
        self.sgmii_speed     = Signal(2)  # SGMII speed.
        self.source          = source = stream.Endpoint([("data", 32), ("error", 1)]) # Data output.

        # PHY ports inputs
        self.data = Signal(32)
        self.char_is_k = Signal(4)
        self.disparity_err = Signal(4)
        self.table_err = Signal(4)
        self.enable = Signal()

        # # #

        # Signals.
        # --------
        count = Signal() # Byte counter for config register.
        invalid = Signal()

        self.comb += invalid.eq(self.table_err)

        # SGMII Timer.
        # ------------
        self.timer = timer = CEInserter()(PCSSGMIITimer(speed=self.sgmii_speed))
        self.comb += timer.ce.eq(self.enable) 

        # Buffer.
        # -------
        self.buffer = buffer = stream.Buffer([("data", 32)], pipe_valid=True, pipe_ready=False)
        self.comb += If(timer.ce & timer.done,
            buffer.source.connect(source, omit={"last", "error"}),
            source.last.eq(buffer.source.valid & ~buffer.sink.valid), # Last when next is not valid.
        )

        # FSM.
        # ----
        self.fsm = fsm = FSM()
        fsm.act("START",
            If(self.enable,
                # Wait for a K-character.
                If(self.char_is_k == 0b0001,
                    # K-character is Config or Idle K28.5.
                    If(self.data[0:8] == K(28, 5),
                        NextValue(count, 0),
                        If((self.data[8:16] == D(21, 5)) | # /C1/.
                          (self.data[8:16] == D( 2, 2)),  # /C2/.
                            self.seen_valid_ci.eq(1),
                            self.seen_config_reg.eq(1),
                            NextValue(self.config_reg, self.data[16:32]),
                        ),
                        # Check for Idle Word.
                        If((self.data[8:16] == D( 5, 6)) | # /I1/.
                           (self.data[8:16] == D(16, 2)),  # /I2/.
                            self.seen_valid_ci.eq(1),
                            NextState("START")
                        )
                       ),
                    # K-character is Start-of-packet /S/.
                       If(self.data[0:8] == K(27, 7),
                        timer.enable.eq(1),
                        buffer.sink.valid.eq(1),
                        buffer.sink.data.eq(Cat(
                            0x55, # First Preamble Byte.
                            self.data[8:32]
                        )),
                        NextState("DATA")
                    )
                )
            )
        )
        fsm.act("DATA",
            If(self.enable,
                If((self.char_is_k == 0) & ~invalid,
                    # Receive Data.
                    timer.enable.eq(1),
                    buffer.sink.valid.eq(timer.done),
                    buffer.sink.data.eq(self.data),
                   ).Elif((self.char_is_k == 1) & (self.data[0:8] == K(29, 7)) & ~invalid, # TODO handle unaligned EPD
                    # K-character is End-of-packet /S/.
                    NextState("START"),
                ).Else(
                    source.error.eq(1),
                    source.last.eq(1),
                    source.valid.eq(1),
                    If(source.ready,
                       NextState("ERROR"),
                    )
                )
            )
        )
        fsm.act("ERROR",
            NextState("START")
        )

# PCS ----------------------------------------------------------------------------------------------

class PCS32(LiteXModule):
    autocsr_exclude = {"ev"}
    def __init__(self, bw=16, check_period=6e-3, breaklink_time=10e-3, more_ack_time=10e-3, sgmii_ack_time=1.6e-3, eth_tx_clk_freq=125e6, with_csr=True):
        self.tx = ClockDomainsRenamer("eth_tx")(PCSTX32())
        self.rx = ClockDomainsRenamer("eth_rx")(PCSRX32())

        # MAC side streams
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
            self.sink.connect(self.tx.sink,     omit={"last_be", "error"}),
            self.rx.source.connect(self.source, omit={"last_be"}),
        ]

        # Pulse Synchronizers.
        # --------------------
        self.seen_valid_ci     = seen_valid_ci     = PulseSynchronizer("eth_rx", "eth_tx")
        self.rx_config_reg_abi = rx_config_reg_abi = PulseSynchronizer("eth_rx", "eth_tx")
        self.rx_config_reg_ack = rx_config_reg_ack = PulseSynchronizer("eth_rx", "eth_tx")
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
        sgmii_speed_valid = Signal()
        sgmii_tx_speed    = Signal(2)
        sgmii_rx_speed    = Signal(2)
        self.comb += [
            is_sgmii.eq(self.lp_abi.o[0]),
            sgmii_speed_valid.eq(self.lp_abi.o[10:12] != 0b11),
            sgmii_tx_speed.eq(Mux(sgmii_speed_valid, self.lp_abi.o[10:12], SGMII_1000MBPS_SPEED)),
            sgmii_rx_speed.eq(Mux(self.lp_abi.i[10:12] != 0b11, self.lp_abi.i[10:12], SGMII_1000MBPS_SPEED)),
            # Detect that link is down:
            # - 1000BASE-X : linkup can be inferred by non-empty reg.
            # - SGMII      : linkup is indicated with bit 15.
            If(~is_sgmii,
                linkdown.eq(self.lp_abi.o == 0),
                self.tx.sgmii_speed.eq(0b10),
                self.rx.sgmii_speed.eq(0b10),
            ).Else(
                linkdown.eq(~self.lp_abi.o[15] | ~sgmii_speed_valid),
                self.tx.sgmii_speed.eq(sgmii_tx_speed),
                self.rx.sgmii_speed.eq(sgmii_rx_speed),
            )
        ]

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
            If(self.rx.seen_config_reg,
                # Consistency Count/Check.
                rx_config_reg_last.eq(self.rx.config_reg),
                If(self.rx.config_reg != rx_config_reg_last,
                    rx_config_reg_count.eq(8 - 1)
                ).Else(
                    If(rx_config_reg_count != 0,
                        rx_config_reg_count.eq(rx_config_reg_count - 1),
                    ).Else(
                        # When RX Config is consistent.
                        # Acknowledgement.
                        If(self.rx.config_reg[14],
                            rx_config_reg_ack.i.eq(1),
                        # Ability match.
                        ).Else(
                            rx_config_reg_abi.i.eq(1),
                        )
                    )
                ),
                self.lp_abi.i.eq(self.rx.config_reg)
            )
        ]

        if with_csr:
            self.add_csr()

    def add_csr(self):
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

        self.sync += [
            self.status.fields.link_up.eq(self.link_up),
            self.status.fields.is_sgmii.eq(self.is_sgmii),
        ]

        self.link_up_timer = link_up_timer = WaitTimer(int(LiteXContext.top.sys_clk_freq))

        self.csr_fsm = fsm = FSM()
        fsm.act("DOWN",
            If(self.link_up,
                NextState("UP")
            )
        )
        fsm.act("UP",
            link_up_timer.wait.eq(1),
            self.ev.link.trigger.eq(link_up_timer.done),
            If(~self.link_up,
                NextState("DOWN"),
            )
        )
