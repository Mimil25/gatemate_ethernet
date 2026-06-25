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
SGMII_100MBPS_SPEED  = 0b01 # unused
SGMII_10MBPS_SPEED   = 0b00 # unused


# PCS TX -------------------------------------------------------------------------------------------

class PCSTX32(LiteXModule):
    def __init__(self):
        self.config_valid = Signal()                               # Config valid.
        self.config_reg   = Signal(16)                             # Config register (16-bit).
        self.sink         = sink = stream.Endpoint([("data", 32)])  # Data input.

        # PHY ports output
        self.char_is_k = Signal(4) # is K output
        self.data = Signal(32) # Data output

        # Signals.
        # --------
        count  = Signal() # Byte counter for config register.
        parity = Signal() # Parity for /R/ extension.
        ctype  = Signal() # Toggles config type.

        # FSM.
        # ----
        self.fsm = fsm = FSM()
        fsm.act("START",
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
                    sink.ready.eq(1),
                    NextValue(self.data, Cat(
                        K(27, 7), # Start-of-packet /S/.
                        sink.data[8:32]
                    )),
                    NextState("DATA")
                ).Else(
                    NextValue(self.data, Cat( # IDLE TODO handle disparity
                        K(28, 5), # D(5, 6), 
                        D(16, 2), 
                        D(5, 6), 
                        D(16, 2), 
                    )),
                    # NextState("IDLE")
                )
            )
        )
        fsm.act("DATA",
            # Send Data.
            sink.ready.eq(1),
            If(sink.valid,
                NextValue(self.char_is_k, 0b0000),
                NextValue(self.data, sink.data),
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
        self.is_idle         = Signal()
        self.source          = source = stream.Endpoint([("data", 32), ("error", 1)]) # Data output.

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
        self.sync += [
            data.eq(self.data),
            char_is_k.eq(self.char_is_k),
            disparity_err.eq(self.disparity_err),
            table_err.eq(self.table_err),
        ]
        
        # Invalid char counter.
        # ---------------------
        self.comb += invalid.eq(table_err | disparity_err)
        self.sync += [
            If((invalid | eth_invalid) & (self.error_count < 0xfe),
                self.error_count.eq(self.error_count + 1),
                self.error_data.eq(data),
                self.error_char_is_k.eq(char_is_k),
            ),
            If((self.error_count == 0xff) & self.seen_valid_ci, # start error count once rx is aligned
                self.error_count.eq(0),
            ),
        ]

        # Buffer.
        # -------
        self.buffer = buffer = stream.Buffer([("data", 32)], pipe_valid=True, pipe_ready=False)
        self.comb += [
            buffer.source.connect(source, omit={"last", "error"}),
            source.last.eq(buffer.source.valid & ~buffer.sink.valid), # Last when next is not valid.
        ]

        # FSM.
        # ----
        self.fsm = fsm = FSM()
        fsm.act("START",
            NextValue(buffer.sink.valid, 0),
            eth_invalid.eq(1),
            # Wait for a K-character.
            If(char_is_k[0] == 1,
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
                        self.is_idle.eq(1),
                        NextState("START"),
                    ),
                ),
                # K-character is Start-of-packet /S/.
                If(data[0:8] == K(27, 7),
                    eth_invalid.eq(0),
                    NextValue(buffer.sink.valid, 1),
                    NextValue(buffer.sink.data, Cat(
                        0x55, # First Preamble Byte.
                        data[8:32]
                    )),
                    NextState("DATA")
                )
            )
        )
        fsm.act("DATA",
            self.seen_valid_ci.eq(1),
            eth_invalid.eq(1),
            If((char_is_k == 0) & ~invalid,
                # Receive Data.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.data, data),
            ),
            If((char_is_k == 1) & (data[0:8] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 0),
                NextState("START"),
                eth_invalid.eq(1),
            ),
            If((char_is_k == 0b0010) & (data[8:16] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.data, data),
                NextState("START"),
            ),
            If((char_is_k == 0b0100) & (data[16:24] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.data, data),
                NextState("START"),
            ),
            If((char_is_k == 0b1000) & (data[24:32] == K(29, 7)) & ~invalid,
                # K-character is End-of-packet /S/.
                eth_invalid.eq(0),
                NextValue(buffer.sink.valid, 1),
                NextValue(buffer.sink.data, data),
                NextState("START"),
            ), #.Else(
            If(invalid | eth_invalid,
                NextValue(buffer.sink.valid, 0),
                source.error.eq(1),
                source.last.eq(1),
                source.valid.eq(1),
                If(source.ready,
                   NextState("START"),
                )
            )
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

        self.align   = Signal(reset=0)
        self.link_up = Signal()
        self.restart = Signal()

        self.lp_abi = BusSynchronizer(16, "eth_rx", "eth_tx")

        # # #

        # Signals.
        # --------
        self.linkdown     = linkdown     = Signal()
        self.autoneg_ack  = autoneg_ack  = Signal()

        seen_ack = Signal()
        seen_abi = Signal()

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
        self.rx_is_idle        = rx_is_idle        = PulseSynchronizer("eth_rx", "eth_tx")
        
        self.comb += [
            seen_valid_ci.i.eq(self.rx.seen_valid_ci),
            rx_is_idle.i.eq(self.rx.is_idle),
        ]

        # Timers.
        # -------
        self.breaklink_timer = breaklink_timer = ClockDomainsRenamer("eth_tx")(WaitTimer(breaklink_time * eth_tx_clk_freq))
        self.more_ack_timer  = more_ack_timer  = ClockDomainsRenamer("eth_tx")(WaitTimer(more_ack_time  * eth_tx_clk_freq))

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
            If(seen_valid_ci.o,
                checker_error.eq(0),
                checker_count.eq(checker_max)
            ),
            If(checker_tick,    checker_error.eq(1)),
            

            If(rx_config_reg_abi.o,
                seen_abi.eq(1),
            ),
            If(rx_config_reg_ack.o,
                seen_ack.eq(1),
            ),
        ]

        # Linkdown Detection.
        # -------------------------
        self.comb += [
            # Detect that link is down:
            # - 1000BASE-X : linkup can be inferred by non-empty reg.
            linkdown.eq(self.lp_abi.o == 0),
        ]

        # TX Config.
        # ----------
        self.comb += [
            self.tx.config_reg[0:5].eq(0),                  # reserved
            self.tx.config_reg[5].eq(1),                    # 1000BASE-X: Full-duplex.
            self.tx.config_reg[6].eq(0),                    # 1000BASE-X: Half-duplex.
            self.tx.config_reg[7:9].eq(0),                  # no pause capability
            self.tx.config_reg[9:12].eq(0),                 # reserved
            self.tx.config_reg[12:14].eq(0),                # TODO remote fault encoding 0: OK, 1: offline, 2: link failure, 3: auto-negotiation error
            self.tx.config_reg[14].eq(autoneg_ack),         # acknowledge
            self.tx.config_reg[15].eq(0),                   # next page req
        ]

        # FSM.
        # ----
        self.fsm = fsm = ClockDomainsRenamer("eth_tx")(FSM())
        self.fsm_state = Signal(3)
        # AN_ENABLE.
        fsm.act("AUTONEG-BREAKLINK",
            self.fsm_state.eq(1),
            self.tx.config_reg.eq(0),
            self.tx.config_valid.eq(1),
            breaklink_timer.wait.eq(1),
            self.align.eq(1),
            If(breaklink_timer.done,
                NextValue(autoneg_ack, 0),
                NextState("AUTONEG-WAIT-ABI")
            )
        )
        # ABILITY_DETECT.
        fsm.act("AUTONEG-WAIT-ABI",
            self.fsm_state.eq(2),
            self.tx.config_valid.eq(1),
            self.align.eq(1),
            If(seen_abi, #rx_config_reg_abi.o, # Got matching abi reg
                NextState("AUTONEG-SEND-MORE-ABI"),
            ),
            If(rx_is_idle.o,
                NextState("RUNNING-NO-AUTONEG"),
            ),
            If(checker_error,
                NextValue(self.restart, 1),
                NextState("AUTONEG-BREAKLINK")
            )
        )
        fsm.act("AUTONEG-SEND-MORE-ABI",
            self.fsm_state.eq(6),
            self.tx.config_valid.eq(1),
            more_ack_timer.wait.eq(1),
            If(more_ack_timer.done,
                NextValue(autoneg_ack, 1),
                NextState("AUTONEG-WAIT-ACK")
            ),
            If(checker_error,
                NextValue(self.restart, 1),
                NextState("AUTONEG-BREAKLINK")
            )
        )
        # ACKNOWLEDGE_DETECT.
        fsm.act("AUTONEG-WAIT-ACK",
            self.fsm_state.eq(3),
            self.tx.config_valid.eq(1),
            If(seen_ack, #rx_config_reg_ack.o,
                NextState("AUTONEG-SEND-MORE-ACK")
            ),
            If(checker_error,
                NextValue(self.restart, 1),
                NextState("AUTONEG-BREAKLINK")
            )
        )
        # COMPLETE_ACKNOWLEDGE.
        fsm.act("AUTONEG-SEND-MORE-ACK",
            self.fsm_state.eq(4),
            self.tx.config_valid.eq(1),
            more_ack_timer.wait.eq(1),
            If(more_ack_timer.done,
                NextState("RUNNING")
            ),
            If(checker_error,
                NextValue(self.restart, 1),
                NextState("AUTONEG-BREAKLINK")
            )
        )
        # LINK_OK.
        fsm.act("RUNNING",
            self.fsm_state.eq(5),
            self.link_up.eq(~linkdown),
            If(checker_error | linkdown,
                NextValue(self.restart, 1),
                NextState("AUTONEG-BREAKLINK")
            )
        )
        # LINK_OK - NO AUTONEG.
        fsm.act("RUNNING-NO-AUTONEG",
            self.fsm_state.eq(7),
            self.link_up.eq(1),
            If(checker_error,
                NextValue(self.restart, 1),
                NextState("AUTONEG-BREAKLINK")
            )
        )

        # RX Config (and consistency check).
        # ----------------------------------
        self.rxrc = rx_config_reg_count  = Signal(4, reset=15)
        rx_config_reg_last   = Signal(16)
        self.sync.eth_rx += [
            rx_config_reg_abi.i.eq(0),
            rx_config_reg_ack.i.eq(0),
            If(self.rx.seen_config_reg,
                # Consistency Count/Check.
                rx_config_reg_last.eq(self.rx.config_reg),
                #If(self.rx.config_reg != rx_config_reg_last,
                If((self.rx.config_reg[0:14] != rx_config_reg_last[0:14]) | (self.rx.config_reg[15] != rx_config_reg_last[15]),
                    rx_config_reg_count.eq(8 - 1)
                ).Else(
                    If(rx_config_reg_count != 0,
                        rx_config_reg_count.eq(rx_config_reg_count - 1),
                    ).Else(
                        # When RX Config is consistent.
                        # Acknowledgement.
                        If(self.rx.config_reg[14],
                            rx_config_reg_ack.i.eq(1),
                        ),
                        # Ability match.
                        rx_config_reg_abi.i.eq(1),
                        self.lp_abi.i.eq(self.rx.config_reg),
                    )
                ),
            ),
        ]

        if with_csr:
            self.add_csr()

    def add_csr(self):
        self.status = CSRStatus(fields=[
            CSRField("link_up",    size=1,  offset=0,  description="Link is up."),
            CSRField("align",      size=1,  offset=1,  description="align"),
            CSRField("restart",      size=1,  offset=2,  description="restart"),
            CSRField("rx_idle",      size=1,  offset=3),
            
            CSRField("fsm_state", size=3,  offset=4),
            CSRField("rx_err", size=8,  offset=8),
            
            CSRField("config_reg", size=16, offset=16, description="Link partner ability register."),
            CSRField("tx_config_reg", size=16, offset=32, description="ability register."),
            
            CSRField("rx_err_charisk", size=4, offset=48),
            CSRField("rx_err_data", size=32, offset=64),
        ])

        self.lp_abi_csr = BusSynchronizer(16, "eth_rx", "sys")

        self.comb += [
            self.lp_abi_csr.i.eq(self.lp_abi.i),
            self.status.fields.config_reg.eq(self.lp_abi_csr.o),
        ]

        self.sync += [
            self.status.fields.link_up.eq(self.link_up),
            self.status.fields.align.eq(self.align),
            self.status.fields.fsm_state.eq(self.fsm_state),
            self.status.fields.rx_err.eq(self.rx.error_count),
            self.status.fields.rx_err_data.eq(self.rx.error_data),
            self.status.fields.rx_err_charisk.eq(self.rx.error_char_is_k),
            self.status.fields.rx_idle.eq(self.rx.is_idle),
            self.status.fields.tx_config_reg.eq(self.tx.config_reg),
        ]
