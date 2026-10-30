# GateMate 1000BASE-X PHY (LiteEth)

## Overview

This repository contains a LiteX design used to test the LiteEth 1000BASE-X PHY implemented with the GateMate `CC_SERDES`.

The PHY uses:
- Hardware 8b/10b encoding and decoding.
- A 32-bit datapath at 31.25 MHz.
- No gearbox in the FPGA fabric.

The original implementation was merged into LiteEth and successfully tested against a LiteX Acorn link partner.

**The main remaining issue is unreliable hardware operation across place-and-route runs, despite apparently satisfactory timing reports.**

The current suspicion is that nextpnr does not correctly model the timing constraints of the `CC_SERDES` interface.

## Current status

### Working

- Basic PHY operation demonstrated on real hardware.
- Link establishment with a LiteX Acorn link partner.
- 8b/10b encoding and decoding through the hardware SERDES.
- Comma detection and RX word alignment.
- Link monitoring and recovery.
- Simulation coverage for loopback, alignment, error injection and link-partner restart.
- TX underflow handling has been fixed.

### Not yet reliable

- Hardware operation depends on place-and-route results.
- RX corruption and packet loss can occur even when nextpnr reports an estimated Fmax of 40–60 MHz, with a 31.25 MHz operating frequency.
- Failures can occur with identical placement seeds.
- Registers buffering the SERDES datapath have been observed surprisingly far from the SERDES pins.
- A reliable relationship between reported timing, placement and actual hardware behaviour has not yet been established.

An additional, potentially independent interoperability issue was observed with the Intel 82599ES: portions of the RX datapath appeared to remain zero. The cause is still unknown; SERDES calibration and configuration are among the hypotheses.

## Main investigation: nextpnr SERDES timing

[**nextpnr issue #1882**](https://github.com/YosysHQ/nextpnr/issues/1882)

The current leading hypothesis is that nextpnr does not properly account for timing between the `CC_SERDES` and the FPGA fabric.

In `himbaechel/uarch/gatemate/delay.cc`, SERDES ports appear to be classified as `TMG_IGNORE`.

If these paths are ignored, the placer has no meaningful timing incentive to keep the interface registers close to the SERDES.

This could explain why:
- Timing reports look healthy.
- SERDES interface registers can be placed far from the SERDES.
- Generated bitstreams can exhibit RX corruption despite meeting the reported timing target.

This is still a hypothesis, not a confirmed root cause. Routing congestion may also contribute, but it has not been established as the explanation.

### Experimental workaround

[**SERDES delay-estimation hack**](https://gist.github.com/Mimil25/0ddd79c3d413124943bef92825a826da)

An experimental nextpnr patch:
- Classifies SERDES datapath ports as registered timing endpoints/startpoints.
- Adds a large placement/routing distance penalty for signals originating in the SERDES region (X=19–26, Y=123–127).

The experiment appears promising, but it should only be considered as a proof that the issue lies in the `CC_SERDES` timing constraints.

A proper solution should model the actual `CC_SERDES` timing requirements and clock relationships.

## Hardware testing

The accompanying LiteX design provides a starting point for reproducing the issue.

### Reference platform

- GateMate EVB (CCGM1A1).
- VexRiscv.
- HyperRAM.
- LiteX Ethernet MAC.
- 32-bit Ethernet datapath at 31.25 MHz.
- 24 MHz system clock.
- Store-and-forward enabled.

### Testing procedure

1. Build and program the GateMate board.
```
./colognechip_gatemate_evb.py --build --flash
```
2. Connect a LiteX Acorn as the reference link partner.
3. Verify link establishment.
4. Test bidirectional ping and sustained `iperf` traffic.
5. Generate bitstreams using multiple placement seeds.
6. Record hardware reliability alongside nextpnr timing reports.

A successful link-up or satisfactory Fmax alone is not sufficient: sustained traffic without packet loss or data corruption is the objective.


## Development priorities

1. **Investigate SERDES timing modelling:** understand and properly implement `getPortTimingClass()`, `getPortClockingInfo()` and the relevant delay estimation functions.
2. **Find a reliable placement solution:** investigate explicit placement constraints or a proper SERDES timing model instead of arbitrary distance penalties.
3. **Establish reproducible hardware tests:** correlate placement seeds, timing reports and actual packet integrity.
4. **Investigate Intel 82599ES interoperability:** treat RX datapath corruption on this link partner as a separate issue.

## References

- [LiteEth PR #292 — Original PHY implementation](https://github.com/enjoy-digital/liteeth/pull/292)
- [LiteEth PR #294 — TX underflow robustness](https://github.com/enjoy-digital/liteeth/pull/294)
- [LiteEth PR #298 — Placement-related experiments](https://github.com/enjoy-digital/liteeth/pull/298)
- [LiteEth PR #299 — MAC timing improvements](https://github.com/enjoy-digital/liteeth/pull/299)
- [LiteEth PR #300 — PCS RX timing improvements](https://github.com/enjoy-digital/liteeth/pull/300)
- [nextpnr issue #1882 — SERDES timing investigation](https://github.com/YosysHQ/nextpnr/issues/1882)
- [(very) Experimental nextpnr patch](https://gist.github.com/Mimil25/0ddd79c3d413124943bef92825a826da)

## Goal

Generate reliable bitstreams that operate at 31.25 MHz across different placement seeds, with no RX corruption or packet loss.

The priority is to understand and fix the interaction between the GateMate `CC_SERDES` and nextpnr's timing-driven placement and routing.
