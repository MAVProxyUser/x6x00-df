# x6x00-df

Direction finding with a [KrakenSDR](https://www.krakenrf.com/) on a Xiegu X6200, for the
[GoldCorpHF](https://github.com/MAVProxyUser/X6200-claude-assist-UI) firmware's page I (the Sentry's direction
finder). A small C daemon of our own in place of KrakenRF's Heimdall DAQ and krakensdr_doa: it opens the Kraken's
five RTL-SDRs, calibrates them with the Kraken's noise source, and sends page I a bearing ten times a second.

Receive only: nothing here transmits.

## What it does

- Opens the Kraken's sticks by serial (1000-1004, CH0-CH4), one at a time, with equal manual gain, every AGC off
  and the R820T's PLL dither off (a dithered PLL sweeps the bearing round the circle every few seconds).
- Calibrates on the Kraken's own noise source: each channel's delay (whole samples, then the rest), phase and gain,
  and each tuner's filter ripple per frequency. About 0.15 s, at the start, after a retune or a gain change, on a
  slip, and every two minutes by default.
- Takes a bearing on every 0.1 s block with MUSIC, Bartlett, Capon, MEM, TNA or ROOT-MUSIC (a straight array's
  bearings from a polynomial's roots, off any grid), for a circular (UCA-3/4/5), straight (ULA-3/4/5) or any other
  layout of up to five antennas, through a channel filter on the target, above a squelch; one source, two or three
  (their bearings together), or as many as the eigenvalues show (AUTO); forward-backward averaging and spatial
  smoothing on a ULA for coherent sources such as multipath.
- With gpsd running: a compass's heading, or the course over the ground while moving, turns bearings into compass
  bearings (`df_gps.c`), and the position goes with them.
- On the radio: the sticks' readers on a core of their own at a modest real-time priority, the engine on another, and
  the USB host's interrupt moved off CPU0 (`-C`).
- Says how far to trust it: a fit (0-100 %), a +- wedge, the other peaks; holds the last good bearing for 30 s when
  the signal stops; leaves out a channel 20 dB under the rest (a dead antenna or cable); catches a sample slip and
  calibrates again.
- Talks to one client, the GUI's page I, on the abstract socket `@x6x00-df` (`SOCK_SEQPACKET`): binary messages out
  (`df_proto.h`: DFST state, DFDO bearing, DFSP spectrum, DFGP gpsd, DFTG the other targets), short text
  commands in.
- Keeps a bearing log (a CSV file a day) and serves the target's IQ in Heimdall's format (ports 5000 and 5001) to a
  program on the network.

- Up to three more targets at once in the passband, each with its own filter, squelch and bearing, on the cores the
  engine leaves (DFTG).

Coming, in ROADMAP.md's order: sample-rate slewing. Not planned: output to the KrakenSDR
apps, mapping services or TAK.

## Files

| File | What |
|---|---|
| `df.c` | the daemon: the sticks, their readers and ring buffers, calibration timing, the socket |
| `df.h` | the engine's interface (no hardware, so it builds and runs on any host) |
| `df_engine.c` | one block in, one estimate out: channel filter, covariance, the bearing, levels, slips |
| `df_cal.c` | FFT and calibration on the noise source |
| `df_chan.c` | the channel filter |
| `df_doa.c` | arrays, steering vectors, the six methods (ROOT-MUSIC's roots by Aberth's iteration), counting sources, forward-backward averaging, spatial smoothing |
| `df_proto.h` | the protocol between the daemon and page I |
| `df_gps.c` | gpsd's reports read: the fix, the course, a compass, and which heading the bearings take |
| `df_sources` | the daemon's sources, one a line (what GoldCorpHF's builds compile) |
| `df_test.c` | the engine on made-up Kraken data: calibration, bearings for each array, slips (`make test`) |
| `df_bench.c` | what the engine costs, five channels at a rate (`make df_bench`) |
| `df_rate.c` | a bench check on the radio: samples per second each stick really delivers |
| `df_slip_test.sh` | from a computer: how often the radio's USB loses a sample with all five streaming |

## Building

On a host, the engine's checks:

    make test

The daemon is built for the radio by GoldCorpHF, which has this repository as its submodule
`tools/rtlspec/x6x00-df`: `tools/rtlsdr/build.sh` for a hot install, the image's `x6x00-rtlspec` buildroot package
for the firmware. It needs the RTL-SDR Blog librtlsdr with `rtlsdr_set_dithering` (GoldCorpHF's
`buildroot/br2_external/package/rtlsdr-blog/0002-the-r820t-pll-dither-switch.patch`). On a Linux host with those,
`make x6x00-df`. `x6x00-df -T 45` runs it with a made-up Kraken (a carrier at 45 degrees), for page I without one.

## On the X6200

The radio's USB host has five receive endpoints, and the Kraken's hub kept one for its status, so only four sticks
streamed. GoldCorpHF's kernel patch 9104 has the host ask that hub about its ports over its control endpoint
instead; with it all five stream whole (B's at 2.1168 MS/s and four at 1.024 MS/s, 20 minutes, every second full).

Tried over the air (2026-11-17, a 915 MHz ISM carrier, all five channels): the bearing held within about 1 degree
across bursts at fits of 73-83 %. The absolute bearing needs the array's real layout.

The plan, the measurements and the decisions:
[docs/plans/kraken_df_plan.md](https://github.com/MAVProxyUser/X6200-claude-assist-UI/blob/main/docs/plans/kraken_df_plan.md)
in GoldCorpHF.

## Licence

LGPL-2.1-or-later (`LICENSE`). Written in C of our own after reading what KrakenRF's Heimdall DAQ and krakensdr_doa
do (the plan's section 1, with file and line references); no code of theirs is in it.
