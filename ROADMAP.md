# x6x00-df: what comes next

The owner, 2026-11-17: roll in what KrakenRF's software has and x6x00-df left out, "minus the sharing" (no web
interface, no KrakenSDR apps, mapping services or TAK); use gpsd when it runs; the IQ server is mandatory; sample-rate
slewing too; real-time priority weighed against this radio (perhaps a core of its own). Passive radar is parked.

The radio: an Allwinner R16, four Cortex-A7 cores at 1.008 GHz with NEON, its USB host (musb) in PIO mode, so every
USB byte is copied by a CPU in an interrupt (about 25,000 a second with five sticks streaming). With five channels at
1.024 MS/s, x6x00-df takes about 80 % of one core today. Its Wi-Fi carries a few MB/s at best. Every item below is
measured on the radio before it is called done.

The owner, later the same day: a web page companion for all of it, as the radio's web configurator has a whole
browser for settings, pictures, use in the moment and tuning; and page I has no easy way to tune to the band to DF,
nor to *follow* what a radio is tuned to on another screen.

In the order they are built:

## 1. A core of its own

Done 2026-11-17: readers on CPU2 at SCHED_FIFO 10, the engine on CPU3, the musb interrupt on CPU1 (seen in
`/proc`). Page I ran 31 minutes with five channels without a slip before it, so slips were not the measure.

- The engine thread pinned to one core, the USB reader thread to another, and the musb interrupt to a third
  (`/proc/irq/N/smp_affinity`), so the PIO copies, the readers and the maths never wait on each other. The GUI and
  the rest keep the remaining core and share the others when they are idle.
- The readers at a modest real-time priority (`SCHED_FIFO` 10): a reader that waits too long is what makes a stick
  lose samples. The engine stays a normal thread: if it falls behind, the ring says so (an overrun), nothing is lost
  silently. Never Heimdall's priority 99 with `sched_rt_runtime_us=-1`: on this radio that would starve the GUI,
  which must feed the base every second (x6200-base-keeper's reason).
- Measured: slips per hour with five channels, before and after (the 30-minute runs of page I).

## 2. Tuning page I, and following a radio

Done 2026-11-17 (GoldCorpHF 9a4bc55): F1 TUNE with FOLLOW A or B, the BAND keys on the presets; a typed frequency
came with the web page instead (the radio has no keypad for it).

- A band key on page I itself (today the presets hide in F2 ARRAY's panel), and the frequency typed with a keyboard.
- FOLLOW: OFF, A or B. Following, page I takes the frequency the X6200 (A) or B's SDR is tuned to, as it changes
  (after the dial rests half a second: each retune means a calibration, about 0.5 s), so a signal found on A or B is
  one key from a bearing. The Kraken covers 24 MHz to 1.7 GHz; below that FOLLOW says so and waits.
- In GoldCorpHF's GUI (page I), with the daemon's retune as it is.

## 3. A web page for the Kraken (GoldCorpHF's web configurator)

First version done 2026-11-17 (GoldCorpHF, Remote > Direction finder): every view and setting above but the custom
layout drawn and the bearing log (they come with 5 and 7) and the IQ server's clients (9). Built through the GUI's
configurator socket rather than a second client here: the GUI owns the Kraken and page I's settings, so one set of
settings serves both, and a page polling it four times a second costs nothing worth measuring.

- A Direction finder page in the radio's web configurator: the compass, the spectrum and the bearings' waterfall at
  the browser's size, every channel's level and state, the targets; tuning (a frequency typed, the band presets,
  FOLLOW, the target on the spectrum by a click); every setting (array and its size, a custom layout drawn and
  edited, method, rate, gain, squelch, calibration interval); the bearing log to download; gpsd's position and the
  heading's source; later the IQ server's clients.
- How: x6x00-df takes more than one client on its socket (page I and the web server); the web server relays it to
  the browser (server-sent events). The Kraken is the GUI's to lend, so the page asks the GUI to start direction
  finding (as page I does) when page I is not open, and lets it go when the last viewer leaves.
- Local only, on the radio's web server, as the rest of the configurator.

## 4. gpsd

Done 2026-11-17: tried with a made-up GPS on a pseudo-terminal added to the radio's gpsd (10 knots, course 135):
page I turned north up, its HEADING row `IN USE: GPS COURSE 135 (GPSD)`.

- When gpsd runs (port 2947), x6x00-df reads its TPV reports: the position, and the course over the ground while
  moving faster than about 2 m/s. In motion the course is the array's heading, so a bearing becomes a compass
  bearing without a heading set by hand (a vehicle with the array fixed to it, ANT 0 forward). An ATT report with a
  heading (a compass behind gpsd) is used in preference, moving or not.
- Page I shows where the heading comes from (GPS COURSE, COMPASS, SET BY HAND) and the position.
- Without gpsd, or with no fix, nothing changes.

## 5. A local bearing log

- Each bearing over the squelch (time, frequency, bearing, fit, +-, position when gpsd has one) in a CSV file on the
  DATA partition, one file a day, off unless page I turns it on. Local only: nothing is sent anywhere. (Is this
  "sharing"? It stays on the radio; the owner may say no.)

## 6. More methods

TNA done; several sources (2, 3, AUTO from the eigenvalues' widest gap: MDL counted noise as sources) and
forward-backward averaging done, in the host checks; ROOT-MUSIC and spatial smoothing to come.

- **TNA** (thermal noise algorithm, `1 / a^H R^-2 a`): a step from Capon, which already inverts R.
- **More than one source**: the number of sources from the eigenvalues (MDL), MUSIC's noise subspace of M-K vectors,
  and the K strongest peaks reported as bearings of their own.
- **Forward-backward averaging and spatial smoothing** for a ULA (coherent sources, multipath); a 5-element UCA is not
  centre-symmetric, so for it they stay off.
- **ROOT-MUSIC** for a ULA (the roots of a polynomial instead of a scan: sharper, cheaper); for a UCA only through the
  phase-mode transform, which KrakenRF's version gets wrong, so it is written from the literature and tested here.
- Each in df_test.c on made-up data (two sources, coherent pairs) before the radio.

## 7. Custom arrays

- Any layout of up to five antennas as x and y positions (metres; x forward, y left), from a file on the DATA
  partition (`/mnt/df_array.txt`), drawn and edited on the web page (3), and chosen on page I as CUSTOM. The engine already steers from positions.

## 8. Several targets at once

- Up to four targets (KrakenRF allows 16; the radio's cores allow about four) inside the Kraken's passband, each with
  its own channel filter, squelch and bearing, sharing one calibration. Channel filtering is the cost: each target
  is another pass over five channels, so the targets run on the cores item 1 leaves.
- Page I: the targets as a list, each with its bearing; the compass shows the chosen one, the waterfall all.

## 9. The IQ server, in Heimdall's format

- TCP port 5000, as Heimdall's: 1024-byte IQ headers (`iq_header_struct`, same field order and natural alignment;
  sync word 0x2bf7b95a) then the five channels' samples, complex float 32, calibrated (whole-sample delay, phase and
  gain corrected) and with Heimdall's sync flags set from our own calibration. So KrakenRF's DoA and passive-radar
  software on a computer can take their samples from the radio.
- The Wi-Fi decides the rate: five channels at 1.024 MS/s in float are 41 MB/s, so the server decimates around a
  chosen frequency (the header says the rate): 64 kS/s a channel is 2.6 MB/s. Clients that ask for more get what the
  link carries, never a stall in the engine: frames are dropped for a slow client and counted.
- A control port as Heimdall's (5001) for what its clients send (centre frequency, gain), so they can tune it;
  page I shows when a client holds the Kraken.

## 10. Sample-rate slewing

- What it is: Heimdall lines the sticks up in hardware, running one stick's sample rate a few ppm fast or slow until
  its samples have slid into line (the RTL2832U's resampler ratio, written while it streams). x6x00-df lines them up
  in software instead, and corrects the fraction of a sample per target, which is exact at the target and cheap.
- Why it is wanted now: the IQ server's samples must be lined up across the whole band, not just at one target, and
  a fractional-delay filter on five full-rate channels costs a core. Slewing does it for nothing on the CPU.
- How: a librtlsdr patch (ours, in GoldCorpHF's rtlsdr-blog package) that sets the resampler ratio with a fraction of
  a ppm, and a loop that slews each stick until its fractional delay is under 0.03 sample, as Heimdall's does, run
  after each calibration while the IQ server has a client.

## Parked: passive radar

- KrakenRF's passive radar (`krakensdr_pr`, a fork at github.com/mfkiwl/krakensdr_pr) and the older pyapril, which
  had the clutter cancellation and cross-ambiguity in Python. With the IQ server in Heimdall's format, krakensdr_pr
  on a computer can use the radio's Kraken as it is (one reference channel, four surveillance). On the radio itself
  the cross-ambiguity function is far beyond the cores at any useful size. Not before the owner says what it is for.
