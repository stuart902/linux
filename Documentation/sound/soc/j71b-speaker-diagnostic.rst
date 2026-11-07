.. SPDX-License-Identifier: GPL-2.0-only

====================================
J71b one-shot speaker diagnostic gate
====================================

This experimental variant is for the iPad 6 Wi-Fi (J71b / iPad7,5 / T8010).
It is not production speaker support, a speaker-protection algorithm, or an
instruction to boot or play audio. Device access, each boot and playback test
require separate approval. It must not replace the installed microphone image
while microphone acceptance is in progress.

Source and purpose
==================

The source baseline is ``1b79f24831f78722e6c5fd205bc06646dd920b32`` from the
J71b audio integration branch, itself based on Orchard ``79df400``. Keep that
baseline pinned: this diagnostic does not integrate later power or suspend
changes. The normal source retains four S16_LE channels in eight DSP_A slots,
48 kHz, a 6.144 MHz bitclock and the kernel's sample division by four.
Neither this division nor a low desktop-volume setting protects a speaker
from thermal, excursion, DC or framing faults.

Upstream Orchard ``f96e0bb`` reports clean tones using a temporary 30 ms
gated test build. The gate is not in the retained production source. This
variant implements its own independently reviewable software gate rather
than claiming to reproduce that unpublished test implementation.

Admission and deadline
======================

``CONFIG_SND_SOC_MAX98357A_J71B_TEST`` defaults off. The two diagnostic platform
configurations select it and use the unique localversion
``-j71b-speaker-window-nov7-r1``. This suffix identifies an experimental build;
it is not evidence that its configuration, modules or device tree were checked.

When selected, on J71b only:

* The codec already owning SD_MODE remains its only GPIO owner. Probe requires
  the TDM property, a present non-sleeping GPIO and zero SD_MODE delay.
  Missing prerequisites fail probe; they do not fall back to ordinary playback.
* The load-time ``j71b_test_arm`` boolean defaults false and is read-only after
  load. Ordinary automatic module loading leaves the amplifier muted. The test
  package must not automatically set this parameter or start playback.
* An admitted ALSA START requires the DAPM speaker switch and consumes a single
  module-wide allowance. A monotonic hard-mode hrtimer is armed before the GPIO
  is raised, with a target expiry 30 ms later. An already elapsed deadline is
  rejected without raising the GPIO.
* The timer lowers SD_MODE. It does not stop DMA, free buffers, change MCA frame
  clocks or claim that the hardware observed the GPIO transition.
* STOP, SUSPEND, PAUSE_PUSH and DAPM power-down mute immediately. RESUME,
  PAUSE_RELEASE, unknown commands and repeat START reject and mute. No stream
  reopen, DAPM cycle or driver rebind rearms the module-wide allowance.
* Device shutdown and devres cleanup terminally mute, then synchronously drain
  the timer before GPIO/private-data release. The timer never takes an ALSA
  stream lock or stores a substream pointer.

A module reload or reboot creates a new lifetime. Neither is an automatic
recovery or retry procedure, and neither is authorized by a previous tone test.
After a fault or uncertain DMA ownership, preserve the session and logs;
do not repeat playback, unload/unbind drivers or assume a userspace timeout
made shutdown safe.

This is a software deadline, not a guaranteed physical 30 ms window. Interrupt
latency, a stalled kernel, GPIO electrical behavior and amplifier timing remain
unqualified. There is no independent hardware cutoff. Keep the existing MCA
ordering: mute the amplifier before frame/bitclock teardown. Do not stop LRCLK
while leaving BCLK and the amplifier active as a substitute for this gate.

Offline validation and build boundary
=====================================

Run the standalone host test without ALSA devices or alsa-lib::

  python3 tools/testing/selftests/alsa/test_max98357a_j71b.py

It compiles actual codec callbacks with mocked GPIO, timer, devres and locking
primitives, both with the diagnostic option on and off, using strict warnings
and ASan/UBSan. It checks admission, consumption, mute/deadline ordering,
late-start rejection, prerequisite/action-registration failures, DAPM/trigger
transitions, teardown and other-board behavior. Full platform-probe unwind and
shutdown are checked only by source wiring assertions. Pthread cases exercise selected callback serializations,
not real kernel IRQ, hrtimer or driver-core concurrency. These tests cannot
establish actual timer latency, physical isolation or acoustic safety.

Before any device test, the separate build must verify the exact source tree,
resolved 4K/16K configuration, kernel release, module architecture/vermagic and
module dependencies, compiled J71b DTB, gate presence and default-unarmed
packaging. Source tests alone are insufficient. Keep the unchanged microphone
source and source receipts. Reuse boot tools only with the existing exact-source
and byte-hash checks. Do not mix modules or DTBs from the speaker-disabled image
with this uniquely versioned candidate.

First use must be a separately approved diagnostic session without automatic
desktop audio. Verify the gate remains closed before considering a low-level
one-slot fixture. The four-slot physical mapping, waveform level, stop behavior,
clock/amp state and concurrent touch remain device qualification questions.
Longer playback, PipeWire, simultaneous capture and suspend/resume are later
acceptance steps, not implied by a passing short tone.

The older reused desktop root may lack the TDM UCM ``PlaybackChannels 4`` line.
Audit its actual content before desktop qualification and include a separately
verified UCM delta if required. A full-root reinstall is not a prerequisite for
preparing this diagnostic, and no image installation is part of the host test.
