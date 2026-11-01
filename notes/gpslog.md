# GPS + radio link logger

## GPS + radio link logger (gps-radio-log feature)

GPS goes on USART1 (PB7 RX at 9600 8N1, AF7) - the only free UART:
USART2 is the RS-422 bus and LPUART1's PC0/PC1 pins are the activity
LEDs. RX-only; PB6 (USART1 TX) is left free until a module needs
configuring. The GPS is kept powered and active permanently (power
budget accepted for this feature), so no PMTK/UBX standby commands.
The feature implies `board` (SD card) and `nb` (UART reads).

NMEA handling: assemble lines, verify the XOR checksum, keep only
RMC/GGA (any talker prefix) and log the raw sentence - raw NMEA
already carries every metric (UTC, lat/lon, fix, sats, HDOP, alt,
speed, course), so the SD log stores sentences verbatim. GGA yields
fix-quality/sats for an RTT status line; GGA and RMC are also parsed
for position (see below). At 9600 baud (~1 char/ms) the 1 ms main
loop keeps up; the blocking sensor sweep (~0.5 s) causes UART
overruns, handled by clearing ICR flags and resyncing on the next '$'.

Display readout: in this build the SSD1306 is dedicated to a lat/long
readout (the TX/RX message banners are cfg'd out; OTA progress still
shows). GGA (quality>0) and RMC (status 'A') coordinates are parsed
to signed decimal degrees - NMEA ddmm.mmmm / dddmm.mmmm split as
deg = all-but-last-two integer digits, min = the rest, deg + min/60,
negated for S/W. f32 throughout (matches the sensor suite; core's
float Display handles the "{:.5}" format, no libm). A quality-0 GGA
or status-'V' RMC clears the fix so the display falls back to
"Acquiring NN". Refreshed at 1 Hz (the fix rate) to avoid flicker;
`Gps::has_pos`/`lat_deg`/`lon_deg` plus `fmt_status`/`fmt_lat`/
`fmt_lon` are the interface. Lines fit the 12-char FONT_10X20 width
(worst case "145.12267 W" = 11 chars).

Radio metrics: poll_recv already read LoRaPacketStatus for RSSI; the
driver now also queues SNR (snr_pkt() numer() = raw quarter-dB) and
signal_rssi_pkt (post-despread RSSI) per RX, TX length/result per TX,
into a small critical-section ring (gpslog::events) drained by the
main loop - same pattern as board::activity, but with payloads. A
STATS line (pkt_rx/pkt_crc/pkt_hdr_err from Get_Stats) logs every
60 s; that is everything the SX126x exposes.

Each drained event is also echoed to RTT (the SD-log line, minus the
trailing newline) so RSSI/SNR are visible live without pulling the
card. Independently, the main-loop receive path prints RSSI on its
always-on "RX #n ... rssi=<dBm>" line (io.last_rssi()) rather than
only under the `debug` feature, so RSSI shows in RTT in every build.

SD format: no filesystem. Header block at LBA 2048 (1 MiB in, clear
of any partition metadata) holds magic "GRL1" + next-LBA; data blocks
follow (1 GiB region, wraps). Blocks are zero-padded ASCII lines
("t=<ms> GPS/RX/TX/STATS ...") so recovery is dd+strings. Header is
checkpointed every 16 blocks (not every block) to cut wear; resume
skips 16+1 blocks past the stored pointer so unrecorded blocks and a
partial tail survive reboots. Partial blocks are synced in place
every 10 s, bounding loss on power failure. SD errors drop the logger
to not-ready and a 10 s retry path re-inits the card, so a card
inserted after boot (or a transient SPI error) recovers.

## GPS-log status line: sats + last RX RSSI + time-since-RX

The OLED top line in gps-radio-log now reads e.g. "08 -95dBm 1m23s"
(sat count, last packet RSSI, elapsed since last RX), replacing the old
"Fix 08 sat" text. Choices:
- The RX timestamp is stamped at the physical layer in io.rs (note_rx,
  alongside last_rssi via platform::millis()), not at mesh.receive().
  This keeps "time since last RX" consistent with last_rssi - both refer
  to the same heard packet, including ones the mesh doesn't surface as
  app messages. Exposed as LoraIo::last_rx_ms() -> Option<u32> (None
  before any RX). Elapsed uses wrapping_sub to ride the DWT millis wrap.
- Three fields don't fit 12 chars of FONT_10X20, so the status line uses
  FONT_7X13 (~18 chars); lat/long below stay at FONT_10X20. Fix state is
  now implied by lat/long appearing rather than the "Fix"/"Acquiring"
  word.
