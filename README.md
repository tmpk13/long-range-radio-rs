# Rust long range radio mesh

LoRa mesh firmware for the Seeed Wio-E5 module (STM32WLE5JC with its built-in
SX1262), with a two-stage bootloader for over-the-air updates and a
basestation node that bridges the mesh to a host PC over UART. Build-time
settings (`ADDRESS`, `LORA_PRESET`, `FW_VERSION`, `RX_BOOST`) are environment
variables documented in `src/config.rs`; optional hardware sits behind the
cargo features in `Cargo.toml`.

- `mise run bootloader`: flash the bootloader, once per board (no RTT output; Ctrl-C once flashed)
- `ADDRESS=2 mise run flash`: build, flash and attach RTT to node 2; append `--features debug` for verbose logging
- `mise run attach`: attach RTT to a running node
- `ADDRESS=10 mise run basestation`: build and flash the basestation node
- `mise run ota-upload -p /dev/ttyUSB0 -t 2 -v 3 firmware.bin`: upload firmware to node 2 as version 3 through the basestation
- `mise run data-relay -p /dev/ttyACM0`: relay mesh data from the basestation as JSON lines
- `mise run build`: build the app, bootloader and basestation without flashing

`mise tasks` lists the rest. Design notes: NOTES.md. Components: ARCHITECTURE.md.
Hardware reference: docs/hardware.md.

*Claude Code was utilized in the development process.*
