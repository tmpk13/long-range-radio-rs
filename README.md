# Rust long range radio mesh

LoRa mesh firmware for the Seeed Wio-E5 module (STM32WLE5JC with its built-in
SX1262), with a two-stage bootloader for over-the-air updates and a
basestation node that bridges the mesh to a host PC over UART. Build-time
settings (`ADDRESS`, `LORA_PRESET`, `FW_VERSION`, `RX_BOOST`) are environment
variables documented in `firmware/src/config.rs`; optional hardware sits behind the
cargo features in `firmware/Cargo.toml`, with `board` for the Forest Sensor Platform
V1. That board's CH572D runs the USB bridge firmware in
[ch572-bridge-rs](https://github.com/tmpk13/ch572-bridge-rs), which turns its USB-C port
into a serial port and a CMSIS-DAP probe for the Wio-E5.

- `mise run bootloader`: flash the bootloader, once per board (no RTT output; Ctrl-C once flashed)
- `ADDRESS=2 mise run flash`: build, flash and attach RTT to node 2; append `--features debug` for verbose logging, `--features board` on the Forest Sensor Platform
- `mise run attach`: attach RTT to a running node
- `ADDRESS=10 mise run basestation`: build and flash the basestation node (`--features board` puts its host link on the board's USB-C)
- `ADDRESS=2 FW_VERSION=3 mise run ota-image`: build `target/firmware.bin` for node 2 as version 3 (pass the node's `--features`)
- `mise run ota-upload -p /dev/ttyUSB0 -t 2 -v 3 target/firmware.bin`: upload that image to node 2 through the basestation
- `mise run data-relay -p /dev/ttyACM0`: relay mesh data from the basestation as JSON lines
- `mise run build`: build the app, bootloader and basestation without flashing

`mise tasks` lists the rest. Testing a new board, from first power on: docs/v1-board-test.md.
Design notes: NOTES.md. Components: ARCHITECTURE.md.
Hardware reference: docs/hardware.md.

*Claude Code was utilized in the development process.*
