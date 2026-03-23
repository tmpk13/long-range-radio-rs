# Rust long range radio mesh

For the RP2040 (Raspberry Pi Pico) with an SX1262 LoRa module, using RTIC.

## Wiring

| RP2040 Pin | SX1262 Function |
|------------|-----------------|
| GP2        | SCK (SPI0)      |
| GP3        | COTI (SPI0)     |
| GP4        | CITO (SPI0)     |
| GP5        | NSS             |
| GP6        | RST             |
| GP7        | BUSY            |
| GP8        | RF_SW / ANT     |
| GP9        | DIO1            |

## Run

Requires a debug probe (e.g. Pi Debug Probe, another Pico running picoprobe) connected via SWD.

```
ADDRESS=1 cargo run --release
ADDRESS=2 cargo run --release
ADDRESS=... cargo run --release

runner = "elf2uf2-rs -d"
```

Claude Code was utilized in the development process.
