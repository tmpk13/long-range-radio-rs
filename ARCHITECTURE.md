# Architecture

## System

Every Wio-E5 carries the bootloader in its first 16 KB and one app: the
mesh node firmware, or on one of them the basestation, which links the
mesh to a host PC. Host and basestation exchange framed commands over a
UART (`basestation/src/uart_proto.rs` and
`basestation/host/basestation_proto.py` implement the same frame format).
That UART is USART1 through a USB-UART adapter, or with the `board`
feature USART2 to the Forest Sensor Platform's CH572D, which bridges it to
USB-C and also serves as the SWD probe.

```mermaid
flowchart LR
    subgraph host["Host PC"]
        upload["ota_upload.py"]
        relay["data_relay.py"]
        probe["probe-rs, OpenOCD"]
    end
    subgraph base["Basestation Wio-E5"]
        bsapp["basestation app"]
    end
    subgraph meshnode["Mesh node Wio-E5"]
        nodeapp["node firmware app"]
        boot["bootloader"]
    end
    upload -->|"UART frames"| bsapp
    relay <-->|"UART frames"| bsapp
    bsapp <-->|"LoRa mesh"| nodeapp
    nodeapp -->|"staged image, boot state"| boot
    probe -->|"SWD"| meshnode
```

The partition map that the bootloader and the apps share is defined once
in [flash-layout/src/lib.rs](flash-layout/src/lib.rs), and each crate's
build script generates its linker `memory.x` from it. The boot-state log
between them, the slot swap and the image CRC live in the same crate
([flash-layout/src/boot.rs](flash-layout/src/boot.rs),
[flash-layout/src/crc32.rs](flash-layout/src/crc32.rs)).

## Node firmware

The radio driver implements a `PacketRadio` trait used by the mesh layer.
The driver manages RX/TX state - entering continuous RX on the first poll,
transitioning to standby before TX, then polling the IRQ register for
`TxDone` or `Timeout` rather than blocking on the DIO1 pin.

Everything below hangs off the single priority-1 RTIC task in
`firmware/src/bin/main.rs`; the cfg-gated blocks are cargo features. That
single priority is what makes `SharedI2c`'s `RefCell` sound.

```mermaid
classDiagram
    class RunTask {
        <<RTIC priority 1>>
        loop()
    }
    class MeshNode
    class LoraIo~R~
    class Sx1262Driver {
        <<PacketRadio>>
        SubGHz SPI3
    }
    class OtaReceiver
    class BootState {
        boot-state log
        confirm(), request_swap()
    }
    class RadioWatch {
        <<supervisor>>
        TX stall: re-init, reset
    }
    class SharedI2c~T~ {
        <<&'static RefCell~BoundedI2c~>>
        PB15 SCL / PA15 SDA
    }
    class Ssd1306
    class Sensors~I2C~ {
        found: Found
        read() Readings
    }
    class Board {
        <<feature board>>
        senses, buck, mppt, sd, led, rail_5v
    }
    class Rs422 {
        <<feature rs422>>
        USART1 Modbus RTU, DE/RE
    }
    class GpsRadioLog {
        <<feature gps-radio-log>>
        USART1 NMEA
    }

    RunTask --> MeshNode
    RunTask --> OtaReceiver
    RunTask --> BootState
    RunTask --> RadioWatch
    OtaReceiver --> BootState : request_swap
    RunTask --> Board
    RunTask --> Rs422 : SEN0600 soil
    RunTask --> GpsRadioLog
    MeshNode --> LoraIo
    LoraIo --> Sx1262Driver
    RunTask --> SharedI2c
    SharedI2c --> Ssd1306
    SharedI2c --> Sensors
    Sensors ..> BME680
    Sensors ..> BMP280
    Sensors ..> SHT45
    Sensors ..> MCP9808
    Sensors ..> SCD41
    Sensors ..> ADXL345
    Sensors ..> AS3935
    Sensors ..> SEN0575
    Sensors ..> BMV080

    class BME680 { 0x76/0x77 }
    class BMP280 { 0x76/0x77 }
    class SHT45 { 0x44 }
    class MCP9808 { 0x18..0x1F }
    class SCD41 { 0x62 }
    class ADXL345 { 0x53/0x1D }
    class AS3935 { 0x03..0x01 }
    class SEN0575 { 0x1D rain }
    class BMV080 { 0x57/0x56 }
```

The 0x1D three-way conflict between ADXL345, MCP9808 and SEN0575 is
resolved in probe order (notes/sensors.md, the SEN0575 section).

## Boot state

The bootloader reads the boot-state log on every boot, carries out what
it asks for, then jumps to the active slot. The log is an append-only
list of records, one per flash double-word; each change appends one, so a
power cut at any point leaves the last durable record in charge, and a
swap or revert resumes at the page copy it was in (notes/boot-ota.md,
"The boot-state log and the swap").

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Idle : page erased
    Idle --> Swap : OtaReceiver staged and checked an image (ImageSize, ImageCrc, Swap 0)
    Swap --> Idle : bootloader refused the image (Refused)
    Swap --> Swap : one page copy done (Swap n+1)
    Swap --> Trial : all 168 copies done (Swapped)
    Trial --> Idle : the app ran healthy for 30 min (Confirmed)
    Trial --> Revert : a reset before Confirmed
    Revert --> Revert : one page copied back (Revert n+1)
    Revert --> Idle : old image back (Confirmed)
```

The record names are `flash_layout::boot::Record`. An image the
bootloader did not start (probe-rs sets the program counter itself)
settles whatever the log had pending at once, so the next boot cannot
revert over a freshly flashed build or copy pages over it.

## Over-the-air update

`mise run ota-upload` hands an image to the basestation, which stages it
in its own DFU slot and pushes it to the target node in 27-byte chunks,
stop-and-wait. The message layouts are in
`firmware/src/ota_protocol.rs` (mesh) and `basestation/src/uart_proto.rs`
(UART).

```mermaid
sequenceDiagram
    participant H as ota_upload.py
    participant B as Basestation
    participant N as Node app
    participant L as Node bootloader
    H->>B: START_OTA (target, size, version, CRC32)
    B-->>H: ACK
    loop 128-byte pieces
        H->>B: FW_DATA
        B-->>H: ACK
    end
    H->>B: BEGIN_TRANSFER
    B-->>H: ACK once the staged image's CRC32 matches
    B->>N: OTA_OFFER (size, chunks, CRC32, version)
    alt not newer, too large, busy, or boot not settled
        N-->>B: OTA_REJECT (reason, BOOT_PENDING while on trial)
    else accepted
        N-->>B: OTA_ACCEPT
        loop every chunk
            B->>N: OTA_CHUNK (index, length, data)
            N-->>B: OTA_CHUNK_ACK (next chunk needed)
        end
        Note over N: CRC32 over its DFU slot matches, the log asks for a swap
        N-->>B: OTA_COMPLETE (CRC32), again for a retried chunk
    end
    B-->>H: OTA_DONE (result), after periodic PROGRESS
    Note over N: about 10 s later the node resets itself
    N->>L: software reset
    Note over L: check size, vector table and CRC32, swap page by page
    L->>N: jump to the new image, on trial
    Note over N: 30 min healthy and it appends Confirmed, a reset before that reverts it
```

The same update over time, for a 48 KB image at the default `fast`
preset. All durations are estimates (notes/boot-ota.md, "OTA timing").

```mermaid
gantt
    title OTA update of a 48 KB image
    dateFormat mm:ss
    axisFormat %M:%S
    todayMarker off
    section Host and basestation
    Upload to the basestation over UART : 00:00, 10s
    Chunks over the mesh (stop-and-wait) : up, 00:10, 1340s
    section Node
    Old app receives into the DFU slot : 00:10, 1340s
    Old app answers retries, then resets : wait, after up, 10s
    Bootloader checks and swaps : swap, after wait, 9s
    New app on trial : crit, trial, after swap, 1800s
    New app confirmed : after trial, 300s
    section Boot log
    Idle : 00:00, 1350s
    Swap : sp, after up, 19s
    Trial : tr, after sp, 1800s
    Idle, Confirmed : after tr, 300s
```

## Watchdogs

The independent watchdog (IWDG) resets the chip when the loop stops; the
rest of the system decides what a reset means and makes sure each one is
reported. `firmware/src/watchdog.rs` says who starts and feeds the IWDG,
`firmware/src/supervisor.rs` holds the health rules, and
`firmware/src/reset_record.rs` the backup register map.

```mermaid
flowchart LR
    subgraph bl["Bootloader"]
        work["swap, revert or check an image<br/>feeds before each page"]
        jump["start the IWDG, leave the<br/>handoff word, jump to the app"]
        park["no vector table in the active slot:<br/>wait for a probe, IWDG off"]
    end
    subgraph app["Node app"]
        pass["main loop pass<br/>one feed per pass"]
        waits["waits with their own deadline<br/>TX poll, SD busy, SD init: feed_now"]
        i2c["I2C waits give up after 25 ms, a failed transfer clears the bus<br/>watchdog resets during I2C: bus left alone 1-24 h"]
        radio["RadioWatch: no TX for 10 min re-inits<br/>the radio, 5 min more resets the chip"]
        ota["OTA_COMPLETE sent:<br/>reset about 10 s later"]
        panic["panic: record file and line, spin"]
        gate["image on trial kept after 30 min up,<br/>a TX in the last 5 min and a packet heard"]
    end
    iwdg{{"IWDG<br/>5 s nominal, 4.7-5.4 s<br/>LSI clock, never stops"}}
    reset(["system reset"])
    record[("TAMP backup registers<br/>cause, boots, loop phase, panic site")]
    log[("boot-state log")]

    jump -- starts --> iwdg
    work -- feeds --> iwdg
    pass -- feeds --> iwdg
    waits -- feeds --> iwdg
    panic -. "stops the feeds" .-> iwdg
    iwdg -- "a pass blocked" --> reset
    radio -- "SoftReset RadioStall" --> reset
    ota -- "SoftReset Ota" --> reset
    reset --> bl
    gate -- "appends Confirmed" --> log
    log -- "trial reset before Confirmed: revert" --> work
    pass -. "phase before each step" .-> record
    reset -. "RCC_CSR flags, read at boot" .-> record
    record -- "I2C strikes" --> i2c
    record -- "boot report W B V N P L" --> pass
```

The boot report goes out in place of the first three heartbeats after a
boot and every 150th after; its fields are documented at `boot_line` in
`firmware/src/bin/main.rs`.

## Pin allocation

Wio-E5 on the Forest Sensor Platform V1 board (`board` feature). Pin
numbers are module pads.

| Function | Pins | Notes |
|----------|------|-------|
| SWD      | PA13 (SWDIO), PA14 (SWCLK), RST | Pads 3, 4, 17; shared by the CH572D (CMSIS-DAP) and the J2 header |
| I2C2     | PB15 (SCL), PA15 (SDA) | Pads 5-6, J8 and J10, 4.7k pull-ups |
| SD card  | PB3 (SCK), PB4 (MISO), PB5 (MOSI), PA0 (CS) | SPI1; no card-detect line |
| RS-485   | PB6 (TX), PB7 (RX), PA9 (DE), PB9 (/RE) | USART1 to the TPT482, `rs422` feature |
| Host link | PA2 (TX), PA3 (RX) | USART2 to the CH572D USB bridge; the basestation's link with `board` |
| Senses   | PB13 (VIN), PA10 (VBAT), PB14 (IBAT) | ADC_IN0/6/1, see `firmware/src/board.rs` for scaling |
| Buck PWM | PB10 | TIM2_CH3, high = switch on |
| 5V_EN    | PC0 | Switched 5 V rail (J7 sensor supply, CH572D) |
| LED      | PC1 | D12, active low |
| RF switch | PA4 (control 1), PA5 (control 2) | Module-internal, not on a pad |

## CH572D USB bridge

Its firmware, with a diagram of the modules, is ch572-bridge-rs: see its
[ARCHITECTURE.md](https://github.com/tmpk13/ch572-bridge-rs/blob/main/ARCHITECTURE.md).
