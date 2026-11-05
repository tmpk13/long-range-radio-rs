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

The partition map that the bootloader and the apps share, and the
boot-state record between them, are defined once in
[flash-layout/src/lib.rs](flash-layout/src/lib.rs); each crate's build
script generates its linker `memory.x` from it.

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
    class SharedI2c~T~ {
        <<&'static RefCell~I2c2~>>
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

The bootloader reads the boot-state word on every boot, acts on it, then
jumps to the active slot. A swap or revert records its page progress, so
either resumes after a power loss.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> BOOT_OK : first flash, page erased
    BOOT_OK --> SWAP_PENDING : OtaReceiver verified a staged image
    SWAP_PENDING --> SWAP_COMPLETE : bootloader swapped the slots
    SWAP_COMPLETE --> BOOT_OK : new app ran confirm_boot
    SWAP_COMPLETE --> REVERT_PENDING : reset before confirm_boot
    REVERT_PENDING --> BOOT_OK : bootloader restored the old image
```

`confirm_boot` runs at the top of `init`, right after the watchdog starts
and before the radio comes up, so only a new app that dies that early is
reverted (notes/boot-ota.md, "Known gaps").

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
    alt not newer than the running version, too large, or busy
        N-->>B: OTA_REJECT (reason)
    else accepted
        N-->>B: OTA_ACCEPT
        loop every chunk
            B->>N: OTA_CHUNK (index, length, data)
            N-->>B: OTA_CHUNK_ACK (next chunk needed)
        end
        Note over N: CRC32 over its DFU slot matches, state SWAP_PENDING
        N-->>B: OTA_COMPLETE (CRC32)
    end
    B-->>H: OTA_DONE (result), after periodic PROGRESS
    Note over N: no reboot, the old app keeps running
    N->>L: next reset (power cycle, watchdog, probe)
    Note over L: swap the slots, SWAP_COMPLETE
    L->>N: jump to the new app, which confirms its boot
```

The same update over time, for a 48 KB image at the default `fast`
preset. All durations are estimates (notes/boot-ota.md, "OTA timing"),
and the wait for a reset can be any length.

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
    Old app runs until a reset : crit, run, after up, 600s
    Bootloader swaps the slots : swap, after run, 8s
    New app confirms its boot : ok, after swap, 1s
    New app runs : after ok, 60s
    section Boot state
    BOOT_OK : 00:00, 1350s
    SWAP_PENDING : sp, after up, 608s
    SWAP_COMPLETE : sc, after sp, 1s
    BOOT_OK : after sc, 60s
```

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
