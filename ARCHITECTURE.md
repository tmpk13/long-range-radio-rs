# Architecture

The radio driver implements a `PacketRadio` trait used by the mesh layer.
The driver manages RX/TX state — entering continuous RX on the first poll,
transitioning to standby before TX, then polling the IRQ register for
`TxDone` or `Timeout` rather than blocking on the DIO1 pin.

Everything below hangs off the single priority-1 RTIC task in
`bin/main.rs`; the cfg-gated blocks are cargo features. That single
priority is what makes `SharedI2c`'s `RefCell` sound.

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
        senses, mppt, sd, leds
    }
    class Rs422 {
        <<feature rs422>>
        USART2 Modbus RTU
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
resolved in probe order — see the SEN0575 section.

## Pin allocation

| Function | Pins | Notes |
|----------|------|-------|
| SWD      | PA13 (SWDIO), PA14 (SWCLK) | Pins 3-4, dedicated |
| I2C2     | PB15 (SCL), PA15 (SDA) | Pins 5-6 |
| UART1    | PB6 (TX), PB7 (RX) | Pins 9-10, gateway serial to Linux box |
| SPI2     | PB13 (SCK), PB14 (MISO), PA10 (MOSI), PB9 (NSS) | Pins 24-27, note PB13 is also boot pin |
| GPIO     | PB4, PB3 | Pins 7-8 |
| RF switch | PA4 (control 1), PA5 (control 2) | Module-internal, not on a pad |

*SPI SCK must remain inactive for boot*
