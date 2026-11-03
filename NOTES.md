# Notes

One topic per file. Read this index, then only the files the task touches.

- `notes/build-flash.md`: the debug feature and debug_println!, pinned crates (embedded-io, ssd1306), the macro build break, OpenOCD unlock, probe-rs detection, the probe-rs reset loop with the display unplugged.
- `notes/boot-ota.md`: two-stage boot (set-vtor, the bootloader link address, the IWDG SR wait), the OtaReceiver stack overflow, basestation UART pins, OTA chunk null-byte truncation.
- `notes/radio.md`: TX packet params, async TX, the XIAO TCXO, dead-radio diagnosis, the sx126x wait_on_busy fix, the SubGhz HAL API reference, LORA_PRESET, RxBoost.
- `notes/mesh.md`: TX/RX timing and the listen period, the DWT millis() wrap behind SendingQueueIsFull, broadcast collisions.
- `notes/sensors.md`: display hot-plug, the I2C sensor suite and RS-422 link, SHT45/MCP9808/BMP280, the sensor sweep broadcast and dashboard capture, the SEN0575 rain gauge.
- `notes/board.md`: the board feature on the Forest Sensor Platform V1: what changed from the old charger board, sense scaling, buck drive, the MPPT controller, the activity LED and switched 5 V rail, RS-485 direction control.
- `notes/gpslog.md`: the gps-radio-log feature and its status line (sats, last RX RSSI, time since RX).
- `notes/audits.md`: radio fixes and the second bug sweep ported back from the esp32c6-gps derivative.
- `notes/ch572.md`: the CH572D USB bridge firmware in `ch572/`: build layout, clock and flash timing, the USB driver's pitfalls, CMSIS-DAP, flashing and getting back into the bootloader, board issues, first bring-up checks.
