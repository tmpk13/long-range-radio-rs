# OTA Firmware Update System

Over-the-air firmware updates for the sx1262-mesh-rs LoRa mesh network,
delivered over the existing mesh radio link.

## Flash Layout

The STM32WLE5JC's 256 KB flash (128 pages of 2 KB) is partitioned as follows:

| Partition    | Pages   | Size   | Address Range               |
|-------------|---------|--------|-----------------------------|
| Bootloader  | 0--7    | 16 KB  | `0x0800_0000`--`0x0800_3FFF` |
| Active (app)| 8--63   | 112 KB | `0x0800_4000`--`0x0801_FFFF` |
| DFU staging | 64--120 | 114 KB | `0x0802_0000`--`0x0803_C7FF` |
| Boot state  | 121--127| 14 KB  | `0x0803_C800`--`0x0803_FFFF` |

The DFU partition is one page larger than Active (57 vs 56 pages). The extra
page is used as scratch space during power-fail-safe swaps.

## Components

### Bootloader (`bootloader/`)

A standalone binary (~2.7 KB) that runs from the first 16 KB of flash.
On every boot it reads a state magic word from page 121 and acts accordingly:

| State magic       | Value        | Action                                    |
|-------------------|--------------|-------------------------------------------|
| `BOOT_OK`         | `0x4F4B4F4B` | Jump to app                               |
| `SWAP_PENDING`    | `0x53574150` | Swap Active <-> DFU page-by-page, then boot |
| `SWAP_COMPLETE`   | `0x444F4E45` | App didn't confirm -- revert, then boot   |
| `REVERT_PENDING`  | `0x52455654` | Copy DFU back to Active, then boot        |
| Erased (`0xFFFF`) | --           | Jump to app (fresh flash)                 |

The swap uses a scratch page (page 120) and tracks progress at `STATE_ADDR + 4`
so it can resume after a power loss mid-swap.

Build and flash:

```sh
cd bootloader
cargo build --release
# Flash with probe-rs (writes to 0x0800_0000 per memory.x)
cargo run --release
```

### Boot State (`src/boot_state.rs`)

Application-side helpers for the boot state partition. Uses the HAL
`Flash` API (`page_erase` + `program_bytes`).

- `confirm_boot(flash)` -- writes `BOOT_OK`. Called during `init` after
  the radio comes up. If the app never calls this before resetting, the
  bootloader reverts to the previous firmware on next boot.
- `request_swap(flash)` -- writes `SWAP_PENDING`. Called after a
  successful OTA transfer to trigger the swap on reboot.

### OTA Protocol (`src/ota_protocol.rs`)

All messages fit in a 32-byte `embedded-nano-mesh` payload. The first byte
is the message type; remaining bytes carry the payload.

| ID     | Name           | Direction       | Payload                                              |
|--------|----------------|-----------------|------------------------------------------------------|
| `0xF0` | `OTA_OFFER`    | Sender -> Target | `firmware_size:u32, total_chunks:u16, crc32:u32, version:u16` |
| `0xF1` | `OTA_ACCEPT`   | Target -> Sender | (empty)                                              |
| `0xF2` | `OTA_REJECT`   | Target -> Sender | `reason:u8`                                          |
| `0xF3` | `OTA_CHUNK`    | Sender -> Target | `chunk_index:u16, data:[u8; 28]`                     |
| `0xF4` | `OTA_CHUNK_ACK`| Target -> Sender | `chunk_index:u16, next_needed:u16`                   |
| `0xF5` | `OTA_ABORT`    | Either           | `reason:u8`                                          |
| `0xF6` | `OTA_COMPLETE` | Target -> Sender | `crc32:u32`                                          |

Reject/abort reason codes: `NO_SPACE` (0x01), `BAD_VERSION` (0x02),
`CRC_MISMATCH` (0x03), `FLASH_ERROR` (0x04), `TIMEOUT` (0x05), `BUSY` (0x06).

Each chunk carries **28 bytes** of firmware data (32 - 1 type - 2 index - 1 reserved).

### OTA Receiver (`src/ota.rs`)

State machine integrated into the RTIC main loop. Incoming mesh messages
with a type byte in `0xF0..=0xF6` are routed to `OtaReceiver::handle_message()`.

Transfer flow:

1. Sender unicasts `OTA_OFFER` with firmware metadata.
2. Target validates size, sends `OTA_ACCEPT` or `OTA_REJECT`.
3. Sender sends `OTA_CHUNK` #0.
4. Target copies 28-byte payload into a 2 KB RAM page buffer.
5. When the buffer is full, it erases the corresponding DFU flash page
   and writes the buffer.
6. Target sends `OTA_CHUNK_ACK` with the next chunk it needs.
7. Repeat until all chunks are received.
8. Target reads back the DFU partition, computes CRC32 (software, no
   external crate), and sends `OTA_COMPLETE` if it matches.
9. On success, `request_swap()` is called and the device reboots into
   the new firmware via the bootloader.

Duplicate or out-of-order chunks are handled by re-ACKing with the
current `next_needed` index.

### Main Loop Integration (`src/bin/main.rs`)

- The `FLASH` peripheral is extracted in `init` and passed as a `Local` resource.
- `boot_state::confirm_boot()` is called during `init` after the radio
  is up, marking the running firmware as healthy.
- The `run` task checks each received mesh message: if the first byte is
  `>= 0xF0` it goes to the OTA receiver; otherwise normal message
  handling runs as before.
- During an active transfer the OLED shows `OTA XX%`.

## Estimated Transfer Times

Stop-and-wait ARQ at ~0.75 s per chunk (SF7/BW125):

| Firmware Size | Chunks | Time     |
|--------------|--------|----------|
| 37 KB (current) | 1,350 | ~17 min |
| 50 KB        | 1,829  | ~23 min  |
| 112 KB (max) | 4,096  | ~51 min  |

## Binary Sizes

| Component   | .text   | .bss   |
|-------------|---------|--------|
| Bootloader  | 2.7 KB  | 2 KB   |
| Application | 36.7 KB | 5.3 KB |

## Building

```sh
# Build everything
cargo build --release

# Build app only (with node address)
ADDRESS=1 cargo build --release -p sx1262-mesh-rs

# Build bootloader only
cargo build --release -p bootloader
```

### OTA Sender (`src/ota_sender.rs`)

State machine for initiating OTA transfers from a gateway node.  Reads
firmware data directly from the DFU flash partition and pushes it
chunk-by-chunk to a target node using stop-and-wait ARQ.

Usage:

1. Flash the new firmware `.bin` into the DFU partition (pages 64+) using
   probe-rs.
2. Compute the CRC: `OtaSender::compute_dfu_crc32(firmware_size)`.
3. Create the sender: `OtaSender::new(firmware_size, crc32, version)`.
4. In the main loop, call `next_message()` to get outgoing OTA messages
   and route incoming OTA responses to `handle_message()`.
5. Retransmission on timeout (~2 s) is handled internally.

### Watchdog Timer (`src/watchdog.rs`)

Independent Watchdog (IWDG) driver for boot safety.  Started in `init`
with a 5-second timeout **before** `confirm_boot()` is called.

- If the application hangs during init or fails to reach the main loop,
  the IWDG resets the MCU.
- The bootloader sees `SWAP_COMPLETE` (app never confirmed) and reverts
  to the previous firmware.
- Once the main loop is running, `watchdog::feed()` is called every
  iteration to prevent spurious resets.
- The IWDG runs from the LSI oscillator (~32 kHz) and **cannot be
  stopped** once started -- this is by design.

### Version Checking

The OTA receiver validates the `version` field in incoming `OTA_OFFER`
messages against the compile-time `FIRMWARE_VERSION` constant (set via
the `FW_VERSION` environment variable, default `1`).  Offers with a
version less than or equal to the running firmware are rejected with
`BAD_VERSION` (0x02).

Build with a specific version:

```sh
FW_VERSION=2 ADDRESS=1 cargo build --release -p sx1262-mesh-rs
```
