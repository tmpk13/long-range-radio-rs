#![no_std]
#![no_main]
#![warn(clippy::large_stack_frames)]
#![allow(clippy::manual_div_ceil)]

use panic_halt as _;

#[macro_use]
extern crate sx1262_mesh_rs;

/// RF frequency in Hz (915 MHz ISM band).
const RF_FREQ: u32 = 915_000_000;

mod uart_proto;

use core::cell::UnsafeCell;

// ── Static page buffer for DFU writes (avoid stack overflow) ───────────────

const PAGE_SIZE: usize = 2048;

struct StaticPageBuf(UnsafeCell<[u8; PAGE_SIZE]>);
unsafe impl Sync for StaticPageBuf {}
static DFU_PAGE_BUF: StaticPageBuf = StaticPageBuf(UnsafeCell::new([0xFF; PAGE_SIZE]));

fn page_buf() -> &'static [u8; PAGE_SIZE] {
    unsafe { &*DFU_PAGE_BUF.0.get() }
}

fn page_buf_mut() -> &'static mut [u8; PAGE_SIZE] {
    unsafe { &mut *DFU_PAGE_BUF.0.get() }
}

// ── Flash constants ────────────────────────────────────────────────────────

const FLASH_PAGE_SIZE: u32 = 2048;
const DFU_PAGE_START: u8 = 64;
const MAX_FW_SIZE: u32 = 56 * FLASH_PAGE_SIZE; // 112 KB

/*
    Basestation Node — Seeed Wio-E5 (STM32WLE5JC)

    Single UART connection via RS232 TTL 3.3V FTDI adapter.
    All commands (OTA + data relay) are multiplexed on one link.

    USART1:
        TX — PB6  (pin 10)
        RX — PB7  (pin 9)

    SX1262 radio: integrated (SubGHz peripheral, same as regular node).
    No display on basestation.
*/

#[rtic::app(device = stm32wlxx_hal::pac, dispatchers = [SPI1])]
mod app {
    use rtic_monotonics::systick::prelude::*;
    systick_monotonic!(Mono, 1000);

    use embedded_hal::serial::Read as SerialRead;
    use stm32wlxx_hal::{
        gpio::pins,
        pac::{FLASH, IWDG},
        subghz::SubGhz,
        uart::{self, Uart1},
    };
    use sx1262_mesh_rs::{
        LoraIo, MeshNode, OtaSender,
        config::{BROADCAST_LIFETIME, MESH_LISTEN_PERIOD_MS, THIS_ADDRESS},
        ota_protocol,
        platform::SYSCLK_HZ,
        radio::Sx1262Driver,
        watchdog,
    };
    use rtt_target::{rprintln, rtt_init, set_print_channel};

    use crate::uart_proto::{self, FrameBuf, FrameParser, cmd, resp, ota_state, err};

    type Radio = Sx1262Driver;
    type HostUart = Uart1<pins::B7, pins::B6>;

    /// Basestation operating state.
    enum BsState {
        /// No OTA in progress.  Relaying data only.
        Idle,
        /// Receiving firmware binary from host over UART.
        ReceivingFw {
            target_addr: u8,
            fw_size: u32,
            fw_version: u16,
            bytes_received: u32,
            page_offset: u16,
            current_page: u16,
        },
        /// Sending firmware to target node over mesh radio.
        Transferring {
            target_addr: u8,
        },
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        io: LoraIo<Radio>,
        mesh: MeshNode,
        flash: FLASH,
        iwdg: IWDG,
        uart: HostUart,
        parser: FrameParser,
        state: BsState,
        ota_sender: Option<OtaSender>,
        tx_frame: FrameBuf,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        let channels = rtt_init! {
            up: {
                0: { size: 1024, name: "Terminal" }
            }
        };
        set_print_channel(channels.up.0);

        // Enable DWT cycle counter for millis()/random()
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();

        // Start SysTick monotonic at default MSI 4 MHz
        Mono::start(cx.core.SYST, SYSCLK_HZ);

        let dp = cx.device;
        let mut flash_periph = dp.FLASH;
        let mut rcc = dp.RCC;

        // Start watchdog (5 s timeout)
        let iwdg = dp.IWDG;
        watchdog::start(&iwdg, 5_000);

        // Confirm boot to the bootloader
        sx1262_mesh_rs::boot_state::confirm_boot(&mut flash_periph);

        // ---- SubGHz radio (integrated SX1262) --------------------------------
        let sg = SubGhz::new(dp.SPI3, &mut rcc);
        let mut radio = Sx1262Driver::new(sg);
        radio.init(super::RF_FREQ);
        radio.print_diagnostics();

        // ---- USART1: PB6 TX, PB7 RX ----------------------------------------
        let gpiob = stm32wlxx_hal::gpio::PortB::split(dp.GPIOB, &mut rcc);

        let uart = cortex_m::interrupt::free(|cs| {
            let u = Uart1::new(dp.USART1, 115_200, uart::Clk::Hsi16, &mut rcc);
            let u = u.enable_rx(gpiob.b7, cs);
            u.enable_tx(gpiob.b6, cs)
        });

        // ---- Mesh networking ------------------------------------------------
        debug_println!(
            "Basestation starting (address={}, freq={} Hz)...",
            THIS_ADDRESS,
            super::RF_FREQ
        );
        let io = LoraIo::new(radio);
        let mesh = MeshNode::new(THIS_ADDRESS, MESH_LISTEN_PERIOD_MS);

        rprintln!("Basestation node {} ready", THIS_ADDRESS);

        run::spawn().unwrap();

        (
            Shared {},
            Local {
                io,
                mesh,
                flash: flash_periph,
                iwdg,
                uart,
                parser: FrameParser::new(),
                state: BsState::Idle,
                ota_sender: None,
                tx_frame: FrameBuf::new(),
            },
        )
    }

    #[task(local = [io, mesh, flash, iwdg, uart, parser, state, ota_sender, tx_frame],
           priority = 1)]
    async fn run(cx: run::Context) {
        let io = cx.local.io;
        let mesh = cx.local.mesh;
        let flash = cx.local.flash;
        let iwdg = cx.local.iwdg;
        let uart = cx.local.uart;
        let parser = cx.local.parser;
        let state = cx.local.state;
        let ota_sender = cx.local.ota_sender;
        let tx_frame = cx.local.tx_frame;

        // Progress reporting interval
        let progress_interval = 500_u32.millis();
        let mut next_progress = Mono::now() + progress_interval;

        loop {
            // ── 1. Feed watchdog ────────────────────────────────────────
            watchdog::feed(iwdg);

            // ── 2. Drive mesh protocol ──────────────────────────────────
            mesh.update(io, sx1262_mesh_rs::platform::millis());

            // ── 3. Poll UART for host commands ──────────────────────────
            loop {
                match uart.read() {
                    Ok(byte) => {
                        if parser.feed(byte) {
                            let frame = parser.frame();
                            handle_cmd(
                                frame.cmd, frame.payload,
                                state, ota_sender, flash, mesh,
                                uart, tx_frame,
                            );
                        }
                    }
                    Err(nb::Error::WouldBlock) => break,
                    Err(_) => break,
                }
            }

            // ── 4. Process received mesh messages ───────────────────────
            if let Some(msg) = mesh.receive() {
                if ota_protocol::is_ota_message(&msg.data) {
                    // Route OTA responses to sender
                    if let Some(sender) = ota_sender.as_mut()
                        && let Some(result) = sender.handle_message(&msg.data)
                    {
                        let result_byte = match result {
                            sx1262_mesh_rs::ota_sender::OtaResult::Success => 0x00,
                            sx1262_mesh_rs::ota_sender::OtaResult::Rejected(r) => r,
                            sx1262_mesh_rs::ota_sender::OtaResult::Aborted(r) => r,
                        };
                        tx_frame.build(resp::OTA_DONE, &[result_byte]);
                        uart_send(uart, tx_frame.as_bytes());
                        rprintln!("OTA done: result=0x{:02X}", result_byte);

                        *ota_sender = None;
                        *state = BsState::Idle;
                    }
                } else {
                    // Regular message — forward to host
                    let src = msg.source;
                    let rssi = io.last_rssi();
                    let rssi_bytes = rssi.to_le_bytes();
                    let data = &msg.data[..];

                    // Payload: src(1) + rssi(2 LE) + data(N)
                    let plen = 3 + data.len();
                    if plen <= uart_proto::MAX_PAYLOAD {
                        let mut payload = [0u8; uart_proto::MAX_PAYLOAD];
                        payload[0] = src;
                        payload[1] = rssi_bytes[0];
                        payload[2] = rssi_bytes[1];
                        payload[3..3 + data.len()].copy_from_slice(data);
                        tx_frame.build(resp::RECV_MSG, &payload[..plen]);
                        uart_send(uart, tx_frame.as_bytes());
                    }
                }
            }

            // ── 5. Drive OTA sender (send next chunk over mesh) ─────────
            if let Some(sender) = ota_sender.as_mut()
                && let BsState::Transferring { target_addr } = state
                && let Some(msg) = sender.next_message()
            {
                mesh.send(&msg.data[..msg.len], *target_addr, BROADCAST_LIFETIME).ok();
            }

            // ── 6. Periodic progress reporting ──────────────────────────
            if Mono::now() >= next_progress {
                next_progress = Mono::now() + progress_interval;

                let (st, pct, sent, total) = match state {
                    BsState::Idle => (ota_state::IDLE, 0u8, 0u16, 0u16),
                    BsState::ReceivingFw { fw_size, bytes_received, .. } => {
                        let pct = if *fw_size > 0 {
                            (*bytes_received * 100 / *fw_size) as u8
                        } else {
                            0
                        };
                        (ota_state::RECEIVING_FW, pct, 0, 0)
                    }
                    BsState::Transferring { .. } => {
                        if let Some(sender) = ota_sender.as_ref() {
                            if let Some((sent, total)) = sender.progress() {
                                let pct = if total > 0 {
                                    (sent as u32 * 100 / total as u32) as u8
                                } else {
                                    0
                                };
                                (ota_state::TRANSFERRING, pct, sent, total)
                            } else {
                                (ota_state::COMPLETE, 100, 0, 0)
                            }
                        } else {
                            (ota_state::IDLE, 0, 0, 0)
                        }
                    }
                };

                let mut payload = [0u8; 6];
                payload[0] = st;
                payload[1] = pct;
                payload[2..4].copy_from_slice(&sent.to_le_bytes());
                payload[4..6].copy_from_slice(&total.to_le_bytes());
                tx_frame.build(resp::PROGRESS, &payload);
                uart_send(uart, tx_frame.as_bytes());
            }

            Mono::delay(1_u32.millis()).await;
        }
    }

    // ── Command handler (OTA + data relay, single UART) ─────────────────────

    #[allow(clippy::large_stack_frames)]
    fn handle_cmd(
        cmd_id: u8,
        payload: &[u8],
        state: &mut BsState,
        ota_sender: &mut Option<OtaSender>,
        flash: &mut FLASH,
        mesh: &mut MeshNode,
        uart: &mut HostUart,
        tx: &mut FrameBuf,
    ) {
        match cmd_id {
            // ── OTA commands ────────────────────────────────────────────
            cmd::START_OTA => {
                if !matches!(state, BsState::Idle) {
                    tx.build(resp::NAK, &[err::INVALID_STATE]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }
                if payload.len() < 7 {
                    tx.build(resp::NAK, &[err::BAD_FRAME]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }

                let target_addr = payload[0];
                let fw_size = u32::from_le_bytes([
                    payload[1], payload[2], payload[3], payload[4],
                ]);
                let fw_version = u16::from_le_bytes([payload[5], payload[6]]);

                if fw_size == 0 || fw_size > super::MAX_FW_SIZE {
                    tx.build(resp::NAK, &[err::BAD_SIZE]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }

                super::page_buf_mut().fill(0xFF);

                rprintln!(
                    "OTA start: target={} size={} ver={}",
                    target_addr, fw_size, fw_version
                );

                *state = BsState::ReceivingFw {
                    target_addr,
                    fw_size,
                    fw_version,
                    bytes_received: 0,
                    page_offset: 0,
                    current_page: 0,
                };

                tx.build(resp::ACK, &[]);
                uart_send(uart, tx.as_bytes());
            }

            cmd::FW_DATA => {
                let (fw_size, bytes_received, page_offset, current_page) = match state {
                    BsState::ReceivingFw {
                        fw_size,
                        bytes_received,
                        page_offset,
                        current_page,
                        ..
                    } => (*fw_size, bytes_received, page_offset, current_page),
                    _ => {
                        tx.build(resp::NAK, &[err::INVALID_STATE]);
                        uart_send(uart, tx.as_bytes());
                        return;
                    }
                };

                if payload.is_empty() {
                    tx.build(resp::NAK, &[err::BAD_FRAME]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }

                let mut data_pos = 0;
                while data_pos < payload.len() {
                    let offset = *page_offset as usize;
                    let space = super::PAGE_SIZE - offset;
                    let to_copy = (payload.len() - data_pos).min(space);

                    super::page_buf_mut()[offset..offset + to_copy]
                        .copy_from_slice(&payload[data_pos..data_pos + to_copy]);
                    *page_offset += to_copy as u16;
                    *bytes_received += to_copy as u32;
                    data_pos += to_copy;

                    if *page_offset >= super::PAGE_SIZE as u16 {
                        let page_idx = super::DFU_PAGE_START + *current_page as u8;
                        if !write_page_to_flash(flash, page_idx) {
                            rprintln!("Flash write error at page {}", page_idx);
                            *state = BsState::Idle;
                            tx.build(resp::NAK, &[err::FLASH_ERROR]);
                            uart_send(uart, tx.as_bytes());
                            return;
                        }
                        *current_page += 1;
                        *page_offset = 0;
                        super::page_buf_mut().fill(0xFF);
                    }
                }

                let all_received = *bytes_received >= fw_size;
                let total_bytes = *bytes_received;
                let final_page_offset = *page_offset;
                let final_current_page = *current_page;

                if all_received && final_page_offset > 0 {
                    let page_idx = super::DFU_PAGE_START + final_current_page as u8;
                    if !write_page_to_flash(flash, page_idx) {
                        rprintln!("Flash write error at final page {}", page_idx);
                        *state = BsState::Idle;
                        tx.build(resp::NAK, &[err::FLASH_ERROR]);
                        uart_send(uart, tx.as_bytes());
                        return;
                    }
                }
                if all_received {
                    rprintln!("Firmware upload complete ({} bytes)", total_bytes);
                }

                tx.build(resp::ACK, &[]);
                uart_send(uart, tx.as_bytes());
            }

            cmd::BEGIN_TRANSFER => {
                let (target_addr, fw_size, fw_version, bytes_received) = match state {
                    BsState::ReceivingFw {
                        target_addr,
                        fw_size,
                        fw_version,
                        bytes_received,
                        ..
                    } => (*target_addr, *fw_size, *fw_version, *bytes_received),
                    _ => {
                        tx.build(resp::NAK, &[err::INVALID_STATE]);
                        uart_send(uart, tx.as_bytes());
                        return;
                    }
                };

                if bytes_received < fw_size {
                    tx.build(resp::NAK, &[err::NO_FW_DATA]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }

                let crc32 = OtaSender::compute_dfu_crc32(fw_size);
                rprintln!(
                    "Starting mesh OTA: target={} size={} crc=0x{:08X} ver={}",
                    target_addr, fw_size, crc32, fw_version
                );

                *ota_sender = Some(OtaSender::new(fw_size, crc32, fw_version));
                *state = BsState::Transferring { target_addr };

                tx.build(resp::ACK, &[]);
                uart_send(uart, tx.as_bytes());
            }

            cmd::QUERY_STATUS => {
                let (st, pct, sent, total) = match state {
                    BsState::Idle => (ota_state::IDLE, 0u8, 0u16, 0u16),
                    BsState::ReceivingFw { fw_size, bytes_received, .. } => {
                        let pct = if *fw_size > 0 {
                            (*bytes_received * 100 / *fw_size) as u8
                        } else {
                            0
                        };
                        (ota_state::RECEIVING_FW, pct, 0, 0)
                    }
                    BsState::Transferring { .. } => {
                        if let Some(sender) = ota_sender.as_ref() {
                            if let Some((s, t)) = sender.progress() {
                                let pct = if t > 0 {
                                    (s as u32 * 100 / t as u32) as u8
                                } else {
                                    0
                                };
                                (ota_state::TRANSFERRING, pct, s, t)
                            } else {
                                (ota_state::COMPLETE, 100, 0, 0)
                            }
                        } else {
                            (ota_state::IDLE, 0, 0, 0)
                        }
                    }
                };

                let mut payload_buf = [0u8; 6];
                payload_buf[0] = st;
                payload_buf[1] = pct;
                payload_buf[2..4].copy_from_slice(&sent.to_le_bytes());
                payload_buf[4..6].copy_from_slice(&total.to_le_bytes());
                tx.build(resp::PROGRESS, &payload_buf);
                uart_send(uart, tx.as_bytes());
            }

            cmd::ABORT_OTA => {
                rprintln!("OTA aborted by host");
                *ota_sender = None;
                *state = BsState::Idle;
                tx.build(resp::ACK, &[]);
                uart_send(uart, tx.as_bytes());
            }

            // ── Data relay commands ─────────────────────────────────────
            cmd::SEND_MSG => {
                if payload.len() < 2 {
                    tx.build(resp::NAK, &[err::BAD_FRAME]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }
                let dest = payload[0];
                let data = &payload[1..];
                match mesh.send(data, dest, BROADCAST_LIFETIME) {
                    Ok(()) => {
                        tx.build(resp::ACK, &[]);
                        uart_send(uart, tx.as_bytes());
                    }
                    Err(_) => {
                        tx.build(resp::NAK, &[err::INVALID_STATE]);
                        uart_send(uart, tx.as_bytes());
                    }
                }
            }

            cmd::SEND_BROADCAST => {
                if payload.is_empty() {
                    tx.build(resp::NAK, &[err::BAD_FRAME]);
                    uart_send(uart, tx.as_bytes());
                    return;
                }
                match mesh.broadcast(payload, BROADCAST_LIFETIME) {
                    Ok(()) => {
                        tx.build(resp::ACK, &[]);
                        uart_send(uart, tx.as_bytes());
                    }
                    Err(_) => {
                        tx.build(resp::NAK, &[err::INVALID_STATE]);
                        uart_send(uart, tx.as_bytes());
                    }
                }
            }

            _ => {
                tx.build(resp::NAK, &[err::BAD_FRAME]);
                uart_send(uart, tx.as_bytes());
            }
        }
    }

    // ── Helpers ─────────────────────────────────────────────────────────────

    /// Write a page buffer to flash at the given page index.
    fn write_page_to_flash(flash_periph: &mut FLASH, page_idx: u8) -> bool {
        use stm32wlxx_hal::flash::{AlignedAddr, Flash, Page};

        let mut flash = Flash::unlock(flash_periph);
        let page = unsafe { Page::from_index_unchecked(page_idx) };

        if unsafe { flash.page_erase(page) }.is_err() {
            return false;
        }

        let addr = unsafe { AlignedAddr::new_unchecked(page.addr()) };
        if unsafe { flash.program_bytes(super::page_buf(), addr) }.is_err() {
            return false;
        }

        true
    }

    /// Blocking send of a byte slice over UART.
    fn uart_send<TX: embedded_hal::serial::Write<u8>>(uart: &mut TX, data: &[u8]) {
        for &byte in data {
            loop {
                match uart.write(byte) {
                    Ok(()) => break,
                    Err(nb::Error::WouldBlock) => continue,
                    Err(_) => break,
                }
            }
        }
        loop {
            match uart.flush() {
                Ok(()) => break,
                Err(nb::Error::WouldBlock) => continue,
                Err(_) => break,
            }
        }
    }
}
