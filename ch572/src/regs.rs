//! CH572 register addresses and bit masks.
//!
//! Names and values match WCH's `CH572SFR.h` from the openwch/ch570 EVT
//! package, so they can be checked against it name by name; the datasheet
//! (CH572DS1) chapter for each group is noted.  Only what this firmware
//! uses is listed.

// ---- System control and clocks (datasheet chapters 4 to 6) --------------

pub const R8_CLK_SYS_CFG: usize = 0x4000_1008;
pub const RB_CLK_SYS_MOD: u8 = 0xC0;
pub const R8_HFCK_PWR_CTRL: usize = 0x4000_100A;
pub const RB_CLK_PLL_PON: u8 = 0x10;
pub const RB_CLK_XT32M_PON: u8 = 0x04;
pub const R8_SLP_POWER_CTRL: usize = 0x4000_100F;
pub const R8_XT32M_TUNE: usize = 0x4000_104E;

pub const R8_SAFE_ACCESS_SIG: usize = 0x4000_1040;
pub const SAFE_ACCESS_SIG1: u8 = 0x57;
pub const SAFE_ACCESS_SIG2: u8 = 0xA8;
pub const SAFE_ACCESS_SIG0: u8 = 0x00;

pub const R8_WDOG_COUNT: usize = 0x4000_1043;
pub const R8_GLOB_ROM_CFG: usize = 0x4000_1044;
pub const RB_ROM_CODE_WE: u8 = 0xC0;
pub const R8_RST_WDOG_CTRL: usize = 0x4000_1046;
pub const RB_WDOG_INT_FLAG: u8 = 0x10;
pub const RB_WDOG_INT_EN: u8 = 0x04;
pub const RB_WDOG_RST_EN: u8 = 0x02;
pub const RB_SOFTWARE_RESET: u8 = 0x01;
pub const R8_GLOB_RESET_KEEP: usize = 0x4000_1047;

pub const R8_FLASH_SCK: usize = 0x4000_1805;
pub const R8_FLASH_CFG: usize = 0x4000_1807;

// ---- Pin functions and GPIO (datasheet chapter 7) -----------------------

pub const R16_PIN_ALTERNATE: usize = 0x4000_1018;
pub const RB_PIN_DEBUG_EN: u16 = 0x4000;
pub const RB_PIN_USB_EN: u16 = 0x2000;
pub const RB_UDP_PU_EN: u16 = 0x1000;
pub const R16_PIN_ALTERNATE_H: usize = 0x4000_101A;
pub const RB_UART_TXD: u16 = 0x0038;
pub const RB_UART_RXD: u16 = 0x0007;

pub const R32_PA_DIR: usize = 0x4000_10A0;
pub const R32_PA_PIN: usize = 0x4000_10A4;
pub const R32_PA_OUT: usize = 0x4000_10A8;
pub const R32_PA_CLR: usize = 0x4000_10AC;
pub const R32_PA_PU: usize = 0x4000_10B0;
pub const R32_PA_PD_DRV: usize = 0x4000_10B4;
pub const R32_PA_SET: usize = 0x4000_10B8;

// ---- UART (datasheet chapter 9) -----------------------------------------

pub const R8_UART_IER: usize = 0x4000_3401;
pub const RB_IER_TXD_EN: u8 = 0x40;
pub const R8_UART_FCR: usize = 0x4000_3402;
pub const RB_FCR_FIFO_EN: u8 = 0x01;
pub const R8_UART_LCR: usize = 0x4000_3403;
pub const RB_LCR_PAR_MOD: u8 = 0x30;
pub const RB_LCR_PAR_EN: u8 = 0x08;
pub const RB_LCR_STOP_BIT: u8 = 0x04;
pub const RB_LCR_WORD_SZ: u8 = 0x03;
pub const R8_UART_LSR: usize = 0x4000_3405;
pub const RB_LSR_TX_FIFO_EMP: u8 = 0x20;
pub const RB_LSR_DATA_RDY: u8 = 0x01;
pub const R8_UART_RBR: usize = 0x4000_3408;
pub const R8_UART_THR: usize = 0x4000_3408;
pub const R8_UART_TFC: usize = 0x4000_340B;
pub const R16_UART_DL: usize = 0x4000_340C;
pub const R8_UART_DIV: usize = 0x4000_340E;

// ---- USB full-speed device (datasheet chapter 16) -----------------------

pub const R8_USB_CTRL: usize = 0x4000_8000;
pub const RB_UC_DEV_PU_EN: u8 = 0x20;
pub const RB_UC_INT_BUSY: u8 = 0x08;
pub const RB_UC_DMA_EN: u8 = 0x01;
pub const R8_UDEV_CTRL: usize = 0x4000_8001;
pub const RB_UD_PD_DIS: u8 = 0x80;
pub const RB_UD_PORT_EN: u8 = 0x01;
pub const R8_USB_INT_EN: usize = 0x4000_8002;
pub const RB_UIE_SUSPEND: u8 = 0x04;
pub const RB_UIE_TRANSFER: u8 = 0x02;
pub const RB_UIE_BUS_RST: u8 = 0x01;
pub const R8_USB_DEV_AD: usize = 0x4000_8003;
pub const MASK_USB_ADDR: u8 = 0x7F;
pub const R8_USB_MIS_ST: usize = 0x4000_8005;
pub const RB_UMS_SUSPEND: u8 = 0x04;
pub const R8_USB_INT_FG: usize = 0x4000_8006;
pub const RB_U_TOG_OK: u8 = 0x40;
pub const RB_UIF_SUSPEND: u8 = 0x04;
pub const RB_UIF_TRANSFER: u8 = 0x02;
pub const RB_UIF_BUS_RST: u8 = 0x01;
pub const R8_USB_INT_ST: usize = 0x4000_8007;
pub const RB_UIS_SETUP_ACT: u8 = 0x80;
pub const MASK_UIS_TOKEN: u8 = 0x30;
pub const UIS_TOKEN_OUT: u8 = 0x00;
pub const UIS_TOKEN_IN: u8 = 0x20;
/// The token field reads 11b when no transaction is pending ("idle").
pub const UIS_TOKEN_SETUP: u8 = 0x30;
pub const MASK_UIS_ENDP: u8 = 0x0F;
pub const R8_USB_RX_LEN: usize = 0x4000_8008;
pub const R8_UEP4_1_MOD: usize = 0x4000_800C;
pub const RB_UEP1_RX_EN: u8 = 0x80;
pub const RB_UEP1_TX_EN: u8 = 0x40;
pub const R8_UEP2_3_MOD: usize = 0x4000_800D;
pub const RB_UEP3_RX_EN: u8 = 0x80;
pub const RB_UEP3_TX_EN: u8 = 0x40;
pub const RB_UEP2_RX_EN: u8 = 0x08;
pub const RB_UEP2_TX_EN: u8 = 0x04;
pub const R16_UEP0_DMA: usize = 0x4000_8010;
pub const R8_UEP0_T_LEN: usize = 0x4000_8020;
pub const R8_UEP0_CTRL: usize = 0x4000_8022;
pub const RB_UEP_R_TOG: u8 = 0x80;
pub const RB_UEP_T_TOG: u8 = 0x40;
pub const MASK_UEP_R_RES: u8 = 0x0C;
pub const UEP_R_RES_ACK: u8 = 0x00;
pub const UEP_R_RES_NAK: u8 = 0x08;
pub const UEP_R_RES_STALL: u8 = 0x0C;
pub const MASK_UEP_T_RES: u8 = 0x03;
pub const UEP_T_RES_ACK: u8 = 0x00;
pub const UEP_T_RES_NAK: u8 = 0x02;
pub const UEP_T_RES_STALL: u8 = 0x03;

/// Per-endpoint register strides for endpoints 0 to 3: the DMA address
/// registers sit 4 bytes apart from R16_UEP0_DMA, and each T_LEN/CTRL
/// pair 4 bytes apart from R8_UEP0_T_LEN/R8_UEP0_CTRL.
pub const UEP_STRIDE: usize = 4;

// ---- SysTick (datasheet section 3.4.4; core_riscv.h, not CH572SFR.h) ----

pub const R32_STK_CTLR: usize = 0xE000_F000;
pub const R32_STK_CNTL: usize = 0xE000_F008;
/// STK_CTLR: count on HCLK rather than HCLK/8.
pub const STK_STCLK: u32 = 0x04;
/// STK_CTLR: counter enable.
pub const STK_STE: u32 = 0x01;
