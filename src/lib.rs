//! `radiohead-rs` — Rust FFI wrapper for RadioHead mesh routing.
//!
//! This crate wraps the C++ RadioHead routing stack (RHMesh, RHRouter,
//! RHReliableDatagram) so it can be driven from Rust.  The actual radio
//! hardware is provided by the user via the [`RadioDriver`] trait —
//! typically backed by an existing Rust SX1262/SX126x crate and `esp-hal`.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │  Your application (Rust)            │
//! ├─────────────────────────────────────┤
//! │  RhMesh  (safe Rust wrapper)        │
//! ├─────────────────────────────────────┤
//! │  C shim  (extern "C")              │
//! ├─────────────────────────────────────┤
//! │  RadioHead C++ routing stack        │
//! │  RHMesh → RHRouter → RHReliable…   │
//! ├─────────────────────────────────────┤
//! │  RustDriver : RHGenericDriver       │
//! │  (calls back into Rust via fn ptrs) │
//! ├─────────────────────────────────────┤
//! │  impl RadioDriver  (you write this) │
//! │  e.g. sx126x crate + esp-hal SPI    │
//! └─────────────────────────────────────┘
//! ```
//!
//! # Platform functions
//!
//! The C++ routing code needs `millis()`, `delay()` and `random()`.
//! You **must** provide these as `#[no_mangle] extern "C"` functions
//! in your final binary:
//!
//! ```rust,ignore
//! #[no_mangle]
//! pub extern "C" fn rh_millis() -> u32 {
//!     // return milliseconds since boot
//! }
//!
//! #[no_mangle]
//! pub extern "C" fn rh_delay(ms: u32) {
//!     // busy-wait or yield for `ms` milliseconds
//! }
//!
//! #[no_mangle]
//! pub extern "C" fn rh_random(min: i32, max: i32) -> i32 {
//!     // return a random number in [min, max)
//! }
//! ```

#![no_std]

mod ffi;

mod driver;
pub use driver::RadioDriver;

mod mesh;
pub use mesh::{MeshError, Message, RhMesh};
