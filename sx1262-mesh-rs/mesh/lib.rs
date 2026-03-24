//! Board-agnostic mesh networking layer wrapping `embedded-nano-mesh`.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │  Your application (Rust)            │
//! ├─────────────────────────────────────┤
//! │  MeshNode  (safe Rust wrapper)      │
//! ├─────────────────────────────────────┤
//! │  LoraIo  (packet ↔ byte-stream)     │
//! ├─────────────────────────────────────┤
//! │  impl PacketRadio (user implemented)│
//! │  e.g. sx126x crate + esp-hal SPI    │
//! └─────────────────────────────────────┘
//! ```
//!
//! Implement [`PacketRadio`] for your radio hardware.  Wrap it in
//! [`LoraIo`] to provide the byte-stream interface that the mesh
//! protocol needs, then use [`MeshNode`] to send and receive.

#![no_std]

mod radio;
pub use radio::PacketRadio;

mod adapter;
pub use adapter::LoraIo;

mod mesh;
pub use mesh::{MeshMessage, MeshNode};

// Re-export types that users will need from embedded-nano-mesh.
pub use embedded_nano_mesh::{LifeTimeType, SendError};
