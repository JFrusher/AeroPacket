//! # aeropacket
//!
//! Bit-packed Landing Zone / Drop Zone telemetry for constrained radio links
//! (LoRa, VHF/UHF audio modems, 2.4 kbps channels).
//!
//! * [`packet`] — the [`Packet`] data model and its fixed little-endian wire layout.
//! * [`codec`] — CRC-16/X.25 + COBS framing. One encoded frame is at most
//!   [`codec::MAX_FRAME_LEN`] (64) bytes including the `0x00` delimiter.
//!
//! The core is `#![no_std]` and allocation-free. The `std` feature (default) adds
//! serde derives and the JSON CLI.
//!
//! ```
//! use aeropacket::{codec, Packet};
//!
//! let packet = Packet::default();
//! let mut frame = [0u8; codec::MAX_FRAME_LEN];
//! let n = codec::encode_frame(&packet, &mut frame).unwrap();
//! // frame[n - 1] is the 0x00 delimiter; strip it before decoding.
//! let decoded = codec::decode_frame(&frame[..n - 1]).unwrap();
//! assert_eq!(decoded, packet);
//! ```

#![no_std]
#![deny(missing_docs)]

pub mod codec;
pub mod error;
pub mod packet;

pub use error::AeropacketError;

// Compiles and runs the README's Rust examples under `cargo test`.
#[cfg(doctest)]
#[doc = include_str!("../README.md")]
struct ReadmeDoctests;
pub use packet::{Obstacle, ObstacleType, Packet, SurfaceCondition};
