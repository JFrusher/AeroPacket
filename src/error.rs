//! Error type shared by the packet and codec layers.

use core::fmt;

/// Every way encoding or decoding can fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AeropacketError {
    /// CRC in the frame does not match the payload.
    InvalidChecksum,
    /// Input or output buffer is smaller than the data needs.
    BufferTooShort,
    /// `obstacle_count` is above the limit or does not match the record bytes present.
    MalformedObstacleData,
    /// `surface_condition` byte is not a known [`crate::SurfaceCondition`].
    InvalidSurfaceCondition(u8),
    /// Obstacle `type_flag` byte is not a known [`crate::ObstacleType`].
    InvalidObstacleType(u8),
    /// `status_flags` has bits set that are not defined in [`crate::packet::status`].
    InvalidStatusFlags(u16),
    /// Frame is not valid COBS, contains a delimiter, or is longer than a max frame.
    InvalidFrame,
}

impl fmt::Display for AeropacketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidChecksum => f.write_str("CRC mismatch"),
            Self::BufferTooShort => f.write_str("buffer too short"),
            Self::MalformedObstacleData => f.write_str("obstacle count does not match records"),
            Self::InvalidSurfaceCondition(v) => write!(f, "unknown surface condition {v}"),
            Self::InvalidObstacleType(v) => write!(f, "unknown obstacle type {v}"),
            Self::InvalidStatusFlags(v) => write!(f, "undefined status flag bits {v:#06x}"),
            Self::InvalidFrame => f.write_str("invalid COBS frame"),
        }
    }
}

impl core::error::Error for AeropacketError {}
