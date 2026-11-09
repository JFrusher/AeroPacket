//! LZ/DZ packet data model and its fixed little-endian wire layout.
//!
//! The struct is a plain Rust struct; the wire layout is defined by
//! [`Packet::to_bytes`] / [`Packet::from_bytes`], not by memory layout, so no
//! `unsafe` casts are needed and field order in memory is irrelevant.
//!
//! ## Static core — 40 bytes, all multi-byte fields little-endian
//!
//! | Off | Size | Field                 | Units / encoding                     |
//! |-----|------|-----------------------|--------------------------------------|
//! |  0  | 1    | `packet_id`           |                                      |
//! |  1  | 1    | `source_node`         |                                      |
//! |  2  | 2    | `timestamp_sec`       | u16 seconds                          |
//! |  4  | 2    | `status_flags`        | bits in [`status`]                   |
//! |  6  | 4    | `target_lat`          | i32, degrees × 10⁷                   |
//! | 10  | 4    | `target_lon`          | i32, degrees × 10⁷                   |
//! | 14  | 2    | `target_alt_cm`       | i16, cm                              |
//! | 16  | 1    | `lz_radius_m`         | m                                    |
//! | 17  | 1    | `ground_slope_pct`    | %                                    |
//! | 18  | 4    | `spatial_reserved`    | pads spatial block to 16             |
//! | 22  | 1    | `surface_wind_dir`    | 0–255 ↦ 0–360°                       |
//! | 23  | 1    | `surface_wind_spd`    | kt                                   |
//! | 24  | 1    | `gust_speed`          | kt                                   |
//! | 25  | 1    | `mew_vector_dir`      | 0–255 ↦ 0–360°                       |
//! | 26  | 1    | `mew_vector_spd`      | kt                                   |
//! | 27  | 2    | `baro_pressure_qnh`   | u16, tenths of hPa                   |
//! | 29  | 1    | `air_temp_c`          | i8, °C                               |
//! | 30  | 1    | `surface_condition`   | [`SurfaceCondition`]                 |
//! | 31  | 1    | `atmos_reserved`      | pads atmospheric block to 10         |
//! | 32  | 1    | `primary_heading`     | 0–255 ↦ 0–360°                       |
//! | 33  | 1    | `obstacle_count`      | 0–[`MAX_OBSTACLES`], = records sent  |
//! | 34  | 1    | `max_obstacle_ht_m`   | m                                    |
//! | 35  | 5    | `reserved`            |                                      |
//!
//! ## Obstacle records — 4 bytes each, follow the core
//!
//! `distance_m` (m), `bearing` (0–255 ↦ 0–360°), `height_agl_m` (m), `type_flag` ([`ObstacleType`]).

use crate::AeropacketError;

/// Size of the static core in bytes.
pub const CORE_LEN: usize = 40;
/// Size of one obstacle record in bytes.
pub const OBSTACLE_LEN: usize = 4;
/// Most obstacle records a packet can carry. Set so a full frame
/// (core + records + CRC + COBS overhead + delimiter) fits in 64 bytes.
pub const MAX_OBSTACLES: usize = 5;
/// Largest serialized payload (before CRC and framing).
pub const MAX_PAYLOAD_LEN: usize = CORE_LEN + MAX_OBSTACLES * OBSTACLE_LEN;

/// Bit meanings for [`Packet::status_flags`]. Undefined bits are rejected on decode.
pub mod status {
    /// Lat/lon/alt come from a valid fix.
    pub const POSITION_VALID: u16 = 1 << 0;
    /// Wind fields are measured, not estimated.
    pub const WIND_VALID: u16 = 1 << 1;
    /// Zone is under or near enemy contact.
    pub const ZONE_HOT: u16 = 1 << 2;
    /// Zone is visibly marked (panel, smoke).
    pub const ZONE_MARKED: u16 = 1 << 3;
    /// Zone is marked for night-vision (IR strobe/chemlight).
    pub const IR_MARKED: u16 = 1 << 4;
    /// Loose surface — brownout/whiteout risk on approach.
    pub const BROWNOUT_RISK: u16 = 1 << 5;
    /// Wires or cables in the approach area.
    pub const WIRES_IN_AREA: u16 = 1 << 6;
    /// Do not use this zone.
    pub const ABORT: u16 = 1 << 7;

    /// Every defined bit.
    pub const KNOWN_MASK: u16 = 0x00FF;

    /// Name ↔ bit table, in bit order (for UIs and the CLI).
    pub const NAMES: [(&str, u16); 8] = [
        ("position_valid", POSITION_VALID),
        ("wind_valid", WIND_VALID),
        ("zone_hot", ZONE_HOT),
        ("zone_marked", ZONE_MARKED),
        ("ir_marked", IR_MARKED),
        ("brownout_risk", BROWNOUT_RISK),
        ("wires_in_area", WIRES_IN_AREA),
        ("abort", ABORT),
    ];
}

/// Ground surface at the zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "std", serde(rename_all = "snake_case"))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum SurfaceCondition {
    #[default]
    Unknown = 0,
    DryGrass = 1,
    WetGrass = 2,
    Dirt = 3,
    Gravel = 4,
    Sand = 5,
    Paved = 6,
    Mud = 7,
    Snow = 8,
    Ice = 9,
    Water = 10,
}

impl TryFrom<u8> for SurfaceCondition {
    type Error = AeropacketError;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        use SurfaceCondition::*;
        Ok(match v {
            0 => Unknown,
            1 => DryGrass,
            2 => WetGrass,
            3 => Dirt,
            4 => Gravel,
            5 => Sand,
            6 => Paved,
            7 => Mud,
            8 => Snow,
            9 => Ice,
            10 => Water,
            _ => return Err(AeropacketError::InvalidSurfaceCondition(v)),
        })
    }
}

/// Kind of obstacle in an [`Obstacle`] record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "std", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "std", serde(rename_all = "snake_case"))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum ObstacleType {
    #[default]
    Unknown = 0,
    Tree = 1,
    PowerLine = 2,
    Tower = 3,
    Building = 4,
    Vehicle = 5,
    Terrain = 6,
    Fence = 7,
}

impl TryFrom<u8> for ObstacleType {
    type Error = AeropacketError;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        use ObstacleType::*;
        Ok(match v {
            0 => Unknown,
            1 => Tree,
            2 => PowerLine,
            3 => Tower,
            4 => Building,
            5 => Vehicle,
            6 => Terrain,
            7 => Fence,
            _ => return Err(AeropacketError::InvalidObstacleType(v)),
        })
    }
}

/// One compact obstacle record (4 bytes on the wire).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Obstacle {
    /// Distance from zone centre, metres.
    pub distance_m: u8,
    /// Bearing from zone centre, 0–255 ↦ 0–360°.
    pub bearing: u8,
    /// Height above ground, metres.
    pub height_agl_m: u8,
    /// Obstacle kind.
    pub type_flag: ObstacleType,
}

/// A full LZ/DZ telemetry packet. See the [module docs](self) for the wire layout.
///
/// Only the first `obstacle_count` entries of `obstacles` are sent; the rest must
/// stay `Obstacle::default()` or a decoded copy will not compare equal.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)] // field meanings live in the layout table above
pub struct Packet {
    // Header
    pub packet_id: u8,
    pub source_node: u8,
    pub timestamp_sec: u16,
    pub status_flags: u16,
    // Spatial core
    pub target_lat: i32,
    pub target_lon: i32,
    pub target_alt_cm: i16,
    pub lz_radius_m: u8,
    pub ground_slope_pct: u8,
    pub spatial_reserved: [u8; 4],
    // Atmospheric & drift
    pub surface_wind_dir: u8,
    pub surface_wind_spd: u8,
    pub gust_speed: u8,
    pub mew_vector_dir: u8,
    pub mew_vector_spd: u8,
    pub baro_pressure_qnh: u16,
    pub air_temp_c: i8,
    pub surface_condition: SurfaceCondition,
    pub atmos_reserved: u8,
    // Hazards & approach
    pub primary_heading: u8,
    pub obstacle_count: u8,
    pub max_obstacle_ht_m: u8,
    pub reserved: [u8; 5],
    // Dynamic hazards
    pub obstacles: [Obstacle; MAX_OBSTACLES],
}

impl Packet {
    /// The obstacle records that go on the wire.
    ///
    /// Panics if `obstacle_count` > [`MAX_OBSTACLES`]; [`Packet::to_bytes`] checks first.
    pub fn active_obstacles(&self) -> &[Obstacle] {
        &self.obstacles[..self.obstacle_count as usize]
    }

    /// Serialized payload length: `CORE_LEN + obstacle_count * OBSTACLE_LEN`.
    pub fn encoded_len(&self) -> usize {
        CORE_LEN + self.obstacle_count as usize * OBSTACLE_LEN
    }

    /// Write the payload into `buf`, returning bytes written.
    ///
    /// Errors: `MalformedObstacleData` if `obstacle_count > MAX_OBSTACLES`,
    /// `InvalidStatusFlags` if undefined bits are set, `BufferTooShort` if `buf` is too small.
    pub fn to_bytes(&self, buf: &mut [u8]) -> Result<usize, AeropacketError> {
        if self.obstacle_count as usize > MAX_OBSTACLES {
            return Err(AeropacketError::MalformedObstacleData);
        }
        if self.status_flags & !status::KNOWN_MASK != 0 {
            return Err(AeropacketError::InvalidStatusFlags(self.status_flags));
        }
        let len = self.encoded_len();
        let b = buf.get_mut(..len).ok_or(AeropacketError::BufferTooShort)?;

        b[0] = self.packet_id;
        b[1] = self.source_node;
        b[2..4].copy_from_slice(&self.timestamp_sec.to_le_bytes());
        b[4..6].copy_from_slice(&self.status_flags.to_le_bytes());
        b[6..10].copy_from_slice(&self.target_lat.to_le_bytes());
        b[10..14].copy_from_slice(&self.target_lon.to_le_bytes());
        b[14..16].copy_from_slice(&self.target_alt_cm.to_le_bytes());
        b[16] = self.lz_radius_m;
        b[17] = self.ground_slope_pct;
        b[18..22].copy_from_slice(&self.spatial_reserved);
        b[22] = self.surface_wind_dir;
        b[23] = self.surface_wind_spd;
        b[24] = self.gust_speed;
        b[25] = self.mew_vector_dir;
        b[26] = self.mew_vector_spd;
        b[27..29].copy_from_slice(&self.baro_pressure_qnh.to_le_bytes());
        b[29] = self.air_temp_c as u8;
        b[30] = self.surface_condition as u8;
        b[31] = self.atmos_reserved;
        b[32] = self.primary_heading;
        b[33] = self.obstacle_count;
        b[34] = self.max_obstacle_ht_m;
        b[35..40].copy_from_slice(&self.reserved);

        for (o, rec) in self
            .active_obstacles()
            .iter()
            .zip(b[CORE_LEN..].chunks_exact_mut(OBSTACLE_LEN))
        {
            rec.copy_from_slice(&[o.distance_m, o.bearing, o.height_agl_m, o.type_flag as u8]);
        }
        Ok(len)
    }

    /// Parse a payload. `bytes` must be exactly the payload (no CRC, no framing).
    ///
    /// Errors: `BufferTooShort` if the core or the declared records are cut off,
    /// `MalformedObstacleData` if the count is over the limit or trailing bytes remain,
    /// plus enum/flag validation errors.
    pub fn from_bytes(bytes: &[u8]) -> Result<Self, AeropacketError> {
        let b = bytes.get(..CORE_LEN).ok_or(AeropacketError::BufferTooShort)?;
        let obstacle_count = b[33];
        if obstacle_count as usize > MAX_OBSTACLES {
            return Err(AeropacketError::MalformedObstacleData);
        }
        let expected = CORE_LEN + obstacle_count as usize * OBSTACLE_LEN;
        if bytes.len() < expected {
            return Err(AeropacketError::BufferTooShort);
        }
        if bytes.len() > expected {
            return Err(AeropacketError::MalformedObstacleData);
        }
        let status_flags = u16::from_le_bytes([b[4], b[5]]);
        if status_flags & !status::KNOWN_MASK != 0 {
            return Err(AeropacketError::InvalidStatusFlags(status_flags));
        }

        let mut obstacles = [Obstacle::default(); MAX_OBSTACLES];
        for (o, rec) in obstacles
            .iter_mut()
            .zip(bytes[CORE_LEN..].chunks_exact(OBSTACLE_LEN))
        {
            *o = Obstacle {
                distance_m: rec[0],
                bearing: rec[1],
                height_agl_m: rec[2],
                type_flag: ObstacleType::try_from(rec[3])?,
            };
        }

        Ok(Self {
            packet_id: b[0],
            source_node: b[1],
            timestamp_sec: u16::from_le_bytes([b[2], b[3]]),
            status_flags,
            target_lat: i32::from_le_bytes([b[6], b[7], b[8], b[9]]),
            target_lon: i32::from_le_bytes([b[10], b[11], b[12], b[13]]),
            target_alt_cm: i16::from_le_bytes([b[14], b[15]]),
            lz_radius_m: b[16],
            ground_slope_pct: b[17],
            spatial_reserved: [b[18], b[19], b[20], b[21]],
            surface_wind_dir: b[22],
            surface_wind_spd: b[23],
            gust_speed: b[24],
            mew_vector_dir: b[25],
            mew_vector_spd: b[26],
            baro_pressure_qnh: u16::from_le_bytes([b[27], b[28]]),
            air_temp_c: b[29] as i8,
            surface_condition: SurfaceCondition::try_from(b[30])?,
            atmos_reserved: b[31],
            primary_heading: b[32],
            obstacle_count,
            max_obstacle_ht_m: b[34],
            reserved: [b[35], b[36], b[37], b[38], b[39]],
            obstacles,
        })
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A packet with every field distinct and non-zero, full obstacle load.
    pub(crate) fn sample() -> Packet {
        let mut p = Packet {
            packet_id: 0x11,
            source_node: 0x22,
            timestamp_sec: 0x3344,
            status_flags: status::POSITION_VALID | status::ZONE_MARKED | status::ABORT,
            target_lat: -339_123_456,
            target_lon: 1_512_345_678,
            target_alt_cm: -1234,
            lz_radius_m: 50,
            ground_slope_pct: 7,
            spatial_reserved: [0xA1, 0xA2, 0xA3, 0xA4],
            surface_wind_dir: 200,
            surface_wind_spd: 12,
            gust_speed: 18,
            mew_vector_dir: 190,
            mew_vector_spd: 9,
            baro_pressure_qnh: 10132,
            air_temp_c: -5,
            surface_condition: SurfaceCondition::Snow,
            atmos_reserved: 0xB1,
            primary_heading: 64,
            obstacle_count: MAX_OBSTACLES as u8,
            max_obstacle_ht_m: 30,
            reserved: [0xC1, 0xC2, 0xC3, 0xC4, 0xC5],
            obstacles: [Obstacle::default(); MAX_OBSTACLES],
        };
        for (i, o) in p.obstacles.iter_mut().enumerate() {
            let i = i as u8;
            *o = Obstacle {
                distance_m: 100 + i,
                bearing: 10 * i,
                height_agl_m: 5 + i,
                type_flag: ObstacleType::try_from(i + 1).unwrap(),
            };
        }
        p
    }

    #[test]
    fn round_trip_every_obstacle_count() {
        for n in 0..=MAX_OBSTACLES {
            let mut p = sample();
            p.obstacle_count = n as u8;
            p.obstacles[n..].fill(Obstacle::default());
            let mut buf = [0u8; MAX_PAYLOAD_LEN];
            let len = p.to_bytes(&mut buf).unwrap();
            assert_eq!(len, CORE_LEN + n * OBSTACLE_LEN);
            assert_eq!(Packet::from_bytes(&buf[..len]).unwrap(), p);
        }
    }

    /// Pins every field to its documented offset so the layout cannot drift.
    #[test]
    fn wire_layout_golden() {
        let mut p = sample();
        p.obstacle_count = 1;
        p.obstacles[1..].fill(Obstacle::default());
        let mut buf = [0u8; MAX_PAYLOAD_LEN];
        let len = p.to_bytes(&mut buf).unwrap();
        #[rustfmt::skip]
        let expected: [u8; 44] = [
            0x11, 0x22, 0x44, 0x33, 0x89, 0x00,             // header
            0x00, 0x63, 0xC9, 0xEB,                         // lat -339123456
            0x4E, 0x90, 0x24, 0x5A,                         // lon 1512345678
            0x2E, 0xFB, 50, 7, 0xA1, 0xA2, 0xA3, 0xA4,      // alt -1234, radius, slope, pad
            200, 12, 18, 190, 9, 0x94, 0x27, 0xFB, 8, 0xB1, // atmos: qnh 10132, temp -5, snow, pad
            64, 1, 30, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5,        // hazards
            100, 0, 5, 1,                                   // obstacle 0
        ];
        assert_eq!(&buf[..len], &expected);
    }

    #[test]
    fn truncated_core_is_buffer_too_short() {
        assert_eq!(Packet::from_bytes(&[0u8; CORE_LEN - 1]), Err(AeropacketError::BufferTooShort));
    }

    #[test]
    fn truncated_obstacles_is_buffer_too_short() {
        let mut buf = [0u8; MAX_PAYLOAD_LEN];
        let len = sample().to_bytes(&mut buf).unwrap();
        assert_eq!(Packet::from_bytes(&buf[..len - 1]), Err(AeropacketError::BufferTooShort));
    }

    #[test]
    fn trailing_bytes_are_malformed() {
        let mut p = sample();
        p.obstacle_count = 2;
        let mut buf = [0u8; MAX_PAYLOAD_LEN];
        let len = p.to_bytes(&mut buf).unwrap();
        assert_eq!(Packet::from_bytes(&buf[..len + 1]), Err(AeropacketError::MalformedObstacleData));
    }

    #[test]
    fn count_over_limit_is_malformed() {
        let mut buf = [0u8; MAX_PAYLOAD_LEN + OBSTACLE_LEN];
        buf[33] = MAX_OBSTACLES as u8 + 1;
        assert_eq!(Packet::from_bytes(&buf), Err(AeropacketError::MalformedObstacleData));
        let mut p = sample();
        p.obstacle_count = MAX_OBSTACLES as u8 + 1;
        assert_eq!(p.to_bytes(&mut buf), Err(AeropacketError::MalformedObstacleData));
    }

    #[test]
    fn bad_enums_and_flags_rejected() {
        let mut buf = [0u8; MAX_PAYLOAD_LEN];
        let len = sample().to_bytes(&mut buf).unwrap();

        let mut b = buf;
        b[30] = 0xEE;
        assert_eq!(Packet::from_bytes(&b[..len]), Err(AeropacketError::InvalidSurfaceCondition(0xEE)));

        let mut b = buf;
        b[CORE_LEN + 3] = 0xEE;
        assert_eq!(Packet::from_bytes(&b[..len]), Err(AeropacketError::InvalidObstacleType(0xEE)));

        let mut b = buf;
        b[5] = 0x01;
        assert!(matches!(Packet::from_bytes(&b[..len]), Err(AeropacketError::InvalidStatusFlags(_))));
    }

    #[test]
    fn small_output_buffer_rejected() {
        let mut buf = [0u8; CORE_LEN];
        assert_eq!(sample().to_bytes(&mut buf), Err(AeropacketError::BufferTooShort));
    }
}
