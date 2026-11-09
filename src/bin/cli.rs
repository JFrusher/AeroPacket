//! `aeropacket` CLI.
//!
//! ```text
//! aeropacket encode <packet.json>   # prints the wire frame as hex (incl. trailing 00)
//! aeropacket decode <hex>           # prints the packet as JSON
//! ```
//!
//! JSON uses human units (degrees, metres, hPa); this file converts them to the
//! fixed-point wire fields and back.

use aeropacket::packet::{MAX_OBSTACLES, status};
use aeropacket::{Obstacle, ObstacleType, Packet, SurfaceCondition, codec};
use serde::{Deserialize, Serialize};
use std::process::ExitCode;

#[derive(Serialize, Deserialize, Debug)]
#[serde(deny_unknown_fields)]
struct PacketJson {
    packet_id: u8,
    source_node: u8,
    timestamp_sec: u16,
    status: Vec<String>,
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f64,
    lz_radius_m: u8,
    ground_slope_pct: u8,
    wind_dir_deg: f64,
    wind_kt: u8,
    gust_kt: u8,
    mew_dir_deg: f64,
    mew_kt: u8,
    qnh_hpa: f64,
    air_temp_c: i8,
    surface: SurfaceCondition,
    primary_heading_deg: f64,
    max_obstacle_ht_m: u8,
    obstacles: Vec<ObstacleJson>,
    // Optional on input (spec says zero); always printed so decode → encode keeps every byte.
    #[serde(default)]
    spatial_reserved: [u8; 4],
    #[serde(default)]
    atmos_reserved: u8,
    #[serde(default)]
    reserved: [u8; 5],
}

#[derive(Serialize, Deserialize, Debug)]
#[serde(deny_unknown_fields)]
struct ObstacleJson {
    distance_m: u8,
    bearing_deg: f64,
    height_agl_m: u8,
    #[serde(rename = "type")]
    kind: ObstacleType,
}

/// Round `v * scale` into `T`, failing if it does not fit.
fn scaled<T: TryFrom<i64>>(name: &str, v: f64, scale: f64) -> Result<T, String> {
    let r = (v * scale).round();
    if !r.is_finite() {
        return Err(format!("{name}: {v} is not a number"));
    }
    T::try_from(r as i64).map_err(|_| format!("{name}: {v} out of range"))
}

/// Degrees [0, 360) → wire byte (256 steps per turn, ~1.4° resolution).
fn deg_to_byte(name: &str, deg: f64) -> Result<u8, String> {
    if !(0.0..360.0).contains(&deg) {
        return Err(format!("{name}: {deg} must be in [0, 360)"));
    }
    Ok(((deg * 256.0 / 360.0).round() as u16 % 256) as u8)
}

fn byte_to_deg(b: u8) -> f64 {
    b as f64 * 360.0 / 256.0
}

fn to_packet(j: &PacketJson) -> Result<Packet, String> {
    if !(-90.0..=90.0).contains(&j.lat_deg) || !(-180.0..=180.0).contains(&j.lon_deg) {
        return Err("lat_deg/lon_deg out of range".into());
    }
    if j.obstacles.len() > MAX_OBSTACLES {
        return Err(format!("at most {MAX_OBSTACLES} obstacles fit in a frame"));
    }
    let mut status_flags = 0u16;
    for name in &j.status {
        let (_, bit) = status::NAMES
            .iter()
            .find(|(n, _)| n == name)
            .ok_or_else(|| format!("unknown status flag {name:?}"))?;
        status_flags |= bit;
    }
    let mut obstacles = [Obstacle::default(); MAX_OBSTACLES];
    for (o, oj) in obstacles.iter_mut().zip(&j.obstacles) {
        *o = Obstacle {
            distance_m: oj.distance_m,
            bearing: deg_to_byte("bearing_deg", oj.bearing_deg)?,
            height_agl_m: oj.height_agl_m,
            type_flag: oj.kind,
        };
    }
    Ok(Packet {
        packet_id: j.packet_id,
        source_node: j.source_node,
        timestamp_sec: j.timestamp_sec,
        status_flags,
        target_lat: scaled("lat_deg", j.lat_deg, 1e7)?,
        target_lon: scaled("lon_deg", j.lon_deg, 1e7)?,
        target_alt_cm: scaled("alt_m", j.alt_m, 100.0)?,
        lz_radius_m: j.lz_radius_m,
        ground_slope_pct: j.ground_slope_pct,
        surface_wind_dir: deg_to_byte("wind_dir_deg", j.wind_dir_deg)?,
        surface_wind_spd: j.wind_kt,
        gust_speed: j.gust_kt,
        mew_vector_dir: deg_to_byte("mew_dir_deg", j.mew_dir_deg)?,
        mew_vector_spd: j.mew_kt,
        baro_pressure_qnh: scaled("qnh_hpa", j.qnh_hpa, 10.0)?,
        air_temp_c: j.air_temp_c,
        surface_condition: j.surface,
        primary_heading: deg_to_byte("primary_heading_deg", j.primary_heading_deg)?,
        obstacle_count: j.obstacles.len() as u8,
        max_obstacle_ht_m: j.max_obstacle_ht_m,
        obstacles,
        spatial_reserved: j.spatial_reserved,
        atmos_reserved: j.atmos_reserved,
        reserved: j.reserved,
    })
}

fn from_packet(p: &Packet) -> PacketJson {
    PacketJson {
        packet_id: p.packet_id,
        source_node: p.source_node,
        timestamp_sec: p.timestamp_sec,
        status: status::NAMES
            .iter()
            .filter(|(_, bit)| p.status_flags & bit != 0)
            .map(|(n, _)| n.to_string())
            .collect(),
        lat_deg: p.target_lat as f64 / 1e7,
        lon_deg: p.target_lon as f64 / 1e7,
        alt_m: p.target_alt_cm as f64 / 100.0,
        lz_radius_m: p.lz_radius_m,
        ground_slope_pct: p.ground_slope_pct,
        wind_dir_deg: byte_to_deg(p.surface_wind_dir),
        wind_kt: p.surface_wind_spd,
        gust_kt: p.gust_speed,
        mew_dir_deg: byte_to_deg(p.mew_vector_dir),
        mew_kt: p.mew_vector_spd,
        qnh_hpa: p.baro_pressure_qnh as f64 / 10.0,
        air_temp_c: p.air_temp_c,
        surface: p.surface_condition,
        primary_heading_deg: byte_to_deg(p.primary_heading),
        max_obstacle_ht_m: p.max_obstacle_ht_m,
        obstacles: p
            .active_obstacles()
            .iter()
            .map(|o| ObstacleJson {
                distance_m: o.distance_m,
                bearing_deg: byte_to_deg(o.bearing),
                height_agl_m: o.height_agl_m,
                kind: o.type_flag,
            })
            .collect(),
        spatial_reserved: p.spatial_reserved,
        atmos_reserved: p.atmos_reserved,
        reserved: p.reserved,
    }
}

fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.len().is_multiple_of(2) {
        return Err("hex has odd length".into());
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).map_err(|e| format!("bad hex: {e}")))
        .collect()
}

fn encode(path: &str) -> Result<String, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
    let json: PacketJson = serde_json::from_str(&text).map_err(|e| format!("{path}: {e}"))?;
    let packet = to_packet(&json)?;
    let mut frame = [0u8; codec::MAX_FRAME_LEN];
    let n = codec::encode_frame(&packet, &mut frame).map_err(|e| e.to_string())?;
    Ok(frame[..n].iter().map(|b| format!("{b:02x}")).collect())
}

fn decode(hex: &str) -> Result<String, String> {
    let frame = parse_hex(hex)?;
    let body = frame
        .strip_suffix(&[codec::DELIMITER])
        .ok_or("frame must end with the 00 delimiter")?;
    let packet = codec::decode_frame(body).map_err(|e| e.to_string())?;
    serde_json::to_string_pretty(&from_packet(&packet)).map_err(|e| e.to_string())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().collect();
    let result = match args.get(1..).unwrap_or_default() {
        [cmd, arg] if cmd == "encode" => encode(arg),
        [cmd, arg] if cmd == "decode" => decode(arg),
        _ => Err("usage: aeropacket encode <packet.json> | aeropacket decode <hex>".into()),
    };
    match result {
        Ok(out) => {
            println!("{out}");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_json_round_trips_through_cli() {
        let hex = encode(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/sample.json")).unwrap();
        let back: PacketJson = serde_json::from_str(&decode(&hex).unwrap()).unwrap();
        let original: PacketJson = serde_json::from_str(include_str!("../../examples/sample.json")).unwrap();
        // Quantized fields come back exact here because sample.json uses on-grid values.
        assert_eq!(format!("{back:?}"), format!("{original:?}"));
    }

    /// Frame with spatial_reserved[0]=0xAB, atmos_reserved=0xCD, reserved[0]=0xEF.
    #[test]
    fn reserved_bytes_survive_decode_encode() {
        let frame = "060107100e2b010d63c9eb4e90245a9a103204ab01010fc80c12c00994271201cd400219ef0101010b78201901c8800f02d7d800";
        let path = std::env::temp_dir().join("aeropacket_reserved.json");
        std::fs::write(&path, decode(frame).unwrap()).unwrap();
        assert_eq!(encode(path.to_str().unwrap()).unwrap(), frame);
    }

    #[test]
    fn deg_byte_wraps_and_rejects() {
        assert_eq!(deg_to_byte("x", 0.0), Ok(0));
        assert_eq!(deg_to_byte("x", 359.9), Ok(0)); // rounds up to a full turn
        assert_eq!(deg_to_byte("x", 90.0), Ok(64));
        assert!(deg_to_byte("x", 360.0).is_err());
        assert!(deg_to_byte("x", -1.0).is_err());
    }
}
