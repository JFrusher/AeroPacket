//! Emulated noisy radio link: encode → byte stream → channel noise → FrameReader.
//!
//! Exact check: the reader must deliver precisely the frames the channel left
//! intact (and whose leading delimiter survived), in order, and nothing else.

use aeropacket::codec::{self, FrameReader};
use aeropacket::packet::MAX_OBSTACLES;
use aeropacket::{Obstacle, ObstacleType, Packet, SurfaceCondition};

/// xorshift64: deterministic, no dependency.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn byte(&mut self) -> u8 {
        self.next() as u8
    }
    /// True with probability `per_million` / 1e6.
    fn chance(&mut self, per_million: u64) -> bool {
        self.next() % 1_000_000 < per_million
    }
}

fn random_packet(rng: &mut Rng, id: u16) -> Packet {
    let count = (rng.next() % (MAX_OBSTACLES as u64 + 1)) as usize;
    let mut p = Packet {
        packet_id: id as u8,
        source_node: rng.byte(),
        timestamp_sec: id, // unique per packet so a mix-up cannot pass as a match
        status_flags: rng.byte() as u16,
        target_lat: rng.next() as i32,
        target_lon: rng.next() as i32,
        target_alt_cm: rng.next() as i16,
        lz_radius_m: rng.byte(),
        surface_wind_dir: rng.byte(),
        baro_pressure_qnh: rng.next() as u16,
        air_temp_c: rng.byte() as i8,
        surface_condition: SurfaceCondition::try_from(rng.byte() % 11).unwrap(),
        obstacle_count: count as u8,
        ..Packet::default()
    };
    for o in &mut p.obstacles[..count] {
        *o = Obstacle {
            distance_m: rng.byte(),
            bearing: rng.byte(),
            height_agl_m: rng.byte(),
            type_flag: ObstacleType::try_from(rng.byte() % 8).unwrap(),
        };
    }
    p
}

/// Run `n` packets through a channel with the given error rates (per million bytes).
/// Returns (expected, received, frames_hit).
fn run_link(seed: u64, n: u16, flip: u64, drop: u64, insert: u64) -> (Vec<Packet>, Vec<Packet>, usize) {
    let mut rng = Rng(seed);
    let mut stream = Vec::new();
    let mut expected = Vec::new();
    let mut hit = 0;

    // Receiver powers up mid-transmission: junk with no leading delimiter.
    stream.extend((0..17).map(|_| rng.byte() | 1));
    let mut prev_delim_ok = false;

    for id in 0..n {
        let p = random_packet(&mut rng, id);
        let mut frame = [0u8; codec::MAX_FRAME_LEN];
        let len = codec::encode_frame(&p, &mut frame).unwrap();

        let mut touched = false;
        let mut delim_ok = true;
        for (i, &b) in frame[..len].iter().enumerate() {
            let is_delim = i == len - 1;
            if rng.chance(insert) {
                stream.push(rng.byte());
                touched = true;
            }
            if rng.chance(drop) {
                touched = true;
                delim_ok &= !is_delim;
                continue;
            }
            let b = if rng.chance(flip) { b ^ (1 << (rng.next() % 8)) } else { b };
            touched |= b != frame[i];
            delim_ok &= !is_delim || b == 0;
            stream.push(b);
        }
        // An inserted 0x00 would also split a frame; any insert already marks it touched.
        if touched {
            hit += 1;
        } else if prev_delim_ok {
            expected.push(p);
        }
        prev_delim_ok = delim_ok;
    }

    let mut reader = FrameReader::new();
    let received = stream
        .iter()
        .filter_map(|&b| reader.push(b))
        .filter_map(Result::ok)
        .collect();
    (expected, received, hit)
}

#[test]
fn clean_link_delivers_everything_after_resync() {
    let (expected, received, hit) = run_link(1, 500, 0, 0, 0);
    assert_eq!(hit, 0);
    assert_eq!(expected.len(), 499, "first frame merges with power-up junk");
    assert_eq!(received, expected);
}

#[test]
fn noisy_link_delivers_exactly_the_intact_frames() {
    // ~1e-3 bit flips, ~2e-4 drops and inserts per byte: a bad VHF link.
    for seed in 1..=20 {
        let (expected, received, hit) = run_link(seed, 2000, 1_000, 200, 200);
        assert!(hit > 50, "seed {seed}: channel too clean to prove anything ({hit} hit)");
        assert_eq!(received, expected, "seed {seed}: {hit} frames hit");
    }
}
