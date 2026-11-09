//! CRC-16/X.25 + COBS framing.
//!
//! Wire frame = `COBS(payload ‖ crc16_le) ‖ 0x00`.
//!
//! COBS removes every `0x00` from the body, so `0x00` only ever marks a frame end.
//! A receiver splits the byte stream on `0x00` and hands each chunk (without the
//! delimiter) to [`decode_frame`]. A corrupted or partial chunk fails CRC or COBS
//! checks and the next `0x00` resyncs the stream.

use crate::packet::MAX_PAYLOAD_LEN;
use crate::{AeropacketError, Packet};
use crc::{CRC_16_IBM_SDLC, Crc};

/// CRC-16/X.25 (a.k.a. IBM-SDLC): poly 0x1021, init 0xFFFF, reflected, xorout 0xFFFF.
const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_SDLC);

/// Frame delimiter byte.
pub const DELIMITER: u8 = 0x00;
/// CRC length in bytes.
pub const CRC_LEN: usize = 2;
/// Largest payload + CRC, before COBS.
const MAX_RAW_LEN: usize = MAX_PAYLOAD_LEN + CRC_LEN;
/// Largest COBS body (frame without the delimiter).
pub const MAX_BODY_LEN: usize = cobs::max_encoding_length(MAX_RAW_LEN);
/// Largest wire frame including the delimiter. Pinned at 64 bytes.
pub const MAX_FRAME_LEN: usize = MAX_BODY_LEN + 1;

const _: () = assert!(MAX_FRAME_LEN == 64, "wire frame budget is 64 bytes");

/// CRC-16/X.25 of `data`.
pub fn crc16(data: &[u8]) -> u16 {
    CRC16.checksum(data)
}

/// Encode `packet` into a complete wire frame (ending in [`DELIMITER`]).
/// Returns bytes written. `out` needs at most [`MAX_FRAME_LEN`] bytes.
pub fn encode_frame(packet: &Packet, out: &mut [u8]) -> Result<usize, AeropacketError> {
    let mut raw = [0u8; MAX_RAW_LEN];
    let len = packet.to_bytes(&mut raw)?;
    let crc = crc16(&raw[..len]);
    raw[len..len + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
    let raw = &raw[..len + CRC_LEN];

    if out.len() < cobs::max_encoding_length(raw.len()) + 1 {
        return Err(AeropacketError::BufferTooShort);
    }
    let n = cobs::encode(raw, out);
    out[n] = DELIMITER;
    Ok(n + 1)
}

/// Decode one frame body (bytes between delimiters, delimiter excluded).
pub fn decode_frame(body: &[u8]) -> Result<Packet, AeropacketError> {
    if body.is_empty() || body.len() > MAX_BODY_LEN {
        return Err(AeropacketError::InvalidFrame);
    }
    let mut raw = [0u8; MAX_RAW_LEN];
    let report = cobs::decode(body, &mut raw).map_err(|_| AeropacketError::InvalidFrame)?;
    // A 0x00 inside the body makes cobs stop early; that is not one clean frame.
    if report.parsed_size() != body.len() {
        return Err(AeropacketError::InvalidFrame);
    }
    let raw = &raw[..report.frame_size()];
    let (payload, crc) = raw
        .split_last_chunk::<CRC_LEN>()
        .ok_or(AeropacketError::BufferTooShort)?;
    if crc16(payload) != u16::from_le_bytes(*crc) {
        return Err(AeropacketError::InvalidChecksum);
    }
    Packet::from_bytes(payload)
}

/// Splits a byte stream on [`DELIMITER`] and decodes each frame.
///
/// Feed bytes one at a time (e.g. from a UART interrupt). Fixed buffer, no allocation.
/// Bytes before the first delimiter may be a partial frame and will report an error;
/// the reader is back in sync after that delimiter.
#[derive(Debug, Clone)]
pub struct FrameReader {
    buf: [u8; MAX_BODY_LEN],
    len: usize,
    overflow: bool,
}

impl Default for FrameReader {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameReader {
    /// An empty reader.
    pub const fn new() -> Self {
        Self { buf: [0; MAX_BODY_LEN], len: 0, overflow: false }
    }

    /// Push one byte. Returns `Some` when a delimiter ends a frame:
    /// the packet, or why that frame was rejected. Back-to-back delimiters
    /// (idle line) return `None`.
    pub fn push(&mut self, byte: u8) -> Option<Result<Packet, AeropacketError>> {
        if byte != DELIMITER {
            match self.buf.get_mut(self.len) {
                Some(slot) => {
                    *slot = byte;
                    self.len += 1;
                }
                None => self.overflow = true,
            }
            return None;
        }
        let result = match (self.len, self.overflow) {
            (0, false) => None,
            (_, true) => Some(Err(AeropacketError::InvalidFrame)),
            (n, false) => Some(decode_frame(&self.buf[..n])),
        };
        self.len = 0;
        self.overflow = false;
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::tests::sample;
    use crate::packet::{MAX_OBSTACLES, Obstacle};

    fn encode(p: &Packet) -> ([u8; MAX_FRAME_LEN], usize) {
        let mut f = [0u8; MAX_FRAME_LEN];
        let n = encode_frame(p, &mut f).unwrap();
        (f, n)
    }

    #[test]
    fn crc_matches_x25_check_value() {
        // Standard catalogue check value for CRC-16/IBM-SDLC over "123456789".
        assert_eq!(crc16(b"123456789"), 0x906E);
    }

    #[test]
    fn full_packet_fills_exactly_64_bytes() {
        let (f, n) = encode(&sample());
        assert_eq!(n, MAX_FRAME_LEN);
        assert_eq!(f[n - 1], DELIMITER);
        assert!(!f[..n - 1].contains(&DELIMITER));
    }

    #[test]
    fn round_trip_every_obstacle_count() {
        for k in 0..=MAX_OBSTACLES {
            let mut p = sample();
            p.obstacle_count = k as u8;
            p.obstacles[k..].fill(Obstacle::default());
            let (f, n) = encode(&p);
            assert_eq!(decode_frame(&f[..n - 1]).unwrap(), p);
        }
    }

    #[test]
    fn every_single_bit_flip_is_caught() {
        let (f, n) = encode(&sample());
        for i in 0..n - 1 {
            for bit in 0..8 {
                let mut g = f;
                g[i] ^= 1 << bit;
                assert!(decode_frame(&g[..n - 1]).is_err(), "byte {i} bit {bit}");
            }
        }
    }

    #[test]
    fn every_truncation_is_an_error() {
        let (f, n) = encode(&sample());
        for cut in 0..n - 1 {
            assert!(decode_frame(&f[..cut]).is_err(), "cut {cut}");
        }
    }

    #[test]
    fn embedded_delimiter_rejected() {
        let (mut f, n) = encode(&sample());
        f[10] = DELIMITER;
        assert_eq!(decode_frame(&f[..n - 1]), Err(AeropacketError::InvalidFrame));
    }

    #[test]
    fn oversize_body_rejected() {
        assert_eq!(decode_frame(&[1u8; MAX_BODY_LEN + 1]), Err(AeropacketError::InvalidFrame));
    }

    #[test]
    fn reader_skips_idle_and_recovers_from_overflow() {
        let (f, n) = encode(&sample());
        let mut r = FrameReader::new();
        assert!(r.push(DELIMITER).is_none());
        for _ in 0..MAX_BODY_LEN + 10 {
            assert!(r.push(0xAA).is_none());
        }
        assert_eq!(r.push(DELIMITER), Some(Err(AeropacketError::InvalidFrame)));
        let out: Option<_> = f[..n].iter().filter_map(|&b| r.push(b)).last();
        assert_eq!(out, Some(Ok(sample())));
    }

    #[test]
    fn small_output_rejected() {
        let mut f = [0u8; MAX_FRAME_LEN - 1];
        assert_eq!(encode_frame(&sample(), &mut f), Err(AeropacketError::BufferTooShort));
    }
}
