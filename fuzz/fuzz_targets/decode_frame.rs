//! Feed arbitrary over-the-air bytes to the decoders. Must never panic.
//! Anything that decodes must re-encode to the same packet.
#![no_main]

use aeropacket::{codec, Packet};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(p) = codec::decode_frame(data) {
        let mut f = [0u8; codec::MAX_FRAME_LEN];
        let n = codec::encode_frame(&p, &mut f).expect("decoded packet must re-encode");
        assert_eq!(codec::decode_frame(&f[..n - 1]), Ok(p));
    }
    if let Ok(p) = Packet::from_bytes(data) {
        let mut b = [0u8; aeropacket::packet::MAX_PAYLOAD_LEN];
        let n = p.to_bytes(&mut b).expect("decoded packet must re-encode");
        assert_eq!(&b[..n], data);
    }
});
