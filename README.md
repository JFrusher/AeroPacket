# aeropacket

Bit-packed LZ/DZ telemetry for slow radio links (LoRa, VHF/UHF modems, 2.4 kbps).
`no_std`, no allocation, no `unsafe`.

## Wire format

```
frame = COBS( payload ‖ CRC-16/X.25 little-endian ) ‖ 0x00
payload = 40-byte core ‖ 0–5 obstacle records × 4 bytes
```

- All multi-byte fields little-endian. Full field table: `src/packet.rs` module docs.
- Max frame is **64 bytes including delimiter** (5 obstacles). Enforced at compile time.
- Receiver splits stream on `0x00`, passes each chunk to `codec::decode_frame`.

## Use

```rust
use aeropacket::{codec, Packet};
let mut frame = [0u8; codec::MAX_FRAME_LEN];
let n = codec::encode_frame(&Packet::default(), &mut frame)?;
let back = codec::decode_frame(&frame[..n - 1])?; // strip delimiter
```

## CLI

```sh
cargo run -- encode examples/sample.json   # -> hex frame
cargo run -- decode <hex>                  # -> JSON (human units)
```

## Check

```sh
cargo test
cargo clippy --all-targets -- -D warnings
cargo build --lib --no-default-features --target thumbv7em-none-eabihf
cd fuzz && cargo +nightly fuzz run decode_frame -- -max_total_time=60
```
