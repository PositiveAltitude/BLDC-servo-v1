# BLDC servo protocol

This allocation-free `no_std` crate is the shared CAN message contract for the
BLDC servo firmware and its controllers.

Another Rust project can depend on the crate directly from this repository:

```toml
[dependencies]
bldc-servo-protocol = { git = "https://github.com/PositiveAltitude/BLDC-servo-v1.git" }
```

Message values implement `ApiEncodeDecode`. Use `api_encode()` to obtain an
eight-byte-or-smaller CAN payload and `Type::api_decode(payload)` to decode it.
The codec retains the existing big-endian, fixed-width bincode wire format.
