# ft6236

[![Crates.io](https://img.shields.io/crates/v/ft6236.svg)](https://crates.io/crates/ft6236)
[![Documentation](https://docs.rs/ft6236/badge.svg)](https://docs.rs/ft6236)
[![License](https://img.shields.io/crates/l/ft6236.svg)](https://github.com/embedded-drivers/ft6236)

Platform-agnostic Rust driver for the FT6236, FT6206, and FT6236U capacitive touch screen controllers, built on top of [`embedded-hal`](https://crates.io/crates/embedded-hal) traits.

## Supported Devices

| Chip | Chip ID |
|------|---------|
| FT6206 | `0x06` |
| FT6236 | `0x36` |
| FT6236U | `0x64` |

## Features

- Multi-touch support (up to 2 simultaneous touch points)
- Touch event detection (press down, lift up, contact)
- Gesture recognition (swipe up/down/left/right, zoom in/out)
- Configurable touch detection threshold
- Hardware reset with proper timing sequence
- Optional [`defmt`](https://github.com/knurling-rs/defmt) support for logging

## Usage

Add the dependency to your `Cargo.toml`:

```toml
[dependencies]
ft6236 = "0.1"
```

To enable `defmt` logging:

```toml
[dependencies]
ft6236 = { version = "0.1", features = ["defmt"] }
```

### Example

```rust,ignore
use ft6236::{FT6236, Config};

// Initialize the touch controller
let mut touch = FT6236::new(i2c);

// Reset the controller (optional, requires a reset pin)
touch.reset(&mut rst_pin, &mut delay).unwrap();

// Initialize with default config (threshold = 0x40)
touch.init(Config::default()).unwrap();

// Read touch points in your main loop
loop {
    if let Some(point) = touch.get_point0().unwrap() {
        // point.x, point.y - touch coordinates
        // point.event - PressDown, LiftUp, or Contact
    }

    if let Some(gesture) = touch.get_gesture().unwrap() {
        // Handle gesture (MoveUp, MoveDown, MoveLeft, MoveRight, ZoomIn, ZoomOut)
    }
}
```

## Note

- `INTn` (interrupt output) is not managed by this driver. You need to configure the interrupt pin externally if you want interrupt-driven touch detection.
- The I2C default address is `0x38`.

## Minimum Supported Rust Version (MSRV)

This crate requires Rust 1.75 or later (due to `embedded-hal` 1.0 requirements).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
