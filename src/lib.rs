//! `no_std` driver for the LDS02RR LIDAR, the 360° scanner of the 1st-gen
//! Xiaomi / Roborock robot vacuums. Its packet format is compatible with the
//! Neato XV-11 sensor.
//!
//! The sensor streams 22-byte packets over UART at 115200 baud, 8N1. Each
//! packet carries four distance readings and the current rotation speed. The
//! sensor has no speed control of its own: the host must drive the spin motor
//! with PWM and close the loop on the reported RPM, which [`Motor`] does with a
//! [`Pid`] controller.
//!
//! - [`Lds02rr`] reads packets from any `embedded-io` (blocking) or
//!   `embedded-io-async` (embassy) reader, e.g. an esp-hal `Uart`.
//! - [`PacketParser`] is the transport-free byte-wise parser underneath it.
//! - [`ScanBuilder`] assembles packets into full 360° [`Scan`]s.
//! - [`Motor`] keeps the spin speed at a target RPM through any
//!   `embedded_hal::pwm::SetDutyCycle` output, e.g. an esp-hal LEDC channel.
//!
//! # Example
//!
//! Read packets, hold the motor at 300 RPM and collect full scans. `rx` is an
//! async UART RX half at [`BAUD_RATE`], `pwm` drives the motor transistor and
//! `elapsed` returns the seconds since its last call.
//!
//! ```no_run
//! use embedded_hal::pwm::SetDutyCycle;
//! use lds02::{Error, Lds02rr, Motor, MotorConfig, ScanBuilder};
//!
//! async fn run<R, P>(rx: R, pwm: P, mut elapsed: impl FnMut() -> f32) -> Result<(), P::Error>
//! where
//!     R: embedded_io_async::Read,
//!     P: SetDutyCycle,
//! {
//!     let mut lidar = Lds02rr::new(rx);
//!     let mut motor = Motor::new(pwm, MotorConfig::default())?;
//!     let mut scans = ScanBuilder::new();
//!     motor.start()?;
//!     loop {
//!         match lidar.read_packet().await {
//!             Ok(packet) => {
//!                 motor.update(packet.rpm(), elapsed())?;
//!                 if let Some(scan) = scans.push(packet) {
//!                     let _front_mm = scan.reading(0).distance();
//!                 }
//!             }
//!             // A corrupted packet: keep reading.
//!             Err(Error::Packet(_)) => {}
//!             Err(_) => return motor.stop(),
//!         }
//!     }
//! }
//! ```
//!
//! # Cargo features
//!
//! - `async` (default): [`Lds02rr::read_packet`] for `embedded-io-async`
//!   readers.
//! - `defmt`: `defmt::Format` for all public types.
//!
//! Complete esp-hal examples (blocking and embassy) are in the
//! `examples/esp32c6` directory of the repository.

#![cfg_attr(not(test), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]
#![warn(missing_docs)]

mod driver;
mod motor;
mod packet;
mod pid;
mod scan;

pub use driver::{Error, Lds02rr};
pub use motor::{Motor, MotorConfig};
pub use packet::{
    BAUD_RATE, PACKET_LEN, PACKETS_PER_REV, Packet, PacketError, PacketParser, READINGS_PER_PACKET,
    Reading, checksum,
};
pub use pid::Pid;
pub use scan::{Scan, ScanBuilder};
