# lds02

`no_std` Rust driver for the **LDS02RR** LIDAR, the 360° scanner of the 1st-gen
Xiaomi / Roborock robot vacuums (packet format compatible with the Neato XV-11),
with PID speed control of its spin motor. Works with
[embassy](https://embassy.dev) and [esp-hal](https://github.com/esp-rs/esp-hal),
or any HAL that implements `embedded-io`, `embedded-io-async` and
`embedded-hal` 1.0.

## Features

- Packet parser with checksum check and automatic resync on a raw byte stream.
- `Lds02rr` reader: `read_packet().await` (async) and `read_packet_blocking()`.
- `ScanBuilder`: assembles packets into full 360° scans, one reading per degree,
  and handles lost packets.
- `Pid`: generic PID controller (derivative on measurement, anti-windup).
- `Motor`: closed-loop RPM control on any `embedded_hal::pwm::SetDutyCycle`.

Cargo features: `async` (default) enables the async reader, `defmt` derives
`defmt::Format` for all public types.

## Usage

```rust
use lds02::{Lds02rr, Motor, MotorConfig, ScanBuilder};

let mut lidar = Lds02rr::new(uart_rx);            // embedded-io(-async) reader
let mut motor = Motor::new(pwm, MotorConfig::default())?; // SetDutyCycle
let mut scans = ScanBuilder::new();
motor.start()?;

loop {
    let packet = lidar.read_packet().await?;
    motor.update(packet.rpm(), dt_seconds)?;
    if let Some(scan) = scans.push(packet) {
        let front_mm = scan.reading(0).distance();
    }
}
```

## Examples

ESP32-C6 examples are in `examples/esp32c6` (LIDAR TX on GPIO5, motor PWM on
GPIO6):

```sh
cd examples/esp32c6
cargo run --release --bin blocking   # blocking esp-hal
cargo run --release --bin embassy    # embassy + async esp-hal, 50 Hz PID task
```

For another ESP chip, change the chip features in `examples/esp32c6/Cargo.toml`,
the target in `.cargo/config.toml` and the pins.

## Hardware

### Interface

| Signal | Notes |
|---|---|
| **5V** | Module supply. Budget ~1 A peak together with the motor. |
| **GND** | Must go straight to supply ground, never through the motor MOSFET (see [Troubleshooting](#troubleshooting-uart-problems)). |
| **TX** | 3.3 V TTL UART, **115200 8N1**, LSB first, idle high. Output only, the LDS has no RX. Connect directly to the MCU UART RX, no level shifter. |
| **MOT+ / MOT−** | Bare brushed DC motor. **No driver inside**: the host must PWM it to hold ~300 RPM. |

> ⚠️ Several LDS02RR hardware revisions exist and connector orientation differs
> between them. **Verify your pinout with a multimeter before powering it:**
> - Motor pair: a few Ω between two pins, continuous to the motor leads.
> - GND → 5V: lowest diode-test reading (~0.5 V, red probe on GND).
> - TX: power 5V (current limit ~500 mA), keep the head **still**, and listen
>   at 115200. The LDS sends a **property packet starting with `0xAA` every
>   ~500 ms** while not rotating. Whichever pin carries it is TX. (This crate
>   skips property packets; it only decodes `0xFA` measurement packets.)

### Motor drive circuit

```text
 5V ──┬───────────────┬────────────── LDS 5V
      │               │
   MOT+ ┐        SS14 / 1N5819 (cathode/stripe → 5V)
  (motor)│            │
   MOT− ┘────────────┴──── Drain
                           AO3400A
 MCU PWM ──[100 Ω]──────── Gate
                 └[100 kΩ]─┤
 GND (motor return) ────── Source ──► star ground
 GND (LDS) ───────────────────────► star ground   (separate wire!)

 Near the LDS: 100 nF + 100…1000 µF between 5V and GND
 Optional: 100 nF ceramic directly across the motor terminals
```

- **MOSFET: AO3400A.** V_GS(th) 0.65–1.45 V, R_DS(on) < 48 mΩ at
  **V_GS = 2.5 V**, so it is fully on from a 3.3 V GPIO.
  - *Avoid BS170 from 3.3 V:* its V_GS(th) can be up to 3.0 V and R_DS(on) is
    only specified at V_GS = 10 V. It may spin the motor, but it is out of spec
    and the speed loop gain varies from part to part.
- **Flyback diode: Schottky SS14 or 1N5819** across the motor. Prefer these over
  a 1N4148, since the motor draws ~0.35 A at start-up.
- **PWM:** 5–20 kHz. Do not drive the motor from a GPIO pin directly.

## Motor speed control

The LDS02RR outputs measurement packets only when the head turns at about
**300 RPM (5 Hz)**. Outside that window it may output nothing at all. Once
rotating, it also stops sending the `0xAA` property packets.

- **Feedback:** every packet carries the speed (`Packet::rpm()`, raw value is
  RPM × 64).
- **Start-up:** no packets arrive until the head is close to speed. `Motor::start`
  applies the `feedforward` duty; then keep calling `Motor::update` with `0.0`
  RPM while no packets arrive, so the integrator ramps the duty up until data
  appears. The `embassy` example does this with a 200 ms timeout.
- **Period:** run the loop at a fixed period, e.g. 20 ms. With embassy, run it
  in a high-priority executor (e.g. an `InterruptExecutor`) so long computations
  elsewhere cannot starve it.

`MotorConfig` gains are in duty-cycle fraction per RPM of error.

1. Set `kp = ki = 0` and find the duty cycle that gives about 300 RPM. Put it in
   `feedforward` (often around 0.5).
2. Raise `ki` until the speed settles on target in about a second.
3. Add `kp` to make it react faster. Keep `kd` at 0 unless it oscillates.

[kaiaai/LDS](https://github.com/kaiaai/LDS) uses **kp = 3e-3, ki = 1e-3,
kd = 0** for the LDS02RR, which is another good starting point.

## Troubleshooting UART problems

### Check: 90/90 packets per revolution

A healthy setup delivers **all 90 packets of every revolution with 0 checksum
errors**, at ~5 revolutions per second. Count it with the crate:

```rust
let mut bad_checksums = 0u32;
loop {
    match lidar.read_packet().await {
        Ok(packet) => {
            if let Some(scan) = scans.push(packet) {
                // e.g. "90/90 packets, 0 bad"
                println!("{}/90 packets, {} bad", scan.packets(), bad_checksums);
                bad_checksums = 0;
            }
        }
        Err(lds02::Error::Packet(_)) => bad_checksums += 1,
        Err(e) => println!("UART error: {e}"), // overrun, framing, noise
    }
}
```

- **Missing packets** (e.g. `84/90`) point to bytes dropped on the host: UART
  overrun, DMA buffer too small, or the RX task starved.
- **Checksum errors** (`Error::Packet`) or **UART errors** (`Error::Io`) point
  to signal integrity, see below.
- **No packets at all** with the laser on and the head spinning: wrong rotation
  speed, see [Motor speed control](#motor-speed-control).

### Measure with a scope, not a multimeter

A DC meter on TX shows the *average* of the bit stream, e.g. ~1.2 V, which is
meaningless for a UART. While spinning, the line is ~85 % busy and many bytes
are full of zero bits (invalid samples, zero strength), so the trace looks
"mostly low with narrow high pulses". That is normal; polarity is still idle
high.

| Symptom on the scope | Cause | Fix |
|---|---|---|
| Negative spikes to about −0.3 V after edges, ringing | Motor switched without a freewheel path; inductive kick couples into shared GND/5V | **Flyback Schottky across the motor.** With it, the spikes disappear and the edges become clean. |
| Low level ~0.5–0.8 V instead of ~0 V, drifting with load | Motor current returning through the same GND wire as the LDS (ground offset) | **Separate GND wires** for the LDS and the MOSFET source, joined only at the supply (star ground). Never return LDS GND through the MOSFET. Add bulk + 100 nF near the LDS. |
| Low level still high when the probe ground is clipped directly on the LDS GND pin | Output stage loaded | Remove any pull-up you added on TX, or use ≥10 kΩ to **3.3 V** (never 5 V). |
| UART decoder shows mostly red / framing errors on a clean-looking trace | Decoder settings | 115200, 8N1, LSB first, idle high, threshold ~1.8 V. The narrowest bit must measure **8.68 µs**. |
| Laser on, head spinning, **no data at all** | Wrong rotation speed | Close the speed loop. A motor run straight from 5 V spins too fast and the LDS goes silent. |

**Target levels before connecting an MCU:** high ≈ 3.0–3.3 V, **low < 0.4 V**.

- STM32H7 guarantees LOW below 0.3·VDD ≈ 0.99 V and HIGH above 0.7·VDD ≈ 2.31 V.
- ESP32 only guarantees LOW below 0.25·VDD ≈ 0.8 V, so a 0.8 V low will
  randomly fail there.

### STM32H7 / embassy notes

- **DMA:** use DMA ring-buffered RX (`embassy_stm32::usart::RingBufferedUartRx`,
  it implements `embedded_io_async::Read`). The stream is ~10 kB/s continuous
  (90 × 22 B × 5 Hz).
- **DMA buffer placement:** DMA1/DMA2 **cannot access DTCM (`0x2000_0000`)**.
  Put the RX buffer in **AXI SRAM (`0x2400_0000`)** or SRAM1–3 (`0x3000_0000`)
  via `memory.x` / `#[link_section]`, otherwise you get DMA transfer errors.
- **D-cache:** if it is enabled, make the DMA buffer region non-cacheable (MPU)
  or invalidate before reading.
- **Other 5 V signals** (e.g. encoders): never feed 5 V into H7 pins. Use level
  translation (e.g. SN74LVC2T45). The LDS TX is 3.3 V and connects directly.

## Protocol reference

22 bytes per packet, **90 packets per revolution**, 4 samples each (1° per
sample):

| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `0xFA` start byte |
| 1 | 1 | index `0xA0`–`0xF9` → first angle = `(index − 0xA0) × 4` |
| 2 | 2 | speed, u16 LE, RPM × 64 |
| 4 | 4×4 | samples: `dist[7:0]`, `flags \| dist[13:8]`, `strength` u16 LE |
| 20 | 2 | checksum, u16 LE, see `lds02::checksum` |

Sample decoding (`Reading`):

- `distance_mm = b0 | ((b1 & 0x3F) << 8)`
- `b1 & 0x80` = invalid
- `b1 & 0x40` = strength warning

`0xFA` can also appear inside sample data. The parser therefore requires the
next byte to be in `0xA0..=0xF9`, and after a checksum failure it resyncs from
the next `0xFA` inside the rejected frame instead of dropping all 22 bytes.

## Tests

```sh
cargo test
```

## References

- Roborock Cullinan: LDS serial format, property/measurement packets, checksum.
  <https://github.com/Roborock-OpenSource/Cullinan>
- kaiaai/LDS: Arduino LDS library; LDS02RR motor PID constants.
  <https://github.com/kaiaai/LDS>
- Maker's Pet: LDS02RR adapter and Raspberry Pi tutorial (TX/MOT_EN/5V/GND,
  300 RPM). <https://makerspet.com/blog/tutorial-connect-xiaomi-lds02rr-lidar-to-raspberry-pi-python/>
- LDS07RR scanner: related Roborock module, reverse-engineered pinout and
  pitfalls. <https://github.com/ArnodeParno/lds07rr-lidar-scanner>
- AO3400A datasheet: <https://www.aosmd.com/res/datasheets/AO3400A.pdf>
- BS170 datasheet: <https://www.onsemi.com/download/data-sheet/pdf/bs170-d.pdf>
