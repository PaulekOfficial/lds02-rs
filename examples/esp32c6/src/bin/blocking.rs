//! LDS02RR with blocking esp-hal drivers.
//!
//! Reads packets in a loop, runs the motor PID on every packet and prints a
//! summary of each full scan.
//!
//! Wiring (ESP32-C6):
//! - LIDAR TX (data out) -> GPIO5
//! - GPIO6 -> gate/base of the low-side transistor switching the motor

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    gpio::DriveMode,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    main,
    time::{Instant, Rate},
    uart::{Config, UartRx},
};
use esp_println::println;
use lds02::{BAUD_RATE, Error, Lds02rr, Motor, MotorConfig, ScanBuilder};

esp_bootloader_esp_idf::esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let rx = UartRx::new(peripherals.UART1, Config::default().with_baudrate(BAUD_RATE))
        .unwrap()
        .with_rx(peripherals.GPIO5);
    let mut lidar = Lds02rr::new(rx);

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut pwm_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(25),
        })
        .unwrap();
    let mut pwm = ledc.channel(channel::Number::Channel0, peripherals.GPIO6);
    pwm.configure(channel::config::Config {
        timer: &pwm_timer,
        duty_pct: 0,
        drive_mode: DriveMode::PushPull,
    })
    .unwrap();

    let mut motor = Motor::new(pwm, MotorConfig::default()).unwrap();
    motor.start().unwrap();

    let mut scans = ScanBuilder::new();
    let mut last_update = Instant::now();
    loop {
        match lidar.read_packet_blocking() {
            Ok(packet) => {
                let dt = last_update.elapsed().as_micros() as f32 / 1e6;
                last_update = Instant::now();
                let duty = motor.update(packet.rpm(), dt).unwrap();

                if let Some(scan) = scans.push(packet) {
                    println!(
                        "{:.1} rpm, duty {:.2}, {}/90 packets, front {:?} mm",
                        scan.rpm(),
                        duty,
                        scan.packets(),
                        scan.reading(0).distance(),
                    );
                }
            }
            Err(Error::Packet(e)) => println!("bad packet: {e}"),
            Err(e) => println!("UART error: {e}"),
        }
    }
}
