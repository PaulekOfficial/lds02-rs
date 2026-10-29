//! LDS02RR with embassy and async esp-hal drivers.
//!
//! - `lidar_task` reads packets, publishes the measured RPM and prints a
//!   summary of each full scan.
//! - `main` runs the motor PID at a fixed 50 Hz. When packets stop (the
//!   sensor goes silent when it spins too slowly or stalls) it feeds 0 RPM,
//!   so the controller pushes the motor back up.
//!
//! Wiring (ESP32-C6):
//! - LIDAR TX (data out) -> GPIO5
//! - GPIO6 -> gate/base of the low-side transistor switching the motor

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker};
use esp_backtrace as _;
use esp_hal::{
    Async,
    gpio::DriveMode,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Config, UartRx},
};
use esp_println::println;
use lds02::{BAUD_RATE, Error, Lds02rr, Motor, MotorConfig, ScanBuilder};

esp_bootloader_esp_idf::esp_app_desc!();

const CONTROL_PERIOD: Duration = Duration::from_millis(20);
/// Without packets for this long the motor counts as stopped.
const RPM_TIMEOUT: Duration = Duration::from_millis(200);

/// Latest speed measured by the LIDAR.
static RPM: Signal<CriticalSectionRawMutex, f32> = Signal::new();

#[embassy_executor::task]
async fn lidar_task(rx: UartRx<'static, Async>) {
    let mut lidar = Lds02rr::new(rx);
    let mut scans = ScanBuilder::new();
    loop {
        match lidar.read_packet().await {
            Ok(packet) => {
                RPM.signal(packet.rpm());
                if let Some(scan) = scans.push(packet) {
                    println!(
                        "{:.1} rpm, {}/90 packets, front {:?} mm",
                        scan.rpm(),
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

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    let rx = UartRx::new(peripherals.UART1, Config::default().with_baudrate(BAUD_RATE))
        .unwrap()
        .with_rx(peripherals.GPIO5)
        .into_async();
    spawner.spawn(lidar_task(rx).unwrap());

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

    let dt = CONTROL_PERIOD.as_micros() as f32 / 1e6;
    let mut rpm = 0.0;
    let mut last_packet = Instant::now();
    let mut ticker = Ticker::every(CONTROL_PERIOD);
    let mut tick: u32 = 0;
    loop {
        ticker.next().await;
        if let Some(measured) = RPM.try_take() {
            rpm = measured;
            last_packet = Instant::now();
        } else if last_packet.elapsed() > RPM_TIMEOUT {
            rpm = 0.0;
        }
        let duty = motor.update(rpm, dt).unwrap();

        tick = tick.wrapping_add(1);
        if tick % 50 == 0 {
            println!("motor: {:.1} rpm, duty {:.2}", rpm, duty);
        }
    }
}
