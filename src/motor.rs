//! Closed-loop speed control of the LIDAR spin motor.

use embedded_hal::pwm::SetDutyCycle;

use crate::pid::Pid;

/// Tuning for [`Motor`]. Gains are in duty-cycle fraction per RPM of error.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotorConfig {
    /// Target speed. The LDS02RR is specified for 300 RPM (5 Hz scans).
    pub target_rpm: f32,
    /// Proportional gain.
    pub kp: f32,
    /// Integral gain, per second.
    pub ki: f32,
    /// Derivative gain, in seconds.
    pub kd: f32,
    /// Duty cycle (0.0..=1.0) that roughly gives `target_rpm`. The PID only
    /// corrects around it, which gives a fast, smooth spin up.
    pub feedforward: f32,
}

impl Default for MotorConfig {
    /// Starting point for a motor driven from 5 V through a low-side
    /// transistor. Tune `feedforward` first, then the gains.
    fn default() -> Self {
        Self {
            target_rpm: 300.0,
            kp: 0.001,
            ki: 0.002,
            kd: 0.0,
            feedforward: 0.6,
        }
    }
}

/// Keeps the spin motor at a target RPM by adjusting a PWM duty cycle.
///
/// Call [`update`](Self::update) with each RPM measurement, e.g. from
/// [`Packet::rpm`](crate::Packet::rpm), or with `0.0` when no packets arrive
/// (the sensor stops sending when it spins too slowly).
#[derive(Debug)]
pub struct Motor<P> {
    pwm: P,
    pid: Pid,
    feedforward: f32,
    duty: f32,
}

impl<P: SetDutyCycle> Motor<P> {
    /// Takes the PWM output. The motor stays off until [`start`](Self::start)
    /// or [`update`](Self::update).
    pub fn new(mut pwm: P, config: MotorConfig) -> Result<Self, P::Error> {
        pwm.set_duty_cycle_fully_off()?;
        let feedforward = config.feedforward.clamp(0.0, 1.0);
        // Limit the PID so feedforward + PID always lands in 0..=1 and the
        // integrator cannot wind up past what the PWM can deliver.
        let pid = Pid::new(config.kp, config.ki, config.kd, config.target_rpm)
            .with_output_limits(-feedforward, 1.0 - feedforward);
        Ok(Self {
            pwm,
            pid,
            feedforward,
            duty: 0.0,
        })
    }

    /// Spins up at the feedforward duty cycle.
    pub fn start(&mut self) -> Result<(), P::Error> {
        self.pid.reset();
        self.set_duty(self.feedforward)
    }

    /// Turns the motor off and clears the controller state.
    pub fn stop(&mut self) -> Result<(), P::Error> {
        self.pid.reset();
        self.set_duty(0.0)
    }

    /// Runs one control step with the measured speed and the time since the
    /// last step in seconds. Returns the applied duty cycle (0.0..=1.0).
    pub fn update(&mut self, rpm: f32, dt: f32) -> Result<f32, P::Error> {
        let duty = self.feedforward + self.pid.update(rpm, dt);
        self.set_duty(duty)?;
        Ok(self.duty)
    }

    /// Current target speed in RPM.
    pub fn target_rpm(&self) -> f32 {
        self.pid.setpoint
    }

    /// Changes the target speed. The controller keeps its state.
    pub fn set_target_rpm(&mut self, rpm: f32) {
        self.pid.setpoint = rpm;
    }

    /// Last applied duty cycle (0.0..=1.0).
    pub fn duty(&self) -> f32 {
        self.duty
    }

    /// Access to the controller for live tuning.
    pub fn pid_mut(&mut self) -> &mut Pid {
        &mut self.pid
    }

    /// Returns the PWM output. The motor keeps its last duty cycle.
    pub fn release(self) -> P {
        self.pwm
    }

    fn set_duty(&mut self, duty: f32) -> Result<(), P::Error> {
        let duty = duty.clamp(0.0, 1.0);
        let max = self.pwm.max_duty_cycle();
        self.pwm.set_duty_cycle((duty * f32::from(max) + 0.5) as u16)?;
        self.duty = duty;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;

    struct FakePwm(u16);

    impl embedded_hal::pwm::ErrorType for FakePwm {
        type Error = Infallible;
    }

    impl SetDutyCycle for FakePwm {
        fn max_duty_cycle(&self) -> u16 {
            1000
        }
        fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Infallible> {
            self.0 = duty;
            Ok(())
        }
    }

    #[test]
    fn start_stop() {
        let mut motor = Motor::new(FakePwm(123), MotorConfig::default()).unwrap();
        assert_eq!(motor.duty(), 0.0);
        motor.start().unwrap();
        assert_eq!(motor.release().0, 600);
    }

    #[test]
    fn duty_stays_in_range() {
        let config = MotorConfig {
            kp: 1.0,
            ..MotorConfig::default()
        };
        let mut motor = Motor::new(FakePwm(0), config).unwrap();
        assert_eq!(motor.update(0.0, 0.01).unwrap(), 1.0);
        assert_eq!(motor.update(1000.0, 0.01).unwrap(), 0.0);
        assert_eq!(motor.release().0, 0);
    }

    #[test]
    fn at_target_holds_feedforward() {
        let mut motor = Motor::new(FakePwm(0), MotorConfig::default()).unwrap();
        assert_eq!(motor.update(300.0, 0.01).unwrap(), 0.6);
    }
}
