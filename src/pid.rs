//! Generic PID controller.

/// PID controller with derivative on measurement and integrator clamping.
///
/// - Derivative acts on the measurement, not the error, so setpoint changes
///   do not cause output spikes.
/// - The integral term is clamped to the output limits, so it does not wind
///   up while the output is saturated.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pid {
    /// Proportional gain.
    pub kp: f32,
    /// Integral gain, per second.
    pub ki: f32,
    /// Derivative gain, in seconds.
    pub kd: f32,
    /// Value the controller drives the measurement to.
    pub setpoint: f32,
    out_min: f32,
    out_max: f32,
    integral: f32,
    last_measurement: Option<f32>,
}

impl Pid {
    /// Creates a controller with unlimited output.
    pub const fn new(kp: f32, ki: f32, kd: f32, setpoint: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            setpoint,
            out_min: f32::NEG_INFINITY,
            out_max: f32::INFINITY,
            integral: 0.0,
            last_measurement: None,
        }
    }

    /// Limits the output (and the integral term) to `min..=max`.
    ///
    /// # Panics
    ///
    /// If `min > max` or either is NaN.
    pub fn with_output_limits(mut self, min: f32, max: f32) -> Self {
        assert!(min <= max, "invalid PID output limits");
        self.out_min = min;
        self.out_max = max;
        self.integral = self.integral.clamp(min, max);
        self
    }

    /// Runs one control step and returns the new output.
    ///
    /// `dt` is the time since the previous call in seconds. With `dt <= 0`
    /// only the proportional term is updated.
    pub fn update(&mut self, measurement: f32, dt: f32) -> f32 {
        let error = self.setpoint - measurement;
        let mut derivative = 0.0;
        if dt > 0.0 {
            self.integral =
                (self.integral + self.ki * error * dt).clamp(self.out_min, self.out_max);
            if let Some(last) = self.last_measurement {
                derivative = -self.kd * (measurement - last) / dt;
            }
        }
        self.last_measurement = Some(measurement);
        (self.kp * error + self.integral + derivative).clamp(self.out_min, self.out_max)
    }

    /// Clears the integral and derivative history.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_measurement = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proportional_only() {
        let mut pid = Pid::new(2.0, 0.0, 0.0, 10.0);
        assert_eq!(pid.update(7.0, 0.1), 6.0);
    }

    #[test]
    fn integral_accumulates_and_clamps() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, 10.0).with_output_limits(-5.0, 5.0);
        assert_eq!(pid.update(0.0, 0.1), 1.0);
        assert_eq!(pid.update(0.0, 0.1), 2.0);
        for _ in 0..100 {
            pid.update(0.0, 0.1);
        }
        assert_eq!(pid.update(0.0, 0.1), 5.0);
        // No windup: one step of negative error pulls the output down at once.
        assert_eq!(pid.update(20.0, 0.1), 4.0);
    }

    #[test]
    fn derivative_on_measurement() {
        let mut pid = Pid::new(0.0, 0.0, 1.0, 0.0);
        assert_eq!(pid.update(0.0, 0.5), 0.0);
        assert_eq!(pid.update(1.0, 0.5), -2.0);
        // A setpoint jump alone does not kick the derivative.
        pid.setpoint = 100.0;
        assert_eq!(pid.update(1.0, 0.5), 0.0);
    }

    #[test]
    fn closes_loop_on_first_order_plant() {
        // Plant: speed follows 500 * duty with a 0.2 s time constant.
        let mut pid = Pid::new(0.001, 0.01, 0.0, 300.0).with_output_limits(0.0, 1.0);
        let mut speed = 0.0;
        let dt = 0.01;
        for _ in 0..2000 {
            let duty = pid.update(speed, dt);
            speed += (500.0 * duty - speed) * dt / 0.2;
        }
        assert!((speed - 300.0f32).abs() < 1.0, "speed {speed}");
    }
}
