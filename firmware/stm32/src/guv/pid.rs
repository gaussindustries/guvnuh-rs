use shared::models::telemetry::telemetry::PidGains;

pub struct PidController {
    gains: PidGains,
    integral: f32,
    prev_error: f32,
    prev_output: f32,
}

impl PidController {
    pub fn new(gains: PidGains) -> Self {
        Self {
            gains,
            integral: 0.0,
            prev_error: 0.0,
            prev_output: 0.0,
        }
    }

    /// Compute one PID step. dt is in seconds.
    pub fn update(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32 {
        let error = setpoint - measured;

        // Proportional
        let p = self.gains.kp * error;

        // Integral with anti-windup (clamp before accumulating)
        self.integral += error * dt;
        let i_raw = self.gains.ki * self.integral;

        // Derivative (on measurement to avoid setpoint kick)
        let derivative = if dt > 0.0 {
            -(measured - self.prev_error) / dt // derivative on measurement
        } else {
            0.0
        };
        let d = self.gains.kd * derivative;

        let mut output = p + i_raw + d;

        // Clamp output
        output = output.clamp(self.gains.output_min, self.gains.output_max);

        // Anti-windup: if output is saturated, stop integrating in that direction
        if output == self.gains.output_max && error > 0.0 {
            self.integral -= error * dt; // undo the accumulation
        } else if output == self.gains.output_min && error < 0.0 {
            self.integral -= error * dt;
        }

        self.prev_error = measured; // store measurement for derivative-on-measurement
        self.prev_output = output;

        output
    }

    /// Hot-swap gains without resetting state
    pub fn set_gains(&mut self, gains: PidGains) {
        self.gains = gains;
    }

    /// Reset integral and derivative state (use on mode transitions)
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.prev_output = 0.0;
    }

    pub fn gains(&self) -> &PidGains {
        &self.gains
    }
}
