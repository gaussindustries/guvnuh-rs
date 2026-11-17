use shared::models::telemetry::telemetry::PidGains;

pub struct PidController {
    gains: PidGains,
    integral: f32,
    /// Previous measurement, for derivative-on-measurement.
    prev_measured: f32,
    prev_output: f32,
    /// False until the first update after new()/reset(). With no previous
    /// measurement yet, that first step skips the derivative term.
    primed: bool,
}

impl PidController {
    pub fn new(gains: PidGains) -> Self {
        Self {
            gains,
            integral: 0.0,
            prev_measured: 0.0,
            prev_output: 0.0,
            primed: false,
        }
    }

    /// Compute one PID step. dt is in seconds.
    pub fn update(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32 {
        // A non-finite input would poison the integral for good. Hold the last
        // output for this step instead.
        if !(setpoint.is_finite() && measured.is_finite() && dt.is_finite()) {
            return self.prev_output;
        }

        let error = setpoint - measured;

        // Proportional
        let p = self.gains.kp * error;

        // Integral (anti-windup below: undone while the output is saturated)
        self.integral += error * dt;
        let i_raw = self.gains.ki * self.integral;

        // Derivative on measurement (avoids setpoint kick). Skipped on the first
        // step after new()/reset(): against a stale 0 rpm, a shaft that is still
        // turning would read as a huge deceleration and slam the output.
        let derivative = if self.primed && dt > 0.0 {
            -(measured - self.prev_measured) / dt
        } else {
            0.0
        };
        let d = self.gains.kd * derivative;

        let mut output = p + i_raw + d;

        // Clamp output. f32::clamp panics if min > max or either is NaN, so a
        // bad gain set from the server gives 0 instead of a panic.
        let (lo, hi) = (self.gains.output_min, self.gains.output_max);
        output = if lo <= hi { output.clamp(lo, hi) } else { 0.0 };

        // Anti-windup: if output is saturated, stop integrating in that direction
        if output == hi && error > 0.0 {
            self.integral -= error * dt; // undo the accumulation
        } else if output == lo && error < 0.0 {
            self.integral -= error * dt;
        }

        self.prev_measured = measured;
        self.primed = true;
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
        self.prev_measured = 0.0;
        self.prev_output = 0.0;
        self.primed = false;
    }

    pub fn gains(&self) -> &PidGains {
        &self.gains
    }
}
