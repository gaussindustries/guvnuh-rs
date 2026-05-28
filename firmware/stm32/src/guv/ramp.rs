/// Linear ramp generator. Produces a value from 0.0 to target over duration_ms,
/// or from current to 0.0 for ramp-down.
pub struct Ramp {
    start_value: f32,
    end_value: f32,
    duration_ms: u32,
    elapsed_ms: u32,
    done: bool,
}

impl Ramp {
    pub fn new(start: f32, end: f32, duration_ms: u32) -> Self {
        Self {
            start_value: start,
            end_value: end,
            duration_ms: if duration_ms == 0 { 1 } else { duration_ms },
            elapsed_ms: 0,
            done: false,
        }
    }

    /// Advance by dt_ms, return current interpolated value
    pub fn tick(&mut self, dt_ms: u32) -> f32 {
        if self.done {
            return self.end_value;
        }
        self.elapsed_ms += dt_ms;
        if self.elapsed_ms >= self.duration_ms {
            self.done = true;
            self.end_value
        } else {
            let t = self.elapsed_ms as f32 / self.duration_ms as f32;
            self.start_value + (self.end_value - self.start_value) * t
        }
    }

    pub fn is_done(&self) -> bool {
        self.done
    }

    pub fn current_value(&self) -> f32 {
        if self.done {
            self.end_value
        } else {
            let t = self.elapsed_ms as f32 / self.duration_ms as f32;
            self.start_value + (self.end_value - self.start_value) * t
        }
    }
}
