// guv/calibrate.rs
//
// Learns the steady-state duty→RPM relationship by stepping through a set of
// duty levels, letting RPM settle at each, and recording (duty, rpm) pairs.
// Pure logic — no RTIC resources. The CALIBRATE state feeds it rpm + dt and
// applies CalOutput, exactly as SPOOLUP feeds Ramp/PidController.

/// Duty levels to sample. Low→high; skip very low duties where static friction
/// dominates and the fit goes nonlinear. Tune to your rig.
const CAL_POINTS: [f32; 4] = [0.25, 0.45, 0.65, 0.85];

/// How long to hold each duty before sampling, in ms. Must exceed the rig's
/// mechanical settling time — RPM has to plateau before we record it.
const SETTLE_MS: u32 = 2500;

/// Averaging window at the end of each hold, in ms. We average RPM over the
/// last WINDOW_MS of the settle period rather than grabbing one noisy sample.
const WINDOW_MS: u32 = 500;

#[derive(Clone, Copy, PartialEq)]
enum Phase {
    Settling, // holding a duty, waiting for RPM to plateau
    Sampling, // in the averaging window, accumulating RPM
    Done,
}

/// What the state machine should do this tick.
pub struct CalOutput {
    /// Duty to command right now (0.0–1.0).
    pub duty: f32,
    /// True once all points are collected and the fit is computed.
    pub done: bool,
    /// Populated only on the tick `done` first goes true.
    pub result: Option<CalResult>,
}

/// The learned characterization. Extend with voltage/Hz/current later once
/// the ADC exists — those become derived from max_rpm or a second cal pass.
#[derive(Clone, Copy, Debug)]
pub struct CalResult {
    /// Slope of the duty→RPM line (RPM per unit duty).
    pub k_rpm_per_duty: f32,
    /// Intercept (RPM at duty=0, from the fit — usually small/negative).
    pub rpm_intercept: f32,
    /// Extrapolated RPM at duty=1.0 — the practical "max RPM".
    pub max_rpm: f32,
}

pub struct Calibrator {
    phase: Phase,
    point_idx: usize,
    elapsed_in_point: u32,
    // running mean over the sampling window
    rpm_accum: f32,
    sample_count: u32,
    // collected (duty, mean_rpm) pairs
    duties: [f32; CAL_POINTS.len()],
    rpms: [f32; CAL_POINTS.len()],
    collected: usize,
    result: Option<CalResult>,
}

impl Calibrator {
    pub fn new() -> Self {
        Self {
            phase: Phase::Settling,
            point_idx: 0,
            elapsed_in_point: 0,
            rpm_accum: 0.0,
            sample_count: 0,
            duties: [0.0; CAL_POINTS.len()],
            rpms: [0.0; CAL_POINTS.len()],
            collected: 0,
            result: None,
        }
    }

    /// Advance one control tick. `rpm` is current measured RPM, `dt_ms` the tick.
    pub fn step(&mut self, rpm: f32, dt_ms: u32) -> CalOutput {
        match self.phase {
            Phase::Done => CalOutput {
                duty: 0.0,
                done: true,
                result: None, // result already delivered on the transition tick
            },

            Phase::Settling => {
                self.elapsed_in_point += dt_ms;
                let duty = CAL_POINTS[self.point_idx];

                // enter sampling window for the tail of the settle period
                if self.elapsed_in_point >= SETTLE_MS.saturating_sub(WINDOW_MS) {
                    self.phase = Phase::Sampling;
                    self.rpm_accum = 0.0;
                    self.sample_count = 0;
                }
                CalOutput {
                    duty,
                    done: false,
                    result: None,
                }
            }

            Phase::Sampling => {
                self.elapsed_in_point += dt_ms;
                let duty = CAL_POINTS[self.point_idx];

                self.rpm_accum += rpm;
                self.sample_count += 1;

                if self.elapsed_in_point >= SETTLE_MS {
                    // record the averaged point
                    let mean = if self.sample_count > 0 {
                        self.rpm_accum / self.sample_count as f32
                    } else {
                        rpm
                    };
                    self.duties[self.collected] = duty;
                    self.rpms[self.collected] = mean;
                    self.collected += 1;

                    // advance to next duty, or finish
                    self.point_idx += 1;
                    self.elapsed_in_point = 0;

                    if self.point_idx >= CAL_POINTS.len() {
                        let result = self.fit();
                        self.result = Some(result);
                        self.phase = Phase::Done;
                        return CalOutput {
                            duty: 0.0,
                            done: true,
                            result: Some(result),
                        };
                    } else {
                        self.phase = Phase::Settling;
                    }
                }
                CalOutput {
                    duty,
                    done: false,
                    result: None,
                }
            }
        }
    }

    /// Least-squares line fit through the collected (duty, rpm) pairs.
    fn fit(&self) -> CalResult {
        let n = self.collected as f32;
        let (mut sx, mut sy, mut sxy, mut sxx) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for i in 0..self.collected {
            let x = self.duties[i];
            let y = self.rpms[i];
            sx += x;
            sy += y;
            sxy += x * y;
            sxx += x * x;
        }
        let denom = n * sxx - sx * sx;
        let (slope, intercept) = if denom.abs() > f32::EPSILON {
            let m = (n * sxy - sx * sy) / denom;
            let b = (sy - m * sx) / n;
            (m, b)
        } else {
            (0.0, 0.0)
        };
        CalResult {
            k_rpm_per_duty: slope,
            rpm_intercept: intercept,
            max_rpm: slope * 1.0 + intercept, // duty = 1.0
        }
    }
}
