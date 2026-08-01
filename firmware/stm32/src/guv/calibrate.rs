// guv/calibrate.rs
//
// Learns the steady-state duty→RPM relationship by holding a set of duty levels,
// letting RPM settle at each, and least-squares fitting a line through the points.
//
// The device computes only what it needs to control with (a linear coefficient for
// feedforward). Raw points are reported upward so the server can do richer analysis
// without firmware changes. See VALIDATION below — a fit is not trusted until it
// passes sanity checks.

/// Duty levels to sample, low→high. Avoid very low duties where static friction
/// dominates and the relationship goes nonlinear. Tune to the rig.
pub const CAL_POINTS: [f32; 4] = [0.30, 0.55, 0.70, 0.85];

/// Hold time per duty level, ms. Must exceed mechanical settling time — if RPM is
/// still climbing when we sample, the fit lies. Watch the trace and lengthen this
/// until each plateau is visibly flat.
const SETTLE_MS: u32 = 2500;

/// Averaging window at the tail of each hold, ms. We average rather than take one
/// noisy sample, and we track stddev as a per-point settling-quality signal.
const WINDOW_MS: u32 = 500;

// ── VALIDATION THRESHOLDS ──
// A fit that fails any of these is not used for feedforward. On a DC motor a bad
// coefficient means sloppy control; on a turbine it means a wrong valve position.
/// Minimum acceptable fit quality. Below this, the relationship isn't linear enough
/// to trust — usually means a point didn't settle or the coupling slipped.
const MIN_R_SQUARED: f32 = 0.95;
/// Slope must be positive and physically plausible (RPM per unit duty).
const MIN_SLOPE: f32 = 1.0;
const MAX_SLOPE: f32 = 100_000.0;
/// Per-point noise ceiling: stddev as a fraction of mean. Above this the point
/// never settled.
const MAX_POINT_CV: f32 = 0.15;

#[derive(Clone, Copy, PartialEq)]
enum Phase {
    Settling,
    Sampling,
    Done,
}

/// One sampled operating point — the "work it took" that gets reported upward.
#[derive(Clone, Copy, Debug, Default)]
pub struct CalPoint {
    pub duty: f32,
    pub rpm_mean: f32,
    pub rpm_stddev: f32,
    pub samples: u32,
}

const MIN_MEANINGFUL_RPM: f32 = 1.0;

impl CalPoint {
    pub fn cv(&self) -> f32 {
        if self.rpm_mean.abs() > MIN_MEANINGFUL_RPM {
            self.rpm_stddev / self.rpm_mean.abs()
        } else {
            f32::INFINITY // fails MAX_POINT_CV → point rejected
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct CalResult {
    pub k_rpm_per_duty: f32,
    pub rpm_intercept: f32,
    pub max_rpm: f32,
    pub r_squared: f32,
    pub points: [CalPoint; CAL_POINTS.len()],
    pub point_count: u8,
    /// False if the fit failed validation — do NOT use for feedforward.
    pub valid: bool,
}

impl CalResult {
    /// Feedforward duty for a target RPM. Returns None if the calibration
    /// shouldn't be trusted or the answer is out of range.
    pub fn feedforward(&self, target_rpm: f32) -> Option<f32> {
        if !self.valid || self.k_rpm_per_duty.abs() < f32::EPSILON {
            return None;
        }
        let duty = (target_rpm - self.rpm_intercept) / self.k_rpm_per_duty;
        if duty.is_finite() && (0.0..=1.0).contains(&duty) {
            Some(duty)
        } else {
            None
        }
    }
}

pub struct CalOutput {
    pub duty: f32,
    pub done: bool,
    /// Present only on the tick calibration completes.
    pub result: Option<CalResult>,
}

pub struct Calibrator {
    phase: Phase,
    point_idx: usize,
    elapsed_in_point: u32,
    // Welford-style accumulation for mean + stddev without storing samples
    accum: f32,
    accum_sq: f32,
    sample_count: u32,
    points: [CalPoint; CAL_POINTS.len()],
    collected: usize,
}

impl Calibrator {
    pub fn new() -> Self {
        Self {
            phase: Phase::Settling,
            point_idx: 0,
            elapsed_in_point: 0,
            accum: 0.0,
            accum_sq: 0.0,
            sample_count: 0,
            points: [CalPoint::default(); CAL_POINTS.len()],
            collected: 0,
        }
    }

    pub fn step(&mut self, rpm: f32, dt_ms: u32) -> CalOutput {
        match self.phase {
            Phase::Done => CalOutput {
                duty: 0.0,
                done: true,
                result: None,
            },

            Phase::Settling => {
                self.elapsed_in_point += dt_ms;
                if self.elapsed_in_point >= SETTLE_MS.saturating_sub(WINDOW_MS) {
                    self.phase = Phase::Sampling;
                    self.accum = 0.0;
                    self.accum_sq = 0.0;
                    self.sample_count = 0;
                }
                CalOutput {
                    duty: CAL_POINTS[self.point_idx],
                    done: false,
                    result: None,
                }
            }

            Phase::Sampling => {
                self.elapsed_in_point += dt_ms;
                let duty = CAL_POINTS[self.point_idx];

                self.accum += rpm;
                self.accum_sq += rpm * rpm;
                self.sample_count += 1;

                if self.elapsed_in_point >= SETTLE_MS {
                    let n = self.sample_count.max(1) as f32;
                    let mean = self.accum / n;
                    let variance = (self.accum_sq / n) - (mean * mean);
                    let stddev = if variance > 0.0 { sqrtf(variance) } else { 0.0 };

                    self.points[self.collected] = CalPoint {
                        duty,
                        rpm_mean: mean,
                        rpm_stddev: stddev,
                        samples: self.sample_count,
                    };
                    self.collected += 1;

                    self.point_idx += 1;
                    self.elapsed_in_point = 0;

                    if self.point_idx >= CAL_POINTS.len() {
                        let result = self.fit();
                        self.phase = Phase::Done;
                        return CalOutput {
                            duty: 0.0,
                            done: true,
                            result: Some(result),
                        };
                    }
                    self.phase = Phase::Settling;
                }
                CalOutput {
                    duty,
                    done: false,
                    result: None,
                }
            }
        }
    }

    /// Least-squares line fit + r² + validation.
    fn fit(&self) -> CalResult {
        let n = self.collected as f32;
        let (mut sx, mut sy, mut sxy, mut sxx) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
        for p in &self.points[..self.collected] {
            sx += p.duty;
            sy += p.rpm_mean;
            sxy += p.duty * p.rpm_mean;
            sxx += p.duty * p.duty;
        }

        let denom = n * sxx - sx * sx;
        let (slope, intercept) = if denom.abs() > f32::EPSILON {
            let m = (n * sxy - sx * sy) / denom;
            (m, (sy - m * sx) / n)
        } else {
            (0.0, 0.0)
        };

        // r² — how well the line explains the points
        let mean_y = sy / n;
        let (mut ss_tot, mut ss_res) = (0.0f32, 0.0f32);
        for p in &self.points[..self.collected] {
            let predicted = slope * p.duty + intercept;
            ss_res += (p.rpm_mean - predicted) * (p.rpm_mean - predicted);
            ss_tot += (p.rpm_mean - mean_y) * (p.rpm_mean - mean_y);
        }
        let r_squared = if ss_tot > f32::EPSILON {
            1.0 - (ss_res / ss_tot)
        } else {
            0.0
        };

        // ── Validation: compute it, but don't trust it blindly ──
        let slope_ok = slope >= MIN_SLOPE && slope <= MAX_SLOPE;
        let fit_ok = r_squared >= MIN_R_SQUARED;
        let points_ok = self.points[..self.collected]
            .iter()
            .all(|p| p.cv() <= MAX_POINT_CV);
        let valid = slope_ok && fit_ok && points_ok && self.collected >= 2;

        CalResult {
            k_rpm_per_duty: slope,
            rpm_intercept: intercept,
            max_rpm: slope + intercept, // duty = 1.0
            r_squared,
            points: self.points,
            point_count: self.collected as u8,
            valid,
        }
    }
}

/// no_std sqrt — libm-free Newton iteration. Swap for `libm::sqrtf` if you add libm.
fn sqrtf(x: f32) -> f32 {
    if x <= 0.0 {
        return 0.0;
    }
    let mut guess = x;
    for _ in 0..8 {
        guess = 0.5 * (guess + x / guess);
    }
    guess
}
