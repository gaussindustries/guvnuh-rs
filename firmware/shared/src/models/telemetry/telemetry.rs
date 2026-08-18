/**
 * i'd like the telemetry to show on a time graph
 * where it keeps drawing and scrolling auto magically
 * (but we can see the history if need be
 * (the screen splits halfway so we always see the current data while exploring the past data,
 * if for some reason we're investigating why there might have been a hiccup perhaps
 * (i'm thinking of having a overview
 * (raw number showing for each thing like rpm, temp, etc)
 * and then giving each data a point/line on a line graph, i'd account for scaling of course)
 *
 */
use crate::models::state::states::{Fault, STATE};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum Uplink {
    Telemetry(Telemetry),
    Calibration(CalibrationReport),
    Hello,    // STM32 announces (re)start — "are you there?"
    HelloAck, // STM32 acknowledges the ESP32's Hello
    WcetReport(WcetReportData),
}

/// Number of states tracked in a WcetReport. Must match the firmware's STATE
/// count and the terminal's decoding. Keep in sync with STATE.
pub const N_STATES: usize = 14;
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct WcetReportData {
    pub per_state_max_us: [f32; N_STATES],
    pub per_state_mean_us: [f32; N_STATES],
    pub global_max_us: f32,
    pub period_us: f32,
    pub samples: u32,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct CalibrationReport {
    pub ts_ms: u32,
    pub k_rpm_per_duty: f32,
    pub rpm_intercept: f32,
    pub max_rpm: f32,
    pub r_squared: f32,
    pub points: [CalPointWire; 5],
    //^^ fixed — must match CAL_POINTS.len() within ~/calibration.rs
    pub point_count: u8,
    pub valid: bool,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, PartialEq)]
pub struct CalPointWire {
    pub duty: f32,
    pub rpm_mean: f32,
    pub rpm_stddev: f32,
    pub samples: u32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct CalPoint {
    pub duty: f32,
    pub rpm_mean: f32,
    pub rpm_stddev: f32, // ← settling quality per point
    pub samples: u32,
}

/// Single telemetry sample — canonical wire format.
/// Serialized via postcard/COBS over UART, deserialized on the server.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Telemetry {
    pub ts_ms: u32,
    pub state: STATE,
    pub rpm: f32,
    pub duty_percent: f32,
    pub v_gen_rms: f32,
    pub i_gen_rms: f32,
    pub freq_gen_hz: f32,
    pub theta_err_rad: f32,
    pub temp_c: f32,
    pub dc_bus_v: f32,
    pub run_mode: Option<RunMode>,
    pub fault: Option<Fault>,
}

/// Run configuration — sent as a "prelude" before Start.
/// Defines how the firmware should behave during the run.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct RunConfig {
    pub mode: RunMode,
    /// Target RPM for closed-loop modes (ignored in Manual/OpenLoop duty mode)
    pub target_rpm: f32,
    /// Ramp-up duration in milliseconds
    pub ramp_up_ms: u32,
    /// Hold duration in milliseconds (0 = hold indefinitely until Stop)
    pub hold_ms: u32,
    /// Ramp-down duration in milliseconds
    pub ramp_down_ms: u32,
    /// PID gains (only used in ClosedLoop mode)
    pub pid: PidGains,
    /// Max duty clamp (0.0–1.0) — safety limit
    pub max_duty_clamp: f32,
    pub target_freq_hz: f32,
    pub target_v_rms: f32,
    pub max_amperage_clamp: f32,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self {
            mode: RunMode::OpenLoop,
            target_rpm: 0.0,
            ramp_up_ms: 5000,
            hold_ms: 0,
            ramp_down_ms: 5000,
            pid: PidGains::default(),
            max_duty_clamp: 1.0,
            target_freq_hz: 60.0,
            target_v_rms: 120.0,
            max_amperage_clamp: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum RunMode {
    /// Direct duty cycle control — no feedback
    OpenLoop,
    /// PID closed-loop on RPM target
    ClosedLoop,
    /// Predefined calibration sequence (ramp up, hold, ramp down)
    Calibrate,
    /// Live manual control — desktop sends LiveAdjust commands in real time
    Manual,
    Generate,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub output_min: f32,
    pub output_max: f32,
}

impl Default for PidGains {
    fn default() -> Self {
        Self {
            kp: 1.0,
            ki: 0.0,
            kd: 0.0,
            output_min: 0.0,
            output_max: 1.0,
        }
    }
}

/// Real-time parameter adjustments while running
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum LiveParam {
    /// Set duty directly (0.0–1.0), only in Manual or OpenLoop mode
    Duty(f32),
    /// Change target RPM on the fly, only in ClosedLoop mode
    TargetRpm(f32),
    /// Hot-swap PID gains without stopping
    PidGains(PidGains),
    /// Adjust max duty clamp
    MaxDutyClamp(f32),

    TargetFreqHz(f32),

    TargetVRms(f32),
}

/// Setpoints sent down from the dashboard (legacy — kept for compatibility)
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Setpoints {
    pub ref_hz: f32,
    pub v_rms: f32,
}

/// Commands from desktop → gaussindustri.es server → ESP32 → STM32
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub enum Command {
    Ping,
    Configure(RunConfig),
    Start,
    Stop,
    EmergencyStop,
    LiveAdjust(LiveParam),
    Set(Setpoints),
    ClearFaults,
    Hello,    // ESP32 announces (re)start — "are you there?"
    HelloAck, // ESP32 acknowledges the STM32's Hello — "yes, I'm here"
    LoadProfile(SetpointProfile),
}

/// Max breakpoints in a profile. Bounds RAM: each ProfilePoint is ~12 bytes,
/// so 16 points ≈ 192 bytes on the wire and in STM32 shared state. Start at 16;
/// raise once the mechanic is proven.
pub const MAX_PROFILE_POINTS: usize = 16;

pub const PROFILE_CMD_BUF: usize = core::mem::size_of::<Command>() + 64;

/// How the segment LEAVING a point behaves — carried per-point so a single
/// profile can ramp between some points and step between others.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum SegmentInterp {
    /// Linear ramp from this point's value to the next point's value.
    Linear,
    /// Hold this point's value until the next point's time, then jump.
    Step,
}

/// What happens once elapsed time passes the LAST breakpoint.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum EndBehavior {
    /// Hold the final point's value forever — until Stop / EStop.
    /// This is "hold indefinitely".
    HoldLast,
    /// Profile is done — the control loop should ramp down / finish.
    Stop,
    /// Restart the profile from t=0 (elapsed wraps modulo total_ms).
    Loop,
}

/// One breakpoint. `interp` describes the segment from THIS point to the next
/// (ignored on the final point — the tail is governed by EndBehavior).
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct ProfilePoint {
    pub t_ms: u32,
    pub target_rpm: f32,
    pub interp: SegmentInterp,
}

impl Default for ProfilePoint {
    fn default() -> Self {
        Self {
            t_ms: 0,
            target_rpm: 0.0,
            interp: SegmentInterp::Linear,
        }
    }
}

/// A setpoint trajectory: sparse breakpoints the STM32 interpolates at loop rate.
/// Fixed-size array (shared is no_std). `count` valid points, the rest padding.
/// Points MUST be sorted by t_ms ascending (the editor guarantees this; eval
/// assumes it).
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct SetpointProfile {
    pub points: [ProfilePoint; MAX_PROFILE_POINTS],
    pub count: u8,
    pub end_behavior: EndBehavior,
    /// Time of the last breakpoint (cached; == points[count-1].t_ms). Used for
    /// Loop wrap and to know when the tail behavior kicks in.
    pub total_ms: u32,
}

impl Default for SetpointProfile {
    fn default() -> Self {
        Self {
            points: [ProfilePoint::default(); MAX_PROFILE_POINTS],
            count: 0,
            end_behavior: EndBehavior::HoldLast,
            total_ms: 0,
        }
    }
}

/// Evaluate the profile at `elapsed_ms`, returning the current target RPM.
/// Assumes points are sorted ascending by t_ms. Pure function — no state.
pub fn eval_profile(profile: &SetpointProfile, elapsed_ms: u32) -> f32 {
    let n = profile.count as usize;
    if n == 0 {
        return 0.0;
    }
    let pts = &profile.points[..n];

    // Single point: hold its value (nothing to interpolate).
    if n == 1 {
        return pts[0].target_rpm;
    }

    // Resolve elapsed against the tail behavior.
    let last_t = pts[n - 1].t_ms;
    let t = if elapsed_ms >= last_t {
        match profile.end_behavior {
            EndBehavior::HoldLast => return pts[n - 1].target_rpm, // hold forever
            EndBehavior::Stop => return pts[n - 1].target_rpm,     // caller detects done separately
            EndBehavior::Loop => {
                if profile.total_ms == 0 {
                    return pts[n - 1].target_rpm;
                }
                elapsed_ms % profile.total_ms // wrap
            }
        }
    } else {
        elapsed_ms
    };

    // Before the first point: hold the first value.
    if t <= pts[0].t_ms {
        return pts[0].target_rpm;
    }

    // Find the segment [a, b] containing t.
    for i in 0..n - 1 {
        let a = pts[i];
        let b = pts[i + 1];
        if t >= a.t_ms && t <= b.t_ms {
            return match a.interp {
                SegmentInterp::Step => a.target_rpm, // hold a until b
                SegmentInterp::Linear => {
                    let span = (b.t_ms - a.t_ms) as f32;
                    if span <= 0.0 {
                        b.target_rpm
                    } else {
                        let frac = (t - a.t_ms) as f32 / span;
                        a.target_rpm + frac * (b.target_rpm - a.target_rpm)
                    }
                }
            };
        }
    }

    // Fallback (shouldn't reach — t is within [first, last] by construction).
    pts[n - 1].target_rpm
}

/// Has a `Stop`-terminated profile finished? (For the control loop to know when
/// to ramp down. HoldLast/Loop never "finish".)
pub fn profile_finished(profile: &SetpointProfile, elapsed_ms: u32) -> bool {
    profile.count > 0 && profile.end_behavior == EndBehavior::Stop && elapsed_ms >= profile.total_ms
}

/// Compiled-in demo profile: ramp 0→1500 over 5s, hold 1500 for 20s, ramp
/// 1500→1800 over 5s, hold 1800 (the 60 Hz target) indefinitely. Runs when no
/// profile is loaded. Tune on hardware.
pub const DEFAULT_PROFILE: SetpointProfile = SetpointProfile {
    points: {
        let mut p = [ProfilePoint {
            t_ms: 0,
            target_rpm: 0.0,
            interp: SegmentInterp::Linear,
        }; MAX_PROFILE_POINTS];
        p[0] = ProfilePoint {
            t_ms: 0,
            target_rpm: 0.0,
            interp: SegmentInterp::Linear,
        };
        p[1] = ProfilePoint {
            t_ms: 5000,
            target_rpm: 1500.0,
            interp: SegmentInterp::Linear,
        };
        p[2] = ProfilePoint {
            t_ms: 25000,
            target_rpm: 1500.0,
            interp: SegmentInterp::Linear,
        };
        p[3] = ProfilePoint {
            t_ms: 30000,
            target_rpm: 1800.0,
            interp: SegmentInterp::Linear,
        };
        p
    },
    count: 4,
    end_behavior: EndBehavior::HoldLast, // hold 1800 forever until Stop
    total_ms: 30000,
};
