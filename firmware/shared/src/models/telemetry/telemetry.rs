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
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct CalibrationReport {
    pub ts_ms: u32,
    pub k_rpm_per_duty: f32,
    pub rpm_intercept: f32,
    pub max_rpm: f32,
    pub r_squared: f32,
    pub points: [CalPointWire; 4], // fixed — matches CAL_POINTS.len()
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

/// Commands from desktop → gaussindustri.es → ESP32 → STM32
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
}
