use core::time::Duration;
use serde::{Serialize, Deserialize};


use crate::models::state::states::STATE;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Telemetry {
    pub ts_ms: u32,
    pub state: STATE,
    pub v_gen_rms: f32,
    pub i_gen_rms: f32,
    pub freq_gen_hz: f32,
    pub theta_err_rad: f32,     // θ_gen - θ_ref
    pub rpm: f32,
    pub temp_c: f32,
    pub dc_bus_v: f32,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Setpoints {
    pub ref_hz: f32,
    pub v_rms: f32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Command {
    Start,
    Stop,
    Set(Setpoints),
    ClearFaults,
    Ping(u32),
}