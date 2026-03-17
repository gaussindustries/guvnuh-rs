use crate::models::state::states::STATE;
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
use serde::{Deserialize, Serialize};

/// Single telemetry sample — canonical wire format.
/// Serialized via postcard/COBS over UART, deserialized on the server.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Telemetry {
    pub ts_ms: u32,
    pub state: STATE,
    pub rpm: f32,
    pub v_gen_rms: f32,
    pub i_gen_rms: f32,
    pub freq_gen_hz: f32,
    pub theta_err_rad: f32,
    pub temp_c: f32,
    pub dc_bus_v: f32,
}

/// Setpoints sent down from the dashboard
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Setpoints {
    pub ref_hz: f32,
    pub v_rms: f32,
}

/// Commands from desktop → gaussindustri.es → ESP32 → STM32
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub enum Command {
    Start,
    Stop,
    Set(Setpoints),
    ClearFaults,
    Ping(u32),
}
