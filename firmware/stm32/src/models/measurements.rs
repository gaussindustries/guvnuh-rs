#[derive(Clone, Copy, Default)]
pub struct Measurements {
    pub rpm: f32,
    pub v_gen_rms: f32,
    pub i_gen_rms: f32,
    pub freq_gen_hz: f32,
    pub dc_bus_v: f32,
    pub temp_c: f32,
}
