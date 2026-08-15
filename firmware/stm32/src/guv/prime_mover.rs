/// The interface between the control loop and whatever physically produces
/// shaft power. The control loop commands *demand* (0.0–1.0 of available
/// output) and requires an emergency-off; it does NOT know whether that demand
/// becomes an inverted-PWM duty on a DC motor or a valve position on a turbine.
///
/// This is the seam the Phase 4 turbine swaps into: a `TurbineController`
/// implementing this trait drops in without touching the PID, feedforward,
/// calibration, or state machine above it. Calibration generalizes with it —
/// what is duty→RPM today becomes demand→RPM generically, the same fit math
/// over a different prime mover's curve.
pub trait PrimeMover {
    /// Command output as a fraction of available: 0.0 = no output (safe idle),
    /// 1.0 = maximum. Clamped internally by the safety demand clamp. Ignored
    /// while disabled. This is what PID + feedforward drive.
    fn set_demand(&mut self, demand: f32);

    /// The last commanded demand fraction (post-clamp).
    fn demand(&self) -> f32;

    /// Arm the prime mover. Output stays off until the first `set_demand`.
    fn enable(&mut self);

    /// Disarm: force to safe idle and ignore demand until re-enabled.
    fn disable(&mut self);

    /// Force to the safe state IMMEDIATELY, bypassing enable state and any
    /// ramping. The safety supervisor calls this on a trip. For a DC motor this
    /// is duty→off; for a turbine, valve→closed (fail-safe). Must be the fastest
    /// path to safe and must not depend on `enabled`.
    fn emergency_off(&mut self);

    /// Limit maximum demand for safety. The control loop cannot command above
    /// this fraction. Applied on the next (and current) `set_demand`.
    fn set_max_demand(&mut self, clamp: f32);

    /// The current max-demand safety clamp.
    fn max_demand(&self) -> f32;

    /// Is the prime mover armed?
    fn is_enabled(&self) -> bool;
}
