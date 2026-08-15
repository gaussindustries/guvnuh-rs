//! # TurbineController — Phase 4 prime-mover skeleton (NON-FUNCTIONAL)
//!
//! This is a **compile-checked design stub**, not working firmware. It exists
//! to prove that the [`PrimeMover`] trait genuinely generalizes: a steam turbine
//! is a completely different actuator from a PWM DC motor — a fail-closed steam
//! valve with thermal preconditions and slow mechanical dynamics — yet it
//! satisfies the *same* interface the control loop already drives. If this stub
//! implements `PrimeMover` honestly, the seam is real: the PID, feedforward,
//! calibration, safety supervisor, and state machine above the trait need no
//! changes to swap a motor for a turbine.
//!
//! Every method body is `todo!()` or a documented placeholder. This will not run
//! a turbine. It is here to (a) document the plug point in code, (b) surface the
//! turbine-specific concerns the interface must accommodate, and (c) let the
//! design question "does the abstraction hold?" be answered by the compiler
//! rather than by hand-waving.
//!
//! ## What changes vs. `MotorController`, and what doesn't
//!
//! **Doesn't change (the whole point):**
//! - `set_demand(0.0..1.0)` is still the command surface. PID + feedforward drive
//!   it identically. For the motor, demand → inverted PWM duty; for the turbine,
//!   demand → steam-valve position. The control loop cannot tell the difference.
//! - `emergency_off()` is still the supervisor's trip action. Motor: duty → off.
//!   Turbine: valve → **slammed shut** (fail-closed). Same contract: fastest path
//!   to safe, independent of `enabled`.
//! - Calibration generalizes for free: what is duty→RPM on the motor becomes
//!   demand→RPM on the turbine — the same least-squares fit over a different
//!   (and nonlinear) prime-mover curve. The `Calibrator` doesn't change.
//!
//! **Does change (hidden below the trait, as it should be):**
//! - The actuator is a **valve**, not a PWM channel. `set_demand` maps to a valve
//!   position through a nonlinear valve characteristic, not a linear duty.
//! - **Fail-closed is a hardware property**, not just a software default. The
//!   valve must spring/actuate closed on loss of power or command. `emergency_off`
//!   commands it, but the real safety is that the valve *cannot* stay open without
//!   active holding — the eventual hardware overspeed trip cuts the valve's
//!   holding signal directly.
//! - **Preconditions exist that a motor never had.** You cannot admit steam to a
//!   cold turbine (thermal shock) or an unpressurized one. `enable()` for a
//!   turbine implies checking that superheater temperature and header pressure
//!   are within a safe admit window — preconditions the DC motor had no analog to.
//!   These are noted here but belong in the sequencing layer (guard conditions),
//!   not baked into the actuator; the actuator only *actuates*.
//! - **Actuator dynamics are slow.** A valve takes real time to stroke. Where the
//!   motor's PWM is effectively instantaneous, the turbine's demand→output has
//!   actuator lag the control loop's tuning must account for. This does not change
//!   the *interface* — `set_demand` is still "command this fraction" — but it
//!   changes the plant the PID sees. That's a tuning concern, not an interface one,
//!   which is exactly why it stays below the trait.
//!
//! ## Why this is a skeleton and not a real driver
//!
//! A functional turbine controller needs the valve-actuator hardware driver
//! (stepper/servo/hydraulic), the valve characteristic curve, the fail-closed
//! wiring, and integration with the thermal/pressure sensing that gates admission.
//! None of that exists yet, and building it against no hardware would be fiction.
//! This stub commits only to the *shape* — the interface the turbine will present —
//! so the seam is visible and the trait's generality is proven at compile time.

#![allow(dead_code, unused_variables)] // skeleton: nothing here is wired up yet

use crate::guv::prime_mover::PrimeMover;

/// Where the fail-closed steam valve is commanded to sit, 0.0 (shut) to 1.0
/// (fully open). This is the turbine's analog of the motor's demand fraction —
/// but note it maps to *valve position*, which relates to shaft power through a
/// nonlinear valve characteristic, not the motor's linear duty.
type ValvePosition = f32;

/// Steam-turbine prime mover (PHASE 4 — SKELETON, DOES NOT RUN).
///
/// Implements [`PrimeMover`] to prove the control loop can drive a turbine with
/// zero changes above the trait. All actuator work is `todo!()`.
pub struct TurbineController {
    /// Commanded valve position (0.0 shut … 1.0 open), post-clamp.
    current_demand: ValvePosition,
    /// Safety clamp — the valve is never commanded past this fraction.
    demand_clamp: f32,
    /// Armed only after admit preconditions are met (thermal/pressure OK).
    /// Unlike the motor, `enabled` for a turbine implies the plant is in a state
    /// where admitting steam is *safe* — the preconditions themselves live in the
    /// sequencing layer; this flag only reflects "the actuator is permitted."
    enabled: bool,
    // FUTURE — the actual hardware, none of which exists yet:
    //   valve_actuator: <stepper / servo / hydraulic driver>,
    //   valve_curve:    <nonlinear position→effective-area map>,
    //   fail_closed:    <the holding signal whose ABSENCE closes the valve>,
    // The fail-closed element is the safety-critical one: the valve closes when
    // this signal drops, so the hardware overspeed trip can cut it directly,
    // independent of this software ever executing.
}

impl TurbineController {
    /// Construct in the safe state: valve shut, disarmed.
    ///
    /// SKELETON: a real constructor takes the valve-actuator driver and the valve
    /// characteristic, and asserts the valve is physically closed before returning.
    pub fn new(/* valve_actuator, valve_curve */) -> Self {
        let mut tc = Self {
            current_demand: 0.0,
            demand_clamp: 1.0,
            enabled: false,
        };
        tc.emergency_off(); // start with the valve commanded shut
        tc
    }

    /// Map a demand fraction (0.0–1.0) to a physical valve position through the
    /// valve's nonlinear characteristic, then command the actuator.
    ///
    /// This is the turbine's analog of `MotorController::apply_demand` (which does
    /// the inverted-PWM translation). The KEY difference: the motor's mapping is
    /// linear (`duty = max * (1 - fraction)`); the valve's is a nonlinear curve
    /// (equal-percentage, linear, or quick-opening trim) that must be characterized
    /// per valve. That nonlinearity is exactly what stays HIDDEN below the trait —
    /// the control loop commands demand, this method owns the valve physics.
    fn apply_valve_position(&mut self, demand: f32) {
        // todo!("map demand → valve position via valve_curve, drive valve_actuator")
    }
}

impl PrimeMover for TurbineController {
    fn set_demand(&mut self, demand: f32) {
        // Same contract as the motor: clamp, store, actuate; ignore if disarmed.
        // The turbine's clamp additionally protects against admitting more steam
        // than the plant can safely handle, but the INTERFACE is identical.
        if !self.enabled {
            return;
        }
        let clamped = demand.clamp(0.0, self.demand_clamp);
        self.current_demand = clamped;
        self.apply_valve_position(clamped);
    }

    fn demand(&self) -> f32 {
        self.current_demand
    }

    fn enable(&mut self) {
        // NOTE: for a turbine, arming is only safe once admit preconditions
        // (superheat temperature, header pressure) are satisfied — but those
        // guards live in the SEQUENCING layer, which decides *when* to call
        // enable(). The actuator itself only records that it is now permitted.
        // This mirrors the motor exactly at the interface; the difference is
        // entirely in what the sequencing layer checks before it calls this.
        self.enabled = true;
    }

    fn disable(&mut self) {
        // Disarm and command the valve shut. Same shape as the motor's disable.
        self.enabled = false;
        self.current_demand = 0.0;
        self.emergency_off();
    }

    fn emergency_off(&mut self) {
        // Supervisor trip action: slam the valve SHUT, immediately, regardless of
        // `enabled`. This is the turbine's `force_off`. Critically, in real
        // hardware the valve is FAIL-CLOSED — it closes on loss of the holding
        // signal — so this software command is the *normal-path* close, while the
        // hardware overspeed trip cuts the holding signal for the *last-resort*
        // close that doesn't depend on this code running at all.
        self.current_demand = 0.0;
        // todo!("command valve_actuator fully closed; assert fail-closed holding released")
    }

    fn set_max_demand(&mut self, clamp: f32) {
        // Identical contract to the motor: cap maximum demand, and if the current
        // command exceeds the new cap, reduce immediately.
        self.demand_clamp = clamp.clamp(0.0, 1.0);
        if self.current_demand > self.demand_clamp {
            self.set_demand(self.demand_clamp);
        }
    }

    fn max_demand(&self) -> f32 {
        self.demand_clamp
    }

    fn is_enabled(&self) -> bool {
        self.enabled
    }
}
