use crate::guv::prime_mover::PrimeMover;
use cortex_m::prelude::_embedded_hal_PwmPin;
use stm32h7xx_hal::{device::TIM1, pwm};

/// DC-motor prime mover: PWM duty proportional to demand.
/// duty = 0 means motor OFF; duty = max means full command.
/// PE9 (TIM1_CH1, Arduino D6) → 74HCT125 → 2-pole RC → 3296 trimmer → SI-6 SIG,
/// so 0% duty is 0 V on SIG. A reset, a crash, a disabled timer or an
/// unpowered Nucleo all land on stopped.
/// TIM1 wiring and max_duty are implementation details hidden behind the
/// PrimeMover interface — the control loop sees only demand 0.0–1.0.
pub struct MotorController {
    pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>,
    max_duty: u16,
    /// Safety clamp — never exceed this demand fraction (0.0–1.0)
    demand_clamp: f32,
    /// Current commanded demand as a fraction (0.0–1.0)
    current_demand: f32,
    enabled: bool,
}

impl MotorController {
    pub fn new(pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>) -> Self {
        let max_duty = pwm.get_max_duty();
        let mut mc = Self {
            pwm,
            max_duty,
            demand_clamp: 1.0,
            current_demand: 0.0,
            enabled: false,
        };
        mc.emergency_off();
        mc
    }

    /// Motor-specific: the raw max PWM duty value (for diagnostics/tests).
    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }

    /// Internal: translate a demand fraction to the PWM duty.
    fn apply_demand(&mut self, fraction: f32) {
        let duty = self.max_duty as f32 * fraction.clamp(0.0, 1.0);
        self.pwm.set_duty(duty as u16);
    }
}

impl PrimeMover for MotorController {
    fn set_demand(&mut self, demand: f32) {
        if !self.enabled {
            return;
        }
        // A NaN or infinite demand (bad feedforward, corrupt config) means stop.
        let clamped = if demand.is_finite() {
            demand.clamp(0.0, self.demand_clamp)
        } else {
            0.0
        };
        self.current_demand = clamped;
        self.apply_demand(clamped);
    }

    fn demand(&self) -> f32 {
        self.current_demand
    }

    fn enable(&mut self) {
        self.enabled = true;
    }

    fn disable(&mut self) {
        self.enabled = false;
        self.current_demand = 0.0;
        self.emergency_off();
    }

    fn emergency_off(&mut self) {
        self.current_demand = 0.0;
        self.pwm.set_duty(0); // 0% duty = 0 V on SIG = motor off
    }

    fn set_max_demand(&mut self, clamp: f32) {
        // f32::clamp panics on a NaN bound, so a NaN clamp from the server must
        // never reach demand_clamp. Treat it as 0 (no drive).
        self.demand_clamp = if clamp.is_finite() {
            clamp.clamp(0.0, 1.0)
        } else {
            0.0
        };
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
