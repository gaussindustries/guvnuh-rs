use crate::guv::prime_mover::PrimeMover;
use cortex_m::prelude::_embedded_hal_PwmPin;
use stm32h7xx_hal::{device::TIM1, pwm};

/// DC-motor prime mover: inverted-PWM control of the shaft-drive motor.
/// Inverted: PWM duty=MAX means motor OFF, duty=0 means motor at 100%.
/// The inversion, TIM1 wiring, and max_duty are implementation details hidden
/// behind the PrimeMover interface — the control loop sees only demand 0.0–1.0.
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

    /// Internal: translate a demand fraction to the inverted PWM duty.
    fn apply_demand(&mut self, fraction: f32) {
        let inverted = self.max_duty as f32 * (1.0 - fraction);
        self.pwm.set_duty(inverted as u16);
    }
}

impl PrimeMover for MotorController {
    fn set_demand(&mut self, demand: f32) {
        if !self.enabled {
            return;
        }
        let clamped = demand.clamp(0.0, self.demand_clamp);
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
        self.pwm.set_duty(self.max_duty); // inverted: max duty = motor off
    }

    fn set_max_demand(&mut self, clamp: f32) {
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
