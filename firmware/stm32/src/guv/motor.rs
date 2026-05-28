use cortex_m::prelude::_embedded_hal_PwmPin;
use stm32h7xx_hal::{device::TIM1, pwm};
/// Abstracts the inverted-PWM motor control logic.
/// Inverted: duty=MAX means motor OFF, duty=0 means motor at 100%.
pub struct MotorController {
    pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>,
    max_duty: u16,
    /// Safety clamp — never exceed this fraction (0.0–1.0)
    duty_clamp: f32,
    /// Current commanded speed as a fraction (0.0–1.0)
    current_speed: f32,
    enabled: bool,
}

impl MotorController {
    pub fn new(pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>) -> Self {
        let max_duty = pwm.get_max_duty();
        let mut mc = Self {
            pwm,
            max_duty,
            duty_clamp: 1.0,
            current_speed: 0.0,
            enabled: false,
        };
        mc.force_off();
        mc
    }

    /// Set speed as a fraction 0.0 (stopped) to 1.0 (full speed).
    /// Clamped by duty_clamp. Does nothing if not enabled.
    pub fn set_speed(&mut self, fraction: f32) {
        if !self.enabled {
            return;
        }
        let clamped = fraction.clamp(0.0, self.duty_clamp);
        self.current_speed = clamped;
        self.apply_duty(clamped);
    }

    /// Get current commanded speed fraction
    pub fn speed(&self) -> f32 {
        self.current_speed
    }

    /// Enable motor output (relay should also be enabled separately)
    pub fn enable(&mut self) {
        self.enabled = true;
    }

    /// Disable motor output and force PWM to off state
    pub fn disable(&mut self) {
        self.enabled = false;
        self.current_speed = 0.0;
        self.force_off();
    }

    /// Emergency: force motor off regardless of enable state
    pub fn force_off(&mut self) {
        self.current_speed = 0.0;
        self.pwm.set_duty(self.max_duty); // inverted: max duty = motor off
    }

    /// Update the safety clamp. Takes effect on next set_speed call.
    pub fn set_duty_clamp(&mut self, clamp: f32) {
        self.duty_clamp = clamp.clamp(0.0, 1.0);
        // If current speed exceeds new clamp, reduce immediately
        if self.current_speed > self.duty_clamp {
            self.set_speed(self.duty_clamp);
        }
    }

    pub fn duty_clamp(&self) -> f32 {
        self.duty_clamp
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }

    /// Internal: apply a speed fraction to the inverted PWM
    fn apply_duty(&mut self, fraction: f32) {
        let inverted = self.max_duty as f32 * (1.0 - fraction);
        self.pwm.set_duty(inverted as u16);
    }
}
