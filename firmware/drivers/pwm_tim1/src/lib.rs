#![no_std]

use stm32h7xx_hal::{
    pac,
    rcc::{CoreClocks, rec},
    pwm::{ PwmExt, Pins, Ch, C1, ComplementaryDisabled},
    hal::PwmPin,      // trait for set_duty/enable/get_max_duty
    time::Hertz,
};

// Associated type alias: the channel produced for a given CH1 pin.
type Ch1Of<PIN> =
    <PIN as Pins<pac::TIM1, Ch<{ C1 }>, ComplementaryDisabled>>::Channel;

pub struct PwmTim1Ch1<CH> {
    ch1: CH,
}

impl PwmTim1Ch1<()> {
    /// Construct TIM1 CH1 PWM on a valid CH1 pin (e.g., PA8<AF1>, PE9<AF1>).
    pub fn new<PIN>(
        tim1: pac::TIM1,
        pin: PIN,
        tim1_rec: rec::Tim1,
        clocks: &CoreClocks,
        freq: Hertz,
    ) -> PwmTim1Ch1<Ch1Of<PIN>>
    where
        PIN: Pins<pac::TIM1, Ch<{ C1 }>, ComplementaryDisabled>,
        Ch1Of<PIN>: PwmPin<Duty = u16>,
    {
        let mut ch1: Ch1Of<PIN> = tim1.pwm(pin, freq, tim1_rec, clocks);
        ch1.set_duty(0);
        ch1.enable();
        PwmTim1Ch1 { ch1 }
    }
}

// Methods that work for any TIM1-CH1 channel type (normal or complementary)
impl<CH> PwmTim1Ch1<CH>
where
    CH: PwmPin<Duty = u16>,
{
    #[inline] pub fn set_duty(&mut self, duty: u16) { self.ch1.set_duty(duty); }
    #[inline] pub fn max_duty(&self) -> u16 { self.ch1.get_max_duty() }
    #[inline] pub fn enable(&mut self) { self.ch1.enable(); }
    #[inline] pub fn disable(&mut self) { self.ch1.disable(); }
}
