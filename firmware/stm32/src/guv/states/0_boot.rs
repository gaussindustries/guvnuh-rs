use stm32h7xx_hal::{
    device::TIM1,
    gpio::{self, Output, PushPull},
    pac,
    prelude::*,
    pwm,
    rcc::CoreClocks, // We only need CoreClocks, not the full Ccdr
};

pub struct Board {
    pub ld1: gpio::Pin<'B', 0, Output<PushPull>>,
    pub ld2: gpio::Pin<'E', 1, Output<PushPull>>,
    pub ld3: gpio::Pin<'B', 14, Output<PushPull>>,

    //this enables motor power, will be used in E-Stop
    pub relay: gpio::Pin<'E', 0, Output<PushPull>>,

    // CHANGED: Store only the Clock configuration, not the whole resource bag
    pub clocks: CoreClocks,

    pub motor_pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>,
}

pub fn setup(dp: pac::Peripherals) -> Board {
    // 1. Power & Clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();

    // 'ccdr' holds the Tokens (peripheral) and the Speeds (clocks)
    let ccdr = rcc
        .sysclk(64.MHz()) // Lower speed for stability
        .freeze(pwrcfg, &dp.SYSCFG);

    // 2. GPIO Split (Consumes GPIOB/E tokens from ccdr)
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // 3. Pin Config
    let mut ld1 = gpiob.pb0.into_push_pull_output();
    let mut ld2 = gpioe.pe1.into_push_pull_output();
    let mut ld3 = gpiob.pb14.into_push_pull_output();

    // PWM Setup
    let pwm_pin = gpioe.pe9.into_alternate::<1>();

    // Consumes TIM1 token from ccdr
    let mut motor_pwm = dp.TIM1.pwm(
        pwm_pin,
        20.kHz(),
        ccdr.peripheral.TIM1, // <--- Token MOVED here
        &ccdr.clocks,
    );

    motor_pwm.enable();
    motor_pwm.set_duty(motor_pwm.get_max_duty());
    let mut relay = gpioe.pe0.into_push_pull_output();

    // 4. Initial Safety State
    ld1.set_low();
    ld2.set_low();
    ld3.set_low();

    //we need to send our pwm signal into the pwm controller first before we enable power to the dc motor
    relay.set_low();

    Board {
        ld1,
        ld2,
        ld3,
        relay,
        motor_pwm,
        clocks: ccdr.clocks,
    }
}
