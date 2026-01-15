use stm32h7xx_hal::{
    gpio::{self, Output, PushPull},
    pac,
    prelude::*,
    rcc::{Ccdr, CoreClocks},
};
//CCDR = Core Clock Distribution and Reset
//
// The Bundle of Resources main.rs needs
pub struct Board {
    pub ld1: gpio::Pin<'B', 0, Output<PushPull>>,  // Green
    pub ld2: gpio::Pin<'E', 1, Output<PushPull>>,  // Yellow (Indicator)
    pub ld3: gpio::Pin<'B', 14, Output<PushPull>>, // Red (Fault)
    pub ccdr: Ccdr,                                // For System Clock
}

// The Setup Function
pub fn setup(dp: pac::Peripherals) -> Board {
    // 1. Power & Clocks
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sysclk(200.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // 2. GPIO Split
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // 3. Pin Config
    let mut ld1 = gpiob.pb0.into_push_pull_output();
    let mut ld2 = gpioe.pe1.into_push_pull_output();
    let mut ld3 = gpiob.pb14.into_push_pull_output();

    // 4. Initial Safety State (All Off)
    ld1.set_low();
    ld2.set_low();
    ld3.set_low();

    Board {
        ld1,
        ld2,
        ld3,
        ccdr,
    }
}
