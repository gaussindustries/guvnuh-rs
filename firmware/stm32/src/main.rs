// SPDX-License-Identifier: MIT OR Apache-2.0
#![no_std]
#![no_main]
defmt::timestamp!("{=u64:us}", { 0 });


use rtic::app;
use stm32h7xx_hal::{
    gpio::{self, GpioExt, Output, PushPull},
    prelude::*,
};

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI0, EXTI1])]
mod app {
    use super::*;
    use cortex_m::asm;
    use defmt_rtt as _;       // defmt logger
	use panic_probe as _;     // panic handler

    #[shared]
    pub struct Shared {
        ld1: gpio::Pin<'B', 0, Output<PushPull>>,
        ld2: gpio::Pin<'E', 1, Output<PushPull>>,
        ld3: gpio::Pin<'B', 14, Output<PushPull>>,
    }

    #[local]
    pub struct Local {
        // keep empty for now; add inputs / EXTI pins later
    }

    #[init]
	fn init(cx: init::Context) -> (Shared, Local) {
		defmt::info!("Initializing governor firmware (RTIC v2)");

		let dp = cx.device;

		// Power & clocks (H7 style)
		let pwr = dp.PWR.constrain();
		let pwrcfg = pwr.freeze();

		let rcc = dp.RCC.constrain();
		let ccdr = rcc
			.sysclk(200.MHz())               // choose your target
			.freeze(pwrcfg, &dp.SYSCFG);     // <-- REQUIRED on H7

		// GPIOB for LD1 (green) & LD3 (red)
		let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
		let mut ld1 = gpiob.pb0.into_push_pull_output();
		let mut ld3 = gpiob.pb14.into_push_pull_output();

		// GPIOE for LD2 (orange)
		let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
		let mut ld2 = gpioe.pe1.into_push_pull_output();
		ld1.set_low();
		ld2.set_low();
		ld3.set_low();

		(Shared { ld1, ld2, ld3 }, Local {})
	}

    // Simple blinky loop — replace with timers/EXTI later
    #[idle(shared = [ld1, ld2, ld3])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cx.shared.ld1.lock(|p| p.toggle());
            asm::delay(50_000_000);

            cx.shared.ld2.lock(|p| p.toggle());
            asm::delay(50_000_00);

            cx.shared.ld3.lock(|p| p.toggle());
            asm::delay(50_000_000);
        }
    }
	
}
