// SPDX-License-Identifier: MIT OR Apache-2.0
#![no_std]
#![no_main]
defmt::timestamp!("{=u64:us}", { 0 });

use defmt_rtt as _;        // link defmt logger
use panic_probe as _;      // link panic handler

use fugit::ExtU64;
use rtic::app;
use rtic_monotonics::systick::prelude::*; // <- bring the macro prelude
use stm32h7xx_hal::{
    gpio::{self, GpioExt, Output, PushPull},
    prelude::*,
};

// Create a SysTick-based monotonic named `Mono` that ticks at 1 kHz
systick_monotonic!(Mono, 1_000);

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI0, EXTI1])]
mod app {
    use super::*;

    #[shared]
    pub struct Shared {
        ld1: gpio::Pin<'B', 0, Output<PushPull>>,   // adjust pins to your board
        ld2: gpio::Pin<'E', 1, Output<PushPull>>,   // e.g., Nucleo H753ZI: LD2 is PE1
        ld3: gpio::Pin<'B', 14, Output<PushPull>>,
    }

    #[local]
    pub struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("rtic + monotonic boot");

        let dp = cx.device;

        // H7 power + clocks
        let pwr = dp.PWR.constrain();
        let pwrcfg = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let ccdr = rcc.sysclk(200.MHz()).freeze(pwrcfg, &dp.SYSCFG);

        // Start the SysTick monotonic at the system clock frequency (Hz)
        Mono::start(cx.core.SYST, ccdr.clocks.sysclk().to_Hz());

        // GPIO
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

        let mut ld1 = gpiob.pb0.into_push_pull_output();
        let mut ld2 = gpioe.pe1.into_push_pull_output();
        let mut ld3 = gpiob.pb14.into_push_pull_output();
        ld1.set_low();
        ld2.set_low();
        ld3.set_low();

        // Kick off periodic tasks
        blink_ld1::spawn().ok();
        blink_ld2::spawn().ok();

        // Demonstrate precise scheduled sequence on LD1/2/3 after 5 seconds
        sequence_demo::spawn_after(5.secs()).ok();

        (Shared { ld1, ld2, ld3 }, Local {})
    }

    // 1 Hz: LD1
    #[task(shared = [ld1])]
    async fn blink_ld1(mut cx: blink_ld1::Context) {
        cx.shared.ld1.lock(|p| p.toggle());
        blink_ld1::spawn_after(1.secs()).ok();
    }

    // 2 Hz: LD2
    #[task(shared = [ld2])]
    async fn blink_ld2(mut cx: blink_ld2::Context) {
        cx.shared.ld2.lock(|p| p.toggle());
        blink_ld2::spawn_after(500.millis()).ok();
    }

    // A mini “light show” to show spawn_at accuracy
    #[task]
    async fn sequence_demo(_: sequence_demo::Context) {
        let t0 = Mono::now();
        step_ld3::spawn_at(t0).ok();                       // t0
        step_ld1::spawn_at(t0 + 100.millis()).ok();        // t0 + 100ms
        step_ld2::spawn_at(t0 + 200.millis()).ok();        // t0 + 200ms
        all_off::spawn_at(t0 + 400.millis()).ok();         // t0 + 400ms
    }

    #[task(shared = [ld3])]
    fn step_ld3(mut cx: step_ld3::Context) { cx.shared.ld3.lock(|p| p.set_high()); }
    #[task(shared = [ld1])]
    fn step_ld1(mut cx: step_ld1::Context) { cx.shared.ld1.lock(|p| p.set_high()); }
    #[task(shared = [ld2])]
    fn step_ld2(mut cx: step_ld2::Context) { cx.shared.ld2.lock(|p| p.set_high()); }

    #[task(shared = [ld1, ld2, ld3])]
    fn all_off(mut cx: all_off::Context) {
        cx.shared.ld1.lock(|p| p.set_low());
        cx.shared.ld2.lock(|p| p.set_low());
        cx.shared.ld3.lock(|p| p.set_low());
    }
}

/*
	stm32 focuses on controlling the system, deffering the data collected to the esp32
	inputs{
		voltage output of load cell
			amperage 
			hence power output (watts)
		rpm of dc motor
		

	}
	outputs{
		pwm control for the dc motor
		data packets to the esp32 for data processing
			esp32 simply collects the data and focuses on outputting that data in real time to our webserver
	}

	overarching states{
		init:
			set up inputs
			set up outputs 
		ready:
			systems are nominal
		startup:
			spin up the dc motor, ensuring the rpm is being read properly
		idle:
			maintaining rpm/voltage output
		wind down:

		e stop:
			vfd for braking?
	}
	tasks:


*/
