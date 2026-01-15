// SPDX-License-Identifier: MIT OR Apache-2.0
#![no_std]
#![no_main]

defmt::timestamp!("{=u64:us}", { 0 });

// Imports
use defmt_rtt as _; // link defmt logger
use panic_probe as _; // link panic handler

use fugit::ExtU64;
use rtic::app;
use rtic_monotonics::systick::prelude::*; // <- bring the macro prelude
use stm32h7xx_hal::{
    gpio::{self, GpioExt, Output, PushPull},
    prelude::*,
};

// Internal Modules
mod models {
    pub mod status;
} // Expose local status
mod app {
    pub mod states {
        pub mod boot; // Linked to 0_boot.rs via #[path] or direct
        pub mod calibrate; // Linked to 1_calibrate.rs
    }
}
// Shared Librarys (for esp32/stm32)
use crate::models::status::WorkerStatus;
use shared::models::state::states::STATE;

// Create a SysTick-based monotonic named `Mono` that ticks at 1 kHz
systick_monotonic!(Mono, 1_000);

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI0, EXTI1])]
mod app {
    use super::*;

    #[shared]
    pub struct Shared {
        //core GPIO
        ld1: gpio::Pin<'B', 0, Output<PushPull>>,  // green
        ld2: gpio::Pin<'E', 1, Output<PushPull>>,  // orange/yellur
        ld3: gpio::Pin<'B', 14, Output<PushPull>>, // red

        //funcs (funk)

        //structs
        pub state: STATE,
    }

    #[local]
    pub struct Local {}

    // --- THE INIT WRAPPER ---
        #[init]
        fn init(cx: init::Context) -> (Shared, Local) {
            defmt::info!("System Boot: Initializing...");

            // 1. DELEGATE TO BOOT.RS
            // The ugly hardware code is gone from main.rs!
            let board = crate::app::states::boot::setup(cx.device);

            // 2. START SCHEDULER
            let sys_freq = board.ccdr.clocks.sysclk().to_Hz();
            Mono::start(cx.core.SYST, sys_freq);

            // 3. SPAWN MANAGER
            state_manager::spawn().ok();

            // 4. RETURN RESOURCES
            (
                Shared {
                    state: STATE::BOOT, // Start at 0
                    ld1: board.ld1,
                    ld2: board.ld2,
                    ld3: board.ld3,
                },
                Local {},
            )
        }

        // --- THE CEO (State Manager) ---
        #[task(priority = 1, shared = [state, ld1, ld2, ld3])]
        fn state_manager(mut cx: state_manager::Context) {

            // 1. Get Timestamp (for timeouts/logic)
            let now = Mono::now().ticks();

            // 2. Lock Resources
            (cx.shared.state, cx.shared.ld2).lock(|state, ld1, ld2, ld3| {

                match *state {
                    // --- STATE 0: BOOT ---
                    STATE::BOOT => {
                        // Logic: Boot.rs already ran during init.
                        // We just confirm and move on.
                        defmt::info!("Boot Complete. Transitioning to CALIBRATE.");
                        *state = STATE::CALIBRATE;
                    },

                    // --- STATE 1: CALIBRATE ---
                    STATE::CALIBRATE => {
                        // Call the worker (Apollo 9 logic)
                        // We assume calibrate::run returns WorkerStatus
                        // let status = crate::app::states::calibrate::run(ld2, now);

                        // For now, let's mock it to verify the structure:
                        let status = WorkerStatus::Running; // Placeholder

                        match status {
                            WorkerStatus::Running => {
                                // Do nothing, wait for next loop
                                ld2.toggle(); // Visual indicator
                            },
                            WorkerStatus::Complete => {
                                defmt::info!("Calibration Passed. System IDLE.");
                                *state = STATE::IDLE;
                            },
                            WorkerStatus::Failed => {
                                defmt::error!("Calibration Failed!");
                                *state = STATE::FAULT;
                            }
                        }
                    },

                    // --- STATE 2: IDLE ---
                    STATE::IDLE => {
                        // Wait for start command...
                    },

                    // --- STATE 9: FAULT ---
                    STATE::FAULT => {
                        // Blink Red LED forever
                        // (We'll move this to 9_fault.rs later)
                    }

                    _ => {}
                }
            });

            // 3. The Heartbeat (Run again in 100ms)
            state_manager::spawn_after(100.millis()).ok();
        }
    }
    // 1 Hz: LD1
    #[task(shared = [ld1])]
    fn blink_ld1(mut cx: blink_ld1::Context) {
        cx.shared.ld1.lock(|p| p.toggle());
        blink_ld1::spawn_after(1.secs()).ok();
    }

    // 2 Hz: LD2
    #[task(shared = [ld2])]
    fn blink_ld2(mut cx: blink_ld2::Context) {
        cx.shared.ld2.lock(|p| p.toggle());
        blink_ld2::spawn_after(500.millis()).ok();
    }

    // A mini “light show” to show spawn_at accuracy
    #[task]
    fn sequence_demo(_: sequence_demo::Context) {
        let t0 = Mono::now();
        step_ld3::spawn_at(t0).ok(); // t0
        step_ld1::spawn_at(t0 + 100.millis()).ok(); // t0 + 100ms
        step_ld2::spawn_at(t0 + 200.millis()).ok(); // t0 + 200ms
        all_off::spawn_at(t0 + 400.millis()).ok(); // t0 + 400ms
    }

    #[task(shared = [ld3])]
    fn step_ld3(mut cx: step_ld3::Context) {
        cx.shared.ld3.lock(|p| p.set_high());
    }
    #[task(shared = [ld1])]
    fn step_ld1(mut cx: step_ld1::Context) {
        cx.shared.ld1.lock(|p| p.set_high());
    }
    #[task(shared = [ld2])]
    fn step_ld2(mut cx: step_ld2::Context) {
        cx.shared.ld2.lock(|p| p.set_high());
    }

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
