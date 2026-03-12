// SPDX-License-Identifier: MIT OR Apache-2.0
#![no_std]
#![no_main]

defmt::timestamp!("{=u64:us}", { 0 });

// Imports
use core::fmt::Write;
use defmt_rtt as _; // link defmt logger
use panic_probe as _; // link panic handler

use rtic::app;
use rtic_monotonics::fugit::ExtU32;
use rtic_monotonics::systick::prelude::*;
use stm32h7xx_hal::{
    device::{TIM1, TIM2},
    gpio::{self, GpioExt, Output, PushPull},
    prelude::*,
    pwm,
    qei::Qei,
};

use cortex_m::peripheral::scb::SystemHandler;

// Internal Modules
mod guv;
mod models;
// Shared Librarys (for esp32/stm32)
use crate::models::status::WorkerStatus;
use shared::models::state::states::STATE;

// Create a SysTick-based monotonic named `Mono` that ticks at 1 kHz
systick_monotonic!(Mono, 1_000);

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use stm32h7xx_hal::qei::Qei;

    use super::*;

    #[shared]
    pub struct Shared {
        //core GPIO
        ld1: gpio::Pin<'B', 0, Output<PushPull>>,  // green
        ld2: gpio::Pin<'E', 1, Output<PushPull>>,  // orange/yellur
        ld3: gpio::Pin<'B', 14, Output<PushPull>>, // red

        relay: gpio::Pin<'E', 0, Output<PushPull>>,

        motor_pwm: pwm::Pwm<TIM1, 0, pwm::ComplementaryDisabled>,

        encoder: Qei<TIM2>,

        tx: stm32h7xx_hal::serial::Tx<stm32h7xx_hal::pac::UART4>,
        rx: stm32h7xx_hal::serial::Rx<stm32h7xx_hal::pac::UART4>,

        //funcs (funk)

        //structs
        pub state: STATE,
    }

    #[local]
    pub struct Local {
        calib_start_time: Option<u32>,
        safety_init_done: bool,
    }

    // --- THE INIT WRAPPER ---
    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("System Boot: Initializing...");

        // 1. DELEGATE TO BOOT.RS
        let mut board = crate::guv::states::boot::setup(cx.device);

        unsafe {
            cx.core.SCB.set_priority(SystemHandler::SysTick, 255);
        }

        // --- SAFETY: ENSURE MOTOR IS OFF ON BOOT ---
        // Inverted Logic: Set Duty to MAX to turn motor OFF.
        let max_duty = board.motor_pwm.get_max_duty();
        board.motor_pwm.set_duty(max_duty);

        // 2. START SCHEDULER
        let sys_freq = board.clocks.sysclk().to_Hz();
        Mono::start(cx.core.SYST, sys_freq);

        // 3. SPAWN MANAGER
        state_manager::spawn().ok();

        rpm_monitor::spawn().ok();

        board.ld3.set_high(); // Red LED on

        // 4. RETURN RESOURCES
        (
            Shared {
                state: STATE::BOOT,
                ld1: board.ld1,
                ld2: board.ld2,
                ld3: board.ld3,
                relay: board.relay,
                motor_pwm: board.motor_pwm,
                encoder: board.encoder,
                tx: board.tx,
                rx: board.rx,
            },
            Local {
                calib_start_time: None,
                safety_init_done: false,
            },
        )
    }

    // --- THE CEO (State Manager) ---
    #[task(priority = 1,
        shared = [
            state,
            ld1,
            ld2,
            ld3,
            relay,
            motor_pwm,
            tx,
            rx
        ],
        local = [
            calib_start_time,
            safety_init_done
        ])]
    async fn state_manager(mut cx: state_manager::Context) {
        let mut ticks: u32 = 0;

        loop {
            cx.shared.state.lock(|state| {
                match *state {
                    STATE::BOOT => {
                        defmt::info!("StateMachine: BOOT");

                        // SAFETY SEQUENCE:
                        // Step 1: Assert 0% speed (Max Duty for inverted logic)
                        if !*cx.local.safety_init_done {
                            cx.shared.motor_pwm.lock(|motor| {
                                let max_duty = motor.get_max_duty();
                                motor.set_duty(max_duty);
                            });
                            cx.shared.relay.lock(|relay| relay.set_low());
                            defmt::info!("Safety init complete.");
                            *cx.local.safety_init_done = true;
                        }

                        // Step 3: enable data stream, ensure it's being sent (uart for now, the server henceforth. perhaps file?)
                        cx.shared.tx.lock(|tx| {
                            writeln!(tx, "HELLO\r").ok();
                        });

                        // Try to read OK response (non-blocking)
                        let got_ok = cx.shared.rx.lock(|rx| {
                            let mut buf = [0u8; 4];
                            let mut i = 0;
                            while i < 4 {
                                match rx.read() {
                                    Ok(b) => {
                                        buf[i] = b;
                                        i += 1;
                                    }
                                    Err(_) => break,
                                }
                            }
                            &buf[..i] == b"OK\r\n" || &buf[..i] == b"OK\r"
                        });

                        if got_ok {
                            defmt::info!("Handshake OK — proceeding to CALIBRATE");
                            cx.shared.ld3.lock(|ld3| ld3.set_low());
                            cx.shared.ld2.lock(|ld2| ld2.set_high());
                            cx.shared.motor_pwm.lock(|motor| {
                                let max_duty = motor.get_max_duty();
                                motor.set_duty(max_duty);
                            });
                            cx.shared.relay.lock(|relay| relay.set_high());
                            *state = STATE::CALIBRATE;
                            ticks = 0;
                        } else {
                            // Still waiting — blink red LED
                            cx.shared.ld3.lock(|ld3| ld3.toggle());
                            defmt::info!("Waiting for ESP32 handshake...");
                            // stay in BOOT, loop will retry in 10ms
                        }
                        ticks = 0;
                    }

                    STATE::CALIBRATE => {
                        let elapsed_ms = ticks * 10;

                        cx.shared.motor_pwm.lock(|motor| {
                            let max_duty = motor.get_max_duty() as f32;

                            // CHANGE 1: Bump Target to 50% or 100% so it definitely moves
                            let target_speed_percent = 1.0; // 100% Speed

                            // --- PHASE 1: RAMP UP (0s -> 5s) ---
                            if elapsed_ms < 5000 {
                                // Progress: 0.0 -> 1.0
                                let progress = elapsed_ms as f32 / 5000.0;
                                let desired_speed = progress * target_speed_percent;

                                // INVERTED MATH (The Magic):
                                // Real Speed 0%   = Duty MAX
                                // Real Speed 100% = Duty 0
                                let inverted_duty = max_duty - (max_duty * desired_speed);
                                motor.set_duty(inverted_duty as u16);

                                if ticks % 10 == 0 {
                                    cx.shared.ld1.lock(|l| l.toggle());
                                }
                            }
                            // --- PHASE 2: HOLD (5s -> 10s) ---
                            else if elapsed_ms < 10000 {
                                // Hold steady at Target Speed
                                let desired_speed = target_speed_percent;

                                let inverted_duty = max_duty - (max_duty * desired_speed);
                                motor.set_duty(inverted_duty as u16);

                                cx.shared.ld1.lock(|l| l.set_high());
                            }
                            // --- PHASE 3: RAMP DOWN (10s -> 15s) ---
                            else if elapsed_ms < 15000 {
                                // Ramp down from Target to 0%
                                let ramp_down_progress = (elapsed_ms - 10000) as f32 / 5000.0;
                                let desired_speed =
                                    target_speed_percent * (1.0 - ramp_down_progress);

                                let inverted_duty = max_duty - (max_duty * desired_speed);
                                motor.set_duty(inverted_duty as u16);

                                if ticks % 20 == 0 {
                                    cx.shared.ld1.lock(|l| l.toggle());
                                }
                            }
                            // --- FINISHED ---
                            else {
                                // Force STOP (Inverted: Max Duty)
                                motor.set_duty(max_duty as u16);

                                // SAFETY: Shut off power to the controller
                                cx.shared.relay.lock(|r| r.set_low());

                                *state = STATE::IDLE;
                                defmt::info!("Calibration Complete. Motor & Relay OFF.");

                                cx.shared.relay.lock(|r| r.set_low());

                                cx.shared.ld2.lock(|l| l.set_low());
                                cx.shared.ld1.lock(|l| l.set_high());
                            }
                        });
                    }

                    STATE::IDLE => {
                        cx.shared.motor_pwm.lock(|motor| {
                            let max = motor.get_max_duty();
                            // Inverted: Max Duty = STOP
                            motor.set_duty(max);
                        });

                        if ticks % 100 == 0 {
                            cx.shared.ld3.lock(|ld3| ld3.toggle());
                        }
                    }
                    _ => {}
                }
            });

            Mono::delay(10u32.millis()).await;
            ticks += 1;
        }
    }

    #[task(priority = 1, shared = [encoder, tx])] // <-- Add tx here
    async fn rpm_monitor(mut cx: rpm_monitor::Context) {
        let mut last_count: u32 = 0;
        let counts_per_rev: f32 = 8192.0;
        let mut loop_counter: u32 = 0;

        loop {
            Mono::delay(10u32.millis()).await;

            // Lock BOTH the encoder and the transmitter
            (&mut cx.shared.encoder, &mut cx.shared.tx).lock(|enc, tx| {
                let current_count = enc.count();
                let delta_counts = current_count.wrapping_sub(last_count);
                last_count = current_count;

                let counts_per_second = (delta_counts as i32 as f32) * 100.0;
                let rpm = (counts_per_second / counts_per_rev) * 60.0 * -1.0;

                // Transmit over physical wire every 100 loops (1 second)
                if loop_counter % 100 == 0 {
                    defmt::info!("Motor RPM: {}", rpm as i32);

                    // THIS WRITES TO THE ESP32!
                    writeln!(tx, "RPM:{}\r", rpm as i32).ok();
                }
            });

            loop_counter = loop_counter.wrapping_add(1);
        }
    }
    // 1 Hz: LD1
    // #[task(shared = [ld1])]
    // async fn blink_ld1(mut cx: blink_ld1::Context) {
    //     cx.shared.ld1.lock(|p| p.toggle());
    //     Mono::delay(1.secs()).await;
    // }

    // // 2 Hz: LD2
    // #[task(shared = [ld2])]
    // async fn blink_ld2(mut cx: blink_ld2::Context) {
    //     cx.shared.ld2.lock(|p| p.toggle());
    //     Mono::delay(500.millis()).await;
    // }

    // // A mini “light show” to show spawn_at accuracy
    // // #[task]
    // // async fn sequence_demo(_: sequence_demo::Context) {
    // //     let t0 = Mono::now();
    // //     step_ld3::spawn_at(t0).ok(); // t0
    // //     step_ld1::spawn_at(t0 + 100.millis()).ok(); // t0 + 100ms
    // //     step_ld2::spawn_at(t0 + 200.millis()).ok(); // t0 + 200ms
    // //     all_off::spawn_at(t0 + 400.millis()).ok(); // t0 + 400ms
    // // }

    // #[task(shared = [ld3])]
    // async fn step_ld3(mut cx: step_ld3::Context) {
    //     cx.shared.ld3.lock(|p| p.set_high());
    // }
    // #[task(shared = [ld1])]
    // async fn step_ld1(mut cx: step_ld1::Context) {
    //     cx.shared.ld1.lock(|p| p.set_high());
    // }
    // #[task(shared = [ld2])]
    // async fn step_ld2(mut cx: step_ld2::Context) {
    //     cx.shared.ld2.lock(|p| p.set_high());
    // }

    // #[task(shared = [ld1, ld2, ld3])]
    // async fn all_off(mut cx: all_off::Context) {
    //     cx.shared.ld1.lock(|p| p.set_low());
    //     cx.shared.ld2.lock(|p| p.set_low());
    //     cx.shared.ld3.lock(|p| p.set_low());
    // }
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
