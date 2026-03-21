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
use nb::block;
use shared::models::state::states::STATE;
use shared::models::telemetry::telemetry::Command;
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
        cmd_buf: [u8; 64], // COBS accumulation buffer for incoming commands
        cmd_len: usize,    // current position in cmd_buf
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
                cmd_buf: [0u8; 64],
                cmd_len: 0,
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
            safety_init_done,
            cmd_buf,
            cmd_len
        ])]
    async fn state_manager(mut cx: state_manager::Context) {
        let mut ticks: u32 = 0;

        loop {
            let current_state = cx.shared.state.lock(|s| *s);
            // ── Check UART for incoming COBS-encoded commands (non-blocking) ──
            // This runs BEFORE the state lock so we have access to local resources
            // without nesting inside the lock closure.
            let received_cmd: Option<Command> = if current_state != STATE::BOOT {
                let cmd_buf = &mut *cx.local.cmd_buf;
                let cmd_len = &mut *cx.local.cmd_len;
                cx.shared.rx.lock(|rx| {
                    loop {
                        match rx.read() {
                            Ok(b) => {
                                if b == 0x00 {
                                    if *cmd_len > 0 {
                                        if let Ok(cmd) = postcard::from_bytes_cobs::<Command>(
                                            &mut cmd_buf[..*cmd_len],
                                        ) {
                                            *cmd_len = 0;
                                            return Some(cmd);
                                        }
                                        *cmd_len = 0;
                                    }
                                } else if *cmd_len < cmd_buf.len() {
                                    cmd_buf[*cmd_len] = b;
                                    *cmd_len += 1;
                                } else {
                                    *cmd_len = 0;
                                }
                            }
                            Err(_) => break,
                        }
                    }
                    None
                })
            } else {
                None
            };

            // ── State machine ──
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

                        // Send HELLO to ESP32 for handshake
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
                            defmt::info!("Handshake OK — waiting for Start command");
                            cx.shared.ld3.lock(|ld3| ld3.set_low());
                            cx.shared.ld2.lock(|ld2| ld2.set_high()); // yellow = idle/waiting
                            cx.shared.motor_pwm.lock(|motor| {
                                let max_duty = motor.get_max_duty();
                                motor.set_duty(max_duty); // motor OFF
                            });
                            // Do NOT enable relay — wait for Start command
                            *state = STATE::IDLE;
                            ticks = 0;
                        } else {
                            // Still waiting — blink red LED
                            cx.shared.ld3.lock(|ld3| ld3.toggle());
                            defmt::info!("Waiting for ESP32 handshake...");
                        }
                        ticks = 0;
                    }

                    STATE::IDLE => {
                        // Motor stays OFF
                        cx.shared.motor_pwm.lock(|motor| {
                            let max = motor.get_max_duty();
                            motor.set_duty(max);
                        });

                        // Blink yellow = waiting for Start command
                        if ticks % 50 == 0 {
                            cx.shared.ld2.lock(|ld2| ld2.toggle());
                        }

                        // Check for Start command from desktop app
                        if let Some(Command::Start) = received_cmd {
                            defmt::info!("START received — beginning calibration");
                            cx.shared.ld2.lock(|ld2| ld2.set_low());
                            cx.shared.ld1.lock(|ld1| ld1.set_high()); // green = running
                            cx.shared.relay.lock(|relay| relay.set_high()); // power on
                            *state = STATE::CALIBRATE;
                            ticks = 0;
                        }
                    }

                    STATE::CALIBRATE => {
                        let elapsed_ms = ticks * 10;

                        // Check for Stop command during calibration
                        if let Some(Command::Stop) = received_cmd {
                            defmt::info!("STOP received — aborting calibration");
                            cx.shared.motor_pwm.lock(|motor| {
                                let max_duty = motor.get_max_duty();
                                motor.set_duty(max_duty as u16); // motor OFF
                            });
                            cx.shared.relay.lock(|r| r.set_low()); // power off
                            cx.shared.ld1.lock(|l| l.set_low());
                            cx.shared.ld2.lock(|l| l.set_high()); // yellow = idle
                            *state = STATE::IDLE;
                            ticks = 0;
                            return;
                        }

                        cx.shared.motor_pwm.lock(|motor| {
                            let max_duty = motor.get_max_duty() as f32;

                            let target_speed_percent = 1.0; // 100% Speed

                            // --- PHASE 1: RAMP UP (0s -> 5s) ---
                            if elapsed_ms < 5000 {
                                let progress = elapsed_ms as f32 / 5000.0;
                                let desired_speed = progress * target_speed_percent;

                                // INVERTED MATH:
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
                                let desired_speed = target_speed_percent;

                                let inverted_duty = max_duty - (max_duty * desired_speed);
                                motor.set_duty(inverted_duty as u16);

                                cx.shared.ld1.lock(|l| l.set_high());
                            }
                            // --- PHASE 3: RAMP DOWN (10s -> 15s) ---
                            else if elapsed_ms < 15000 {
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
                                defmt::info!(
                                    "Calibration Complete. Motor & Relay OFF. Returning to IDLE."
                                );

                                cx.shared.ld1.lock(|l| l.set_low());
                                cx.shared.ld2.lock(|l| l.set_high()); // yellow = idle again
                            }
                        });
                    }

                    _ => {}
                }
            });

            Mono::delay(10u32.millis()).await;
            ticks += 1;
        }
    }

    #[task(priority = 1, shared = [encoder, tx, state])]
    async fn rpm_monitor(mut cx: rpm_monitor::Context) {
        let mut last_count: u32 = 0;
        let counts_per_rev: f32 = 8192.0;
        let mut loop_counter: u32 = 0;
        let mut ts_ms: u32 = 0;

        loop {
            Mono::delay(10u32.millis()).await;
            ts_ms = ts_ms.wrapping_add(10);

            (
                &mut cx.shared.encoder,
                &mut cx.shared.tx,
                &mut cx.shared.state,
            )
                .lock(|enc, tx, state| {
                    let current_count = enc.count();
                    let delta_counts = current_count.wrapping_sub(last_count);
                    last_count = current_count;

                    let counts_per_second = (delta_counts as i32 as f32) * 100.0;
                    let rpm = (counts_per_second / counts_per_rev) * 60.0 * -1.0;

                    // Build telemetry frame
                    let frame = shared::models::telemetry::telemetry::Telemetry {
                        ts_ms,
                        state: *state,
                        rpm,
                        v_gen_rms: 0.0,
                        i_gen_rms: 0.0,
                        freq_gen_hz: 0.0,
                        theta_err_rad: 0.0,
                        temp_c: 0.0,
                        dc_bus_v: 0.0,
                    };

                    // Serialize to COBS — max postcard size for this struct is ~40 bytes
                    if *state != STATE::BOOT {
                        let mut buf = [0u8; 64];
                        if let Ok(encoded) = postcard::to_slice_cobs(&frame, &mut buf) {
                            for byte in encoded.iter() {
                                block!(tx.write(*byte)).ok();
                            }
                        }
                    }

                    if loop_counter % 100 == 0 {
                        defmt::info!("RPM: {} STATE: {}", rpm as i32, state.as_str());
                    }
                });

            loop_counter = loop_counter.wrapping_add(1);
        }
    }
}
