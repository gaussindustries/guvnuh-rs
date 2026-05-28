// SPDX-License-Identifier: MIT OR Apache-2.0
#![no_std]
#![no_main]

defmt::timestamp!("{=u64:us}", { 0 });

use core::fmt::Write;
use defmt_rtt as _;
use panic_probe as _;

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

mod guv;
mod models;

use crate::models::status::WorkerStatus;
use nb::block;
use shared::models::state::states::{Fault, STATE};
use shared::models::telemetry::telemetry::{Command, LiveParam, PidGains, RunConfig, RunMode};

systick_monotonic!(Mono, 1_000);

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use super::*;
    use stm32h7xx_hal::qei::Qei;
    type Motor = crate::guv::motor::MotorController;
    type Cfg = Option<RunConfig>;
    #[shared]
    pub struct Shared {
        // GPIO
        ld1: gpio::Pin<'B', 0, Output<PushPull>>,  // green
        ld2: gpio::Pin<'E', 1, Output<PushPull>>,  // yellow
        ld3: gpio::Pin<'B', 14, Output<PushPull>>, // red
        relay: gpio::Pin<'E', 0, Output<PushPull>>,

        // Motor (wraps inverted PWM logic)
        motor: crate::guv::motor::MotorController,

        // Encoder
        encoder: Qei<TIM2>,

        // UART to ESP32
        tx: stm32h7xx_hal::serial::Tx<stm32h7xx_hal::pac::UART4>,
        rx: stm32h7xx_hal::serial::Rx<stm32h7xx_hal::pac::UART4>,

        // State
        pub state: STATE,
        pub run_config: Option<RunConfig>,
        pub current_rpm: f32,
        pub last_fault: Option<Fault>,
    }

    #[local]
    pub struct Local {
        safety_init_done: bool,
        cmd_buf: [u8; 128],
        cmd_len: usize,
        pid: crate::guv::pid::PidController,
        ramp: Option<crate::guv::ramp::Ramp>,
        run_elapsed_ms: u32,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("System Boot: Initializing...");

        let mut board = crate::guv::states::boot::setup(cx.device);

        unsafe {
            cx.core.SCB.set_priority(SystemHandler::SysTick, 255);
        }

        // Wrap PWM in MotorController — constructor forces motor off
        let motor = crate::guv::motor::MotorController::new(board.motor_pwm);

        let sys_freq = board.clocks.sysclk().to_Hz();
        Mono::start(cx.core.SYST, sys_freq);

        state_manager::spawn().ok();
        rpm_monitor::spawn().ok();

        board.ld3.set_high(); // Red = booting

        (
            Shared {
                state: STATE::BOOT,
                ld1: board.ld1,
                ld2: board.ld2,
                ld3: board.ld3,
                relay: board.relay,
                motor,
                encoder: board.encoder,
                tx: board.tx,
                rx: board.rx,
                run_config: None,
                current_rpm: 0.0,
                last_fault: None,
            },
            Local {
                safety_init_done: false,
                cmd_buf: [0u8; 128],
                cmd_len: 0,
                pid: crate::guv::pid::PidController::new(PidGains::default()),
                ramp: None,
                run_elapsed_ms: 0,
            },
        )
    }

    // ────────────────────────────────────────────
    //  State Manager — the main control loop
    // ────────────────────────────────────────────
    #[task(priority = 1,
        shared = [state, ld1, ld2, ld3, relay, motor, tx, rx, run_config, current_rpm, last_fault],
        local = [safety_init_done, cmd_buf, cmd_len, pid, ramp, run_elapsed_ms]
    )]
    async fn state_manager(mut cx: state_manager::Context) {
        let dt_ms: u32 = 10;

        loop {
            // ── Read incoming COBS command (non-blocking) ──
            let current_state = cx.shared.state.lock(|s| *s);

            let received_cmd: Option<Command> = if current_state != STATE::BOOT {
                let buf = &mut *cx.local.cmd_buf;
                let len = &mut *cx.local.cmd_len;
                cx.shared.rx.lock(|rx| {
                    loop {
                        match rx.read() {
                            Ok(b) => {
                                if b == 0x00 {
                                    if *len > 0 {
                                        if let Ok(cmd) =
                                            postcard::from_bytes_cobs::<Command>(&mut buf[..*len])
                                        {
                                            *len = 0;
                                            return Some(cmd);
                                        }
                                        *len = 0;
                                    }
                                } else if *len < buf.len() {
                                    buf[*len] = b;
                                    *len += 1;
                                } else {
                                    *len = 0;
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

            // ── ESTOP — always honored, any state ──
            if let Some(Command::EmergencyStop) = received_cmd {
                defmt::warn!("!! EMERGENCY STOP !!");
                cx.shared.motor.lock(|m| m.force_off());
                cx.shared.relay.lock(|r| r.set_low());
                cx.shared.state.lock(|s| *s = STATE::ESTOP);
                cx.local.pid.reset();
                *cx.local.ramp = None;
                *cx.local.run_elapsed_ms = 0;
                cx.shared.ld1.lock(|l| l.set_low());
                cx.shared.ld2.lock(|l| l.set_low());
                cx.shared.ld3.lock(|l| l.set_high()); // red solid
                Mono::delay(dt_ms.millis()).await;
                continue;
            }

            // ── State machine ──
            match current_state {
                // ─── BOOT: safety init + ESP32 handshake ───
                STATE::BOOT => {
                    if !*cx.local.safety_init_done {
                        cx.shared.motor.lock(|m| m.force_off());
                        cx.shared.relay.lock(|r| r.set_low());
                        defmt::info!("Safety init complete.");
                        *cx.local.safety_init_done = true;
                    }

                    cx.shared.tx.lock(|tx| {
                        writeln!(tx, "HELLO\r").ok();
                    });

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
                        defmt::info!("Handshake OK — entering IDLE");
                        cx.shared.ld3.lock(|l| l.set_low());
                        cx.shared.ld2.lock(|l| l.set_high());
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                    } else {
                        cx.shared.ld3.lock(|l| l.toggle());
                    }
                }

                // ─── IDLE: motor off, waiting for Configure ───
                STATE::IDLE => {
                    cx.shared.motor.lock(|m| m.disable());

                    match received_cmd {
                        Some(Command::Configure(config)) => {
                            defmt::info!(
                                "Config received: mode={} target_rpm={}",
                                match config.mode {
                                    RunMode::OpenLoop => "OpenLoop",
                                    RunMode::ClosedLoop => "ClosedLoop",
                                    RunMode::Calibrate => "Calibrate",
                                    RunMode::Manual => "Manual",
                                    RunMode::Generate => "Generate",
                                },
                                config.target_rpm as i32
                            );
                            cx.shared
                                .motor
                                .lock(|m| m.set_duty_clamp(config.max_duty_clamp));
                            cx.shared.run_config.lock(|rc| *rc = Some(config));
                            cx.shared.last_fault.lock(|f| *f = None);
                            cx.shared.state.lock(|s| *s = STATE::CONFIGURED);
                            // Green solid = configured
                            cx.shared.ld2.lock(|l| l.set_low());
                            cx.shared.ld1.lock(|l| l.set_high());
                        }
                        Some(Command::ClearFaults) => {
                            cx.shared.last_fault.lock(|f| *f = None);
                            defmt::info!("Faults cleared");
                        }
                        _ => {
                            // Slow yellow blink = idle
                            static mut IDLE_TICK: u32 = 0;
                            unsafe {
                                IDLE_TICK += 1;
                            }
                            if unsafe { IDLE_TICK } % 50 == 0 {
                                cx.shared.ld2.lock(|l| l.toggle());
                            }
                        }
                    }
                }

                // ─── CONFIGURED: RunConfig loaded, waiting for Start ───
                STATE::CONFIGURED => {
                    match received_cmd {
                        Some(Command::Start) => {
                            let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());
                            defmt::info!(
                                "START — mode={}",
                                match config.mode {
                                    RunMode::OpenLoop => "OpenLoop",
                                    RunMode::ClosedLoop => "ClosedLoop",
                                    RunMode::Calibrate => "Calibrate",
                                    RunMode::Manual => "Manual",
                                    RunMode::Generate => "Generate",
                                }
                            );

                            // Power on
                            cx.shared.relay.lock(|r| r.set_high());
                            cx.shared.motor.lock(|m| m.enable());
                            cx.local.pid.reset();
                            *cx.local.run_elapsed_ms = 0;

                            // Set up ramp for non-manual modes
                            if config.mode != RunMode::Manual {
                                *cx.local.ramp =
                                    Some(crate::guv::ramp::Ramp::new(0.0, 1.0, config.ramp_up_ms));
                            } else {
                                *cx.local.ramp = None;
                            }

                            if config.mode == RunMode::ClosedLoop
                                || config.mode == RunMode::Generate
                            {
                                cx.local.pid.set_gains(config.pid);
                            }

                            let next = match config.mode {
                                RunMode::Calibrate => STATE::CALIBRATE,
                                RunMode::Manual => STATE::MANUAL,
                                RunMode::Generate => STATE::SPOOLUP,
                                _ => STATE::SPOOLUP, // OpenLoop and ClosedLoop also spool up
                            };
                            cx.shared.state.lock(|s| *s = next);
                        }
                        Some(Command::Configure(config)) => {
                            // Allow reconfiguration before Start
                            defmt::info!("Reconfigure received");
                            cx.shared
                                .motor
                                .lock(|m| m.set_duty_clamp(config.max_duty_clamp));
                            cx.shared.run_config.lock(|rc| *rc = Some(config));
                        }
                        Some(Command::Stop) => {
                            defmt::info!("Stop in CONFIGURED — back to IDLE");
                            cx.shared.run_config.lock(|rc| *rc = None);
                            cx.shared.state.lock(|s| *s = STATE::IDLE);
                            cx.shared.ld1.lock(|l| l.set_low());
                            cx.shared.ld2.lock(|l| l.set_high());
                        }
                        _ => {
                            // Steady green = ready
                            cx.shared.ld1.lock(|l| l.set_high());
                        }
                    }
                }

                // ─── CALIBRATE: simple motor test sequence ───
                STATE::CALIBRATE => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());
                    *cx.local.run_elapsed_ms += dt_ms;
                    let elapsed = *cx.local.run_elapsed_ms;

                    if let Some(Command::Stop) = received_cmd {
                        defmt::info!("STOP during calibration");
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // Phase 1: ramp up
                    if elapsed < config.ramp_up_ms {
                        let progress = elapsed as f32 / config.ramp_up_ms as f32;
                        cx.shared.motor.lock(|m| m.set_speed(progress));
                        if (elapsed / 100) % 2 == 0 {
                            cx.shared.ld1.lock(|l| l.toggle());
                        }
                    }
                    // Phase 2: hold
                    else if config.hold_ms == 0 || elapsed < config.ramp_up_ms + config.hold_ms {
                        cx.shared.motor.lock(|m| m.set_speed(1.0));
                        cx.shared.ld1.lock(|l| l.set_high());

                        // If hold_ms == 0, hold indefinitely (until Stop)
                        if config.hold_ms == 0 {
                            // just hold
                        }
                    }
                    // Phase 3: ramp down
                    else if elapsed < config.ramp_up_ms + config.hold_ms + config.ramp_down_ms {
                        let rd_elapsed = elapsed - config.ramp_up_ms - config.hold_ms;
                        let progress = 1.0 - (rd_elapsed as f32 / config.ramp_down_ms as f32);
                        cx.shared.motor.lock(|m| m.set_speed(progress));
                        if (elapsed / 200) % 2 == 0 {
                            cx.shared.ld1.lock(|l| l.toggle());
                        }
                    }
                    // Done
                    else {
                        defmt::info!("Calibration complete — IDLE");
                        cx.shared.motor.lock(|m| m.disable());
                        cx.shared.relay.lock(|r| r.set_low());
                        *cx.local.run_elapsed_ms = 0;
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                        cx.shared.ld1.lock(|l| l.set_low());
                        cx.shared.ld2.lock(|l| l.set_high());
                    }
                }

                // ─── SPOOLUP: ramp motor to target RPM ───
                STATE::SPOOLUP => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());
                    *cx.local.run_elapsed_ms += dt_ms;

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // Apply live adjustments
                    if let Some(Command::LiveAdjust(param)) = received_cmd {
                        match param {
                            LiveParam::Duty(d) => {
                                cx.shared.motor.lock(
                                    |m: &mut crate::guv::motor::MotorController| m.set_speed(d),
                                );
                            }
                            LiveParam::TargetRpm(rpm) => {
                                cx.shared.run_config.lock(|rc: &mut Option<RunConfig>| {
                                    if let Some(ref mut c) = rc {
                                        c.target_rpm = rpm;
                                    }
                                });
                            }
                            LiveParam::PidGains(gains) => {
                                cx.local.pid.set_gains(gains);
                            }
                            LiveParam::MaxDutyClamp(clamp) => {
                                cx.shared.motor.lock(
                                    |m: &mut crate::guv::motor::MotorController| {
                                        m.set_duty_clamp(clamp)
                                    },
                                );
                            }
                            LiveParam::TargetFreqHz(hz) => {
                                cx.shared.run_config.lock(|rc: &mut Option<RunConfig>| {
                                    if let Some(ref mut c) = rc {
                                        c.target_freq_hz = hz;
                                    }
                                });
                            }
                            LiveParam::TargetVRms(v) => {
                                cx.shared.run_config.lock(|rc: &mut Option<RunConfig>| {
                                    if let Some(ref mut c) = rc {
                                        c.target_v_rms = v;
                                    }
                                });
                            }
                        }
                    }

                    // Ramp up
                    if let Some(ref mut ramp) = cx.local.ramp {
                        let val = ramp.tick(dt_ms);
                        match config.mode {
                            RunMode::ClosedLoop | RunMode::Generate => {
                                let target_rpm = config.target_rpm * val;
                                let measured = cx.shared.current_rpm.lock(|r| *r);
                                let dt_s = dt_ms as f32 / 1000.0;
                                let output = cx.local.pid.update(target_rpm, measured, dt_s);
                                cx.shared.motor.lock(|m| m.set_speed(output));
                            }
                            _ => {
                                cx.shared.motor.lock(|m| m.set_speed(val));
                            }
                        }

                        if ramp.is_done() {
                            *cx.local.ramp = None;
                            defmt::info!("Spoolup complete");

                            // For Generate mode, advance through the sequence
                            // For OpenLoop/ClosedLoop, stay in steady state here
                            match config.mode {
                                RunMode::Generate => {
                                    cx.shared.state.lock(|s| *s = STATE::EXCITE);
                                    defmt::info!("→ EXCITE");
                                }
                                _ => {
                                    // OpenLoop/ClosedLoop: just hold at speed
                                    // (stays in SPOOLUP as a "running" state)
                                }
                            }
                        }
                    } else {
                        // Post-ramp steady state for OpenLoop/ClosedLoop
                        match config.mode {
                            RunMode::ClosedLoop => {
                                let measured = cx.shared.current_rpm.lock(|r| *r);
                                let dt_s = dt_ms as f32 / 1000.0;
                                let output = cx.local.pid.update(config.target_rpm, measured, dt_s);
                                cx.shared.motor.lock(|m| m.set_speed(output));
                            }
                            _ => {
                                // OpenLoop holds whatever duty the ramp ended at
                            }
                        }

                        // Check hold timeout (0 = indefinite)
                        if config.hold_ms > 0
                            && *cx.local.run_elapsed_ms >= config.ramp_up_ms + config.hold_ms
                        {
                            defmt::info!("Hold complete — ramping down");
                            let speed = cx.shared.motor.lock(|m| m.speed());
                            *cx.local.ramp =
                                Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                            cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        }
                    }

                    // LED: fast green blink during spoolup
                    if (*cx.local.run_elapsed_ms / 100) % 2 == 0 {
                        cx.shared.ld1.lock(|l| l.toggle());
                    }
                }

                // ─── EXCITE: energize capacitor bank (stub for now) ───
                STATE::EXCITE => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // TODO: monitor v_gen_rms from ADC
                    // When voltage builds to target_v_rms, advance to PLL_LOCK
                    // For now, auto-advance after 3 seconds as placeholder
                    *cx.local.run_elapsed_ms += dt_ms;
                    if *cx.local.run_elapsed_ms > config.ramp_up_ms + 3000 {
                        defmt::info!("Excitation nominal → PLL_LOCK");
                        cx.shared.state.lock(|s| *s = STATE::PLL_LOCK);
                    }

                    // LED: yellow fast blink
                    cx.shared.ld2.lock(|l| l.toggle());
                }

                // ─── PLL_LOCK: phase-lock to reference (stub) ───
                STATE::PLL_LOCK => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // TODO: phase detector + loop filter on theta_err_rad
                    // When |theta_err| < threshold, advance to READY
                    // Placeholder: auto-advance after 2 seconds
                    *cx.local.run_elapsed_ms += dt_ms;
                    if *cx.local.run_elapsed_ms > config.ramp_up_ms + 5000 {
                        defmt::info!("PLL locked → READY");
                        cx.shared.state.lock(|s| *s = STATE::READY);
                        // One green blink to signal lock
                        cx.shared.ld1.lock(|l| l.set_high());
                    }

                    // LED: yellow blink slowing as "lock" approaches
                    let elapsed_in_pll = *cx.local.run_elapsed_ms - config.ramp_up_ms - 3000;
                    let blink_rate = 50u32.saturating_sub(elapsed_in_pll / 100).max(5);
                    if (elapsed_in_pll / 10) % blink_rate < blink_rate / 2 {
                        cx.shared.ld2.lock(|l| l.set_high());
                    } else {
                        cx.shared.ld2.lock(|l| l.set_low());
                    }
                }

                // ─── READY: locked, gate closed, waiting for load ───
                STATE::READY => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // Maintain PID on RPM to hold speed
                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let dt_s = dt_ms as f32 / 1000.0;
                    let output = cx.local.pid.update(config.target_rpm, measured, dt_s);
                    cx.shared.motor.lock(|m| m.set_speed(output));

                    // Apply live adjustments
                    if let Some(Command::LiveAdjust(param)) = received_cmd {
                        match param {
                            LiveParam::Duty(d) => {
                                cx.shared.motor.lock(
                                    |m: &mut crate::guv::motor::MotorController| m.set_speed(d),
                                );
                            }
                            LiveParam::TargetRpm(rpm) => {
                                cx.shared.run_config.lock(|rc: &mut Option<RunConfig>| {
                                    if let Some(ref mut c) = rc {
                                        c.target_rpm = rpm;
                                    }
                                });
                            }
                            LiveParam::PidGains(gains) => {
                                cx.local.pid.set_gains(gains);
                            }
                            LiveParam::MaxDutyClamp(clamp) => {
                                cx.shared.motor.lock(
                                    |m: &mut crate::guv::motor::MotorController| {
                                        m.set_duty_clamp(clamp)
                                    },
                                );
                            }
                            LiveParam::TargetFreqHz(hz) => {
                                cx.shared.run_config.lock(|rc: &mut Option<RunConfig>| {
                                    if let Some(ref mut c) = rc {
                                        c.target_freq_hz = hz;
                                    }
                                });
                            }
                            LiveParam::TargetVRms(v) => {
                                cx.shared.run_config.lock(|rc: &mut Option<RunConfig>| {
                                    if let Some(ref mut c) = rc {
                                        c.target_v_rms = v;
                                    }
                                });
                            }
                        }
                    }

                    // TODO: auto-transition to GENERATE when load is connected
                    // or when desktop sends a "gate open" command
                    // For now, desktop sends Start again to open gate (reuse command)
                    if let Some(Command::Start) = received_cmd {
                        defmt::info!("Gate open → GENERATE");
                        cx.shared.state.lock(|s| *s = STATE::GENERATE);
                    }

                    // LED: steady green
                    cx.shared.ld1.lock(|l| l.set_high());
                    cx.shared.ld2.lock(|l| l.set_low());
                }

                // ─── GENERATE: serving load ───
                STATE::GENERATE => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // Closed-loop RPM/frequency regulation
                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let dt_s = dt_ms as f32 / 1000.0;
                    let output = cx.local.pid.update(config.target_rpm, measured, dt_s);
                    cx.shared.motor.lock(|m| m.set_speed(output));

                    // Apply live adjustments
                    if let Some(Command::LiveAdjust(param)) = received_cmd {
                        match param {
                            LiveParam::Duty(d) => {
                                cx.shared.motor.lock(|m: &mut Motor| m.set_speed(d));
                            }
                            LiveParam::TargetRpm(rpm) => {
                                cx.shared.run_config.lock(|rc: &mut Cfg| {
                                    if let Some(ref mut c) = rc {
                                        c.target_rpm = rpm;
                                    }
                                });
                            }
                            LiveParam::PidGains(gains) => {
                                cx.local.pid.set_gains(gains);
                            }
                            LiveParam::MaxDutyClamp(clamp) => {
                                cx.shared
                                    .motor
                                    .lock(|m: &mut Motor| m.set_duty_clamp(clamp));
                            }
                            LiveParam::TargetFreqHz(hz) => {
                                cx.shared.run_config.lock(|rc: &mut Cfg| {
                                    if let Some(ref mut c) = rc {
                                        c.target_freq_hz = hz;
                                    }
                                });
                            }
                            LiveParam::TargetVRms(v) => {
                                cx.shared.run_config.lock(|rc: &mut Cfg| {
                                    if let Some(ref mut c) = rc {
                                        c.target_v_rms = v;
                                    }
                                });
                            }
                        }
                    }

                    // TODO: detect load rejection (large delta in current/RPM)
                    // If |rpm - target| > threshold, transition to LOAD_REJECTION

                    // LED: steady green
                    cx.shared.ld1.lock(|l| l.set_high());
                }

                // ─── LOAD_REJECTION: rapid load change recovery ───
                STATE::LOAD_REJECTION => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.speed());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // Aggressive PID recovery — same controller, just let it work
                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let dt_s = dt_ms as f32 / 1000.0;
                    let output = cx.local.pid.update(config.target_rpm, measured, dt_s);
                    cx.shared.motor.lock(|m| m.set_speed(output));

                    // TODO: when RPM stabilizes within tolerance, return to GENERATE
                    let error = (config.target_rpm - measured).abs();
                    if error < config.target_rpm * 0.02 {
                        // Within 2% — recovered
                        defmt::info!("Load rejection recovered → GENERATE");
                        cx.shared.state.lock(|s| *s = STATE::GENERATE);
                    }

                    // LED: fast green/yellow alternating
                    cx.shared.ld1.lock(|l| l.toggle());
                    cx.shared.ld2.lock(|l| l.toggle());
                }

                // ─── MANUAL: live desktop control ───
                STATE::MANUAL => {
                    match received_cmd {
                        Some(Command::LiveAdjust(LiveParam::Duty(d))) => {
                            cx.shared.motor.lock(|m| m.set_speed(d));
                        }
                        Some(Command::LiveAdjust(LiveParam::MaxDutyClamp(c))) => {
                            cx.shared.motor.lock(|m| m.set_duty_clamp(c));
                        }
                        Some(Command::LiveAdjust(LiveParam::TargetRpm(rpm))) => {
                            // Switch to closed-loop on the fly
                            cx.shared.run_config.lock(|rc| {
                                if let Some(ref mut c) = rc {
                                    c.target_rpm = rpm;
                                }
                            });
                        }
                        Some(Command::Stop) => {
                            let speed = cx.shared.motor.lock(|m| m.speed());
                            let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());
                            *cx.local.ramp =
                                Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                            cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        }
                        _ => {}
                    }

                    // LED: green slow blink = manual
                    static mut MANUAL_TICK: u32 = 0;
                    unsafe {
                        MANUAL_TICK += 1;
                    }
                    if unsafe { MANUAL_TICK } % 25 == 0 {
                        cx.shared.ld1.lock(|l| l.toggle());
                    }
                }

                // ─── RAMP_DOWN: graceful shutdown ───
                STATE::RAMP_DOWN => {
                    if let Some(ref mut ramp) = cx.local.ramp {
                        let val = ramp.tick(dt_ms);
                        cx.shared.motor.lock(|m| m.set_speed(val));

                        if ramp.is_done() {
                            defmt::info!("Ramp down complete → IDLE");
                            cx.shared.motor.lock(|m| m.disable());
                            cx.shared.relay.lock(|r| r.set_low());
                            *cx.local.ramp = None;
                            *cx.local.run_elapsed_ms = 0;
                            cx.shared.state.lock(|s| *s = STATE::IDLE);
                            cx.shared.ld1.lock(|l| l.set_low());
                            cx.shared.ld2.lock(|l| l.set_high());
                        }
                    } else {
                        // No ramp set — immediate off
                        cx.shared.motor.lock(|m| m.disable());
                        cx.shared.relay.lock(|r| r.set_low());
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                    }

                    // LED: green fading (toggling slower and slower)
                    let rd_tick = cx
                        .local
                        .ramp
                        .as_ref()
                        .map(|r| if r.is_done() { 1 } else { 10 })
                        .unwrap_or(1);
                    if (*cx.local.run_elapsed_ms / 100) % 2 == 0 {
                        cx.shared.ld1.lock(|l| l.toggle());
                    }
                }

                // ─── FAULT: something went wrong ───
                STATE::FAULT => {
                    cx.shared.motor.lock(|m| m.force_off());
                    cx.shared.relay.lock(|r| r.set_low());

                    // Red solid
                    cx.shared.ld3.lock(|l| l.set_high());
                    cx.shared.ld1.lock(|l| l.set_low());

                    // Yellow blinks the fault code (blink count = last state before fault)
                    // TODO: implement blink-count pattern using last_fault

                    match received_cmd {
                        Some(Command::ClearFaults) | Some(Command::Stop) => {
                            defmt::info!("Faults cleared → IDLE");
                            cx.shared.last_fault.lock(|f| *f = None);
                            cx.shared.state.lock(|s| *s = STATE::IDLE);
                            cx.shared.ld3.lock(|l| l.set_low());
                        }
                        _ => {}
                    }
                }

                // ─── ESTOP: physical button pressed ───
                STATE::ESTOP => {
                    cx.shared.motor.lock(|m| m.force_off());
                    cx.shared.relay.lock(|r| r.set_low());

                    // Red + yellow alternating flash
                    static mut ESTOP_TICK: u32 = 0;
                    unsafe {
                        ESTOP_TICK += 1;
                    }
                    if unsafe { ESTOP_TICK } % 25 == 0 {
                        cx.shared.ld3.lock(|l| l.toggle());
                        cx.shared.ld2.lock(|l| l.toggle());
                    }

                    // Only ClearFaults or Stop can exit ESTOP
                    match received_cmd {
                        Some(Command::ClearFaults) | Some(Command::Stop) => {
                            defmt::info!("ESTOP cleared → IDLE");
                            cx.shared.state.lock(|s| *s = STATE::IDLE);
                            cx.shared.ld3.lock(|l| l.set_low());
                            cx.shared.ld2.lock(|l| l.set_low());
                        }
                        _ => {}
                    }
                }

                _ => {}
            }

            Mono::delay(dt_ms.millis()).await;
        }
    }

    // ────────────────────────────────────────────
    //  RPM Monitor — encoder reading + telemetry
    // ────────────────────────────────────────────
    #[task(priority = 1, shared = [encoder, tx, state, current_rpm, motor, run_config, last_fault])]
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
                &mut cx.shared.current_rpm,
                &mut cx.shared.motor,
                &mut cx.shared.run_config,
                &mut cx.shared.last_fault,
            )
                .lock(
                    |enc, tx, state, current_rpm, motor, run_config, last_fault| {
                        let current_count = enc.count();
                        let delta = current_count.wrapping_sub(last_count);
                        last_count = current_count;

                        let counts_per_second = (delta as i32 as f32) * 100.0;
                        let rpm = (counts_per_second / counts_per_rev) * 60.0 * -1.0;

                        *current_rpm = rpm;

                        let frame = shared::models::telemetry::telemetry::Telemetry {
                            ts_ms,
                            state: *state,
                            rpm,
                            duty_percent: motor.speed(),
                            v_gen_rms: 0.0,
                            i_gen_rms: 0.0,
                            freq_gen_hz: 0.0,
                            theta_err_rad: 0.0,
                            temp_c: 0.0,
                            dc_bus_v: 0.0,
                            run_mode: run_config.map(|c| c.mode),
                            fault: *last_fault,
                        };

                        if *state != STATE::BOOT {
                            let mut buf = [0u8; 96];
                            if let Ok(encoded) = postcard::to_slice_cobs(&frame, &mut buf) {
                                for byte in encoded.iter() {
                                    block!(tx.write(*byte)).ok();
                                }
                            }
                        }

                        if loop_counter % 100 == 0 {
                            defmt::info!(
                                "RPM: {} DUTY: {}% STATE: {}",
                                rpm as i32,
                                (motor.speed() * 100.0) as i32,
                                state.as_str()
                            );
                        }
                    },
                );

            loop_counter = loop_counter.wrapping_add(1);
        }
    }

    // ────────────────────────────────────────────
    //  Helper: apply live parameter adjustment
    // ────────────────────────────────────────────
    // fn apply_live_param(cx: &mut state_manager::Context, param: LiveParam) {
    //     match param {
    //         LiveParam::Duty(d) => {
    //             cx.shared.motor.lock(|m| m.set_speed(d));
    //         }
    //         LiveParam::TargetRpm(rpm) => {
    //             cx.shared.run_config.lock(|rc| {
    //                 if let Some(ref mut c) = rc {
    //                     c.target_rpm = rpm;
    //                 }
    //             });
    //         }
    //         LiveParam::PidGains(gains) => {
    //             cx.local.pid.set_gains(gains);
    //         }
    //         LiveParam::MaxDutyClamp(clamp) => {
    //             cx.shared.motor.lock(|m| m.set_duty_clamp(clamp));
    //         }
    //         LiveParam::TargetFreqHz(hz) => {
    //             cx.shared.run_config.lock(|rc| {
    //                 if let Some(ref mut c) = rc {
    //                     c.target_freq_hz = hz;
    //                 }
    //             });
    //         }
    //         LiveParam::TargetVRms(v) => {
    //             cx.shared.run_config.lock(|rc| {
    //                 if let Some(ref mut c) = rc {
    //                     c.target_v_rms = v;
    //                 }
    //             });
    //         }
    //     }
    // }
}
