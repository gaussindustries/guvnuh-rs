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

use crate::guv::prime_mover::PrimeMover;
use crate::models::{measurements::Measurements, status::WorkerStatus};
use nb::block;
use shared::models::state::states::{Fault, STATE};
use shared::models::telemetry::telemetry::{Command, LiveParam, PidGains, RunConfig, RunMode};

systick_monotonic!(Mono, 1_000);

/// Human-readable command name for defmt logging (Command doesn't derive defmt::Format).
fn cmd_name(c: &Command) -> &'static str {
    match c {
        Command::Ping => "Ping",
        Command::Configure(_) => "Configure",
        Command::Start => "Start",
        Command::Stop => "Stop",
        Command::EmergencyStop => "EmergencyStop",
        Command::LiveAdjust(_) => "LiveAdjust",
        Command::Set(_) => "Set",
        Command::ClearFaults => "ClearFaults",
        Command::Hello => "Hello",
        Command::HelloAck => "HelloAck",
    }
}

// ── Safety limits (compiled-in; the server may only TUNE within these bounds) ──
/// Active from boot with zero config — the autonomous-mode ceiling.
pub const OVERSPEED_LIMIT_DEFAULT: f32 = 2800.0;
/// Absolute bounds any server-commanded limit is clamped to. The server can
/// never disable protection nor set an instant-trip value.
pub const OVERSPEED_LIMIT_MIN: f32 = 500.0;
pub const OVERSPEED_LIMIT_MAX: f32 = 3000.0; // ← true mechanical never-exceed
/// RPM readings outside this window (or non-finite) mean the sensor is lying.
pub const RPM_PLAUSIBLE_MAX: f32 = 4000.0;

/// Clamp a server-commanded overspeed limit into the safe envelope.
#[inline]
pub fn clamp_overspeed_limit(requested: f32) -> f32 {
    if !requested.is_finite() {
        return OVERSPEED_LIMIT_DEFAULT;
    }
    requested.clamp(OVERSPEED_LIMIT_MIN, OVERSPEED_LIMIT_MAX)
}

#[app(device = stm32h7xx_hal::pac, dispatchers = [EXTI0, EXTI1, EXTI2])]
mod app {
    use super::*;
    use shared::models::telemetry::telemetry::Uplink;
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
        pub cmd_in: heapless::Deque<Command, 8>,

        pub measurements: crate::models::measurements::Measurements,
        pub calibration: Option<crate::guv::calibrate::CalResult>,
        pub pending_report: Option<shared::models::telemetry::telemetry::CalibrationReport>,

        /// Overspeed trip ceiling. Boots at the compiled default so protection
        /// is active with zero config; the command path may later set it (clamped).
        /// Comms loss never removes it — the local supervisor keeps enforcing it.
        pub overspeed_limit: f32,
    }

    #[local]
    pub struct Local {
        safety_init_done: bool,
        cmd_buf: [u8; 128], // owned by the UART4 ISR
        cmd_len: usize,
        pid: crate::guv::pid::PidController,
        ramp: Option<crate::guv::ramp::Ramp>,
        run_elapsed_ms: u32,
        calibrator: Option<crate::guv::calibrate::Calibrator>,
        boot_hello_attempts: u32,
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
        telemetry_task::spawn().ok();
        safety_supervisor::spawn().ok();

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
                cmd_in: heapless::Deque::new(),
                calibration: None,
                pending_report: None,
                measurements: crate::models::measurements::Measurements::default(),
                overspeed_limit: crate::OVERSPEED_LIMIT_DEFAULT,
            },
            Local {
                safety_init_done: false,
                cmd_buf: [0u8; 128],
                cmd_len: 0,
                pid: crate::guv::pid::PidController::new(PidGains::default()),
                ramp: None,
                run_elapsed_ms: 0,
                calibrator: None,
                boot_hello_attempts: 0,
            },
        )
    }
    // ────────────────────────────────────────────
    //  Safety Supervisor — independent, comms-agnostic, PREEMPTIVE
    //  Priority 2: preempts state_manager & telemetry (both pri 1). Runs at
    //  500 Hz — 5× the control loop — so overspeed is caught fast. Reads ONLY
    //  local sensors; no dependency on ESP32 or server. This layer keeps the
    //  machine safe even when everything upstream is dead.
    // ────────────────────────────────────────────
    #[task(priority = 2, shared = [current_rpm, overspeed_limit, motor, relay, state, last_fault, ld3])]
    async fn safety_supervisor(mut cx: safety_supervisor::Context) {
        const SUPERVISE_MS: u32 = 2; // 500 Hz

        loop {
            Mono::delay(SUPERVISE_MS.millis()).await;

            let rpm = cx.shared.current_rpm.lock(|r| *r);
            let limit = cx.shared.overspeed_limit.lock(|l| *l);

            // ── evaluate LOCAL trip conditions (no comms involved) ──
            let trip: Option<Fault> = if !rpm.is_finite() || rpm.abs() > crate::RPM_PLAUSIBLE_MAX {
                Some(Fault::SensorOutOfRange)
            } else if rpm > limit {
                Some(Fault::Overspeed)
            } else {
                None
            };

            if let Some(fault) = trip {
                // Actuators OFF first — fastest path to safe.
                cx.shared.motor.lock(|m| m.emergency_off());
                cx.shared.relay.lock(|r| r.set_low());

                // Transition + log only on the EDGE (don't spam while coasting down).
                let already_faulted = cx
                    .shared
                    .state
                    .lock(|s| matches!(*s, STATE::FAULT | STATE::ESTOP));

                if !already_faulted {
                    cx.shared.last_fault.lock(|f| *f = Some(fault));
                    cx.shared.state.lock(|s| *s = STATE::FAULT);
                    cx.shared.ld3.lock(|l| l.set_high());

                    if rpm > limit && rpm.is_finite() {
                        defmt::error!(
                            "!! OVERSPEED TRIP !! rpm={} limit={}",
                            rpm as i32,
                            limit as i32
                        );
                    } else {
                        defmt::error!("!! SENSOR TRIP !! implausible rpm={}", rpm as i32);
                    }
                }
                // Keep forcing off every cycle while faulted — the supervisor
                // doesn't rely on state_manager running to hold safe.
            }
        }
    }
    // ────────────────────────────────────────────
    //  State Manager — the main control loop
    // ────────────────────────────────────────────
    #[task(priority = 1,
        shared = [state, ld1, ld2, ld3, relay, motor, tx, rx, run_config, current_rpm, last_fault, cmd_in, calibration, pending_report],
        local = [safety_init_done, pid, ramp, run_elapsed_ms, calibrator, boot_hello_attempts]
    )]
    async fn state_manager(mut cx: state_manager::Context) {
        let dt_ms: u32 = 10;

        loop {
            let current_state = cx.shared.state.lock(|s| *s);

            // ── Pull one decoded command from the ISR-fed queue ──
            let received_cmd: Option<Command> = cx.shared.cmd_in.lock(|q| q.pop_front());

            // ── ESTOP — always honored, any state ──
            if let Some(Command::EmergencyStop) = received_cmd {
                defmt::warn!("!! EMERGENCY STOP !!");
                cx.shared.motor.lock(|m| m.emergency_off());
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

            // ── ESP32 (re)start announcement — reply from ANY state ──
            // The ESP32 rebooted and is re-announcing. Ack it so the link
            // re-syncs, but DO NOT change state — the machine keeps doing
            // exactly what it was doing. Comms absence never stopped us;
            // comms return never disrupts us.
            if let Some(Command::Hello) = received_cmd {
                defmt::info!("ESP32 Hello — re-syncing link (state unchanged)");
                cx.shared.tx.lock(|tx| {
                    let mut buf = [0u8; 16];
                    if let Ok(enc) = postcard::to_slice_cobs(&Uplink::HelloAck, &mut buf) {
                        for b in enc.iter() {
                            block!(tx.write(*b)).ok();
                        }
                    }
                });
                Mono::delay(dt_ms.millis()).await;
                continue;
            }

            // ── State machine ──
            match current_state {
                // ─── BOOT: safety init + framed Hello handshake ───
                STATE::BOOT => {
                    if !*cx.local.safety_init_done {
                        cx.shared.motor.lock(|m| m.emergency_off());
                        cx.shared.relay.lock(|r| r.set_low());
                        // Arm the RX ISR ONCE, up front. The handshake now flows
                        // through the same COBS/command path as everything else,
                        // so the ISR must be live to receive HelloAck. This ends
                        // the old polling-vs-ISR conflict.
                        cx.shared.rx.lock(|rx| rx.listen());
                        defmt::info!("Safety init complete; RX armed.");
                        *cx.local.safety_init_done = true;
                    }

                    // Announce ourselves: framed Uplink::Hello via TX.
                    cx.shared.tx.lock(|tx| {
                        let mut buf = [0u8; 16];
                        if let Ok(enc) = postcard::to_slice_cobs(&Uplink::Hello, &mut buf) {
                            for b in enc.iter() {
                                block!(tx.write(*b)).ok();
                            }
                        }
                    });

                    // Did a HelloAck land in the command queue? (ISR decoded it.)
                    // Drain it out; leave any other queued commands intact.
                    let acked = cx.shared.cmd_in.lock(|q| {
                        let mut found = false;
                        let mut keep: heapless::Deque<Command, 8> = heapless::Deque::new();
                        while let Some(c) = q.pop_front() {
                            if matches!(c, Command::HelloAck) {
                                found = true;
                            } else {
                                let _ = keep.push_back(c);
                            }
                        }
                        *q = keep;
                        found
                    });

                    if acked {
                        defmt::info!("HelloAck received — entering IDLE");
                        *cx.local.boot_hello_attempts = 0;
                        cx.shared.ld3.lock(|l| l.set_low());
                        cx.shared.ld2.lock(|l| l.set_high());
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                    } else if *cx.local.boot_hello_attempts >= 100 {
                        // ~1s of retries with no ack. Proceed anyway — a dead or
                        // still-booting ESP32 must NOT brick us in BOOT. When the
                        // ESP32 comes up it will send its own Hello, which the
                        // mid-run handler acks and the link attaches.
                        defmt::warn!(
                            "No HelloAck after 1s — proceeding to IDLE, comms will attach later"
                        );
                        *cx.local.boot_hello_attempts = 0;
                        cx.shared.ld3.lock(|l| l.set_low());
                        cx.shared.ld2.lock(|l| l.set_high());
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                    } else {
                        *cx.local.boot_hello_attempts += 1;
                        cx.shared.ld3.lock(|l| l.toggle()); // blink red while trying
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
                                .lock(|m| m.set_max_demand(config.max_duty_clamp));
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

                            //calibrate for our coefficents
                            if config.mode == RunMode::Calibrate {
                                *cx.local.calibrator =
                                    Some(crate::guv::calibrate::Calibrator::new());
                            }
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
                                .lock(|m| m.set_max_demand(config.max_duty_clamp));
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

                // ─── CALIBRATE: compute k, square(r) value derived from rpm, duty pairs ───
                STATE::CALIBRATE => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        defmt::info!("STOP during calibration");
                        let speed = cx.shared.motor.lock(|m| m.demand());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        *cx.local.calibrator = None;
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    let measured = cx.shared.current_rpm.lock(|r| *r);

                    let out = match cx.local.calibrator.as_mut() {
                        Some(cal) => cal.step(measured, dt_ms),
                        None => {
                            cx.shared.state.lock(|s| *s = STATE::IDLE);
                            Mono::delay(dt_ms.millis()).await;
                            continue;
                        }
                    };

                    cx.shared.motor.lock(|m| m.set_demand(out.duty));

                    if let Some(res) = out.result {
                        if res.valid {
                            defmt::info!(
                                "CAL OK: k={} rpm/duty  intercept={}  max_rpm={}  r2={}%",
                                res.k_rpm_per_duty as i32,
                                res.rpm_intercept as i32,
                                res.max_rpm as i32,
                                (res.r_squared * 100.0) as i32
                            );
                        } else {
                            defmt::warn!(
                                "CAL REJECTED: k={} r2={}% — feedforward disabled",
                                res.k_rpm_per_duty as i32,
                                (res.r_squared * 100.0) as i32
                            );
                        }
                        for p in &res.points[..res.point_count as usize] {
                            defmt::info!(
                                "  pt duty={}% rpm={} sd={} n={}",
                                (p.duty * 100.0) as i32,
                                p.rpm_mean as i32,
                                p.rpm_stddev as i32,
                                p.samples
                            );
                        }
                        cx.shared.calibration.lock(|c| *c = Some(res));

                        // build wire report and queue it for telemetry_task to send (even if rejected)
                        let mut points =
                            [shared::models::telemetry::telemetry::CalPointWire::default();
                                crate::guv::calibrate::CAL_POINTS.len()];
                        for (i, p) in res.points.iter().enumerate() {
                            points[i] = shared::models::telemetry::telemetry::CalPointWire {
                                duty: p.duty,
                                rpm_mean: p.rpm_mean,
                                rpm_stddev: p.rpm_stddev,
                                samples: p.samples,
                            };
                        }
                        let report = shared::models::telemetry::telemetry::CalibrationReport {
                            ts_ms: *cx.local.run_elapsed_ms,
                            k_rpm_per_duty: res.k_rpm_per_duty,
                            rpm_intercept: res.rpm_intercept,
                            max_rpm: res.max_rpm,
                            r_squared: res.r_squared,
                            points,
                            point_count: res.point_count,
                            valid: res.valid,
                        };
                        cx.shared.pending_report.lock(|slot| *slot = Some(report));
                    }

                    if out.done {
                        cx.shared.motor.lock(|m| m.disable());
                        cx.shared.relay.lock(|r| r.set_low());
                        *cx.local.calibrator = None;
                        *cx.local.run_elapsed_ms = 0;
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                        cx.shared.ld1.lock(|l| l.set_low());
                        cx.shared.ld2.lock(|l| l.set_high());
                    } else {
                        *cx.local.run_elapsed_ms += dt_ms;
                        if (*cx.local.run_elapsed_ms / 150) % 2 == 0 {
                            cx.shared.ld1.lock(|l| l.toggle());
                        }
                    }
                }

                // ─── SPOOLUP: ramp motor to target RPM ───
                STATE::SPOOLUP => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());
                    *cx.local.run_elapsed_ms += dt_ms;

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.demand());
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
                                cx.shared.motor.lock(|m: &mut Motor| m.set_demand(d));
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
                                    .lock(|m: &mut Motor| m.set_max_demand(clamp));
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

                    // Ramp up
                    if let Some(ref mut ramp) = cx.local.ramp {
                        let val = ramp.tick(dt_ms);
                        match config.mode {
                            RunMode::ClosedLoop | RunMode::Generate => {
                                let target_rpm = config.target_rpm * val;
                                let measured = cx.shared.current_rpm.lock(|r| *r);
                                let dt_s = dt_ms as f32 / 1000.0;

                                let ff = cx
                                    .shared
                                    .calibration
                                    .lock(|c| c.and_then(|cal| cal.feedforward(target_rpm)))
                                    .unwrap_or(0.0);

                                let trim = cx.local.pid.update(target_rpm, measured, dt_s);
                                cx.shared
                                    .motor
                                    .lock(|m| m.set_demand((ff + trim).clamp(0.0, 1.0)));
                            }
                            _ => {
                                cx.shared.motor.lock(|m| m.set_demand(val));
                            }
                        }

                        if ramp.is_done() {
                            *cx.local.ramp = None;
                            defmt::info!("Spoolup complete");

                            match config.mode {
                                RunMode::Generate => {
                                    cx.shared.state.lock(|s| *s = STATE::EXCITE);
                                    defmt::info!("→ EXCITE");
                                }
                                _ => {
                                    // OpenLoop/ClosedLoop: hold at speed (stay in SPOOLUP)
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
                                cx.shared.motor.lock(|m| m.set_demand(output));
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
                            let speed = cx.shared.motor.lock(|m| m.demand());
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
                        let speed = cx.shared.motor.lock(|m| m.demand());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // TODO: monitor v_gen_rms from ADC; advance when voltage builds.
                    // Placeholder: auto-advance after 3 seconds.
                    *cx.local.run_elapsed_ms += dt_ms;
                    if *cx.local.run_elapsed_ms > config.ramp_up_ms + 3000 {
                        defmt::info!("Excitation nominal → PLL_LOCK");
                        cx.shared.state.lock(|s| *s = STATE::PLL_LOCK);
                    }

                    cx.shared.ld2.lock(|l| l.toggle());
                }

                // ─── PLL_LOCK: phase-lock to reference (stub) ───
                STATE::PLL_LOCK => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.demand());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    // TODO: phase detector + loop filter on theta_err_rad.
                    // Placeholder: auto-advance after 2 seconds past excite.
                    *cx.local.run_elapsed_ms += dt_ms;
                    if *cx.local.run_elapsed_ms > config.ramp_up_ms + 5000 {
                        defmt::info!("PLL locked → READY");
                        cx.shared.state.lock(|s| *s = STATE::READY);
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
                        let speed = cx.shared.motor.lock(|m| m.demand());
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
                    cx.shared.motor.lock(|m| m.set_demand(output));

                    // Apply live adjustments
                    if let Some(Command::LiveAdjust(param)) = received_cmd {
                        match param {
                            LiveParam::Duty(d) => {
                                cx.shared.motor.lock(|m: &mut Motor| m.set_demand(d));
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
                                    .lock(|m: &mut Motor| m.set_max_demand(clamp));
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

                    // Second Start = gate open → GENERATE (placeholder gate control)
                    if let Some(Command::Start) = received_cmd {
                        defmt::info!("Gate open → GENERATE");
                        cx.shared.state.lock(|s| *s = STATE::GENERATE);
                    }

                    cx.shared.ld1.lock(|l| l.set_high());
                    cx.shared.ld2.lock(|l| l.set_low());
                }

                // ─── GENERATE: serving load ───
                STATE::GENERATE => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.demand());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let dt_s = dt_ms as f32 / 1000.0;
                    let output = cx.local.pid.update(config.target_rpm, measured, dt_s);
                    cx.shared.motor.lock(|m| m.set_demand(output));

                    // Apply live adjustments
                    if let Some(Command::LiveAdjust(param)) = received_cmd {
                        match param {
                            LiveParam::Duty(d) => {
                                cx.shared.motor.lock(|m: &mut Motor| m.set_demand(d));
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
                                    .lock(|m: &mut Motor| m.set_max_demand(clamp));
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

                    cx.shared.ld1.lock(|l| l.set_high());
                }

                // ─── LOAD_REJECTION: rapid load change recovery ───
                STATE::LOAD_REJECTION => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());

                    if let Some(Command::Stop) = received_cmd {
                        let speed = cx.shared.motor.lock(|m| m.demand());
                        *cx.local.ramp =
                            Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                        cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        Mono::delay(dt_ms.millis()).await;
                        continue;
                    }

                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let dt_s = dt_ms as f32 / 1000.0;
                    let output = cx.local.pid.update(config.target_rpm, measured, dt_s);
                    cx.shared.motor.lock(|m| m.set_demand(output));

                    let error = (config.target_rpm - measured).abs();
                    if error < config.target_rpm * 0.02 {
                        defmt::info!("Load rejection recovered → GENERATE");
                        cx.shared.state.lock(|s| *s = STATE::GENERATE);
                    }

                    cx.shared.ld1.lock(|l| l.toggle());
                    cx.shared.ld2.lock(|l| l.toggle());
                }

                // ─── MANUAL: live desktop control ───
                STATE::MANUAL => {
                    match received_cmd {
                        Some(Command::LiveAdjust(LiveParam::Duty(d))) => {
                            cx.shared.motor.lock(|m| m.set_demand(d));
                        }
                        Some(Command::LiveAdjust(LiveParam::MaxDutyClamp(c))) => {
                            cx.shared.motor.lock(|m| m.set_max_demand(c));
                        }
                        Some(Command::LiveAdjust(LiveParam::TargetRpm(rpm))) => {
                            cx.shared.run_config.lock(|rc| {
                                if let Some(ref mut c) = rc {
                                    c.target_rpm = rpm;
                                }
                            });
                        }
                        Some(Command::Stop) => {
                            let speed = cx.shared.motor.lock(|m| m.demand());
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
                        cx.shared.motor.lock(|m| m.set_demand(val));

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
                        cx.shared.motor.lock(|m| m.disable());
                        cx.shared.relay.lock(|r| r.set_low());
                        cx.shared.state.lock(|s| *s = STATE::IDLE);
                    }

                    if (*cx.local.run_elapsed_ms / 100) % 2 == 0 {
                        cx.shared.ld1.lock(|l| l.toggle());
                    }
                }

                // ─── FAULT ───
                STATE::FAULT => {
                    cx.shared.motor.lock(|m| m.emergency_off());
                    cx.shared.relay.lock(|r| r.set_low());

                    cx.shared.ld3.lock(|l| l.set_high());
                    cx.shared.ld1.lock(|l| l.set_low());

                    // TODO: yellow blink-count pattern of last state before fault

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

                // ─── ESTOP ───
                STATE::ESTOP => {
                    cx.shared.motor.lock(|m| m.emergency_off());
                    cx.shared.relay.lock(|r| r.set_low());

                    static mut ESTOP_TICK: u32 = 0;
                    unsafe {
                        ESTOP_TICK += 1;
                    }
                    if unsafe { ESTOP_TICK } % 25 == 0 {
                        cx.shared.ld3.lock(|l| l.toggle());
                        cx.shared.ld2.lock(|l| l.toggle());
                    }

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
    #[task(priority = 1, shared = [encoder, tx, state, current_rpm, motor, run_config, last_fault, measurements, pending_report])]
    async fn telemetry_task(mut cx: telemetry_task::Context) {
        let mut last_count: u32 = 0;
        let counts_per_rev: f32 = 8192.0;
        let mut loop_counter: u32 = 0;
        let mut ts_ms: u32 = 0;
        loop {
            Mono::delay(10u32.millis()).await;
            ts_ms = ts_ms.wrapping_add(10);

            // drain the calibration outbox (atomic take)
            let report = cx.shared.pending_report.lock(|slot| slot.take());
            (
                &mut cx.shared.encoder,
                &mut cx.shared.tx,
                &mut cx.shared.state,
                &mut cx.shared.current_rpm,
                &mut cx.shared.motor,
                &mut cx.shared.run_config,
                &mut cx.shared.last_fault,
                &mut cx.shared.measurements,
            )
                .lock(
                    |enc, tx, state, current_rpm, motor, run_config, last_fault, meas| {
                        // ── acquisition: encoder → rpm ──
                        let current_count = enc.count();
                        let delta = current_count.wrapping_sub(last_count);
                        last_count = current_count;
                        let counts_per_second = (delta as i32 as f32) * 100.0;
                        let rpm = (counts_per_second / counts_per_rev) * 60.0 * -1.0;

                        // write into BOTH: control loop reads current_rpm, telemetry reads measurements.
                        // when the ADC lands, a separate `measurement` task fills meas.v_gen_rms etc.
                        *current_rpm = rpm;
                        meas.rpm = rpm;

                        // ── assembly: frame is built FROM measurements ──
                        let frame = shared::models::telemetry::telemetry::Telemetry {
                            ts_ms,
                            state: *state,
                            rpm: meas.rpm,
                            duty_percent: motor.demand(),
                            v_gen_rms: meas.v_gen_rms,
                            i_gen_rms: meas.i_gen_rms,
                            freq_gen_hz: meas.freq_gen_hz,
                            theta_err_rad: 0.0, // still a stub — no sensor
                            temp_c: meas.temp_c,
                            dc_bus_v: meas.dc_bus_v,
                            run_mode: run_config.map(|c| c.mode),
                            fault: *last_fault,
                        };

                        if *state != STATE::BOOT {
                            let mut buf = [0u8; 96];
                            if let Ok(encoded) =
                                postcard::to_slice_cobs(&Uplink::Telemetry(frame), &mut buf)
                            {
                                for byte in encoded.iter() {
                                    block!(tx.write(*byte)).ok();
                                }
                            }

                            // send a pending calibration report, if one was queued
                            if let Some(rep) = report {
                                let mut rbuf = [0u8; 96];
                                if let Ok(encoded) =
                                    postcard::to_slice_cobs(&Uplink::Calibration(rep), &mut rbuf)
                                {
                                    for byte in encoded.iter() {
                                        block!(tx.write(*byte)).ok();
                                    }
                                    defmt::info!("Calibration report sent");
                                }
                            }
                        }

                        if loop_counter % 100 == 0 {
                            defmt::info!(
                                "RPM: {} DUTY: {}% STATE: {}",
                                rpm as i32,
                                (motor.demand() * 100.0) as i32,
                                state.as_str()
                            );
                        }
                    },
                );
            loop_counter = loop_counter.wrapping_add(1);
        }
    }

    // ────────────────────────────────────────────
    //  UART4 RX ISR — drains FIFO, decodes COBS, queues commands
    // ────────────────────────────────────────────
    #[task(binds = UART4, priority = 3, shared = [rx, cmd_in], local = [cmd_buf, cmd_len])]
    fn uart4_rx(mut cx: uart4_rx::Context) {
        let buf = &mut *cx.local.cmd_buf;
        let len = &mut *cx.local.cmd_len;

        cx.shared.rx.lock(|rx| {
            loop {
                match rx.read() {
                    Ok(b) => {
                        if b == 0x00 {
                            if *len > 0 {
                                match postcard::from_bytes_cobs::<Command>(&mut buf[..*len]) {
                                    Ok(cmd) => {
                                        defmt::info!("CMD recv: {}", cmd_name(&cmd));
                                        cx.shared.cmd_in.lock(|q| {
                                            if q.push_back(cmd).is_err() {
                                                defmt::warn!("cmd queue full — dropped");
                                            }
                                        });
                                    }
                                    Err(_) => defmt::warn!("cmd decode fail ({} bytes)", *len),
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
                    Err(_) => break, // WouldBlock: FIFO drained
                }
            }
        });
    }
    // FUTURE — do not add until ADS131M04 is wired
    // #[task(priority = 1, shared = [encoder, measurements], local = [/* adc handle, sample buffer */])]
    // async fn measurement(mut cx: measurement::Context) {
    //     loop {
    //         Mono::delay(10u32.millis()).await;
    //         // read encoder → rpm  (moves here, out of telemetry_task)
    //         // compute RMS/freq from the adc_sampler's buffer
    //         cx.shared.measurements.lock(|m| {
    //             m.rpm = /* encoder */;
    //             m.v_gen_rms = /* computed */;
    //             m.i_gen_rms = /* computed */;
    //             m.freq_gen_hz = /* zero-cross */;
    //             m.dc_bus_v = /* adc channel */;
    //         });
    //     }
    // }
}
