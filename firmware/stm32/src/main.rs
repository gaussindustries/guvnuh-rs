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
    gpio::{self, ExtiPin, GpioExt, Input, Output, PushPull},
    prelude::*,
    pwm,
    qei::Qei,
};

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::{DCB, DWT};

mod guv;
mod models;

use crate::guv::prime_mover::PrimeMover;
use crate::guv::wcet::{CycleTimer, WcetTable};
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
        Command::SetLoad(_) => "SetLoad",
        Command::StepLoadUp => "StepLoadUp",
        Command::StepLoadDown => "StepLoadDown",
        Command::LoadProfile(_) => "LoadProfile",
        Command::SetAlignPhase(_) => "SetAlignPhase",
    }
}

/// Map a STATE to a stable table index for WCET tracking. Order must be stable
/// (append new states at the end) so banked numbers stay comparable.
fn state_index(s: STATE) -> usize {
    match s {
        STATE::BOOT => 0,
        STATE::CALIBRATE => 1,
        STATE::IDLE => 2,
        STATE::CONFIGURED => 3,
        STATE::SPOOLUP => 4,
        STATE::EXCITE => 5,
        STATE::PLL_LOCK => 6,
        STATE::READY => 7,
        STATE::GENERATE => 8,
        STATE::LOAD_REJECTION => 9,
        STATE::RAMP_DOWN => 10,
        STATE::MANUAL => 11,
        STATE::FAULT => 12,
        STATE::ESTOP => 13,
    }
}

fn enable_cycle_counter(dcb: &mut DCB, dwt: &mut DWT) {
    dcb.enable_trace(); // DEMCR.TRCENA = 1 (required for DWT)
    dwt.enable_cycle_counter(); // DWT.CTRL.CYCCNTENA = 1
}

// ── Safety limits (compiled-in; the server may only TUNE within these bounds) ──

/// Load rejection = the DC-bus current (2 ms mean) falls this far (A) below
/// its ~20 ms running baseline. One 2.2 kΩ step at 310 V is ~0.14 A, so this
/// is half a step.
pub const LOAD_REJECT_DROP_A: f32 = 0.07;
/// Only arm the detector while at least this much load current flows (A).
pub const LOAD_REJECT_MIN_A: f32 = 0.10;
/// Baseline smoothing per 2 ms supervisor tick (~20 ms time constant).
pub const LOAD_REJECT_BASELINE_ALPHA: f32 = 0.1;
/// DC bus ceiling (V). Normal bus is 310–325 V at 230 V line-to-line; the
/// DC channel reads up to ~405 V.
pub const DC_OVERVOLT_LIMIT: f32 = 380.0;
/// DC bus current ceiling (A). Reachable only with R40 = 0.05 Ω (~0.59 A full
/// scale); with the 0.1 Ω part the channel clips at ~0.29 A (boot log warns).
pub const DC_OVERCURRENT_LIMIT: f32 = 0.45;
/// Active from boot with zero config — the autonomous-mode ceiling.
/// 2200 rpm ≈ 1.2× the SEIG's ~1850 rpm for 60 Hz.
pub const OVERSPEED_LIMIT_DEFAULT: f32 = 2200.0;
/// Absolute bounds any server-commanded limit is clamped to. The server can
/// never disable protection nor set an instant-trip value.
pub const OVERSPEED_LIMIT_MIN: f32 = 500.0;
pub const OVERSPEED_LIMIT_MAX: f32 = 3000.0; // ← true mechanical never-exceed
/// RPM readings outside this window (or non-finite) mean the sensor is lying.
pub const RPM_PLAUSIBLE_MAX: f32 = 4000.0;
/// Turning backwards faster than this (rpm) trips SensorOutOfRange. A reversed
/// motor or encoder would otherwise make the speed loop run away.
pub const REVERSE_RPM_TRIP: f32 = 200.0;
/// Encoder sign: forward rotation must read POSITIVE rpm. Flip this (not the
/// motor leads) if the shaft has to turn one particular way.
pub const ENCODER_SIGN: f32 = -1.0;
/// Independent watchdog timeout (ms). The 500 Hz supervisor feeds it. A hang,
/// panic or HardFault stops the feeding and resets the MCU, which drops the
/// relay (PE0) and the PWM pin (pull-down) — a crash lands on stopped.
pub const WATCHDOG_TIMEOUT_MS: u32 = 100;

// ── LOAD_REJECTION recovery (10 ms control ticks) ──
/// Minimum time spent in LOAD_REJECTION (ticks).
pub const LR_MIN_TICKS: u32 = 20;
/// Speed must stay within ±2% of target this long to count as recovered.
pub const LR_SETTLE_TICKS: u32 = 30;
/// Give up waiting for ±2% and return to the previous state after this.
pub const LR_TIMEOUT_TICKS: u32 = 500;

/// A sudden drop in DC-bus current seen by the supervisor. The state machine
/// decides what to do with it (LOAD_REJECTION in READY/GENERATE).
#[derive(Clone, Copy)]
pub struct LoadRejectEvent {
    pub i_before: f32,
    pub i_after: f32,
}

/// Feedforward (from calibration) plus PID trim, clamped to 0..1. Every state
/// that holds speed uses this, so hand-offs between them are bumpless.
pub fn speed_hold(
    cal: Option<crate::guv::calibrate::CalResult>,
    pid: &mut crate::guv::pid::PidController,
    target_rpm: f32,
    measured_rpm: f32,
    dt_s: f32,
) -> f32 {
    let ff = cal.and_then(|c| c.feedforward(target_rpm)).unwrap_or(0.0);
    (ff + pid.update(target_rpm, measured_rpm, dt_s)).clamp(0.0, 1.0)
}

// ── Deadline monitoring (execution-time budget per control iteration) ──

/// The control loop period in cycles. At SYSCLK_HZ and 100 Hz (10 ms):
///   64_000_000 / 100 = 640_000 cycles per 10 ms period.
/// Keep in sync with the loop rate (dt_ms) and SYSCLK.
pub const CONTROL_PERIOD_CYCLES: u32 = crate::guv::wcet::SYSCLK_HZ / 100;

/// Fraction of the period a state's execution may occupy before it counts as an
/// overrun. 0.8 = 80% — loose enough that only genuine degradation trips (the
/// loop normally runs at single-digit % utilization), leaving headroom for the
/// priority-2 supervisor preemption + the UART ISR + jitter. Tighten for more
/// sensitivity.
pub const DEADLINE_BUDGET_FRAC: f32 = 0.8;

/// The per-iteration cycle budget, derived. Execution above this = one overrun.
pub const DEADLINE_BUDGET_CYCLES: u32 =
    (CONTROL_PERIOD_CYCLES as f32 * DEADLINE_BUDGET_FRAC) as u32;

/// How many CONSECUTIVE overruns in a deadline-bearing state before tripping.
/// 3 = ~30 ms of sustained overrun — a real degradation, not a one-off blip
/// (cold-cache first-hit, a single scheduling hiccup). Prevents nuisance trips.
pub const DEADLINE_MISS_STREAK: u32 = 3;

/// Whether a state is time-critical enough to enforce an execution-time
/// deadline. Active-control states have hard timing requirements; BOOT / IDLE /
/// CONFIGURED / FAULT / ESTOP do not (a slow IDLE tick endangers nothing).
fn state_has_deadline(s: STATE) -> bool {
    matches!(
        s,
        STATE::SPOOLUP
            | STATE::EXCITE
            | STATE::PLL_LOCK
            | STATE::READY
            | STATE::GENERATE
            | STATE::LOAD_REJECTION
            | STATE::RAMP_DOWN
            | STATE::CALIBRATE
            | STATE::MANUAL
    )
}

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
    use shared::models::telemetry::telemetry::{Uplink, DEFAULT_PROFILE};
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

        //load
        load_bank: crate::guv::load_bank::LoadBank,

        adc: crate::guv::adc::Ads131Pair,
        adc_acc: crate::guv::adc::Accumulator,
        adc_meas: crate::guv::adc::Measurements,

        // State
        pub state: STATE,
        pub run_config: Option<RunConfig>,
        pub current_rpm: f32,
        pub last_fault: Option<Fault>,
        pub cmd_in: heapless::Deque<Command, 8>,
        pub align_target: crate::guv::adc::AlignPhase,
        pub measurements: crate::models::measurements::Measurements,
        /// Set by the supervisor on a sudden DC-bus current drop; taken by
        /// state_manager.
        pub load_reject: Option<crate::LoadRejectEvent>,

        pub calibration: Option<crate::guv::calibrate::CalResult>,
        pub pending_report: Option<shared::models::telemetry::telemetry::CalibrationReport>,

        /// Overspeed trip ceiling. Boots at the compiled default so protection
        /// is active with zero config; the command path may later set it (clamped).
        /// Comms loss never removes it — the local supervisor keeps enforcing it.
        pub overspeed_limit: f32,

        /// Active setpoint profile, if one is loaded. None = use static
        /// target_rpm (backward compatible) OR the compiled DEFAULT_PROFILE for
        /// a profile run — see the control-loop integration below.
        pub active_profile: Option<shared::models::telemetry::telemetry::SetpointProfile>,

        pub wcet: WcetTable,
    }

    #[local]
    pub struct Local {
        safety_init_done: bool,
        // Sized for the largest Command: a LoadProfile always encodes all
        // MAX_PROFILE_POINTS slots (~200 B COBS), so 128 B dropped every profile.
        cmd_buf: [u8; shared::models::telemetry::telemetry::PROFILE_CMD_BUF], // owned by the UART4 ISR
        cmd_len: usize,
        pid: crate::guv::pid::PidController,
        ramp: Option<crate::guv::ramp::Ramp>,
        run_elapsed_ms: u32,
        /// Profile clock value on the previous SPOOLUP tick — edge-detects which
        /// breakpoints were crossed so each action fires exactly once.
        profile_prev_ms: Option<u32>,
        calibrator: Option<crate::guv::calibrate::Calibrator>,
        boot_hello_attempts: u32,
        adc_drdy: crate::guv::adc::Drdy,
        /// Supervisor: running DC-bus current baseline (A) for load rejection.
        i_base: f32,
        i_base_ok: bool,
        /// Supervisor: ticks before another load-rejection event may fire.
        lr_holdoff: u32,
        estop: gpio::Pin<'E', 6, Input>,
        /// Supervisor: independent watchdog, fed every 2 ms.
        wdg: stm32h7xx_hal::independent_watchdog::IndependentWatchdog,
        /// state_manager: LOAD_REJECTION bookkeeping.
        lr_ticks: u32,
        lr_settled: u32,
        lr_from: STATE,
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
        defmt::info!("INIT: setup() returned");
        let adc = crate::guv::adc::Ads131Pair::new(
            board.adc_spi,
            board.adc_cs1,
            board.adc_cs2,
            board.adc_sync,
            board.adc_rst,
        );

        defmt::info!("INIT: Ads131Pair::new() complete");
        let load_bank = crate::guv::load_bank::LoadBank::new(board.load_step_1, board.load_step_2);

        if crate::DC_OVERCURRENT_LIMIT >= crate::guv::adc::I_DC_FULL_SCALE {
            defmt::warn!(
                "DC over-current trip ({} mA) is above the current channel's full scale ({} mA) with R40 = {} mOhm, so it cannot fire. Fit the 0.05 Ohm shunt and set SHUNT_OHMS in adc.rs.",
                (crate::DC_OVERCURRENT_LIMIT * 1000.0) as i32,
                (crate::guv::adc::I_DC_FULL_SCALE * 1000.0) as i32,
                (crate::guv::adc::SHUNT_OHMS * 1000.0) as i32
            );
        }
        state_manager::spawn().ok();
        telemetry_task::spawn().ok();
        safety_supervisor::spawn().ok();
        //for wcet calc
        cx.core.DCB.enable_trace();
        cx.core.DWT.enable_cycle_counter();
        board.ld3.set_high(); // Red = booting

        // Start the watchdog last: the ADC bring-up above can take a while.
        // From here the supervisor must feed it every 2 ms.
        let mut wdg = board.iwdg;
        wdg.start(crate::WATCHDOG_TIMEOUT_MS.millis());
        defmt::info!("INIT: watchdog armed ({} ms)", crate::WATCHDOG_TIMEOUT_MS);
        (
            Shared {
                state: STATE::BOOT,
                ld1: board.ld1,
                ld2: board.ld2,
                ld3: board.ld3,
                relay: board.motor_relay,
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
                active_profile: Some(DEFAULT_PROFILE),
                wcet: WcetTable::new(),
                load_bank,
                adc,
                adc_acc: crate::guv::adc::Accumulator::default(),
                adc_meas: crate::guv::adc::Measurements::default(),
                align_target: crate::guv::adc::AlignPhase::default(),
                load_reject: None,
            },
            Local {
                safety_init_done: false,
                cmd_buf: [0u8; shared::models::telemetry::telemetry::PROFILE_CMD_BUF],
                cmd_len: 0,
                pid: crate::guv::pid::PidController::new(PidGains::default()),
                ramp: None,
                run_elapsed_ms: 0,
                profile_prev_ms: None,
                calibrator: None,
                boot_hello_attempts: 0,
                i_base: 0.0,
                i_base_ok: false,
                lr_holdoff: 0,
                adc_drdy: board.adc_drdy,
                estop: board.estop,
                wdg,
                lr_ticks: 0,
                lr_settled: 0,
                lr_from: STATE::GENERATE,
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
    #[task(priority = 2,
        shared = [current_rpm, overspeed_limit, motor, relay,
            state, last_fault, ld3, ld2, ld1, adc_acc, load_bank, load_reject],
        local = [i_base, i_base_ok, lr_holdoff, estop, wdg])]
    async fn safety_supervisor(mut cx: safety_supervisor::Context) {
        const SUPERVISE_MS: u32 = 2; // 500 Hz

        loop {
            Mono::delay(SUPERVISE_MS.millis()).await;

            // Proof of life. If this task stops running (hang, panic,
            // HardFault, interrupt storm) the IWDG resets the MCU.
            cx.local.wdg.feed();

            // ── PHYSICAL E-STOP — highest priority, fail-safe ──
            // Pin HIGH = E-stop (button pressed OR wire broken). LOW = closed/running.
            let estop_active = cx.local.estop.is_high();

            if estop_active {
                // Kill everything immediately, every cycle it's held.
                cx.shared.motor.lock(|m| m.emergency_off());
                cx.shared.relay.lock(|r| r.set_low());
                cx.shared.load_bank.lock(|lb| lb.emergency_shed());

                // Transition to ESTOP on the edge (don't spam).
                let already_estopped = cx.shared.state.lock(|s| matches!(*s, STATE::ESTOP));
                if !already_estopped {
                    defmt::error!("!! PHYSICAL E-STOP ENGAGED !!");
                    cx.shared.state.lock(|s| *s = STATE::ESTOP);
                    cx.shared
                        .last_fault
                        .lock(|f| *f = Some(Fault::EmergencyStop));
                    cx.shared.ld3.lock(|l| l.set_high());
                    cx.shared.ld1.lock(|l| l.set_low());
                    cx.shared.ld2.lock(|l| l.set_low());
                }
                // Skip the rest of the supervisor while E-stopped — hold safe.
                continue;
            }

            let rpm = cx.shared.current_rpm.lock(|r| *r);
            let limit = cx.shared.overspeed_limit.lock(|l| *l);

            // ── evaluate LOCAL trip conditions (no comms involved) ──
            // Overspeed on |rpm|: a reversed encoder or motor must not blind it.
            let trip: Option<Fault> = if !rpm.is_finite() || rpm.abs() > crate::RPM_PLAUSIBLE_MAX {
                Some(Fault::SensorOutOfRange)
            } else if rpm.abs() > limit {
                Some(Fault::Overspeed)
            } else if rpm < -crate::REVERSE_RPM_TRIP {
                Some(Fault::SensorOutOfRange)
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

                    if rpm.is_finite() && rpm.abs() > limit && rpm.abs() <= crate::RPM_PLAUSIBLE_MAX
                    {
                        defmt::error!(
                            "!! OVERSPEED TRIP !! rpm={} limit={}",
                            rpm as i32,
                            limit as i32
                        );
                    } else if rpm.is_finite() && rpm < -crate::REVERSE_RPM_TRIP {
                        defmt::error!(
                            "!! REVERSE ROTATION !! rpm={} — swap A+/A- or flip ENCODER_SIGN",
                            rpm as i32
                        );
                    } else {
                        defmt::error!("!! SENSOR TRIP !! implausible rpm={}", rpm as i32);
                    }
                }
                // Keep forcing off every cycle while faulted — the supervisor
                // doesn't rely on state_manager running to hold safe.
            }
            // ── ADC-based fast protection: DC-bus means over the last 2 ms ──
            // None = no samples (ADC offline / not converting): skip, never
            // act on zeros.
            let fast = cx.shared.adc_acc.lock(|acc| acc.take_fast_dc(1.0));
            let Some(prot) = fast else {
                continue;
            };

            // LOAD REJECTION: the DC-bus current drops well below its recent
            // baseline → the generator just lost load → the shaft will speed
            // up. That is a control event, not a fault: report it and let the
            // state machine respond (LOAD_REJECTION in READY/GENERATE). The
            // overspeed trip above stays the backstop.
            if !*cx.local.i_base_ok {
                *cx.local.i_base = prot.i_dc;
                *cx.local.i_base_ok = true;
            }
            let base = *cx.local.i_base;
            if *cx.local.lr_holdoff > 0 {
                *cx.local.lr_holdoff -= 1;
            }
            let dropped =
                base > crate::LOAD_REJECT_MIN_A && base - prot.i_dc > crate::LOAD_REJECT_DROP_A;
            if dropped && *cx.local.lr_holdoff == 0 {
                defmt::warn!(
                    "!! LOAD REJECTION !! i_dc {} mA -> {} mA",
                    (base * 1000.0) as i32,
                    (prot.i_dc * 1000.0) as i32
                );
                cx.shared.load_reject.lock(|e| {
                    *e = Some(crate::LoadRejectEvent {
                        i_before: base,
                        i_after: prot.i_dc,
                    })
                });
                *cx.local.i_base = prot.i_dc; // re-baseline at the new load
                *cx.local.lr_holdoff = 50; // 100 ms before another event
            } else {
                *cx.local.i_base = base + (prot.i_dc - base) * crate::LOAD_REJECT_BASELINE_ALPHA;
            }

            // fast DC-bus electrical trips (local, no comms):
            if prot.v_dc > crate::DC_OVERVOLT_LIMIT || prot.i_dc > crate::DC_OVERCURRENT_LIMIT {
                let already_faulted = cx
                    .shared
                    .state
                    .lock(|s| matches!(*s, STATE::FAULT | STATE::ESTOP));
                if !already_faulted {
                    defmt::error!(
                        "!! DC BUS TRIP !! v={} i={}",
                        prot.v_dc as i32,
                        prot.i_dc as i32
                    );
                    cx.shared.load_bank.lock(|lb| lb.emergency_shed());
                    cx.shared.motor.lock(|m| m.emergency_off());
                    cx.shared.relay.lock(|r| r.set_low());
                    cx.shared.last_fault.lock(|f| {
                        *f = Some(if prot.v_dc > crate::DC_OVERVOLT_LIMIT {
                            Fault::OverVoltage
                        } else {
                            Fault::OverCurrent
                        })
                    });
                    cx.shared.state.lock(|s| *s = STATE::FAULT);
                    cx.shared.ld3.lock(|l| l.set_high());
                }
            }
        }
    }
    // ────────────────────────────────────────────
    //  State Manager — the main control loop
    // ────────────────────────────────────────────
    #[task(priority = 1,
        shared = [state, ld1, ld2, ld3, relay, load_bank, motor, tx, rx, run_config,
            current_rpm, last_fault, cmd_in, calibration, pending_report, active_profile,
            wcet, adc_acc, adc_meas, align_target, load_reject],
        local = [safety_init_done, pid, ramp, run_elapsed_ms, profile_prev_ms, calibrator, boot_hello_attempts,
            lr_ticks, lr_settled, lr_from]
    )]
    async fn state_manager(mut cx: state_manager::Context) {
        let dt_ms: u32 = 10;

        loop {
            let current_state = cx.shared.state.lock(|s| *s);

            // ── Pull one decoded command from the ISR-fed queue ──
            let received_cmd: Option<Command> = cx.shared.cmd_in.lock(|q| q.pop_front());

            // ── High-priority commands honored in ANY state (ESTOP, Hello, Load) ──
            // Match by reference so `received_cmd` stays available for the state machine below.
            match &received_cmd {
                Some(Command::EmergencyStop) => {
                    defmt::warn!("!! EMERGENCY STOP !!");
                    cx.shared.motor.lock(|m| m.emergency_off());
                    cx.shared.relay.lock(|r| r.set_low());
                    cx.shared.load_bank.lock(|lb| lb.emergency_shed());
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
                Some(Command::Hello) => {
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
                Some(Command::StepLoadUp) => {
                    cx.shared.load_bank.lock(|lb| lb.step_up());
                    // no continue — fall through to the rest of the loop
                }
                Some(Command::StepLoadDown) => {
                    cx.shared.load_bank.lock(|lb| lb.step_down());
                }
                Some(Command::SetLoad(level)) => {
                    let target = match level {
                        shared::models::telemetry::telemetry::LoadLevel::None => {
                            crate::guv::load_bank::LoadLevel::None
                        }
                        shared::models::telemetry::telemetry::LoadLevel::One => {
                            crate::guv::load_bank::LoadLevel::One
                        }
                        shared::models::telemetry::telemetry::LoadLevel::Both => {
                            crate::guv::load_bank::LoadLevel::Both
                        }
                    };
                    cx.shared.load_bank.lock(|lb| lb.set_level(target));
                }
                Some(Command::SetAlignPhase(p)) => {
                    let t = match p {
                        shared::models::telemetry::telemetry::AlignPhase::A => {
                            crate::guv::adc::AlignPhase::A
                        }
                        shared::models::telemetry::telemetry::AlignPhase::B => {
                            crate::guv::adc::AlignPhase::B
                        }
                        shared::models::telemetry::telemetry::AlignPhase::C => {
                            crate::guv::adc::AlignPhase::C
                        }
                    };
                    cx.shared.align_target.lock(|a| *a = t);
                    defmt::info!("Align target set to {}", t);
                }
                _ => {
                    // all other commands (Configure/Start/Stop/LiveAdjust/etc.) are handled
                    // by the state machine below — do nothing here, let them fall through.
                }
            }

            let _wcet_timer = CycleTimer::start();
            let align = cx.shared.align_target.lock(|a| *a);
            // ── ADC: fold in the latest completed 100 ms AC window (if one
            // closed since last tick) and the DC means of the last 10 ms ──
            let (win, dc) = cx
                .shared
                .adc_acc
                .lock(|acc| (acc.take_window(), acc.take_control_dc(1.0)));
            cx.shared.adc_meas.lock(|am| {
                if let Some(w) = win {
                    crate::guv::adc::finalize_window(&w, 1.0, am);
                }
                if let Some(d) = dc {
                    am.apply_dc(&d);
                }
                am.select(align);
            });

            // ── Load rejection reported by the supervisor ──
            if let Some(ev) = cx.shared.load_reject.lock(|e| e.take()) {
                // Only READY/GENERATE get the recovery state; the conditional
                // write never overrides a FAULT/ESTOP the supervisor just set.
                let entered = cx.shared.state.lock(|s| {
                    if matches!(*s, STATE::READY | STATE::GENERATE) {
                        *s = STATE::LOAD_REJECTION;
                        true
                    } else {
                        false
                    }
                });
                if entered {
                    *cx.local.lr_from = current_state;
                    *cx.local.lr_ticks = 0;
                    *cx.local.lr_settled = 0;
                    defmt::warn!(
                        "Load rejection {} mA -> {} mA in {=str} -> LOAD_REJECTION",
                        (ev.i_before * 1000.0) as i32,
                        (ev.i_after * 1000.0) as i32,
                        current_state.as_str()
                    );
                } else {
                    defmt::info!(
                        "Load drop {} mA -> {} mA in {=str}: speed loop absorbs it",
                        (ev.i_before * 1000.0) as i32,
                        (ev.i_after * 1000.0) as i32,
                        current_state.as_str()
                    );
                }
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

                    // Did a HelloAck arrive? The loop top already popped ONE
                    // command into `received_cmd` this tick — check that first,
                    // or an ack that arrives once per tick is always stolen and
                    // the handshake only ever "succeeds" via the 1 s timeout.
                    // Then drain any further HelloAcks still queued; leave other
                    // queued commands intact.
                    let popped_ack = matches!(received_cmd, Some(Command::HelloAck));
                    let queued_ack = cx.shared.cmd_in.lock(|q| {
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
                    let acked = popped_ack || queued_ack;

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
                        Some(Command::LoadProfile(profile)) => {
                            defmt::info!("Profile loaded: {} points", profile.count);
                            cx.shared.active_profile.lock(|p| *p = Some(profile));
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
                        Some(Command::LoadProfile(profile)) => {
                            defmt::info!("Profile loaded: {} points", profile.count);
                            cx.shared.active_profile.lock(|p| *p = Some(profile));
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

                // ─── SPOOLUP: ramp motor to target RPM (or walk a profile) ───
                STATE::SPOOLUP => {
                    let config = cx.shared.run_config.lock(|rc| rc.unwrap_or_default());
                    *cx.local.run_elapsed_ms += dt_ms;
                    let elapsed = *cx.local.run_elapsed_ms;

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

                    // Is a setpoint profile loaded? If so it drives the target
                    // directly (its own first segment handles ramp-in), and we
                    // skip the classic ramp entirely for closed-loop modes.
                    let has_profile = cx.shared.active_profile.lock(|ap| ap.is_some());

                    // ── Profile-scheduled load steps + phase alignment ──
                    // Fire each breakpoint's actions once, as the profile clock
                    // crosses it — same clock and same modes that drive the RPM
                    // target below. The LoadBank still enforces the ordered path.
                    if has_profile && matches!(config.mode, RunMode::ClosedLoop | RunMode::Generate)
                    {
                        // run_elapsed_ms strictly increases within a run; if it
                        // didn't, the clock was reset for a new run → first tick.
                        let prev = match *cx.local.profile_prev_ms {
                            Some(p) if p < elapsed => Some(p),
                            _ => None,
                        };
                        let profile = cx.shared.active_profile.lock(|ap| *ap);
                        if let Some(p) = profile {
                            shared::models::telemetry::telemetry::for_each_crossed_point(
                                &p,
                                prev,
                                elapsed,
                                |pt| {
                                    use crate::guv::adc::AlignPhase as AA;
                                    use crate::guv::load_bank::LoadLevel as LL;
                                    use shared::models::telemetry::telemetry::{
                                        AlignPhase as WA, LoadAction as LA,
                                    };

                                    let load_name = match pt.load_action {
                                        LA::None => None,
                                        LA::HalfLoad => Some("HalfLoad"),
                                        LA::FullLoad => Some("FullLoad"),
                                        LA::StepDown => Some("StepDown"),
                                        LA::ShedAll => Some("ShedAll"),
                                    };
                                    if let Some(name) = load_name {
                                        defmt::info!("Profile t={}ms → load {=str}", elapsed, name);
                                        cx.shared.load_bank.lock(|lb| match pt.load_action {
                                            LA::None => {}
                                            LA::HalfLoad => lb.set_level(LL::One),
                                            LA::FullLoad => lb.set_level(LL::Both),
                                            LA::StepDown => lb.step_down(),
                                            LA::ShedAll => lb.set_level(LL::None),
                                        });
                                    }

                                    if let Some(ph) = pt.align_phase {
                                        let (target, name) = match ph {
                                            WA::A => (AA::A, "A"),
                                            WA::B => (AA::B, "B"),
                                            WA::C => (AA::C, "C"),
                                        };
                                        defmt::info!(
                                            "Profile t={}ms → align phase {=str}",
                                            elapsed,
                                            name
                                        );
                                        cx.shared.align_target.lock(|a| *a = target);
                                    }
                                },
                            );
                        }
                        *cx.local.profile_prev_ms = Some(elapsed);
                    } else {
                        *cx.local.profile_prev_ms = None;
                    }

                    // Ramp up (classic ramp — used when NO profile, or for the
                    // open-loop / generate spool)
                    if let Some(ref mut ramp) = cx.local.ramp {
                        let val = ramp.tick(dt_ms);
                        match config.mode {
                            RunMode::ClosedLoop | RunMode::Generate => {
                                // Target: profile if loaded, else ramped static target.
                                let target_rpm = if has_profile {
                                    cx.shared.active_profile.lock(|ap| {
                                        ap.as_ref().map_or(config.target_rpm, |p| {
                                            shared::models::telemetry::telemetry::eval_profile(
                                                p, elapsed,
                                            )
                                        })
                                    })
                                } else {
                                    config.target_rpm * val
                                };
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
                                // Target: profile if loaded, else static.
                                let target_rpm = cx.shared.active_profile.lock(|ap| {
                                    ap.as_ref().map_or(config.target_rpm, |p| {
                                        shared::models::telemetry::telemetry::eval_profile(
                                            p, elapsed,
                                        )
                                    })
                                });
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
                                // OpenLoop holds whatever duty the ramp ended at
                            }
                        }

                        // Profile-driven termination: a Stop-terminated profile
                        // that has run its course ramps down.
                        let profile_done = cx.shared.active_profile.lock(|ap| {
                            ap.as_ref().map_or(false, |p| {
                                shared::models::telemetry::telemetry::profile_finished(p, elapsed)
                            })
                        });

                        if profile_done {
                            defmt::info!("Profile finished — ramping down");
                            let speed = cx.shared.motor.lock(|m| m.demand());
                            *cx.local.ramp =
                                Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                            cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        } else if !has_profile
                            && config.hold_ms > 0
                            && elapsed >= config.ramp_up_ms + config.hold_ms
                        {
                            // Classic hold timeout (only when NO profile — a
                            // profile governs its own duration via EndBehavior).
                            defmt::info!("Hold complete — ramping down");
                            let speed = cx.shared.motor.lock(|m| m.demand());
                            *cx.local.ramp =
                                Some(crate::guv::ramp::Ramp::new(speed, 0.0, config.ramp_down_ms));
                            cx.shared.state.lock(|s| *s = STATE::RAMP_DOWN);
                        }
                    }

                    // LED: fast green blink during spoolup
                    if (elapsed / 100) % 2 == 0 {
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

                    // Hold speed while the caps excite the generator (the
                    // excitation load would otherwise drag it down open-loop).
                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let cal = cx.shared.calibration.lock(|c| *c);
                    let duty = crate::speed_hold(
                        cal,
                        cx.local.pid,
                        config.target_rpm,
                        measured,
                        dt_ms as f32 / 1000.0,
                    );
                    cx.shared.motor.lock(|m| m.set_demand(duty));

                    // TODO: monitor v_a_rms from the ADC; advance when voltage builds.
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

                    // Hold speed while (eventually) trimming phase.
                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let cal = cx.shared.calibration.lock(|c| *c);
                    let duty = crate::speed_hold(
                        cal,
                        cx.local.pid,
                        config.target_rpm,
                        measured,
                        dt_ms as f32 / 1000.0,
                    );
                    cx.shared.motor.lock(|m| m.set_demand(duty));

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

                    // Hold speed: feedforward + PID, same law as SPOOLUP.
                    let measured = cx.shared.current_rpm.lock(|r| *r);
                    let dt_s = dt_ms as f32 / 1000.0;
                    let cal = cx.shared.calibration.lock(|c| *c);
                    let output =
                        crate::speed_hold(cal, cx.local.pid, config.target_rpm, measured, dt_s);
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
                    let cal = cx.shared.calibration.lock(|c| *c);
                    let output =
                        crate::speed_hold(cal, cx.local.pid, config.target_rpm, measured, dt_s);
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

                    // Load rejection is detected by the supervisor (DC-bus
                    // current drop) and routed here at the top of the loop.

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
                    let cal = cx.shared.calibration.lock(|c| *c);
                    let output =
                        crate::speed_hold(cal, cx.local.pid, config.target_rpm, measured, dt_s);
                    cx.shared.motor.lock(|m| m.set_demand(output));

                    // Recovered = at least LR_MIN_TICKS in this state AND
                    // LR_SETTLE_TICKS in a row within ±2% of target. (Checking
                    // the error alone would exit on the very first tick, before
                    // the shaft has even started to speed up.)
                    *cx.local.lr_ticks += 1;
                    let error = (config.target_rpm - measured).abs();
                    if error < config.target_rpm * 0.02 {
                        *cx.local.lr_settled += 1;
                    } else {
                        *cx.local.lr_settled = 0;
                    }
                    let back = *cx.local.lr_from;
                    let settled = *cx.local.lr_ticks >= crate::LR_MIN_TICKS
                        && *cx.local.lr_settled >= crate::LR_SETTLE_TICKS;
                    let timed_out = *cx.local.lr_ticks >= crate::LR_TIMEOUT_TICKS;
                    if settled || timed_out {
                        if settled {
                            defmt::info!(
                                "Load rejection recovered in {} ms -> {=str}",
                                *cx.local.lr_ticks * dt_ms,
                                back.as_str()
                            );
                        } else {
                            defmt::warn!(
                                "Load rejection: not within 2% after {} ms -> {=str} anyway",
                                *cx.local.lr_ticks * dt_ms,
                                back.as_str()
                            );
                        }
                        cx.shared.state.lock(|s| {
                            if *s == STATE::LOAD_REJECTION {
                                *s = back;
                            }
                        });
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
                    cx.shared.load_bank.lock(|lb| lb.emergency_shed());

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

            // ── record timing + evaluate the execution-time deadline ──
            let elapsed = _wcet_timer.stop();
            let idx = state_index(current_state);
            let has_deadline = state_has_deadline(current_state);
            let deadline_missed = cx.shared.wcet.lock(|w| {
                w.record_and_check(
                    idx,
                    elapsed,
                    has_deadline,
                    crate::DEADLINE_BUDGET_CYCLES,
                    crate::DEADLINE_MISS_STREAK,
                )
            });

            if deadline_missed {
                // The real-time guarantee degraded — fail safe through the
                // existing fault path (same as any other trip).
                defmt::error!(
                    "!! DEADLINE MISS in {} — {} cyc > {} budget, {}x consecutive → FAULT",
                    // state name via your existing helper if you have one, else idx
                    idx,
                    elapsed,
                    crate::DEADLINE_BUDGET_CYCLES,
                    crate::DEADLINE_MISS_STREAK
                );
                cx.shared.motor.lock(|m| m.emergency_off());
                cx.shared.relay.lock(|r| r.set_low());
                cx.shared
                    .last_fault
                    .lock(|f| *f = Some(Fault::DeadlineMiss));
                cx.shared.state.lock(|s| *s = STATE::FAULT);
                cx.local.pid.reset();
                *cx.local.ramp = None;
                *cx.local.run_elapsed_ms = 0;
                cx.shared.ld3.lock(|l| l.set_high()); // red
                Mono::delay(dt_ms.millis()).await;
                continue;
            }

            Mono::delay(dt_ms.millis()).await;
        }
    }

    // ────────────────────────────────────────────
    //  RPM Monitor — encoder reading + telemetry
    // ────────────────────────────────────────────
    #[task(priority = 1, shared = [encoder, tx, state, current_rpm, motor, run_config, last_fault, measurements, pending_report, adc_meas, load_bank, align_target])]
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

            // Values that don't need the supervisor-shared lock, read first.
            let adc_m = cx.shared.adc_meas.lock(|am| *am);
            let load_lvl = cx.shared.load_bank.lock(|lb| lb.level());

            // ── 1. Acquire + build the frame. This lock includes resources the
            // priority-2 supervisor uses (current_rpm, motor, state,
            // last_fault), so it blocks the supervisor while held: keep it to
            // a few microseconds — no UART traffic in here. ──
            let (frame, rpm, demand) = (
                &mut cx.shared.encoder,
                &mut cx.shared.state,
                &mut cx.shared.current_rpm,
                &mut cx.shared.motor,
                &mut cx.shared.run_config,
                &mut cx.shared.last_fault,
                &mut cx.shared.measurements,
            )
                .lock(
                    |enc, state, current_rpm, motor, run_config, last_fault, meas| {
                        // ── acquisition: encoder → rpm ──
                        let current_count = enc.count();
                        let delta = current_count.wrapping_sub(last_count);
                        last_count = current_count;
                        let counts_per_second = (delta as i32 as f32) * 100.0;
                        let rpm = (counts_per_second / counts_per_rev) * 60.0 * crate::ENCODER_SIGN;

                        // write into BOTH: control loop reads current_rpm, telemetry reads measurements.
                        *current_rpm = rpm;
                        meas.rpm = rpm;
                        let wire_load = match load_lvl {
                            crate::guv::load_bank::LoadLevel::None => {
                                shared::models::telemetry::telemetry::LoadLevel::None
                            }
                            crate::guv::load_bank::LoadLevel::One => {
                                shared::models::telemetry::telemetry::LoadLevel::One
                            }
                            crate::guv::load_bank::LoadLevel::Both => {
                                shared::models::telemetry::telemetry::LoadLevel::Both
                            }
                        };
                        // map the selected align target to its wire enum
                        let align_wire = cx.shared.align_target.lock(|a| match *a {
                            crate::guv::adc::AlignPhase::A => {
                                shared::models::telemetry::telemetry::AlignPhase::A
                            }
                            crate::guv::adc::AlignPhase::B => {
                                shared::models::telemetry::telemetry::AlignPhase::B
                            }
                            crate::guv::adc::AlignPhase::C => {
                                shared::models::telemetry::telemetry::AlignPhase::C
                            }
                        });

                        // ── assembly: frame is built FROM the ADC measurements ──
                        let frame = shared::models::telemetry::telemetry::Telemetry {
                            ts_ms,
                            state: *state,
                            rpm: meas.rpm,
                            duty_percent: motor.demand(),
                            // phase alignment (chip 1 — reference + 3 motor phases):
                            v_ref_rms: adc_m.v_ref_rms,
                            v_a_rms: adc_m.v_a_rms,
                            v_b_rms: adc_m.v_b_rms,
                            v_c_rms: adc_m.v_c_rms,
                            phase_err_a: adc_m.phase_err_a,
                            phase_err_b: adc_m.phase_err_b,
                            phase_err_c: adc_m.phase_err_c,
                            theta_err_rad: adc_m.phase_err_selected,
                            align_phase: align_wire,
                            // rectifier DC bus (chip 2) + line frequency:
                            v_dc: adc_m.v_dc,
                            i_dc: adc_m.i_dc,
                            freq_hz: adc_m.freq_hz,
                            temp_c: meas.temp_c,
                            load_level: wire_load,
                            run_mode: run_config.map(|c| c.mode),
                            fault: *last_fault,
                        };
                        (frame, rpm, motor.demand())
                    },
                );

            // ── 2. Transmit holding only `tx` (shared with priority-1 tasks
            // only), so the ~7 ms of UART time no longer blocks the
            // supervisor. ──
            if frame.state != STATE::BOOT {
                cx.shared.tx.lock(|tx| {
                    let mut buf = [0u8; 192];
                    if let Ok(encoded) =
                        postcard::to_slice_cobs(&Uplink::Telemetry(frame), &mut buf)
                    {
                        for byte in encoded.iter() {
                            block!(tx.write(*byte)).ok();
                        }
                    }

                    // send a pending calibration report, if one was queued
                    if let Some(rep) = report {
                        let mut rbuf = [0u8; 192];
                        if let Ok(encoded) =
                            postcard::to_slice_cobs(&Uplink::Calibration(rep), &mut rbuf)
                        {
                            for byte in encoded.iter() {
                                block!(tx.write(*byte)).ok();
                            }
                            defmt::info!("Calibration report sent");
                        }
                    }
                });
            }

            if loop_counter % 100 == 0 {
                defmt::info!(
                    "RPM: {} DUTY: {}% STATE: {}",
                    rpm as i32,
                    (demand * 100.0) as i32,
                    frame.state.as_str()
                );
            }
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
    // ─────────────────────────────────────────────────────────────────────────────
    // C) DRDY ISR — fires at ADC_SAMPLE_RATE (~4kHz). Lean: read frame, accumulate.
    //    Priority 3 (above 500Hz supervise=2, 100Hz control=1) — a missed sample
    //    corrupts RMS, but this is tiny. DRDY is PD5 → EXTI line 5 → EXTI9_5.
    //    Pending bit is cleared FIRST so a DRDY edge that lands during the read
    //    re-pends the IRQ instead of being lost.
    // ─────────────────────────────────────────────────────────────────────────────
    #[task(binds = EXTI9_5, priority = 3, shared = [adc, adc_acc], local = [adc_drdy])]
    fn adc_drdy_isr(mut cx: adc_drdy_isr::Context) {
        cx.local.adc_drdy.clear_interrupt_pending_bit();
        if let Some(frame) = cx.shared.adc.lock(|adc| adc.read_frame()) {
            cx.shared.adc_acc.lock(|acc| acc.add(&frame));
        }
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
