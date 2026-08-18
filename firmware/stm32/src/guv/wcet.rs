// src/guv/wcet.rs — observed-WCET instrumentation via DWT cycle counter.
//
// Measures per-STATE execution time of the control loop and keeps a running
// max (observed worst case) + a simple mean, per state. Zero heap, fixed arrays.
//
// This is MEASUREMENT-BASED timing (max observed over N runs), NOT a proven
// static WCET bound. Report it as such.

#![allow(dead_code)]

use cortex_m::peripheral::DWT;

/// Number of states we track (mirror your STATE enum's count).
pub const N_STATES: usize = 14;

/// Sysclk in Hz — for converting cycles → microseconds. MUST match your clock.
pub const SYSCLK_HZ: u32 = 64_000_000;

/// Per-state timing stats, all in CPU cycles.
#[derive(Clone, Copy)]
pub struct StateTiming {
    pub max_cycles: u32,  // observed worst case
    pub last_cycles: u32, // most recent
    pub sum_cycles: u64,  // for mean
    pub count: u32,       // iterations measured
}

impl StateTiming {
    const fn new() -> Self {
        Self {
            max_cycles: 0,
            last_cycles: 0,
            sum_cycles: 0,
            count: 0,
        }
    }
    pub fn mean_cycles(&self) -> u32 {
        if self.count == 0 {
            0
        } else {
            (self.sum_cycles / self.count as u64) as u32
        }
    }
    pub fn max_us(&self) -> f32 {
        self.max_cycles as f32 / (SYSCLK_HZ as f32 / 1_000_000.0)
    }
    pub fn mean_us(&self) -> f32 {
        self.mean_cycles() as f32 / (SYSCLK_HZ as f32 / 1_000_000.0)
    }
}

/// The timing table: one StateTiming per state, indexed by state-as-usize.
pub struct WcetTable {
    states: [StateTiming; N_STATES],
    /// Consecutive-overrun counter per state. Reset to 0 on any in-budget tick;
    /// incremented on an overrun. Trips when it reaches the streak threshold.
    overrun_streak: [u32; N_STATES],
}

impl WcetTable {
    pub const fn new() -> Self {
        Self {
            states: [StateTiming::new(); N_STATES],
            overrun_streak: [0; N_STATES],
        }
    }

    /// Record an elapsed measurement for a given state index.
    #[inline]
    pub fn record(&mut self, state_idx: usize, cycles: u32) {
        if state_idx >= N_STATES {
            return;
        }
        let s = &mut self.states[state_idx];
        s.last_cycles = cycles;
        s.count = s.count.saturating_add(1);
        s.sum_cycles = s.sum_cycles.saturating_add(cycles as u64);
        if cycles > s.max_cycles {
            s.max_cycles = cycles;
        }
    }

    pub fn get(&self, state_idx: usize) -> Option<StateTiming> {
        self.states.get(state_idx).copied()
    }

    /// Worst case across ALL states — the headline number.
    pub fn global_max_cycles(&self) -> u32 {
        self.states.iter().map(|s| s.max_cycles).max().unwrap_or(0)
    }
    pub fn global_max_us(&self) -> f32 {
        self.global_max_cycles() as f32 / (SYSCLK_HZ as f32 / 1_000_000.0)
    }

    #[inline]
    pub fn record_and_check(
        &mut self,
        state_idx: usize,
        cycles: u32,
        has_deadline: bool,
        budget_cycles: u32,
        streak_limit: u32,
    ) -> bool {
        // always record the timing (even non-deadline states — good for the report)
        self.record(state_idx, cycles);

        if state_idx >= N_STATES || !has_deadline {
            return false;
        }

        if cycles > budget_cycles {
            // overrun — extend the streak
            self.overrun_streak[state_idx] = self.overrun_streak[state_idx].saturating_add(1);
            // trip exactly when we REACH the limit (so it fires once, not every
            // subsequent tick — the fault transition will leave this state anyway)
            self.overrun_streak[state_idx] >= streak_limit
        } else {
            // in budget — reset the streak
            self.overrun_streak[state_idx] = 0;
            false
        }
    }

    /// Current overrun streak for a state (diagnostic).
    pub fn overrun_streak(&self, state_idx: usize) -> u32 {
        self.overrun_streak.get(state_idx).copied().unwrap_or(0)
    }
}

/// RAII-style scoped timer: reads CYCCNT on new(), and on `stop()` returns the
/// elapsed cycles (wrapping-safe). Use explicitly (no Drop, so you control when
/// the measurement ends and can hand the result to the table).
pub struct CycleTimer {
    start: u32,
}
impl CycleTimer {
    #[inline]
    pub fn start() -> Self {
        Self {
            start: DWT::cycle_count(),
        }
    }
    #[inline]
    pub fn stop(self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.start)
    }
}
