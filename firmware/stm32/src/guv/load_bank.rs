use stm32h7xx_hal::gpio::{self, Output, PushPull};

/// How many load steps are currently engaged (0, 1, or 2). Because of the
/// series-ground wiring, valid states are ONLY: none, step-1-only, or both.
/// "step-2-only" is electrically impossible (2 needs 1's ground) and this type
/// makes it unrepresentable.
#[derive(Clone, Copy, PartialEq, Eq, Debug, defmt::Format)]
pub enum LoadLevel {
    None, // both open
    One,  // step 1 closed only
    Both, // step 1 + step 2 closed
}

/// Owns the two load-step relays and enforces the ordered-switching wiring
/// constraint. All load changes go through here.
pub struct LoadBank {
    step_1: gpio::Pin<'E', 15, Output<PushPull>>,
    step_2: gpio::Pin<'E', 8, Output<PushPull>>,
    level: LoadLevel,
}

impl LoadBank {
    /// Construct from the two relay pins. Assumes both start OPEN (set_low in boot).
    pub fn new(
        step_1: gpio::Pin<'E', 15, Output<PushPull>>,
        step_2: gpio::Pin<'E', 8, Output<PushPull>>,
    ) -> Self {
        Self {
            step_1,
            step_2,
            level: LoadLevel::None,
        }
    }

    pub fn level(&self) -> LoadLevel {
        self.level
    }

    /// Engage the next load step UP (None → One → Both). Enforces close order:
    /// step 1 must be closed before step 2. No-op if already at Both.
    pub fn step_up(&mut self) {
        match self.level {
            LoadLevel::None => {
                // close step 1 first — it provides the ground path step 2 needs.
                self.step_1.set_high();
                self.level = LoadLevel::One;
                defmt::info!("LoadBank: step 1 CLOSED (level=One)");
            }
            LoadLevel::One => {
                // step 1 already closed → step 2 now has a return path; safe to close.
                self.step_2.set_high();
                self.level = LoadLevel::Both;
                defmt::info!("LoadBank: step 2 CLOSED (level=Both)");
            }
            LoadLevel::Both => {
                defmt::warn!("LoadBank: step_up ignored — already at Both");
            }
        }
    }

    /// Shed one load step DOWN gracefully (Both → One → None). Opens step 2
    /// before step 1 (reverse order) for a smooth two-step reduction — gentler
    /// on the generator than dropping everything at once.
    pub fn step_down(&mut self) {
        match self.level {
            LoadLevel::Both => {
                // open step 2 first (it has its own relay); step 1 still carries load.
                self.step_2.set_low();
                self.level = LoadLevel::One;
                defmt::info!("LoadBank: step 2 OPEN (level=One)");
            }
            LoadLevel::One => {
                self.step_1.set_low();
                self.level = LoadLevel::None;
                defmt::info!("LoadBank: step 1 OPEN (level=None)");
            }
            LoadLevel::None => {
                defmt::warn!("LoadBank: step_down ignored — already at None");
            }
        }
    }

    /// Set a specific level, taking the correct ordered path to get there.
    /// Steps one at a time through valid intermediate states (never violates the
    /// close-1-before-2 / open-2-before-1 rules).
    pub fn set_level(&mut self, target: LoadLevel) {
        while self.level != target {
            match (self.level, target) {
                // need to go up
                (LoadLevel::None, LoadLevel::One | LoadLevel::Both)
                | (LoadLevel::One, LoadLevel::Both) => self.step_up(),
                // need to go down
                (LoadLevel::Both, LoadLevel::One | LoadLevel::None)
                | (LoadLevel::One, LoadLevel::None) => self.step_down(),
                _ => break, // already there (shouldn't hit due to while guard)
            }
        }
    }

    /// IMMEDIATE shed for fault / E-stop / load-rejection. Opens step 1 FIRST,
    /// which cuts step 2 by proxy (removes its ground) AND step 1 simultaneously
    /// → all load drops at once. Use when safety beats smoothness. Then ensure
    /// step 2's relay is also de-energized for a clean state.
    pub fn emergency_shed(&mut self) {
        // open 1 → kills both electrically (2 loses its ground return).
        self.step_1.set_low();
        // also de-energize 2's coil so we land in a clean, fully-open state.
        self.step_2.set_low();
        self.level = LoadLevel::None;
        defmt::warn!("LoadBank: EMERGENCY SHED — all load dropped (opened step 1 first)");
    }

    /// Force both open with no ordering guarantees — only for init / known-safe
    /// resets. Prefer emergency_shed() for runtime safety cuts.
    pub fn force_open(&mut self) {
        self.step_1.set_low();
        self.step_2.set_low();
        self.level = LoadLevel::None;
    }
}
