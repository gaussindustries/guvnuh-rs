// (combined adc.rs — SPI driver + phase-alignment measurement layer)
// ═════════════════════════════════════════════════════════════════════════════
// DUAL ADS131M04 DRIVER + PHASE-ALIGNMENT MEASUREMENT
//
// Two ADS131M04 24-bit delta-sigma ADCs share SPI1 with separate CS lines and
// tied SYNC (coherent sampling). Chip1 carries the AC voltages used for phase
// alignment (reference + 3 motor phases); Chip2 carries the rectifier DC bus.
// The external 8.192 MHz oscillator (CTS CB3LV) must feed both CLKIN pins or the
// ADCs never sample.
// ═════════════════════════════════════════════════════════════════════════════

#![allow(dead_code)]

use stm32h7xx_hal::gpio::{self, Input, Output, PushPull};
// Bring the blocking SPI Transfer trait into scope for spi.transfer(&mut buf).
// (cortex_m re-exports the embedded-hal 0.2 blocking traits under this alias.)
use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer as _;

// ─────────────────────────────────────────────────────────────────────────────
// Master enable: false until the ADC chips + 8.192 MHz oscillator are wired.
// When false, Ads131Pair::new() skips the blocking SPI config and the DRDY
// interrupt is left un-armed (see 0_boot.rs), so the board boots clean with the
// ADC dormant and telemetry electrical fields read 0.
// ─────────────────────────────────────────────────────────────────────────────
pub const ADC_HARDWARE_ENABLED: bool = false;

// ─────────────────────────────────────────────────────────────────────────────
// TUNABLE: ADC sample rate (SPS per channel). The DRDY pin fires at this rate.
//   Samples per 60 Hz cycle = ADC_SAMPLE_RATE / 60
//     2000 → 33/cycle (light)   4000 → 67/cycle (default)   8000 → 133/cycle
// Faster = finer phase resolution + better RMS, but heavier ISR. Validate WCET.
// ─────────────────────────────────────────────────────────────────────────────
pub const ADC_SAMPLE_RATE: u32 = 4000; // SPS per channel
pub const AC_LINE_HZ: f32 = 60.0;
pub const SAMPLES_PER_CYCLE: u32 = ADC_SAMPLE_RATE / (AC_LINE_HZ as u32);

// ── ADS131M04 register map (subset) ──
const REG_ID: u8 = 0x00;
const REG_STATUS: u8 = 0x01;
const REG_MODE: u8 = 0x02;
const REG_CLOCK: u8 = 0x03;
const REG_GAIN1: u8 = 0x04;
const REG_CFG: u8 = 0x06;

const CMD_NULL: u16 = 0x0000;
const CMD_RESET: u16 = 0x0011;
const CMD_STANDBY: u16 = 0x0022;
const CMD_WAKEUP: u16 = 0x0033;

// ─────────────────────────────────────────────────────────────────────────────
// PIN ASSIGNMENTS (match 0_boot.rs):
//   SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (AF5)
//   CS1=PD3, CS2=PD4, DRDY=PD5 (EXTI), SYNC=PD6, RST=PD7
//   CLKIN = external 8.192 MHz oscillator to both chips (not an MCU pin)
// ─────────────────────────────────────────────────────────────────────────────
pub type Cs1 = gpio::Pin<'D', 3, Output<PushPull>>;
pub type Cs2 = gpio::Pin<'D', 4, Output<PushPull>>;
pub type Drdy = gpio::Pin<'D', 5, Input>;
pub type Sync = gpio::Pin<'D', 6, Output<PushPull>>;
pub type Rst = gpio::Pin<'D', 7, Output<PushPull>>;

pub type AdcSpi = stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>;

// ── RREG/WREG command builders ──
#[inline]
fn wreg_cmd(addr: u8, count: u8) -> u16 {
    0x6000 | ((addr as u16) << 7) | ((count as u16) - 1)
}
#[inline]
fn rreg_cmd(addr: u8, count: u8) -> u16 {
    0xA000 | ((addr as u16) << 7) | ((count as u16) - 1)
}

/// Map desired sample rate → OSR register bits. With 8.192 MHz CLKIN:
/// OSR=1024 → 4000 SPS, OSR=2048 → 2000, OSR=512 → 8000.
fn osr_bits_for_rate(rate: u32) -> u16 {
    match rate {
        r if r >= 8000 => 0b010, // OSR 512  → 8000 SPS
        r if r >= 4000 => 0b011, // OSR 1024 → 4000 SPS (default)
        _ => 0b100,              // OSR 2048 → 2000 SPS
    }
}

/// Big-endian 24-bit two's-complement bytes → i32 (sign-extended).
#[inline]
fn be24_to_i32(b: &[u8]) -> i32 {
    let raw = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | (b[2] as u32);
    if raw & 0x0080_0000 != 0 {
        (raw | 0xFF00_0000) as i32
    } else {
        raw as i32
    }
}

#[derive(Clone, Copy)]
enum Chip {
    One,
    Two,
}

/// The driver: owns SPI + control pins.
pub struct Ads131Pair {
    spi: AdcSpi,
    cs1: Cs1,
    cs2: Cs2,
    sync: Sync,
    rst: Rst,
    gain: f32,
}

impl Ads131Pair {
    /// Construct + (conditionally) configure both chips. When ADC_HARDWARE_ENABLED
    /// is false, skips the blocking SPI config so the board boots with the ADC
    /// absent.
    pub fn new(spi: AdcSpi, cs1: Cs1, cs2: Cs2, mut sync: Sync, mut rst: Rst) -> Self {
        // hardware reset both chips (RST tied, active low)
        rst.set_low();
        cortex_m::asm::delay(64_000); // ~1ms at 64MHz
        rst.set_high();
        cortex_m::asm::delay(640_000); // ~10ms settle
        sync.set_high(); // SYNC high = normal

        let mut me = Self {
            spi,
            cs1,
            cs2,
            sync,
            rst,
            gain: 1.0,
        };
        me.cs1.set_high();
        me.cs2.set_high();

        if ADC_HARDWARE_ENABLED {
            me.configure();
        } else {
            defmt::warn!("ADS131M04 DISABLED (ADC_HARDWARE_ENABLED=false) — SPI config skipped");
        }
        me
    }

    /// Configure both chips identically: OSR (→ sample rate), gain, channel enable.
    fn configure(&mut self) {
        let osr_bits = osr_bits_for_rate(ADC_SAMPLE_RATE);
        let clock_val: u16 = 0x000E | (osr_bits << 2); // enable 4 ch + OSR
        let gain_val: u16 = 0x0000; // gain 1 all channels
        for chip in [Chip::One, Chip::Two] {
            self.write_reg(chip, REG_CLOCK, clock_val);
            self.write_reg(chip, REG_GAIN1, gain_val);
        }
        self.gain = 1.0;
        defmt::info!(
            "ADS131M04 pair configured: OSR bits={=u16}, rate≈{=u32} SPS, gain=1",
            osr_bits,
            ADC_SAMPLE_RATE
        );
    }

    /// Read one coherent frame from BOTH chips. Called from the DRDY ISR.
    ///   Chip1 → [ref, v_a, v_b, v_c]   Chip2 → [v_dc, i_dc, spare, spare]
    pub fn read_frame(&mut self) -> RawFrame {
        let c1 = self.read_chip(Chip::One);
        let c2 = self.read_chip(Chip::Two);
        RawFrame {
            v_ref: c1[0],
            v_a: c1[1],
            v_b: c1[2],
            v_c: c1[3],
            v_dc: c2[0],
            i_dc: c2[1],
        }
    }

    /// Read the 4 channel words from one chip. Frame = status + 4×24bit + crc = 18 bytes.
    fn read_chip(&mut self, chip: Chip) -> [i32; 4] {
        self.select(chip, true);
        let mut buf = [0u8; 18];
        let _ = self.spi.transfer(&mut buf);
        self.select(chip, false);
        [
            be24_to_i32(&buf[3..6]),
            be24_to_i32(&buf[6..9]),
            be24_to_i32(&buf[9..12]),
            be24_to_i32(&buf[12..15]),
        ]
    }

    fn write_reg(&mut self, chip: Chip, addr: u8, val: u16) {
        self.select(chip, true);
        let cmd = wreg_cmd(addr, 1);
        let mut tx = [
            (cmd >> 8) as u8,
            (cmd & 0xFF) as u8,
            0,
            (val >> 8) as u8,
            (val & 0xFF) as u8,
            0,
        ];
        let _ = self.spi.transfer(&mut tx);
        self.select(chip, false);
    }

    #[inline]
    fn select(&mut self, chip: Chip, active: bool) {
        match chip {
            Chip::One => {
                if active {
                    self.cs1.set_low()
                } else {
                    self.cs1.set_high()
                }
            }
            Chip::Two => {
                if active {
                    self.cs2.set_low()
                } else {
                    self.cs2.set_high()
                }
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// (measurement layer follows — appended below)
// ═════════════════════════════════════════════════════════════════════════════

/// Which motor phase the PLL aligns the reference to. Set from the terminal via
/// Command::SetAlignPhase; defaults to A. Mirror this enum in the shared crate
/// so the wire Command can carry it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum AlignPhase {
    A,
    B,
    C,
}

impl Default for AlignPhase {
    fn default() -> Self {
        AlignPhase::A
    }
}

/// One coherent sample set. Chip1 = reference + 3 motor phase voltages (sampled
/// the same instant, so phase differences are meaningful). Chip2 = rectifier DC.
#[derive(Clone, Copy, Default)]
pub struct RawFrame {
    pub v_ref: i32, // reference phase (chip1 ch0)
    pub v_a: i32,   // motor phase A   (chip1 ch1)
    pub v_b: i32,   // motor phase B   (chip1 ch2)
    pub v_c: i32,   // motor phase C   (chip1 ch3)
    pub v_dc: i32,  // rectifier DC voltage (chip2 ch0)
    pub i_dc: i32,  // rectifier DC current (chip2 ch1)
                    // chip2 ch2/ch3 spare — add fields here when you wire them.
}

/// Per-channel zero-crossing tracker. Records the fractional sample index of the
/// most recent RISING zero-crossing within the current window, for phase math.
#[derive(Clone, Copy, Default)]
struct CrossTracker {
    last_sign: i8,       // sign of previous sample (+1 / -1)
    last_cross_idx: f32, // sample index of most recent rising crossing this window
    have_cross: bool,    // saw at least one crossing this window
    crossings: u32,      // rising crossings this window (for frequency)
}

impl CrossTracker {
    #[inline]
    fn add(&mut self, sample: f64, idx: u32) {
        let sign: i8 = if sample >= 0.0 { 1 } else { -1 };
        if self.last_sign < 0 && sign > 0 {
            // rising zero-crossing between (idx-1) and idx. Linear-interpolate the
            // sub-sample crossing point isn't tracked here (we use the integer idx);
            // for finer phase, interpolate using prev/cur magnitudes.
            self.last_cross_idx = idx as f32;
            self.have_cross = true;
            self.crossings += 1;
        }
        self.last_sign = sign;
    }
    #[inline]
    fn reset_window(&mut self) {
        // keep last_sign for continuity across windows; clear per-window state.
        self.have_cross = false;
        self.crossings = 0;
    }
}

/// Running accumulators between finalize calls. Tracks RMS (sum-of-squares) for
/// magnitudes AND zero-crossings for phase alignment. Kept lean — no divide/sqrt
/// in the ISR; those happen in finalize.
#[derive(Clone, Copy, Default)]
pub struct Accumulator {
    // sum of squares for RMS magnitudes
    pub sq_ref: f64,
    pub sq_a: f64,
    pub sq_b: f64,
    pub sq_c: f64,
    // DC-bus running sums (mean, not RMS)
    pub sum_v_dc: f64,
    pub sum_i_dc: f64,
    // zero-crossing trackers (phase alignment)
    x_ref: CrossTracker,
    x_a: CrossTracker,
    x_b: CrossTracker,
    x_c: CrossTracker,
    // sample count this window
    pub n: u32,
}

impl Accumulator {
    #[inline]
    pub fn reset(&mut self) {
        self.sq_ref = 0.0;
        self.sq_a = 0.0;
        self.sq_b = 0.0;
        self.sq_c = 0.0;
        self.sum_v_dc = 0.0;
        self.sum_i_dc = 0.0;
        self.x_ref.reset_window();
        self.x_a.reset_window();
        self.x_b.reset_window();
        self.x_c.reset_window();
        self.n = 0;
    }

    /// Add one coherent sample (called from the DRDY ISR — keep LEAN).
    #[inline]
    pub fn add(&mut self, f: &RawFrame) {
        let vr = f.v_ref as f64;
        let va = f.v_a as f64;
        let vb = f.v_b as f64;
        let vc = f.v_c as f64;

        self.sq_ref += vr * vr;
        self.sq_a += va * va;
        self.sq_b += vb * vb;
        self.sq_c += vc * vc;

        self.sum_v_dc += f.v_dc as f64;
        self.sum_i_dc += f.i_dc as f64;

        let idx = self.n;
        self.x_ref.add(vr, idx);
        self.x_a.add(va, idx);
        self.x_b.add(vb, idx);
        self.x_c.add(vc, idx);

        self.n = self.n.wrapping_add(1);
    }
}

/// Finalized electrical + alignment scalars. Shipped in telemetry and used by
/// the control loop (PLL_LOCK reads theta_err_rad; the governor reads i_dc for
/// feedforward and v_dc for protection).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct Measurements {
    // magnitudes (RMS, SI volts/amps after scaling)
    pub v_ref_rms: f32,
    pub v_a_rms: f32,
    pub v_b_rms: f32,
    pub v_c_rms: f32,
    pub v_dc: f32,
    pub i_dc: f32,
    pub freq_hz: f32, // line frequency from the reference channel

    // phase error (radians, wrapped ±π) of each motor phase RELATIVE to reference.
    // Positive = phase leads reference. PLL drives the SELECTED one to zero.
    pub phase_err_a: f32,
    pub phase_err_b: f32,
    pub phase_err_c: f32,
    /// The error for the currently-selected align target — this is theta_err_rad.
    pub phase_err_selected: f32,
}

// ── scaling helpers (fill ratios once the front-end is built) ──
const ADC_FS_VOLTS: f32 = 1.2;
const ADC_COUNTS: f32 = 8_388_608.0; // 2^23

// PLACEHOLDER ratios — replace with your measured divider/PT/shunt values:
const REF_V_RATIO: f32 = 1.0; // reference-phase divider/PT → real volts per pin-volt
const PHASE_V_RATIO: f32 = 1.0; // motor-phase divider/PT
const DC_V_RATIO: f32 = 1.0; // DC-bus divider
const DC_I_RATIO: f32 = 1.0; // 1 / (shunt × INA240 gain) → real amps per pin-volt

#[inline]
fn counts_to_pin_volts(raw: f64, gain: f32) -> f32 {
    (raw as f32) * (ADC_FS_VOLTS / ADC_COUNTS) / gain
}

#[inline]
fn rms_counts(sq: f64, n: f64) -> f64 {
    libm::sqrt(sq / n)
}

/// Wrap a phase difference (radians) into ±π.
#[inline]
fn wrap_pi(mut x: f32) -> f32 {
    use core::f32::consts::PI;
    while x > PI {
        x -= 2.0 * PI;
    }
    while x < -PI {
        x += 2.0 * PI;
    }
    x
}

/// Phase error of a target channel vs the reference, in radians. Uses the
/// difference in rising-crossing sample index over one window, scaled to the
/// samples-per-cycle. Returns 0 if either lacks a crossing this window.
#[inline]
fn phase_err(target: &CrossTracker, reference: &CrossTracker) -> f32 {
    use core::f32::consts::PI;
    if !target.have_cross || !reference.have_cross {
        return 0.0;
    }
    let samples_per_cycle = SAMPLES_PER_CYCLE as f32;
    let d_samples = target.last_cross_idx - reference.last_cross_idx;
    let err = (d_samples / samples_per_cycle) * 2.0 * PI;
    wrap_pi(err)
}

/// FULL finalize (100 Hz control tick): magnitudes + all phase errors + the
/// selected error. `align` picks which phase's error becomes phase_err_selected
/// (→ theta_err_rad). Caller resets the accumulator after.
pub fn finalize_full(acc: &Accumulator, gain: f32, align: AlignPhase) -> Measurements {
    if acc.n == 0 {
        return Measurements::default();
    }
    let n = acc.n as f64;

    let v_ref_rms = counts_to_pin_volts(rms_counts(acc.sq_ref, n), gain) * REF_V_RATIO;
    let v_a_rms = counts_to_pin_volts(rms_counts(acc.sq_a, n), gain) * PHASE_V_RATIO;
    let v_b_rms = counts_to_pin_volts(rms_counts(acc.sq_b, n), gain) * PHASE_V_RATIO;
    let v_c_rms = counts_to_pin_volts(rms_counts(acc.sq_c, n), gain) * PHASE_V_RATIO;

    let v_dc = counts_to_pin_volts(acc.sum_v_dc / n, gain) * DC_V_RATIO;
    let i_dc = counts_to_pin_volts(acc.sum_i_dc / n, gain) * DC_I_RATIO;

    // frequency from the reference channel's crossings over the window
    let window_s = n / (ADC_SAMPLE_RATE as f64);
    let freq_hz = if window_s > 0.0 {
        (acc.x_ref.crossings as f64 / window_s) as f32
    } else {
        0.0
    };

    // phase errors of each motor phase vs the reference
    let phase_err_a = phase_err(&acc.x_a, &acc.x_ref);
    let phase_err_b = phase_err(&acc.x_b, &acc.x_ref);
    let phase_err_c = phase_err(&acc.x_c, &acc.x_ref);
    let phase_err_selected = match align {
        AlignPhase::A => phase_err_a,
        AlignPhase::B => phase_err_b,
        AlignPhase::C => phase_err_c,
    };

    Measurements {
        v_ref_rms,
        v_a_rms,
        v_b_rms,
        v_c_rms,
        v_dc,
        i_dc,
        freq_hz,
        phase_err_a,
        phase_err_b,
        phase_err_c,
        phase_err_selected,
    }
}

/// FAST protection subset (500 Hz, with the overspeed supervisor): DC bus V/I
/// (over-volt/over-current + the feedforward source) + line frequency. Does NOT
/// reset the accumulator (the 100 Hz finalize owns the reset).
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct ProtectionScalars {
    pub v_dc: f32,
    pub i_dc: f32, // ← load-feedforward source: rising i_dc = load increasing
    pub freq_hz: f32,
}

pub fn finalize_protection(acc: &Accumulator, gain: f32) -> ProtectionScalars {
    if acc.n == 0 {
        return ProtectionScalars::default();
    }
    let n = acc.n as f64;
    let v_dc = counts_to_pin_volts(acc.sum_v_dc / n, gain) * DC_V_RATIO;
    let i_dc = counts_to_pin_volts(acc.sum_i_dc / n, gain) * DC_I_RATIO;
    let window_s = n / (ADC_SAMPLE_RATE as f64);
    let freq_hz = if window_s > 0.0 {
        (acc.x_ref.crossings as f64 / window_s) as f32
    } else {
        0.0
    };
    ProtectionScalars {
        v_dc,
        i_dc,
        freq_hz,
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// read_frame — update the channel mapping in Ads131Pair::read_frame to:
//
//   pub fn read_frame(&mut self) -> RawFrame {
//       let c1 = self.read_chip(Chip::One); // [ref, v_a, v_b, v_c]
//       let c2 = self.read_chip(Chip::Two); // [v_dc, i_dc, spare, spare]
//       RawFrame {
//           v_ref: c1[0], v_a: c1[1], v_b: c1[2], v_c: c1[3],
//           v_dc:  c2[0], i_dc: c2[1],
//       }
//   }
//
// ═════════════════════════════════════════════════════════════════════════════
