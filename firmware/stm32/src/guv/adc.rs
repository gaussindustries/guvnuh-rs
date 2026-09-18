// (combined driver — part1 + part2 merged into guv/adc.rs)
// ═════════════════════════════════════════════════════════════════════════════
// DUAL ADS131M04 DRIVER — 3-phase power measurement front-end
//
// SYSTEM ROLE (per the architecture split):
//   The STM32 samples the 3-phase AC waveforms + DC bus FAST, and computes the
//   CONTROL-CRITICAL electrical scalars locally: per-phase RMS V/I, real power,
//   power factor, frequency, DC bus V/I. These are CONTROL INPUTS, not just
//   telemetry — a governor's whole job is matching mechanical input to
//   electrical output, so it must KNOW the electrical output in real time, and
//   the fastest/most-dangerous failure modes (load rejection → overspeed, esp.
//   for a steam turbine) are detected FROM these computed values and must be
//   handled LOCALLY with no terminal round-trip.
//
//   The STM32 does NOT do analytical work (harmonics, THD, spectra, trends) —
//   that's the terminal's job, from the shipped scalars. STM32 runs the machine;
//   ESP32 relays; terminal analyzes.
//
// TWO CHIPS, ONE BUS, COHERENT SAMPLING:
//   Two ADS131M04s share SPI1 (SCK/MISO/MOSI) with separate CS lines. Their SYNC
//   pins are tied → both sample SIMULTANEOUSLY (coherent), essential for real
//   power (V×I must be sampled at the same instant per phase). Readout is
//   sequential over the shared bus; sampling is synchronized by SYNC.
//
//   Chip #1: Ch0-2 = phase A/B/C VOLTAGE, Ch3 = DC bus VOLTAGE
//   Chip #2: Ch0-2 = phase A/B/C CURRENT, Ch3 = DC bus CURRENT
//
// CLKIN: external 8.192 MHz oscillator (CTS CB3LV-3C-8M192000) feeds both chips.
// ═════════════════════════════════════════════════════════════════════════════

#![allow(dead_code)]

use libm::sqrt;
use stm32h7xx_hal::{
    gpio::{self, Input, Output, PushPull},
    hal::blocking::spi::Transfer,
};
pub const ADC_HARDWARE_ENABLED: bool = false;
// ─────────────────────────────────────────────────────────────────────────────
// TUNABLE: ADC sample rate — see the big comment. Editable knob trading ADC
// fidelity vs. CPU/ISR load. The DRDY pin fires at this rate.
//
//   Samples per 60 Hz cycle = ADC_SAMPLE_RATE / 60
//     2000 SPS ≈ 33 samples/cycle  → solid RMS/power, lightest CPU
//     4000 SPS ≈ 67 samples/cycle  → good RMS + low-order harmonics (DEFAULT)
//     8000 SPS ≈ 133 samples/cycle → harmonics headroom, heavier ISR
//
// The ADS131M04's own output data rate (ODR) is set by OSR (oversampling ratio)
// against the 8.192 MHz CLKIN: ODR = CLKIN / (2 * OSR * 2). Pick OSR to land near
// ADC_SAMPLE_RATE (see osr_for_rate below). Tune against measured WCET — the
// control-loop deadline instrumentation shows the ISR + finalize cost. At 64 MHz
// sysclk, if the loop tightens, drop to 2000; if you need on-device harmonics,
// raise to 8000 (and confirm the ISR still fits the budget).
// ─────────────────────────────────────────────────────────────────────────────
pub const ADC_SAMPLE_RATE: u32 = 4000; // SPS per channel

pub const AC_LINE_HZ: f32 = 60.0;
/// Samples accumulated per 60 Hz cycle at the current rate.
pub const SAMPLES_PER_CYCLE: u32 = ADC_SAMPLE_RATE / (AC_LINE_HZ as u32);

// ── ADS131M04 register map (subset we use) ──
const REG_ID: u8 = 0x00;
const REG_STATUS: u8 = 0x01;
const REG_MODE: u8 = 0x02;
const REG_CLOCK: u8 = 0x03;
const REG_GAIN1: u8 = 0x04; // PGA gain for ch0-3
const REG_CFG: u8 = 0x06;
// per-channel config base (CHx_CFG at 0x09 + 5*x)
const REG_CH0_CFG: u8 = 0x09;

// SPI commands
const CMD_NULL: u16 = 0x0000;
const CMD_RESET: u16 = 0x0011;
const CMD_STANDBY: u16 = 0x0022;
const CMD_WAKEUP: u16 = 0x0033;
// RREG / WREG are 011a aaaa accc cccc — built in helpers below.

/// Which physical quantity a channel carries, for scaling raw counts → SI units.
#[derive(Clone, Copy)]
pub enum ChannelKind {
    PhaseVoltage, // via PT / iso-amp
    PhaseCurrent, // via CT + burden
    DcVoltage,    // via divider
    DcCurrent,    // via shunt + INA240
}

// ═════════════════════════════════════════════════════════════════════════════
// PIN ASSIGNMENTS (control lines) — placed on the free PD bank + PA5/6/7 for SPI1.
//
//   SPI1:  SCK = PA5, MISO = PA6, MOSI = PA7   (AF5)
//   CS1  = PD8   (chip #1 chip-select, active low)
//   CS2  = PD9   (chip #2 chip-select, active low)
//   DRDY = PD10  (data-ready, shared — both chips DRDY tied; falling edge = new frame)
//   SYNC = PD11  (sync/reset, tied to both chips — drives coherent sampling)
//   RST  = PD12  (hardware reset, tied to both chips, active low)
//
//   (CLKIN is the external 8.192 MHz oscillator, wired to both chips — not an MCU pin.)
// ═════════════════════════════════════════════════════════════════════════════

pub type Cs1 = gpio::Pin<'D', 3, Output<PushPull>>;
pub type Cs2 = gpio::Pin<'D', 4, Output<PushPull>>;
pub type Drdy = gpio::Pin<'D', 5, Input>;
pub type Sync = gpio::Pin<'D', 6, Output<PushPull>>;
pub type Rst = gpio::Pin<'D', 7, Output<PushPull>>;

/// The SPI1 peripheral, configured for the ADS131M04 (CPOL=0, CPHA=1, MSB first).
pub type AdcSpi = stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>;

// ═════════════════════════════════════════════════════════════════════════════
// SCALING — raw 24-bit counts → SI units. FILL THESE IN once the front-end
// (PT/CT/divider/shunt+INA240 values) is finalized. Each is: full-scale volts at
// the ADC pin × the external divider/transformer/shunt ratio ÷ ADC counts.
//
// ADS131M04: ±1.2 V differential full-scale at gain=1 → 2^23 counts = 1.2 V.
// So volts_at_pin = raw_counts * (1.2 / 8_388_608.0) / gain.
// Then multiply by the front-end ratio to get the real-world quantity.
// ═════════════════════════════════════════════════════════════════════════════
const ADC_FS_VOLTS: f32 = 1.2;
const ADC_COUNTS: f32 = 8_388_608.0; // 2^23

// PLACEHOLDER ratios — replace with your measured/designed front-end values:
const PHASE_V_RATIO: f32 = 1.0; // (PT ratio × iso-amp gain) → real phase volts per pin-volt
const PHASE_I_RATIO: f32 = 1.0; // (CT ratio / burden) → real amps per pin-volt
const DC_V_RATIO: f32 = 1.0; //    (divider ratio) → real DC volts per pin-volt
const DC_I_RATIO: f32 = 1.0; //    (1 / (shunt × INA240 gain)) → real amps per pin-volt

#[inline]
fn counts_to_pin_volts(raw: i32, gain: f32) -> f32 {
    (raw as f32) * (ADC_FS_VOLTS / ADC_COUNTS) / gain
}

// (continued in part 2 — driver struct, config, ISR, accumulation, finalize)
// ═════════════════════════════════════════════════════════════════════════════
// DUAL ADS131M04 DRIVER — Part 2: driver, accumulation, finalize
// (continues adc_ads131m04_part1.rs — same module)
// ═════════════════════════════════════════════════════════════════════════════

// ── RREG/WREG command builders ──
// WREG: 011 aaaaa a ccccccc  → 0x6000 | (addr<<7) | (count-1)
// RREG: 101 aaaaa a ccccccc  → 0xA000 | (addr<<7) | (count-1)
#[inline]
fn wreg_cmd(addr: u8, count: u8) -> u16 {
    0x6000 | ((addr as u16) << 7) | ((count as u16) - 1)
}
#[inline]
fn rreg_cmd(addr: u8, count: u8) -> u16 {
    0xA000 | ((addr as u16) << 7) | ((count as u16) - 1)
}

/// One coherent sample set: all 8 channels, both chips, same instant.
/// Chip1 = voltages (A,B,C,DC), Chip2 = currents (A,B,C,DC).
#[derive(Clone, Copy, Default)]
pub struct RawFrame {
    pub v_a: i32,
    pub v_b: i32,
    pub v_c: i32,
    pub v_dc: i32,
    pub i_a: i32,
    pub i_b: i32,
    pub i_c: i32,
    pub i_dc: i32,
}

/// Running accumulators between finalize calls. The ISR adds each sample's
/// squared value (for RMS) and V*I product (for real power). No divide/sqrt in
/// the ISR — that happens in finalize. Uses i64/f64 accumulators to avoid
/// overflow/precision loss over thousands of samples.
#[derive(Clone, Copy, Default)]
pub struct Accumulator {
    // sum of squares, per channel, for RMS
    pub sq_v_a: f64,
    pub sq_v_b: f64,
    pub sq_v_c: f64,
    pub sq_i_a: f64,
    pub sq_i_b: f64,
    pub sq_i_c: f64,
    // sum of V*I products, per phase, for real power
    pub p_a: f64,
    pub p_b: f64,
    pub p_c: f64,
    // DC bus: simple running sum (DC → mean, not RMS)
    pub sum_v_dc: f64,
    pub sum_i_dc: f64,
    // count of samples accumulated
    pub n: u32,
    // ── frequency detection: zero-crossing tracking on phase A voltage ──
    pub last_v_a_sign: i8, // sign of previous v_a sample (+1/-1)
    pub crossings: u32,    // rising zero-crossings counted this window
}

impl Accumulator {
    #[inline]
    pub fn reset(&mut self) {
        *self = Accumulator {
            last_v_a_sign: self.last_v_a_sign, // carry sign across windows for continuity
            ..Default::default()
        };
    }

    /// Add one coherent sample (called from the DRDY ISR — keep LEAN).
    #[inline]
    pub fn add(&mut self, f: &RawFrame) {
        let va = f.v_a as f64;
        let vb = f.v_b as f64;
        let vc = f.v_c as f64;
        let ia = f.i_a as f64;
        let ib = f.i_b as f64;
        let ic = f.i_c as f64;

        self.sq_v_a += va * va;
        self.sq_v_b += vb * vb;
        self.sq_v_c += vc * vc;
        self.sq_i_a += ia * ia;
        self.sq_i_b += ib * ib;
        self.sq_i_c += ic * ic;

        self.p_a += va * ia;
        self.p_b += vb * ib;
        self.p_c += vc * ic;

        self.sum_v_dc += f.v_dc as f64;
        self.sum_i_dc += f.i_dc as f64;

        // rising zero-crossing on phase A voltage → frequency
        let sign: i8 = if va >= 0.0 { 1 } else { -1 };
        if self.last_v_a_sign < 0 && sign > 0 {
            self.crossings += 1;
        }
        self.last_v_a_sign = sign;

        self.n = self.n.wrapping_add(1);
    }
}

/// Finalized electrical scalars — the CONTROL-CRITICAL measurements the STM32
/// computes locally and both (a) acts on for control/protection and (b) ships
/// via telemetry. All SI units.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct Measurements {
    pub v_a_rms: f32,
    pub v_b_rms: f32,
    pub v_c_rms: f32,
    pub i_a_rms: f32,
    pub i_b_rms: f32,
    pub i_c_rms: f32,
    pub p_a: f32, // real power per phase (W)
    pub p_b: f32,
    pub p_c: f32,
    pub p_total: f32, // total real power (W) — the load-rejection sentinel
    pub pf: f32,      // total power factor
    pub v_dc: f32,    // DC bus voltage (V)
    pub i_dc: f32,    // DC bus current (A)
    pub freq_hz: f32, // output frequency (Hz)
}

/// The driver: owns SPI + control pins + the shared accumulator.
pub struct Ads131Pair {
    spi: AdcSpi,
    cs1: Cs1,
    cs2: Cs2,
    sync: Sync,
    rst: Rst,
    gain: f32, // PGA gain applied to all channels (config below)
}

impl Ads131Pair {
    /// Construct + configure both chips. Call once in init (after SPI + pins built
    /// in 0_boot). Does hardware reset, sets mode/clock/gain, enables channels.
    pub fn new(spi: AdcSpi, cs1: Cs1, cs2: Cs2, mut sync: Sync, mut rst: Rst) -> Self {
        rst.set_low();
        cortex_m::asm::delay(64_000);
        rst.set_high();
        cortex_m::asm::delay(640_000);
        sync.set_high();

        let mut me = Self {
            spi,
            cs1,
            cs2,
            sync,
            rst,
            gain: 1.0,
        };

        if ADC_HARDWARE_ENABLED {
            me.configure(); // SPI register writes — only when the chips can respond
        } else {
            defmt::warn!("ADS131M04 DISABLED (ADC_HARDWARE_ENABLED=false) — SPI config skipped");
        }
        me
    }

    /// Configure both chips identically: gain, OSR (→ sample rate), channel enable.
    fn configure(&mut self) {
        // OSR chosen so ODR ≈ ADC_SAMPLE_RATE. See osr_for_rate.
        let osr_bits = osr_bits_for_rate(ADC_SAMPLE_RATE);
        // MODE / CLOCK / GAIN register values — see datasheet §. These are the
        // key ones; adjust bit-fields to your exact needs.
        // CLOCK reg: set OSR field + enable all 4 channels.
        let clock_val: u16 = 0x000E | (osr_bits << 2) /* OSR */ ;
        // GAIN1: gain=1 for all channels (bits per channel). 0 = gain 1.
        let gain_val: u16 = 0x0000;

        for chip in [Chip::One, Chip::Two] {
            self.write_reg(chip, REG_CLOCK, clock_val);
            self.write_reg(chip, REG_GAIN1, gain_val);
            // MODE reg: default (24-bit words, etc.) — write if you need to change.
        }
        self.gain = 1.0;
        defmt::info!(
            "ADS131M04 pair configured: OSR bits={=u16}, rate≈{=u32} SPS, gain=1",
            osr_bits,
            ADC_SAMPLE_RATE
        );
    }

    /// Read one coherent frame from BOTH chips. Called from the DRDY ISR.
    /// The ADS131M04 frame is: STATUS word + 4 channel words (24-bit each) + CRC.
    /// We read chip1 (voltages) then chip2 (currents) over the shared bus.
    pub fn read_frame(&mut self) -> RawFrame {
        let v = self.read_chip(Chip::One); // [ch0..ch3] = v_a,v_b,v_c,v_dc
        let i = self.read_chip(Chip::Two); // [ch0..ch3] = i_a,i_b,i_c,i_dc
        RawFrame {
            v_a: v[0],
            v_b: v[1],
            v_c: v[2],
            v_dc: v[3],
            i_a: i[0],
            i_b: i[1],
            i_c: i[2],
            i_dc: i[3],
        }
    }

    /// Read the 4 channel words from one chip. Frame = status + 4×24bit + crc.
    /// At 24-bit word size that's 6 words of 3 bytes = 18 bytes.
    fn read_chip(&mut self, chip: Chip) -> [i32; 4] {
        self.select(chip, true);
        // send NULL command, clock out the full frame.
        let mut buf = [0u8; 18]; // status(3) + 4ch(3 each = 12) + crc(3)
                                 // TX all-zeros (NULL cmd), RX the frame.
        let _ = self.spi.transfer(&mut buf);
        self.select(chip, false);

        // words: [0..3]=status, [3..6]=ch0, [6..9]=ch1, [9..12]=ch2, [12..15]=ch3, [15..18]=crc
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
            0, // pad to 24-bit word
            (val >> 8) as u8,
            (val & 0xFF) as u8,
            0,
        ];
        let _ = self.spi.transfer(&mut tx);
        self.select(chip, false);
    }

    #[inline]
    fn select(&mut self, chip: Chip, active: bool) {
        // CS active low
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

#[derive(Clone, Copy)]
enum Chip {
    One,
    Two,
}

/// Big-endian 24-bit two's-complement bytes → i32 (sign-extended).
#[inline]
fn be24_to_i32(b: &[u8]) -> i32 {
    let raw = ((b[0] as u32) << 16) | ((b[1] as u32) << 8) | (b[2] as u32);
    // sign-extend 24→32
    if raw & 0x0080_0000 != 0 {
        (raw | 0xFF00_0000) as i32
    } else {
        raw as i32
    }
}

/// Map desired sample rate → OSR register bits (ODR = CLKIN / (2 * OSR)). With
/// 8.192 MHz CLKIN: OSR=1024 → 4000 SPS, OSR=2048 → 2000, OSR=512 → 8000.
fn osr_bits_for_rate(rate: u32) -> u16 {
    match rate {
        r if r >= 8000 => 0b010, // OSR 512  → 8000 SPS
        r if r >= 4000 => 0b011, // OSR 1024 → 4000 SPS (default)
        _ => 0b100,              // OSR 2048 → 2000 SPS
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// FINALIZE — turn accumulated sums into SI scalars. Two entry points at two
// rates, per the protection-vs-control split.
// ═════════════════════════════════════════════════════════════════════════════

/// FULL finalize (called at the 100 Hz control tick): all scalars for governing
/// + telemetry. Consumes the accumulator (caller resets after).
pub fn finalize_full(acc: &Accumulator, gain: f32) -> Measurements {
    if acc.n == 0 {
        return Measurements::default();
    }
    let n = acc.n as f64;
    let scale_v = |sq: f64, ratio: f32| -> f32 {
        // RMS in pin-volts, then × front-end ratio
        let rms_counts = sqrt(sq / n);
        counts_to_pin_volts(rms_counts as i32, gain) * ratio
    };
    // NOTE: converting the sqrt(mean-square) of counts via counts_to_pin_volts is
    // approximate (it treats the RMS count as a raw count); for exactness, scale
    // per-sample. Kept compact here — refine if you need lab-grade accuracy.

    let v_a_rms = scale_v(acc.sq_v_a, PHASE_V_RATIO);
    let v_b_rms = scale_v(acc.sq_v_b, PHASE_V_RATIO);
    let v_c_rms = scale_v(acc.sq_v_c, PHASE_V_RATIO);
    let i_a_rms = scale_v(acc.sq_i_a, PHASE_I_RATIO);
    let i_b_rms = scale_v(acc.sq_i_b, PHASE_I_RATIO);
    let i_c_rms = scale_v(acc.sq_i_c, PHASE_I_RATIO);

    // real power per phase: mean of (V*I) products, scaled by both ratios
    let vscale = (ADC_FS_VOLTS / ADC_COUNTS) / gain;
    let p_scale = vscale * vscale * (PHASE_V_RATIO * PHASE_I_RATIO);
    let p_a = ((acc.p_a / n) * p_scale as f64) as f32;
    let p_b = ((acc.p_b / n) * p_scale as f64) as f32;
    let p_c = ((acc.p_c / n) * p_scale as f64) as f32;
    let p_total = p_a + p_b + p_c;

    // apparent power for PF
    let s_total = v_a_rms * i_a_rms + v_b_rms * i_b_rms + v_c_rms * i_c_rms;
    let pf = if s_total.abs() > f32::EPSILON {
        (p_total / s_total).clamp(-1.0, 1.0)
    } else {
        0.0
    };

    // DC bus: mean of counts → pin volts → ratio
    let v_dc = counts_to_pin_volts((acc.sum_v_dc / n) as i32, gain) * DC_V_RATIO;
    let i_dc = counts_to_pin_volts((acc.sum_i_dc / n) as i32, gain) * DC_I_RATIO;

    // frequency: rising crossings over the sample window → Hz
    // window duration = n / ADC_SAMPLE_RATE seconds; freq = crossings / duration
    let window_s = n / (ADC_SAMPLE_RATE as f64);
    let freq_hz = if window_s > 0.0 {
        (acc.crossings as f64 / window_s) as f32
    } else {
        0.0
    };

    Measurements {
        v_a_rms,
        v_b_rms,
        v_c_rms,
        i_a_rms,
        i_b_rms,
        i_c_rms,
        p_a,
        p_b,
        p_c,
        p_total,
        pf,
        v_dc,
        i_dc,
        freq_hz,
    }
}

/// FAST protection subset (called at 500 Hz alongside the overspeed task): only
/// the safety-critical scalars needed to catch fast failure modes LOCALLY —
/// total real power (load-rejection sentinel), DC bus voltage (overvoltage), DC
/// current (overcurrent), frequency (overspeed corroboration). Cheaper than the
/// full finalize; does NOT reset the accumulator (the 100Hz full finalize owns
/// the reset), so this reads the running partial sums.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct ProtectionScalars {
    pub p_total: f32,
    pub v_dc: f32,
    pub i_dc: f32,
    pub freq_hz: f32,
}

pub fn finalize_protection(acc: &Accumulator, gain: f32) -> ProtectionScalars {
    if acc.n == 0 {
        return ProtectionScalars::default();
    }
    let n = acc.n as f64;
    let vscale = (ADC_FS_VOLTS / ADC_COUNTS) / gain;
    let p_scale = vscale * vscale * (PHASE_V_RATIO * PHASE_I_RATIO);
    let p_total = (((acc.p_a + acc.p_b + acc.p_c) / n) * p_scale as f64) as f32;
    let v_dc = counts_to_pin_volts((acc.sum_v_dc / n) as i32, gain) * DC_V_RATIO;
    let i_dc = counts_to_pin_volts((acc.sum_i_dc / n) as i32, gain) * DC_I_RATIO;
    let window_s = n / (ADC_SAMPLE_RATE as f64);
    let freq_hz = if window_s > 0.0 {
        (acc.crossings as f64 / window_s) as f32
    } else {
        0.0
    };
    ProtectionScalars {
        p_total,
        v_dc,
        i_dc,
        freq_hz,
    }
}
