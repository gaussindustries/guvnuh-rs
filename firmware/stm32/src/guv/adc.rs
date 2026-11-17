// (combined adc.rs — SPI driver + phase-alignment measurement layer)
// ═════════════════════════════════════════════════════════════════════════════
// DUAL ADS131M04 DRIVER + PHASE-ALIGNMENT MEASUREMENT
//
// Two ADS131M04 24-bit delta-sigma ADCs share SPI1 with separate CS lines and
// one tied SYNC/RESET line (coherent sampling). Chip1 carries the AC voltages
// used for phase alignment (reference + 3 motor phases); Chip2 carries the
// rectifier DC bus. The external 8.192 MHz oscillator (CTS CB3LV) must feed
// both CLKIN pins or the ADCs never convert (no DRDY, no data).
//
// Bring-up is fail-safe: every init-time SPI exchange is bounded, each chip's
// ID register is checked, and register writes are read back. If anything fails
// the driver logs why and goes OFFLINE (the DRDY ISR then skips reads) instead
// of hanging the boot.
// ═════════════════════════════════════════════════════════════════════════════

#![allow(dead_code)]

use stm32h7xx_hal::gpio::{self, Input, Output, PushPull};
use stm32h7xx_hal::hal::spi::FullDuplex;
use stm32h7xx_hal::nb;
// Bring the blocking SPI Transfer trait into scope for spi.transfer(&mut buf).
// (cortex_m re-exports the embedded-hal 0.2 blocking traits under this alias.)
use cortex_m::prelude::_embedded_hal_blocking_spi_Transfer as _;

// ─────────────────────────────────────────────────────────────────────────────
// Master enable. When false, Ads131Pair::new() skips all SPI traffic and the
// DRDY interrupt is left un-armed (see 0_boot.rs), so the board boots clean
// with the ADC dormant and telemetry electrical fields read 0.
// ─────────────────────────────────────────────────────────────────────────────
pub const ADC_HARDWARE_ENABLED: bool = true;

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

/// ID register reads 0x24xx on an ADS131M04 (upper byte = 0x24: 4 channels).
const ID_EXPECTED_HI: u16 = 0x24;

// ── SPI framing ──
// Default word length is 24 bits (MODE.WLENGTH = 01). Every frame is SIX words:
// command/response + 4 channels + CRC. The ADS131M04 only accepts SHORT frames
// while its ADCs are disabled (datasheet §8.5.1.11) — they're enabled by
// default, so EVERY frame here is a full 18 bytes.
const WORD: usize = 3;
const FRAME: usize = 6 * WORD;

/// Spin budget for one SPI byte at init. A byte at 8 MHz SCLK takes ~1 µs;
/// 100k polls is tens of ms — only an MCU-side stall can exhaust it.
const SPI_SPIN_LIMIT: u32 = 100_000;

/// SPI1 runs from a 64 MHz kernel clock (PLL1_Q); CFG1.MBR[2:0] divides it:
/// 010=/8 → 8 MHz, 011=/16 → 4 MHz, 100=/32 → 2 MHz, 101=/64 → 1 MHz.
/// Bring-up sweeps slowest → fastest to find what the wiring supports.
const SPI_SPEEDS: [(u8, u32); 4] = [
    (0b101, 1_000),
    (0b100, 2_000),
    (0b011, 4_000),
    (0b010, 8_000),
];

/// Slowest SCLK the DRDY ISR can live with: two 18-byte frames (288 bits) per
/// sample must take <= half the sample period. 4 kSPS → ≥ 2304 kHz.
const MIN_RUN_KHZ: u32 = (2 * FRAME as u32 * 8) * ADC_SAMPLE_RATE * 2 / 1000;

/// ID reads per chip per speed; all must be valid for the speed to pass.
const ID_TRIES: u8 = 3;

// ─────────────────────────────────────────────────────────────────────────────
// PIN ASSIGNMENTS (match 0_boot.rs):
//   SPI1: SCLK=PA5 (D13), MISO=PA6 (D12, ← both DOUT), MOSI=PB5 (D11, → both DIN)
//         NOT PA7: on the Nucleo-H753ZI, PA7 belongs to the Ethernet PHY.
//   CS1=PD3, CS2=PD4, DRDY=PD5 (EXTI9_5), SYNC/RESET=PD6 (tied to both chips)
//   PD7: NOT CONNECTED — the ADS131M04 has no separate RESET pin; SYNC/RESET
//        (PD6) does both. PD7 is parked low and never toggled.
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

/// Map desired sample rate → OSR[2:0]. With 8.192 MHz CLKIN (HR mode):
/// OSR=512 → 8000 SPS, OSR=1024 → 4000, OSR=2048 → 2000.
fn osr_bits_for_rate(rate: u32) -> u16 {
    match rate {
        r if r >= 8000 => 0b010, // OSR 512  → 8000 SPS
        r if r >= 4000 => 0b011, // OSR 1024 → 4000 SPS (default)
        _ => 0b100,              // OSR 2048 → 2000 SPS
    }
}

/// CLOCK register (0x03, reset 0x0F0E):
///   [11:8] CH3_EN..CH0_EN   [5] TBM   [4:2] OSR[2:0]   [1:0] PWR (10 = high-res)
/// All four channels must stay ENABLED or the ADC stops converting (no DRDY).
fn clock_reg_value(rate: u32) -> u16 {
    0x0F00 | (osr_bits_for_rate(rate) << 2) | 0b10
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

/// A frame with one command word (and optionally one data word); rest zero.
fn frame_with(cmd: u16, data: Option<u16>) -> [u8; FRAME] {
    let mut f = [0u8; FRAME];
    f[0] = (cmd >> 8) as u8;
    f[1] = cmd as u8;
    if let Some(d) = data {
        f[WORD] = (d >> 8) as u8;
        f[WORD + 1] = d as u8;
    }
    f
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Chip {
    One,
    Two,
}

impl Chip {
    fn n(self) -> u8 {
        match self {
            Chip::One => 1,
            Chip::Two => 2,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AdcError {
    /// The MCU's SPI peripheral stopped making progress (SR logged).
    SpiStall,
    /// The SPI peripheral reported an error flag (logged).
    SpiFault,
    /// A chip did not identify as an ADS131M04.
    BadId,
    /// A register read back different from what was written.
    WriteMismatch,
}

fn spi_err_name(e: &stm32h7xx_hal::spi::Error) -> &'static str {
    use stm32h7xx_hal::spi::Error as E;
    match e {
        E::Overrun => "overrun",
        E::ModeFault => "mode fault",
        E::Crc => "crc",
        E::DuplexFailed => "duplex failed (TX finished but nothing received)",
        _ => "other",
    }
}

/// Dump SPI1's status + control registers when a transfer stalls.
fn report_spi_stall(what: &'static str, dir: &'static str, byte: usize) {
    // SAFETY: read-only register access for diagnostics.
    let (sr, cr1) = unsafe {
        let spi = &*stm32h7xx_hal::pac::SPI1::ptr();
        (spi.sr.read().bits(), spi.cr1.read().bits())
    };
    defmt::error!(
        "ADC SPI STALL during {=str} ({=str}, byte {=usize}): SR=0x{=u32:x} CR1=0x{=u32:x} [SPE={=bool} CSTART={=bool} TXP={=bool} RXP={=bool} TXC={=bool} OVR={=bool} MODF={=bool} SUSP={=bool}]",
        what,
        dir,
        byte,
        sr,
        cr1,
        cr1 & 1 != 0,
        cr1 & (1 << 9) != 0,
        sr & (1 << 1) != 0,
        sr & 1 != 0,
        sr & (1 << 12) != 0,
        sr & (1 << 6) != 0,
        sr & (1 << 9) != 0,
        sr & (1 << 11) != 0
    );
}

/// GPIO output-speed (slew) settings to sweep on SCLK (PA5) and MOSI (PB5).
/// OSPEEDR: 00 low, 01 medium, 10 high, 11 very high. If the garbage changes
/// with edge rate, the fault is on the clock line (ringing / slow-edge noise).
const EDGE_SPEEDS: [(u8, &str); 4] = [
    (0b00, "low"),
    (0b01, "medium"),
    (0b10, "high"),
    (0b11, "very-high"),
];

/// Set the output slew rate of SCLK (PA5) and MOSI (PB5).
fn set_edge_speed(speed: u8) {
    // SAFETY: init-time; only the 2-bit OSPEEDR field of pin 5 on ports A and B.
    unsafe {
        let mask = !(0b11u32 << 10);
        let val = (speed as u32 & 0b11) << 10;
        (*stm32h7xx_hal::pac::GPIOA::ptr())
            .ospeedr
            .modify(|r, w| w.bits((r.bits() & mask) | val));
        (*stm32h7xx_hal::pac::GPIOB::ptr())
            .ospeedr
            .modify(|r, w| w.bits((r.bits() & mask) | val));
    }
}

/// Count DRDY (PD5) falling edges for ~10 ms using the EXTI line-5 pending
/// latch (set by 0_boot's falling-edge config; interrupts are still masked
/// during init, so nothing else consumes it). A converting ADS131M04 at 4 kSPS
/// gives ~40. Zero means the chips are not converting.
fn count_drdy_edges() -> u32 {
    // SAFETY: init-time, read/clear of EXTI line-5 pending bit only.
    unsafe {
        let exti = &*stm32h7xx_hal::pac::EXTI::ptr();
        exti.cpupr1.write(|w| w.pr5().set_bit()); // clear stale
        let mut n = 0;
        for _ in 0..10_000 {
            if exti.cpupr1.read().pr5().bit_is_set() {
                n += 1;
                exti.cpupr1.write(|w| w.pr5().set_bit());
            }
            cortex_m::asm::delay(64); // ~1 µs @ 64 MHz → loop ≈ 10+ ms
        }
        n
    }
}

/// The driver: owns SPI + control pins.
pub struct Ads131Pair {
    spi: AdcSpi,
    cs1: Cs1,
    cs2: Cs2,
    sync: Sync,
    rst: Rst,
    gain: f32,
    /// True only after both chips identified and accepted their configuration.
    /// The DRDY ISR reads frames only while online.
    pub online: bool,
}

impl Ads131Pair {
    /// Construct and (when enabled) bring up both chips. Never hangs: a failed
    /// step logs the reason and leaves the pair OFFLINE so the rest of the
    /// firmware still boots.
    pub fn new(spi: AdcSpi, cs1: Cs1, cs2: Cs2, sync: Sync, mut rst: Rst) -> Self {
        rst.set_low(); // PD7 unused — park it

        let mut me = Self {
            spi,
            cs1,
            cs2,
            sync,
            rst,
            gain: 1.0,
            online: false,
        };
        me.cs1.set_high();
        me.cs2.set_high();

        if !ADC_HARDWARE_ENABLED {
            defmt::warn!("ADS131M04 DISABLED (ADC_HARDWARE_ENABLED=false) — SPI skipped");
            return me;
        }

        // ── 1. Reset both chips via the shared SYNC/RESET pin (PD6) ──
        // Low ≥ tw(RSL) = 2048 tCLKIN (250 µs @ 8.192 MHz) resets; then wait
        // ≥ tREGACQ (5 µs) before talking. 2 ms each side is generous.
        defmt::info!("ADC 1/4: reset pulse on SYNC/RESET (PD6)");
        me.sync.set_low();
        cortex_m::asm::delay(128_000); // ~2 ms @ 64 MHz
        me.sync.set_high();
        cortex_m::asm::delay(128_000);

        // ── 2. Identify each chip (proves SCLK, MOSI, MISO, CS, power) ──
        // ── 1b. Are the chips converting? (needs CLKIN + released SYNC/RESET) ──
        let drdy = count_drdy_edges();
        if drdy == 0 {
            defmt::error!("DRDY: 0 edges in ~10 ms — ADCs not converting. The 8.192 MHz oscillator is not reaching CLKIN (pin 17) (or the oscillator is unpowered / its enable pin is low), or SYNC/RESET (pin 11) is stuck low. (SPI register access does not need CLKIN, so this is separate from any ID problem.)");
        } else {
            defmt::info!("DRDY: {=u32} edges in ~10 ms (expect ~40 at 4 kSPS) — the chip(s) on the DRDY line are converting, so CLKIN is running", drdy);
        }

        // ── 2. Identify both chips, sweeping SCLK to find what the wiring supports ──
        defmt::info!(
            "ADC 2/4: ID sweep (need >= {=u32} kHz for {=u32} SPS)",
            MIN_RUN_KHZ,
            ADC_SAMPLE_RATE
        );
        // best = (mbr, khz, edge); highest passing kHz wins, slowest edge on ties
        let mut best: Option<(u8, u32, u8)> = None;
        for (edge, edge_name) in EDGE_SPEEDS {
            set_edge_speed(edge);
            for (mbr, khz) in SPI_SPEEDS {
                Self::set_spi_mbr(mbr);
                let (pass1, id1) = me.id_stable(Chip::One);
                let (pass2, id2) = me.id_stable(Chip::Two);
                let pass = pass1 && pass2;
                defmt::info!(
                    "  edge {=str} / SPI {=u32} kHz: ADS#1 0x{=u16:x}  ADS#2 0x{=u16:x}  {=str}",
                    edge_name,
                    khz,
                    id1,
                    id2,
                    if pass { "PASS" } else { "FAIL" }
                );
                if pass && best.map_or(true, |(_, b_khz, _)| khz > b_khz) {
                    best = Some((mbr, khz, edge));
                }
            }
        }

        match best.map(|(mbr, khz, edge)| {
            set_edge_speed(edge);
            (mbr, khz)
        }) {
            None => {
                // No speed works → not a speed problem. Diagnose at the slowest
                // speed and dump the raw frames so they can be decoded.
                set_edge_speed(EDGE_SPEEDS[0].0);
                Self::set_spi_mbr(SPI_SPEEDS[0].0);
                me.check_id(Chip::One);
                me.check_id(Chip::Two);
                me.dump_id_frames(Chip::One);
                me.bus_probe();
                defmt::error!("ADC OFFLINE — no SPI speed gives a valid ID (see above)");
                return me;
            }
            Some((_, khz)) if khz < MIN_RUN_KHZ => {
                defmt::error!(
                    "ADC OFFLINE — wiring is only reliable up to {=u32} kHz but {=u32} SPS needs >= {=u32} kHz. Shorten SCLK/MISO/MOSI, run a GND wire alongside SCLK, add ~33 ohm in series at the PA5 end",
                    khz,
                    ADC_SAMPLE_RATE,
                    MIN_RUN_KHZ
                );
                return me;
            }
            Some((mbr, khz)) => {
                Self::set_spi_mbr(mbr);
                defmt::info!("ADC: running SPI at {=u32} kHz", khz);
            }
        }

        // ── 2b. Did the SYNC/RESET pulse actually reset the chips? ──
        // Nothing has written CLOCK yet this boot, so it must read its reset
        // value 0x0F0E. Anything else means an old value survived the pulse.
        for chip in [Chip::One, Chip::Two] {
            match me.read_reg(chip, REG_CLOCK) {
                Ok(0x0F0E) => defmt::info!("ADS#{=u8} CLOCK=0x0f0e after reset — reset works", chip.n()),
                Ok(v) => defmt::warn!(
                    "ADS#{=u8} CLOCK=0x{=u16:x} after reset (want 0x0f0e) — the reset pulse is NOT reaching this chip: check PD6 to SYNC/RESET (pin 11). Configure will overwrite it, but a real reset matters",
                    chip.n(),
                    v
                ),
                Err(_) => {}
            }
        }

        // ── 3. Configure + verify ──
        defmt::info!("ADC 3/4: configuring");
        if let Err(e) = me.configure() {
            defmt::error!("ADC OFFLINE — configure failed: {}", e);
            return me;
        }

        // ── 4. Re-sync both chips so their conversions start together ──
        // A short SYNC pulse (> 1 tCLKIN, < tw(RSL)) realigns the filters.
        me.sync.set_low();
        cortex_m::asm::delay(640); // ~10 µs: well above 1 tCLKIN, far below 250 µs
        me.sync.set_high();

        me.online = true;
        defmt::info!(
            "ADC 4/4: ONLINE — {=u32} SPS/ch, gain 1, both chips synced",
            ADC_SAMPLE_RATE
        );
        me
    }

    /// Bounded full-duplex exchange, byte by byte. Init-time only (the DRDY ISR
    /// uses the faster FIFO-pipelined HAL transfer once the bus is proven).
    fn xfer(&mut self, buf: &mut [u8], what: &'static str) -> Result<(), AdcError> {
        for i in 0..buf.len() {
            let mut spins: u32 = 0;
            loop {
                match self.spi.send(buf[i]) {
                    Ok(()) => break,
                    Err(nb::Error::WouldBlock) => {
                        spins += 1;
                        if spins >= SPI_SPIN_LIMIT {
                            report_spi_stall(what, "send", i);
                            return Err(AdcError::SpiStall);
                        }
                    }
                    Err(nb::Error::Other(e)) => {
                        defmt::error!(
                            "ADC SPI error during {=str} send: {=str}",
                            what,
                            spi_err_name(&e)
                        );
                        return Err(AdcError::SpiFault);
                    }
                }
            }
            spins = 0;
            loop {
                match self.spi.read() {
                    Ok(v) => {
                        buf[i] = v;
                        break;
                    }
                    Err(nb::Error::WouldBlock) => {
                        spins += 1;
                        if spins >= SPI_SPIN_LIMIT {
                            report_spi_stall(what, "read", i);
                            return Err(AdcError::SpiStall);
                        }
                    }
                    Err(nb::Error::Other(e)) => {
                        defmt::error!(
                            "ADC SPI error during {=str} read: {=str}",
                            what,
                            spi_err_name(&e)
                        );
                        return Err(AdcError::SpiFault);
                    }
                }
            }
        }
        Ok(())
    }

    /// One full 18-byte frame with CS asserted around it.
    fn exchange(
        &mut self,
        chip: Chip,
        frame: [u8; FRAME],
        what: &'static str,
    ) -> Result<[u8; FRAME], AdcError> {
        let mut buf = frame;
        self.select(chip, true);
        let r = self.xfer(&mut buf, what);
        self.select(chip, false);
        r.map(|_| buf)
    }

    /// Read one register: RREG in this frame, its value comes back as the
    /// response word of the NEXT frame (we clock a NULL to collect it).
    fn read_reg(&mut self, chip: Chip, addr: u8) -> Result<u16, AdcError> {
        self.exchange(chip, frame_with(rreg_cmd(addr, 1), None), "RREG")?;
        let resp = self.exchange(chip, frame_with(CMD_NULL, None), "RREG response")?;
        Ok(((resp[0] as u16) << 8) | resp[1] as u16)
    }

    fn write_reg(&mut self, chip: Chip, addr: u8, val: u16) -> Result<(), AdcError> {
        self.exchange(chip, frame_with(wreg_cmd(addr, 1), Some(val)), "WREG")?;
        Ok(())
    }

    /// Change SCLK by rewriting CFG1.MBR. The SPI must be disabled while MBR
    /// changes; nothing is mid-transfer here (every init exchange completes and
    /// both CS lines are high), and CR1 is restored exactly as the HAL sets it.
    fn set_spi_mbr(mbr: u8) {
        // SAFETY: init-time only, single owner of SPI1, CS lines idle high.
        unsafe {
            let spi = &*stm32h7xx_hal::pac::SPI1::ptr();
            spi.cr1
                .write(|w| w.ssi().slave_not_selected().spe().disabled());
            spi.cfg1.modify(|_, w| w.mbr().bits(mbr));
            spi.cr1
                .write(|w| w.ssi().slave_not_selected().spe().enabled());
        }
    }

    /// Read the ID ID_TRIES times; pass only if every read is a valid 0x24xx.
    /// Returns (pass, last value read) — 0xDEAD marks an SPI-level failure.
    fn id_stable(&mut self, chip: Chip) -> (bool, u16) {
        let mut last = 0xDEAD;
        let mut all_ok = true;
        for _ in 0..ID_TRIES {
            match self.read_reg(chip, REG_ID) {
                Ok(id) => {
                    last = id;
                    if id >> 8 != ID_EXPECTED_HI {
                        all_ok = false;
                    }
                }
                Err(_) => {
                    last = 0xDEAD;
                    all_ok = false;
                }
            }
        }
        (all_ok, last)
    }

    /// MISO bus probe: clock one frame with NO chip selected (idle bus should
    /// read all ff via the pull-up), then one NULL frame per chip alone.
    fn bus_probe(&mut self) {
        let mut idle = [0u8; FRAME];
        if self.xfer(&mut idle, "probe idle").is_ok() {
            let all_ff = idle.iter().all(|b| *b == 0xFF);
            if all_ff {
                defmt::info!(
                    "PROBE idle (no CS): all ff — nothing drives MISO when deselected (good)"
                );
            } else {
                defmt::error!("PROBE idle (no CS): {=[u8]:x} — something drives MISO with BOTH chip selects high: a chip whose CS (pin 12) is not connected/stuck low, or the MISO wire is on another signal", idle);
            }
        }
        for chip in [Chip::One, Chip::Two] {
            if let Ok(f) = self.exchange(chip, frame_with(CMD_NULL, None), "probe chip") {
                defmt::error!("PROBE ADS#{=u8} alone, NULL frame: {=[u8]:x}", chip.n(), f);
            }
        }
    }

    /// Log the raw 18-byte frames of an ID read: the RREG frame (response to
    /// whatever came before) and the NULL frame that should carry the ID.
    fn dump_id_frames(&mut self, chip: Chip) {
        let a = self.exchange(chip, frame_with(rreg_cmd(REG_ID, 1), None), "dump RREG");
        let b = self.exchange(chip, frame_with(CMD_NULL, None), "dump NULL");
        if let (Ok(a), Ok(b)) = (a, b) {
            defmt::error!("ADS#{=u8} raw RREG frame: {=[u8]:x}", chip.n(), a);
            defmt::error!("ADS#{=u8} raw NULL frame: {=[u8]:x}", chip.n(), b);
        }
    }

    /// Read ID and say plainly what a bad value means on the bench.
    fn check_id(&mut self, chip: Chip) -> bool {
        match self.read_reg(chip, REG_ID) {
            Ok(id) if id >> 8 == ID_EXPECTED_HI => {
                defmt::info!("ADS#{=u8} ID=0x{=u16:x} OK", chip.n(), id);
                true
            }
            Ok(0x0000) => {
                defmt::error!(
                    "ADS#{=u8} ID=0x0000 — the chip is driving DOUT low but never shifts: SCLK (PA5) is not reaching it. Wire PA5 to SCLK on BOTH chips",
                    chip.n()
                );
                false
            }
            Ok(0xFFFF) => {
                defmt::error!(
                    "ADS#{=u8} ID=0xFFFF — nothing drives MISO (pull-up wins): check PA6 to DOUT, this chip's CS, its 3.3 V (DVDD/AVDD) and that its GND is tied to the Nucleo GND",
                    chip.n()
                );
                false
            }
            Ok(id) if id >> 8 == 0x05 => {
                defmt::error!(
                    "ADS#{=u8} returned 0x{=u16:x} (the STATUS word, not ID) — the read command never arrived: check MOSI (PB5) to DIN",
                    chip.n(),
                    id
                );
                false
            }
            Ok(id) => {
                defmt::error!(
                    "ADS#{=u8} ID=0x{=u16:x} — unexpected (want 0x24xx): bits garbled; check SCLK wiring, wire length/ground, try lower SPI speed",
                    chip.n(),
                    id
                );
                false
            }
            Err(_) => {
                defmt::error!(
                    "ADS#{=u8} ID read failed — SPI peripheral problem (see SR above)",
                    chip.n()
                );
                false
            }
        }
    }

    /// Configure both chips identically (OSR → sample rate, all channels on,
    /// gain 1) and read CLOCK back to prove the write landed.
    fn configure(&mut self) -> Result<(), AdcError> {
        let clock_val = clock_reg_value(ADC_SAMPLE_RATE);
        let gain_val: u16 = 0x0000; // gain 1 on all channels
        for chip in [Chip::One, Chip::Two] {
            self.write_reg(chip, REG_CLOCK, clock_val)?;
            self.write_reg(chip, REG_GAIN1, gain_val)?;
            let got = self.read_reg(chip, REG_CLOCK)?;
            if got != clock_val {
                defmt::error!(
                    "ADS#{=u8} CLOCK readback 0x{=u16:x} != written 0x{=u16:x}",
                    chip.n(),
                    got,
                    clock_val
                );
                return Err(AdcError::WriteMismatch);
            }
            defmt::info!("ADS#{=u8} CLOCK=0x{=u16:x} verified", chip.n(), got);
        }
        self.gain = 1.0;
        Ok(())
    }

    /// Read one coherent frame from BOTH chips. Called from the DRDY ISR.
    ///   Chip 1 → [ref, v_a, v_b, v_c]
    ///   Chip 2 → [i_dc, v_dc, GND, GND]  (rev 11: AIN0 = AMC1302 U8 on the R40
    ///            shunt, AIN1 = AMC1311B U14 on the DC divider)
    /// Returns None while offline or if a transfer errors.
    pub fn read_frame(&mut self) -> Option<RawFrame> {
        if !self.online {
            return None;
        }
        let c1 = self.read_chip(Chip::One)?;
        let c2 = self.read_chip(Chip::Two)?;
        Some(RawFrame {
            v_ref: c1[0],
            v_a: c1[1],
            v_b: c1[2],
            v_c: c1[3],
            v_dc: c2[1],
            i_dc: c2[0],
        })
    }

    /// Read the 4 channel words from one chip: status + 4×24-bit + CRC = 18 bytes
    /// (a NULL command frame). Fast path: the HAL's FIFO-pipelined transfer.
    fn read_chip(&mut self, chip: Chip) -> Option<[i32; 4]> {
        self.select(chip, true);
        let mut buf = [0u8; FRAME];
        let ok = self.spi.transfer(&mut buf).is_ok();
        self.select(chip, false);
        if !ok {
            return None;
        }
        Some([
            be24_to_i32(&buf[3..6]),
            be24_to_i32(&buf[6..9]),
            be24_to_i32(&buf[9..12]),
            be24_to_i32(&buf[12..15]),
        ])
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
// MEASUREMENT LAYER (no hardware dependencies below this line)
//
// One 4 kSPS sample stream feeds three windows. Each window is reset only by
// the task that consumes it, so nobody throws away data another task needs:
//
//   AC window   ref + A/B/C   WINDOW_SAMPLES = 100 ms, about 6 cycles at 60 Hz.
//                             Closed by the DRDY ISR; finalized once by the
//                             100 Hz control task (sqrt/atan2 happen there).
//   CONTROL DC  v_dc, i_dc    drained by state_manager every 10 ms.
//   FAST DC     v_dc, i_dc    drained by safety_supervisor every 2 ms.
//
// AC math runs on (sample − bias), where bias is the previous window's mean
// (the ~0.6 V front-end offset). RMS is therefore the AC component only, and
// the zero-crossing detector sees a signal centred on zero.
//
// Zero crossings are rising edges with hysteresis (the signal must dip below
// −HYST before the next rising edge counts) and linear interpolation between
// samples, so frequency and phase resolve to a fraction of a sample (~0.1° at
// 60 Hz) instead of the 5.4° a whole sample would give.
//
// Phase: at each reference rising crossing, every motor phase's most recent
// rising crossing gives its angle against the reference. POSITIVE = the phase
// LEADS the reference (crosses earlier). Angles are vector-averaged over the
// window, so there are no ±π wrap artefacts.
//
// A phase with no valid signal reports PHASE_INVALID_RAD (π, i.e. 180°), never
// 0 — "no data" must never look like "in phase".
// ═════════════════════════════════════════════════════════════════════════════

/// Which motor phase the PLL aligns the reference to. Set from the terminal via
/// Command::SetAlignPhase; defaults to A. Mirrored by the shared crate's
/// AlignPhase on the wire.
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

/// One coherent sample set. Chip 1 = reference + 3 motor phase voltages,
/// sampled at the same instant, so phase differences are meaningful.
/// Chip 2 = rectifier DC bus.
#[derive(Clone, Copy, Default)]
pub struct RawFrame {
    pub v_ref: i32, // reference phase       (chip 1 ch0, AMC1311 U7)
    pub v_a: i32,   // motor phase A          (chip 1 ch1, U6)
    pub v_b: i32,   // motor phase B          (chip 1 ch2, U5)
    pub v_c: i32,   // motor phase C          (chip 1 ch3, U4)
    pub v_dc: i32,  // rectifier DC voltage   (chip 2 ch1, U14)
    pub i_dc: i32,  // rectifier DC current   (chip 2 ch0, AMC1302 U8 across R40)
}

// ── window + detector tuning ──

/// AC window length in samples: 100 ms (about 6 cycles at 60 Hz).
pub const WINDOW_SAMPLES: u32 = ADC_SAMPLE_RATE / 10;
/// Zero-crossing hysteresis at the ADS input (V). About 2.5 V on a motor
/// phase and 1.7 V on the mains reference — well above the noise floor.
const CROSS_HYST_VOLTS: f32 = 0.005;
/// The same hysteresis in ADS counts (integer hot path).
const CROSS_HYST_COUNTS: i32 = (CROSS_HYST_VOLTS * ADC_COUNTS / ADC_FS_VOLTS) as i32;
/// Crossings only pair into a period inside this band (Hz).
const F_MIN_HZ: f64 = 40.0;
const F_MAX_HZ: f64 = 80.0;
/// Reference cycles recorded per window (100 ms at 80 Hz is 8; headroom).
const MAX_CYCLES: usize = 12;
/// Reported for a phase with no valid crossings this window.
pub const PHASE_INVALID_RAD: f32 = core::f32::consts::PI;

// ── scaling (rev 11 front end) ──

const ADC_FS_VOLTS: f32 = 1.2;
const ADC_COUNTS: f32 = 8_388_608.0; // 2^23

/// Each isolator output reaches its ADS131M04 input through 1 kΩ + 1 kΩ
/// (R1–R13). Against the ADS's ~300 kΩ differential input impedance (gain 1,
/// fMOD 4.096 MHz) that divider loses ~0.66%; this puts it back. Calibration
/// against a known input trims whatever remains.
const FILTER_LOADING: f32 = (300_000.0 + 2_000.0) / 300_000.0;

// RATIO = real-world units per isolator-output volt. Voltage channels
// (AMC1311B, gain 1): (Rtop + Rbot) / Rbot, Rtop = 3 × 330 kΩ = 990 kΩ.
const REF_V_RATIO: f32 = 337.7; // R41 2.94 kΩ (US). EU part 1.54 kΩ → 643.9
const PHASE_V_RATIO: f32 = 496.0; // R36/R37/R39 2.00 kΩ
const DC_V_RATIO: f32 = 337.7; // R45 2.94 kΩ, no bias → full scale ≈ 405 V

/// R40 sense resistor (Ω). Rev 11 fits 0.1 Ω (L4CL1206LR100). With the
/// AMC1302's gain of 41 the ADS clips at 1.2 V / 41 / 0.1 Ω ≈ 0.29 A — right
/// where both load steps sit at a 310–325 V bus. A 0.05 Ω part on the same
/// footprint doubles the range; change this constant with it.
pub const SHUNT_OHMS: f32 = 0.1;
const AMC1302_GAIN: f32 = 41.0;
const DC_I_RATIO: f32 = 1.0 / (SHUNT_OHMS * AMC1302_GAIN);
/// Current at the ADS clip (A). Readings at or above this mean "at least this".
pub const I_DC_FULL_SCALE: f32 = ADC_FS_VOLTS * FILTER_LOADING * DC_I_RATIO;

/// ADS counts → isolator output volts (filter loading removed).
#[inline]
fn counts_to_amc_volts(raw: f64, gain: f32) -> f32 {
    (raw as f32) * (ADC_FS_VOLTS / ADC_COUNTS) / gain * FILTER_LOADING
}

/// Rising zero-crossing detector with hysteresis + linear interpolation.
/// Integer per sample; floating point only on an actual crossing.
#[derive(Clone, Copy, Default)]
struct Edge {
    armed: bool, // dipped below −hyst since the last rising crossing
    prev: i32,   // previous (sample − bias), counts
    have_prev: bool,
    last_t: f64, // absolute time (samples) of the latest rising crossing
    have_last: bool,
}

impl Edge {
    /// `x` = sample − bias (counts), `t` = sample index of `x`.
    /// Returns the interpolated crossing time on a rising crossing.
    #[inline]
    fn add(&mut self, x: i32, t: u32, hyst: i32) -> Option<f64> {
        let mut out = None;
        if x < -hyst {
            self.armed = true;
        }
        if self.armed && self.have_prev && self.prev < 0 && x >= 0 {
            // The zero lies between t-1 (prev < 0) and t (x >= 0).
            let frac = (-self.prev) as f64 / (x - self.prev) as f64;
            let tc = t as f64 - 1.0 + frac;
            self.armed = false;
            self.last_t = tc;
            self.have_last = true;
            out = Some(tc);
        }
        self.prev = x;
        self.have_prev = true;
        out
    }
}

/// Everything one AC window collected. Produced by the ISR, consumed by
/// `finalize_window` in the 100 Hz task. Index 0 = ref, 1..=3 = A, B, C.
#[derive(Clone, Copy, Default)]
pub struct WindowStats {
    n: u32,
    sum: [i64; 4],  // Σ(x − bias), counts — exact
    sum2: [i64; 4], // Σ(x − bias)², counts² — ≤ 2^55 over a window, fits
    cross_n: [u32; 4],
    cross_first: [f64; 4],
    cross_last: [f64; 4],
    // Running (Σx, Σx², n) captured at each channel's first and latest
    // crossing, so RMS can be taken over whole cycles of that channel.
    snap_first: [(i64, i64, u32); 4],
    snap_last: [(i64, i64, u32); 4],
    phase: [[f32; MAX_CYCLES]; 3], // per-cycle angle of A/B/C vs ref (rad)
    phase_n: [u8; 3],
}

/// DC bus means over one drained window.
#[derive(Clone, Copy, Default, defmt::Format)]
pub struct DcScalars {
    pub v_dc: f32,
    pub i_dc: f32, // load-feedforward / load-rejection source
}

/// Running state fed by the DRDY ISR. The per-sample path is integer only
/// (exact, and cheap even without the M7's double-precision FPU enabled);
/// floating point runs once per zero crossing.
#[derive(Clone, Copy, Default)]
pub struct Accumulator {
    t: u32,          // sample clock (wraps after ~12 days; harmless)
    bias: [i32; 4],  // previous window's mean per AC channel (counts)
    have_bias: bool, // false until the first window completes
    edge: [Edge; 4],
    win: WindowStats,          // window being built
    done: Option<WindowStats>, // last completed window, not yet taken
    ctl_v: i64,
    ctl_i: i64,
    ctl_n: u32,
    fast_v: i64,
    fast_i: i64,
    fast_n: u32,
}

impl Accumulator {
    /// Add one coherent sample. Called from the DRDY ISR.
    #[inline]
    pub fn add(&mut self, f: &RawFrame) {
        let raw = [f.v_ref, f.v_a, f.v_b, f.v_c];
        let t = self.t;

        let mut ref_cross: Option<(f64, f64)> = None; // (crossing time, period)
        for k in 0..4 {
            let x = raw[k].wrapping_sub(self.bias[k]);
            let xl = x as i64;
            self.win.sum[k] += xl;
            self.win.sum2[k] += xl * xl;
            let snap = (self.win.sum[k], self.win.sum2[k], self.win.n + 1);

            if self.have_bias {
                let prev_last = self.edge[k].last_t;
                let had_last = self.edge[k].have_last;
                if let Some(tc) = self.edge[k].add(x, t, CROSS_HYST_COUNTS) {
                    if self.win.cross_n[k] == 0 {
                        self.win.cross_first[k] = tc;
                        self.win.snap_first[k] = snap;
                    }
                    self.win.cross_last[k] = tc;
                    self.win.snap_last[k] = snap;
                    self.win.cross_n[k] += 1;
                    if k == 0 && had_last {
                        let period = tc - prev_last;
                        let min_period = ADC_SAMPLE_RATE as f64 / F_MAX_HZ;
                        let max_period = ADC_SAMPLE_RATE as f64 / F_MIN_HZ;
                        if period >= min_period && period <= max_period {
                            ref_cross = Some((tc, period));
                        }
                    }
                }
            }
        }

        // Angle of each motor phase against this reference crossing.
        if let Some((tc, period)) = ref_cross {
            use core::f64::consts::PI;
            for p in 0..3 {
                let e = &self.edge[p + 1];
                if !e.have_last {
                    continue;
                }
                // d > 0: the phase crossed d samples BEFORE the reference → leads.
                let d = tc - e.last_t;
                if d.abs() >= 1.5 * period {
                    continue; // stale — that phase has stopped crossing
                }
                let mut th = 2.0 * PI * d / period;
                while th > PI {
                    th -= 2.0 * PI;
                }
                while th <= -PI {
                    th += 2.0 * PI;
                }
                let n = self.win.phase_n[p] as usize;
                if n < MAX_CYCLES {
                    self.win.phase[p][n] = th as f32;
                    self.win.phase_n[p] += 1;
                }
            }
        }

        let vdc = f.v_dc as i64;
        let idc = f.i_dc as i64;
        self.ctl_v += vdc;
        self.ctl_i += idc;
        self.ctl_n += 1;
        self.fast_v += vdc;
        self.fast_i += idc;
        self.fast_n += 1;

        self.win.n += 1;
        self.t = self.t.wrapping_add(1);

        if self.win.n >= WINDOW_SAMPLES {
            self.close_window();
        }
    }

    /// Publish the window, re-centre each channel on its new mean, start fresh.
    fn close_window(&mut self) {
        let n = self.win.n as i64;
        for k in 0..4 {
            let shift = (self.win.sum[k] / n) as i32; // mean of (x − old bias)
            self.bias[k] = self.bias[k].wrapping_add(shift);
            // Keep the detector's previous sample on the same reference.
            self.edge[k].prev = self.edge[k].prev.wrapping_sub(shift);
        }
        if !self.have_bias {
            // The first window only seeded the bias; its crossings were not
            // measured, so don't publish it.
            self.have_bias = true;
        } else {
            self.done = Some(self.win);
        }
        self.win = WindowStats::default();
    }

    /// Take the latest completed AC window (once). Called by the 100 Hz task.
    pub fn take_window(&mut self) -> Option<WindowStats> {
        self.done.take()
    }

    /// DC means since the last call. Called by the 100 Hz task.
    pub fn take_control_dc(&mut self, gain: f32) -> Option<DcScalars> {
        let r = dc_mean(self.ctl_v, self.ctl_i, self.ctl_n, gain);
        self.ctl_v = 0;
        self.ctl_i = 0;
        self.ctl_n = 0;
        r
    }

    /// DC means since the last call. Called by the 500 Hz supervisor.
    pub fn take_fast_dc(&mut self, gain: f32) -> Option<DcScalars> {
        let r = dc_mean(self.fast_v, self.fast_i, self.fast_n, gain);
        self.fast_v = 0;
        self.fast_i = 0;
        self.fast_n = 0;
        r
    }
}

fn dc_mean(sum_v: i64, sum_i: i64, n: u32, gain: f32) -> Option<DcScalars> {
    if n == 0 {
        return None; // no samples (ADC offline or not converting)
    }
    let n = n as f64;
    Some(DcScalars {
        v_dc: counts_to_amc_volts(sum_v as f64 / n, gain) * DC_V_RATIO,
        i_dc: counts_to_amc_volts(sum_i as f64 / n, gain) * DC_I_RATIO,
    })
}

/// Finalized electrical + alignment values. Shipped in telemetry and used by
/// the control loop (PLL_LOCK reads phase_err_selected → theta_err_rad; the
/// governor and supervisor read v_dc / i_dc).
#[derive(Clone, Copy, defmt::Format)]
pub struct Measurements {
    // magnitudes (RMS, real volts / amps)
    pub v_ref_rms: f32,
    pub v_a_rms: f32,
    pub v_b_rms: f32,
    pub v_c_rms: f32,
    pub v_dc: f32,
    pub i_dc: f32,
    /// Reference (mains) frequency, Hz. 0 = no valid reference this window.
    pub freq_hz: f32,

    /// Each motor phase vs the reference, radians in (−π, π]; positive = the
    /// phase leads. PHASE_INVALID_RAD when that phase had no valid crossings.
    pub phase_err_a: f32,
    pub phase_err_b: f32,
    pub phase_err_c: f32,
    pub phase_valid: [bool; 3],
    /// Frequency of each motor phase (generator electrical frequency), Hz.
    pub phase_freq_hz: [f32; 3],

    /// The selected align target's values (see `select`).
    pub phase_err_selected: f32,
    pub gen_freq_hz: f32,
    pub selected_valid: bool,
}

impl Default for Measurements {
    fn default() -> Self {
        Self {
            v_ref_rms: 0.0,
            v_a_rms: 0.0,
            v_b_rms: 0.0,
            v_c_rms: 0.0,
            v_dc: 0.0,
            i_dc: 0.0,
            freq_hz: 0.0,
            phase_err_a: PHASE_INVALID_RAD,
            phase_err_b: PHASE_INVALID_RAD,
            phase_err_c: PHASE_INVALID_RAD,
            phase_valid: [false; 3],
            phase_freq_hz: [0.0; 3],
            phase_err_selected: PHASE_INVALID_RAD,
            gen_freq_hz: 0.0,
            selected_valid: false,
        }
    }
}

impl Measurements {
    /// Point the "selected" fields at the align target. Cheap; call every tick
    /// so a SetAlignPhase takes effect immediately, not at the next window.
    pub fn select(&mut self, align: AlignPhase) {
        let i = match align {
            AlignPhase::A => 0,
            AlignPhase::B => 1,
            AlignPhase::C => 2,
        };
        self.phase_err_selected = [self.phase_err_a, self.phase_err_b, self.phase_err_c][i];
        self.gen_freq_hz = self.phase_freq_hz[i];
        self.selected_valid = self.phase_valid[i];
    }

    /// Fold in fresh DC means.
    pub fn apply_dc(&mut self, dc: &DcScalars) {
        self.v_dc = dc.v_dc;
        self.i_dc = dc.i_dc;
    }
}

/// Frequency from the first and last interpolated crossings of a window.
fn window_freq(w: &WindowStats, k: usize) -> f32 {
    let n = w.cross_n[k];
    let span = w.cross_last[k] - w.cross_first[k];
    if n < 2 || span <= 0.0 {
        return 0.0;
    }
    let f = (n - 1) as f64 * ADC_SAMPLE_RATE as f64 / span;
    if f < F_MIN_HZ || f > F_MAX_HZ {
        0.0
    } else {
        f as f32
    }
}

/// Fold a completed AC window into `m` (magnitudes, frequencies, phases).
/// Leaves v_dc / i_dc alone (those come from `apply_dc`).
pub fn finalize_window(w: &WindowStats, gain: f32, m: &mut Measurements) {
    if w.n == 0 {
        return;
    }
    // RMS over whole cycles of each channel (first → latest crossing), so an
    // off-nominal frequency doesn't leave a partial cycle in the average.
    // Channels with fewer than two crossings fall back to the whole window.
    let rms = |k: usize| -> f64 {
        let (s0, q0, n0) = w.snap_first[k];
        let (s1, q1, n1) = w.snap_last[k];
        let (sum, sum2, cnt) = if w.cross_n[k] >= 2 && n1 > n0 {
            (s1 - s0, q1 - q0, (n1 - n0) as f64)
        } else {
            (w.sum[k], w.sum2[k], w.n as f64)
        };
        let mean = sum as f64 / cnt;
        let var = sum2 as f64 / cnt - mean * mean;
        libm::sqrt(if var > 0.0 { var } else { 0.0 })
    };
    m.v_ref_rms = counts_to_amc_volts(rms(0), gain) * REF_V_RATIO;
    m.v_a_rms = counts_to_amc_volts(rms(1), gain) * PHASE_V_RATIO;
    m.v_b_rms = counts_to_amc_volts(rms(2), gain) * PHASE_V_RATIO;
    m.v_c_rms = counts_to_amc_volts(rms(3), gain) * PHASE_V_RATIO;

    m.freq_hz = window_freq(w, 0);
    let mut err = [PHASE_INVALID_RAD; 3];
    for p in 0..3 {
        m.phase_freq_hz[p] = window_freq(w, p + 1);
        let cnt = w.phase_n[p] as usize;
        if cnt == 0 {
            m.phase_valid[p] = false;
            continue;
        }
        let (mut s, mut c) = (0.0f32, 0.0f32);
        for th in &w.phase[p][..cnt] {
            s += libm::sinf(*th);
            c += libm::cosf(*th);
        }
        err[p] = libm::atan2f(s, c);
        m.phase_valid[p] = true;
    }
    m.phase_err_a = err[0];
    m.phase_err_b = err[1];
    m.phase_err_c = err[2];
}
