# Guv'nuh – Resilient Micro-Grid Control Platform

![Guv'nuh Alpha Preview](githubMedia/HQ_Still_Preview_Alpha.png)

![Guv'nuh Live Demo](githubMedia/preview_v6.gif)

A laboratory-scale governor architecture that converts a standard DC motor into a
**secure, programmable micro-generator testbed**. This platform demonstrates a
"Zero-Trust" industrial control loop, fusing hard real-time safety (STM32/RTIC)
with secure cloud telemetry (Rust/Dioxus/SurrealDB).

---

## Project Purpose

> To engineer a governor reference design that decouples **Physics** (safety
> critical) from **Connectivity** (telemetry), demonstrating the complete
> engineering pipeline from firmware to cloud SCADA — with the architecture
> designed against IEC 61508 and IEEE 1547 as guiding standards.

---

## Functional Scope

### Phase 1 — Governor Brains (Current)

| ID | Requirement | Target Value | Status |
| --- | --- | --- | --- |
| **F-01** | Closed-loop speed regulation (PID) | ±2% droop, 0–1 kW range | 🔄 In Progress |
| **F-02** | Overspeed trip to safe torque-off | < 12 ms (planned hardware interrupt) | 📋 Planned |
| **F-03** | Load Rejection Response | Recovery < 2s (100% → 0% Load Step) | 🔄 In Progress |
| **F-04** | Async Boot MCU Handshake | Machine locked until gateway ready | ✅ Complete |
| **F-05** | Telemetry Segregation | "Air-gapped" UART Link | ✅ Complete |
| **F-06** | Intranet Telemetry Stream | Live Telemetry over TCP/WiFi | ✅ Complete |
| **F-07** | Prelude-Based Run Configuration | `Configure(RunConfig)` before `Start` | ✅ Complete |
| **F-08** | Live Parameter Adjustment | Duty / RPM / PID gains adjustable mid-run | ✅ Complete |
| **F-09** | Interrupt-Driven Command RX | Overrun-free UART command reception | ✅ Complete |
| **F-10** | Trial Run Recording | Per-trial telemetry to SurrealDB | ✅ Complete |
| **F-11** | Telemetry Visualization | Plotly.js time-series with metric toggles | ✅ Complete |
| **F-12** | REST Hardware API | Stateless control + data endpoints on :3001 | ✅ Complete |

### Phase 2 — EtherCAT Integration

Replaces the UART telemetry link with an EtherCAT fieldbus, enabling
deterministic sub-millisecond communication between the Governor and
downstream field devices. This phase targets industrial interoperability
and positions the platform against real SCADA deployments.

### Phase 3 — 48V Storage Converters

Integrates bidirectional DC-DC converters for a 48V battery storage bank,
enabling islanded operation, load leveling, and grid-forming capability.
Adds a second control loop for state-of-charge management alongside the
existing speed governor.

### Phase 4 — Micro Steam Turbine *(Future Reference)*

> ⚠️ High temperature and pressure — deferred until Phase 1 control logic
> is fully validated and hardened.

Replaces the DC motor testbed with a purpose-built micro steam turbine as
the prime mover, completing the generator set. The Governor architecture
is designed from the ground up to support this transition — the state
machine, safety interlocks, and telemetry pipeline require no fundamental
changes, only physical plant reconfiguration.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                       SAFETY DOMAIN                          │
│                                                              │
│   AMT102-V Encoder ──► STM32H753 (RTIC)                     │
│   ACS772 Current   ──►   ├── State Machine (14 states)      │
│   ADS131M04 ADC    ──►   ├── PID Loop (WIP)                 │
│                          ├── Ramp Generator                  │
│                          ├── MotorController (inverted PWM)  │
│                          ├── UART4 RX ISR (cmd decode)       │
│                          └── UART TX/RX ──────────────┐      │
└───────────────────────────────────────────────────────┼──────┘
                                                        │ UART
┌───────────────────────────────────────────────────────┼──────┐
│                    TELEMETRY DOMAIN                   │      │
│                                                       ▼     │
│   ESP32-WROOM ◄──── UART RX (GPIO16)                        │
│       ├── Handshake Manager                                  │
│       ├── WiFi (esp-wifi + embassy-net)                      │
│       ├── TCP Server :3000 (bidirectional) ──────────────►  │
│       └── CMD forwarding (TCP → UART → STM32)                │
└──────────────────────────────────────────────────────────────┘
                                          │ TCP/WiFi
┌─────────────────────────────────────────▼────────────────────┐
│                     COMMAND DOMAIN                            │
│                                                              │
│   gaussindustri.es Server (Dioxus Fullstack + Axum)          │
│       ├── ingest_loop: ESP32 TCP → postcard/COBS decode      │
│       ├── frame sanity gate: reject non-finite / OOR values  │
│       ├── SurrealDB: trial + telemetry storage               │
│       ├── live_buffer: frame-indexed ring for polling        │
│       └── REST API :3001 (hardware control + trial data)     │
│                                                              │
│   Desktop Terminal (Dioxus Desktop)                          │
│       ├── Run Configurator: RunMode + params + presets       │
│       ├── Trial Control: Configure→Start / Stop / E-Stop     │
│       ├── Live Control: real-time duty adjustment (Manual)   │
│       ├── Connection Status: ESP32 + Server + STM32 state    │
│       ├── Trials Dashboard: accordion list of recorded runs  │
│       └── Plotly.js Charts: RPM, V, I, Freq, Temp, DC Bus   │
└──────────────────────────────────────────────────────────────┘
```

### 1. The Governor (STM32H753 + RTIC)

- **Role:** The "Physics Engine."
- **Responsibility:** State machine, PWM motor control, encoder RPM sampling,
  safety interlocks, command reception.
- **Command Reception:** A dedicated **UART4 RX interrupt** (priority 3) drains
  the FIFO the instant bytes arrive, accumulates COBS frames, decodes them into
  `Command`s, and hands them to the priority-1 state machine through a
  `heapless::Deque` queue. This decouples reception speed from the 10 ms control
  tick and eliminates FIFO overrun on multi-byte command frames (e.g. a full
  `Configure(RunConfig)`).
- **State Machine:** `BOOT` → `IDLE` → `CONFIGURED` → *(mode-dependent)*, with
  a graceful `RAMP_DOWN` on stop and `FAULT` / `ESTOP` reachable from any state.
- **Boot Handshake:** STM32 sends `HELLO` over UART until the ESP32 responds
  `OK`, then arms the RX interrupt and transitions to `IDLE`.
- **Motor Abstraction:** `MotorController` encapsulates the inverted-PWM logic
  (duty MAX = motor off) behind a `set_speed(fraction)` API with a configurable
  safety duty clamp.

### 2. The Gateway (ESP32-WROOM + esp-hal)

- **Role:** The "Scribe."
- **Responsibility:** Receives telemetry from the STM32 over UART, connects to
  WiFi, and streams data to TCP clients on port 3000. Forwards commands from
  the server back to the STM32 over UART.
- **Isolation:** Connected via UART only. The ESP32 cannot directly access
  any Governor control variables — all communication is through a sanitized
  COBS message protocol. The gateway is a dumb byte pipe: it forwards frames
  without interpreting their contents.

### 3. The Server (Dioxus Fullstack + Axum + SurrealDB)

- **Role:** The "Command Center."
- **Ingest Loop:** Connects to ESP32 over TCP, decodes postcard/COBS telemetry
  frames, and stores to SurrealDB during active trials. A sanity gate rejects
  frames with non-finite or physically impossible values so transient wire
  corruption never reaches storage or the chart.
- **Trial System:** Each trial run creates a `trial` record with start/stop
  timestamps, frame count, and status. Telemetry frames are tagged with
  `trial_id` for isolated retrieval.
- **Live Buffer:** A frame-indexed ring buffer feeds the desktop's polling
  chart with a monotonic `ts_s` x-axis derived from frame index (100 Hz), so
  the trace is independent of the STM32 boot clock.
- **REST API (`:3001`):** Stateless hardware control and data query endpoints
  consumed by the desktop terminal.

### 4. The Desktop Terminal (Dioxus Desktop)

- **Role:** The "Operator Console."
- **Run Configurator:** Select a `RunMode`, set target RPM, ramp/hold timings,
  PID gains, and a max-duty safety clamp — or apply a named **preset** with one
  click. The full `RunConfig` is sent as a prelude before the run begins.
- **Trial Control:** Start sends `Configure` then `Start`; Stop ramps down
  gracefully; a dedicated Calibrate button runs the calibration preset; an
  always-available **E-Stop** kills the motor immediately.
- **Live Control:** In Manual mode, a duty slider streams real-time
  `LiveAdjust(Duty)` commands to the motor while it runs.
- **State-Aware Trials:** The terminal reads the live STM32 state and
  auto-closes a trial when the hardware returns to `IDLE` on its own (e.g. after
  a calibration sequence or a timed hold completes).
- **Trials Dashboard:** Accordion list of all recorded trials with status,
  timestamps, and frame counts. Expand to view interactive Plotly.js charts.
- **Metric Toggles:** RPM, Voltage RMS, Current RMS, Frequency, Temperature,
  DC Bus Voltage — each independently toggleable with dual y-axes.
- **Export:** One-click PNG export of charts for reports and client deliverables.

---

## State Machine

The Governor's behavior after `Start` is determined by the `RunMode` supplied in
the preceding `Configure` prelude:

```
BOOT ──► IDLE ──(Configure)──► CONFIGURED ──(Start)──┐
                                                     │
        ┌────────────────────────────────────────────┤
        │                                             │
   RunMode::Calibrate ──► CALIBRATE ──────────────────┤ (ramp/hold/ramp → IDLE)
   RunMode::Manual    ──► MANUAL ────────────────────┤ (live duty control)
   RunMode::OpenLoop  ──► SPOOLUP ───────────────────┤ (ramp, no feedback)
   RunMode::ClosedLoop──► SPOOLUP ───────────────────┤ (PID on RPM)
   RunMode::Generate  ──► SPOOLUP ──► EXCITE ──► PLL_LOCK ──► READY ──► GENERATE
                                                                          │
                                                                   LOAD_REJECTION
                                                                          │
        Any running state ──(Stop)──► RAMP_DOWN ──► IDLE                  │
        Any state ──(EmergencyStop)──► ESTOP                             │
        Any state ──(fault)──► FAULT                                     │
        FAULT / ESTOP ──(ClearFaults | Stop)──► IDLE ◄───────────────────┘
```

`CALIBRATE`, `OpenLoop`, and `ClosedLoop` are validated on hardware today. The
full generator sequence (`EXCITE → PLL_LOCK → READY → GENERATE → LOAD_REJECTION`)
is scaffolded with time-based placeholders pending ADC sensor integration.

---

## REST API Reference

All endpoints served on `:3001`.

| Method | Endpoint | Description |
| --- | --- | --- |
| `GET` | `/api/hw/status` | ESP32 connection state, trial active, current trial ID |
| `POST` | `/api/hw/configure` | Send a `RunConfig` prelude without starting a trial |
| `POST` | `/api/hw/start` | Send `Configure` + `Start`, create trial record |
| `POST` | `/api/hw/adjust` | Send a `LiveAdjust(LiveParam)` while running |
| `POST` | `/api/hw/stop` | Send `Command::Stop`, close trial with frame count |
| `POST` | `/api/hw/estop` | Send `Command::EmergencyStop`, mark trial estopped |
| `GET` | `/api/hw/live` | Poll live frames since a counter (frame-indexed) |
| `GET` | `/api/trials` | List all trials, newest first |
| `GET` | `/api/trials/{id}` | Single trial metadata + all telemetry frames |
| `DELETE` | `/api/trials/{id}` | Delete trial and associated telemetry frames |

---

## Data Flow

```
Desktop "Start Trial" → POST /api/hw/start (body: RunConfig)
  → send Command::Configure(RunConfig) → TCP → ESP32 → UART → STM32
      → STM32: IDLE → CONFIGURED
  → SurrealDB: CREATE trial record
  → send Command::Start → TCP → ESP32 → UART → STM32
      → STM32: CONFIGURED → (mode-dependent run state)

STM32 telemetry (100Hz) → UART → ESP32 → TCP → Server
  → postcard::from_bytes_cobs::<Telemetry>()
    → sanity gate (finite + in-range)
    → live_buffer::push_frame()   (feeds desktop chart)
    → SurrealDB: CREATE telemetry { trial_id, rpm, duty_percent, state, ... }

Desktop live duty drag (Manual) → POST /api/hw/adjust {"Duty": 0.3}
  → Command::LiveAdjust(Duty(0.3)) → TCP → ESP32 → UART → STM32

Desktop "Stop Trial" → POST /api/hw/stop
  → Command::Stop → TCP → ESP32 → UART → STM32: → RAMP_DOWN → IDLE
  → SurrealDB: UPDATE trial SET status='completed', frame_count=N
```

---

## Wire Protocol

All firmware↔server communication uses **postcard** serialization with
**COBS** framing (0x00 delimiter).

```rust
// shared/src/models/telemetry/telemetry.rs

pub struct Telemetry {
    pub ts_ms: u32,
    pub state: STATE,
    pub rpm: f32,
    pub duty_percent: f32,
    pub v_gen_rms: f32,
    pub i_gen_rms: f32,
    pub freq_gen_hz: f32,
    pub theta_err_rad: f32,
    pub temp_c: f32,
    pub dc_bus_v: f32,
    pub run_mode: Option<RunMode>,
    pub fault: Option<Fault>,
}

pub enum Command {
    Ping,
    Configure(RunConfig),   // prelude sent before Start
    Start,
    Stop,
    EmergencyStop,
    LiveAdjust(LiveParam),  // real-time parameter changes
    Set(Setpoints),         // legacy
    ClearFaults,
}

pub struct RunConfig {
    pub mode: RunMode,          // OpenLoop | ClosedLoop | Calibrate | Manual | Generate
    pub target_rpm: f32,
    pub ramp_up_ms: u32,
    pub hold_ms: u32,           // 0 = hold indefinitely until Stop
    pub ramp_down_ms: u32,
    pub pid: PidGains,          // kp, ki, kd, output_min, output_max
    pub max_duty_clamp: f32,    // 0.0–1.0 safety limit
    pub target_freq_hz: f32,
    pub target_v_rms: f32,
}

pub enum LiveParam {
    Duty(f32),
    TargetRpm(f32),
    PidGains(PidGains),
    MaxDutyClamp(f32),
    TargetFreqHz(f32),
    TargetVRms(f32),
}
```

---

## Technology Stack

| Layer | Implementation | Rationale |
| --- | --- | --- |
| **Safety Core** | Rust RTIC on STM32H753 | Zero-cost abstractions, data-race freedom |
| **Command RX** | UART4 interrupt + heapless queue | Overrun-free reception, decoupled from control tick |
| **Telemetry Gateway** | esp-hal + embassy-net on ESP32 | no_std WiFi, memory-safe networking |
| **Transport** | UART (3.3V) + TCP/WiFi | Hardware isolation between domains |
| **Backend** | Axum + SurrealDB | Type-safe async Rust API |
| **Frontend** | Dioxus Fullstack + Desktop | Shared types from firmware to UI |
| **Visualization** | Plotly.js | Interactive charts, PNG export for reports |
| **Desktop** | Dioxus Desktop + reqwest | Native operator console |
| **Control** | PID + Ramp generator (PID tuning WIP) | Smooth speed transitions, closed-loop regulation |
| **Safety** | Hardware Interrupt + Watchdog (WIP) | Fail-safe torque-off (design target: SIL-2) |

---

## Repository Structure (Cargo Workspace)

```text
.
├── shared/                    ↳ Wire-safe data models (no_std, serde + postcard)
│   └── src/models/
│       ├── state/states.rs    ↳ STATE enum (14 states) + Fault enum
│       └── telemetry/         ↳ Telemetry, Command, RunConfig, LiveParam, PidGains
├── firmware/
│   ├── stm32/                 ↳ Hard Real-Time Governor (RTIC, no_std)
│   │   └── src/
│   │       ├── main.rs        ↳ State machine, UART4 RX ISR, RPM monitor
│   │       └── guv/
│   │           ├── motor.rs   ↳ MotorController (inverted PWM abstraction)
│   │           ├── pid.rs     ↳ PID controller (anti-windup, hot-swap gains)
│   │           ├── ramp.rs    ↳ Linear ramp generator
│   │           └── states/    ↳ boot, calibrate, idle, estop, fault...
│   └── esp32/                 ↳ Telemetry Gateway (esp-hal, no_std)
│       └── src/main.rs        ↳ Handshake, WiFi, TCP server, UART bridge
├── docs/
│   ├── 00_requirements/       ↳ Project Charter
│   ├── 45_µcu_reference/      ↳ STM32H753 + ESP32-WROOM datasheets
│   ├── 50_motor_drive_data/   ↳ ACS772, ADS131M04, INA240, AMT102-V
│   └── 90_release_notes/      ↳ Roadmap
├── hardware/                  ↳ KiCad sources, BOMs (WIP)
├── githubMedia/               ↳ Preview images
└── ci/                        ↳ GitHub Actions (Cross-compile + Test)
```

---

## Current State (July 2026)

### Validated End-to-End on Hardware

- ✅ STM32H753 boots, asserts safe PWM state, enables relay
- ✅ UART handshake between STM32 and ESP32 (`HELLO` / `OK`)
- ✅ AMT102-V rotary encoder reading RPM via QEI (TIM2)
- ✅ PWM motor control via TIM1 (inverted logic) behind `MotorController`
- ✅ Telemetry transmitted over UART to ESP32 (postcard/COBS, 100Hz)
- ✅ ESP32 connects to 2.4GHz WiFi via esp-wifi + embassy-net
- ✅ TCP server on port 3000 streams live telemetry to server
- ✅ Bidirectional pipeline — desktop commands forwarded to STM32
- ✅ Interrupt-driven UART4 command RX — no FIFO overrun on `Configure` frames
- ✅ Prelude-based configuration — `Configure(RunConfig)` gates `Start`
- ✅ RunMode branching — Calibrate / Manual / OpenLoop / ClosedLoop validated
- ✅ Live parameter adjustment — real-time duty control in Manual mode
- ✅ Graceful `RAMP_DOWN` on stop; immediate `EmergencyStop` path
- ✅ Server decodes COBS frames with sanity gate, stores to SurrealDB
- ✅ Trial run system — start/stop from desktop, per-trial DB storage
- ✅ State-driven trial termination — auto-close when hardware returns to IDLE
- ✅ REST API on :3001 (status, configure, start, adjust, stop, estop, live, trials)
- ✅ Desktop terminal with run configurator, presets, and live STM32 state readout
- ✅ Trials dashboard with accordion, Plotly.js charts, metric toggles
- ✅ PNG export for client reports

### In Progress

- 🔄 PID closed-loop speed control tuning
- 🔄 Load rejection detection + recovery logic
- 🔄 ADC sensor integration (voltage, current, frequency, temperature)
- 🔄 Full generator sequence (EXCITE / PLL_LOCK / READY / GENERATE)
- 🔄 Live WebSocket telemetry dashboard (web frontend)
- 🔄 VPS deployment with TLS

### Planned

- 📋 Hardware-interrupt overspeed trip to safe torque-off
- 📋 Watchdog-backed fail-safe supervision

---

## Build & Flash

### Prerequisites
```bash
# STM32 toolchain
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools

# ESP32 toolchain
cargo install espup espflash
espup install
. ~/export-esp.sh  # add to ~/.bashrc

# Server
cargo install dioxus-cli
```

### Environment
```bash
# firmware/esp32/.env
WIFI_SSID=your_ssid
WIFI_PASS=your_password
TCP_PORT=3000

# gaussindustri.es/.env
SURREAL_ENDPOINT=ws://127.0.0.1:8000
SURREAL_USER=root
SURREAL_PASS=root
SURREAL_NS=gaussindustries
SURREAL_DB=main
ESP32_ADDR=192.168.1.189:3000
```

### Run
```bash
# Terminal 1 — Database
surreal start --user root --pass root

# Terminal 2 — Server
cd gaussindustri.es && dx serve

# Terminal 3 — Firmware
cd 00_Guv'nuh
cargo stm32-flash
. ~/export-esp.sh && cargo esp32-flash

# Terminal 4 — Desktop Terminal
cd gauss-terminal && dx serve --platform desktop
```

### Verify
```bash
# Check ESP32 TCP
nc <esp32_ip> 3000

# Check REST API
curl http://localhost:3001/api/hw/status
curl http://localhost:3001/api/trials

# Send a calibration prelude + start (example)
curl -X POST http://localhost:3001/api/hw/start \
  -H "Content-Type: application/json" \
  -d '{"mode":"Calibrate","target_rpm":0,"ramp_up_ms":5000,"hold_ms":5000,
       "ramp_down_ms":5000,"pid":{"kp":1.0,"ki":0.0,"kd":0.0,
       "output_min":0.0,"output_max":1.0},"max_duty_clamp":1.0,
       "target_freq_hz":60.0,"target_v_rms":120.0}'
```

---

## Standards & Design Targets

The architecture is designed with the following standards as references. They
inform the safety life-cycle, interconnection behavior, and coding discipline —
they are design targets guiding development, not claims of certified compliance.

- **IEC 61508 (SIL-2)** – Functional safety life-cycle & diagnostics *(target)*
- **IEEE 1547** – Interconnection and interoperability of distributed energy resources
- **Rust 2024 / high-integrity guidelines** – Data-race freedom, `#![no_std]` firmware

---

© 2026 Juan Carlos Mancilla Jr · MIT License
