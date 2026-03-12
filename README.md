Based on what we've actually built, here's the updated README:

```markdown
# Guv'nuh – Resilient Micro-Grid Control Platform

A laboratory-scale governor architecture that converts a standard DC motor into a
**secure, programmable micro-generator testbed**. This platform demonstrates a
"Zero-Trust" industrial control loop, fusing hard real-time safety (STM32/RTIC)
with secure cloud telemetry (Rust/Dioxus/SurrealDB).

---

## Project Purpose

> To engineer a NERC-CIP compliant governor reference design that decouples
> **Physics** (Safety Critical) from **Connectivity** (Telemetry), demonstrating
> the complete engineering pipeline from PCB design to Cloud SCADA.

---

## Functional Scope (Phase-1)

| ID | Requirement | Target Value | Status |
| --- | --- | --- | --- |
| **F-01** | Closed-loop speed regulation | ±2% droop, 0–1 kW range | 🔄 In Progress |
| **F-02** | Overspeed trip to safe torque-off | < 12 ms (Hardware Interrupt) | 🔄 In Progress |
| **F-03** | Load Rejection Response | Recovery < 2s (100% → 0% Load Step) | 🔄 In Progress |
| **F-04** | "Apollo 9" Boot Handshake | Machine locked until Cloud Authorization | ✅ Complete |
| **F-05** | Telemetry Segregation | "Air-gapped" UART Link | ✅ Complete |
| **F-06** | Intranet Telemetry Stream | Live RPM over TCP/WiFi | ✅ Complete |

---

## Repository Structure (Cargo Workspace)

This project utilizes a **Rust Monorepo** architecture to ensure type safety
across firmware and cloud layers.

```text
.
├── shared/                  ↳ Common Structs (Wire-safe data models, no_std)
├── firmware/
│   ├── stm32/               ↳ Hard Real-Time Governor (RTIC, no_std)
│   │   └── src/guv/states/  ↳ boot, calibrate, idle, estop, fault...
│   └── esp32/               ↳ Telemetry Gateway (esp-hal, no_std)
├── docs/
│   ├── 00_requirements/     ↳ Project Charter
│   ├── 45_µcu_reference/    ↳ STM32H753 + ESP32-WROOM datasheets
│   ├── 50_motor_drive_data/ ↳ ACS772, ADS131M04, INA240, AMT102-V
│   └── 90_release_notes/    ↳ Roadmap
├── hardware/                ↳ KiCad sources, BOMs (WIP)
└── ci/                      ↳ GitHub Actions (Cross-compile + Test)
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     SAFETY DOMAIN                        │
│                                                          │
│   AMT102-V Encoder ──► STM32H753 (RTIC)                 │
│   ACS772 Current   ──►   ├── State Machine               │
│   ADS131M04 ADC    ──►   ├── PID Loop (WIP)             │
│                          ├── PWM Motor Control           │
│                          └── UART TX/RX ──────────────┐  │
└──────────────────────────────────────────────────────-─┼─┘
                                                         │ UART
┌────────────────────────────────────────────────────────┼─┐
│                   TELEMETRY DOMAIN                      │  │
│                                                         ▼  │
│   ESP32-WROOM ◄──── UART RX (GPIO16)                      │
│       ├── Handshake Manager                               │
│       ├── WiFi (esp-wifi + embassy-net)                   │
│       └── TCP Server :3000 ──────────────────────────►    │
└───────────────────────────────────────────────────────────┘
                                          │ TCP/WiFi
┌─────────────────────────────────────────▼─────────────────┐
│                    COMMAND DOMAIN (WIP)                    │
│                                                            │
│   Rust API (Axum + SurrealDB)                             │
│       └── Dioxus Frontend (Live Telemetry Dashboard)      │
└────────────────────────────────────────────────────────────┘
```

### 1. The Governor (STM32H753 + RTIC)

- **Role:** The "Physics Engine."
- **Responsibility:** State machine, PWM motor control, encoder RPM sampling,
  safety interlocks.
- **Key Feature:** Priority-based preemption. `ESTOP` (Priority 5) and
  `Load Rejection` (Priority 3) instantly preempt telemetry tasks (Priority 1).
- **State Machine:** `BOOT` → `CALIBRATE` → `IDLE` → `GENERATE` →
  `LOAD_REJECTION` → `ESTOP` / `FAULT`
- **Boot Handshake:** STM32 repeatedly sends `HELLO` over UART until ESP32
  responds `OK`, ensuring the telemetry uplink is established before any
  mechanical sequence begins.

### 2. The Gateway (ESP32-WROOM + esp-hal)

- **Role:** The "Scribe."
- **Responsibility:** Receives telemetry from the STM32 over UART, connects to
  WiFi, and streams data to TCP clients on port 3000. Forwards commands from
  the desktop back to the STM32.
- **Isolation:** Connected via UART only. The ESP32 cannot directly access
  any Governor control variables — all communication is through a sanitized
  message protocol.
- **Current Status:** Live RPM streaming over intranet TCP validated. VPS/TLS
  uplink is next.

### 3. The Cloud (Dioxus + SurrealDB) — WIP

- **Role:** The "Command Center."
- **Responsibility:** Authorization server ("Permissive Action Link") and
  live dashboard.
- **Tech:** Rust full-stack. The frontend, backend, and firmware all share
  the exact same data types via the `shared` crate.

---

## Technology Stack

| Layer | Implementation | Rationale |
| --- | --- | --- |
| **Safety Core** | Rust RTIC on STM32H753 | Zero-cost abstractions, data-race freedom |
| **Telemetry Gateway** | esp-hal + embassy-net on ESP32 | no_std WiFi, memory-safe networking |
| **Transport** | UART (3.3V) + TCP/WiFi | Hardware isolation between domains |
| **Backend** | Axum + SurrealDB (WIP) | Type-safe, async Rust API |
| **Frontend** | Dioxus (WIP) | Shared types from firmware to UI |
| **Control** | PID + Feed-Forward (WIP) | Handling drastic load rejection events |
| **Safety** | Hardware Interrupt + Watchdog (WIP) | SIL-2 compliant fail-safe |

---

## Current State (March 2026)

The following has been validated end-to-end on hardware:

- ✅ STM32H753 boots, asserts safe PWM state, enables relay
- ✅ UART handshake between STM32 and ESP32 (`HELLO` / `OK`)  
- ✅ AMT102-V rotary encoder reading RPM via QEI (TIM2)
- ✅ PWM motor control via TIM1 (inverted logic, 20kHz)
- ✅ RPM telemetry transmitted over UART to ESP32 at 1Hz
- ✅ ESP32 connects to 2.4GHz WiFi via esp-wifi + embassy-net
- ✅ TCP server on port 3000 streams live RPM data to desktop
- ✅ Bidirectional pipeline — desktop commands forwarded to STM32
- 🔄 PID closed-loop speed control
- 🔄 Load rejection state + recovery logic
- 🔄 Rust API (Axum + SurrealDB) on VPS
- 🔄 Dioxus live dashboard

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
```

### Flash

```bash
# Flash STM32 Governor
cargo stm32-flash

# Flash ESP32 Gateway
# Requires firmware/esp32/.env with WIFI_SSID and WIFI_PASS
cargo esp32-flash
```

### Test Live Telemetry

```bash
# After both boards are running and ESP32 prints its IP:
nc <esp32_ip> 3000
```

---

## Milestone Road-map (Jan – June 2026)

| Month | Deliverable | Status |
| --- | --- | --- |
| **Jan** | M-G Set & RTIC Boot | ✅ Complete |
| **Feb** | "Apollo 9" Handshake + UART Telemetry | ✅ Complete |
| **Mar** | WiFi Gateway + TCP Streaming | ✅ Complete |
| **Apr** | PID & Feed-Forward, Load Rejection | 🔄 In Progress |
| **May** | Cloud Dashboard (Axum + Dioxus) | 🔄 Upcoming |
| **Jun** | Portfolio Release + CI/CD Freeze | 🔄 Upcoming |

---

## Standards Referenced

- **IEC 61508 SIL-2** – Safety life-cycle & diagnostics
- **NERC CIP-003** – Cyber Security — Security Management Controls  
- **IEEE 1547** – Interconnection and Interoperability of Distributed Energy Resources
- **MISRA C / Rust 2024** – High-integrity coding guidelines

---

© 2026 Juan Carlos Mancilla Jr · MIT License
```
