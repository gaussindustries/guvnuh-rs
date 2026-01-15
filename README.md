# Guv'nuh – Resilient Micro-Grid Control Platform

A laboratory‑scale governor architecture that converts a standard induction machine into a **secure, programmable micro‑generator**. This platform demonstrates a "Zero-Trust" industrial control loop, fusing hard real-time safety (STM32/RTIC) with secure cloud telemetry (Rust/Dioxus/SurrealDB).

---

##    Project Purpose

> To engineer a NERC-CIP compliant governor reference design that decouples **Physics** (Safety Critical) from **Connectivity** (Telemetry), demonstrating the complete engineering pipeline from PCB design to Cloud SCADA.

---

##    Functional Scope (Phase‑1)

| ID | Requirement | Target Value |
| --- | --- | --- |
| **F‑01** | Closed‑loop speed regulation | ±2 % droop, 0–1 kW range |
| **F‑02** | Overspeed trip to safe torque‑off | < 12 ms (Hardware Interrupt) |
| **F‑03** | **Load Rejection Response** | **Recovery < 2s (100%  0% Load Step)** |
| **F‑04** | "Apollo 9" Boot Handshake | Machine locked until Cloud Authorization |
| **F‑05** | Telemetry Segregation | "Air-gapped" DMA UART Link |

*Phase‑2 targets EtherCAT integration; Phase‑3 integrates 48V storage converters.*

---

##    Repository Structure (Cargo Workspace)

This project utilizes a **Rust Monorepo** architecture to ensure Type Safety across Firmware and Cloud.

```text
.
├── shared/                  ↳ Common Structs (Wire-safe data models)
├── firmware/
│   ├── governor-stm32/      ↳ Hard Real-Time (RTIC, No-std)
│   │   └── src/app/states/  ↳ 8_load_rejection.rs, 10_estop.rs
│   └── gateway-esp32/       ↳ Telemetry Gateway (Rust std / ESP-IDF)
├── hardware/                ↳ KiCad sources, BOMs, Gerbers
├── simulations/             ↳ MATLAB/Python Feed-forward analysis
└── ci/                      ↳ GitHub Actions (Cross-compile + Test)

```

---

##    System Architecture

### 1. The Governor (STM32H7 + RTIC)

* **Role:** The "Physics Engine."
* **Responsibility:** Manages the PID Loop, Safety Interlocks, and Valve Actuation (Once Steam Turbine is built).
* **Key Feature:** **Priority-Based Preemption.** The `ESTOP` interrupt (Priority 5) and `Load Rejection` logic (Priority 3) can instantly preempt telemetry tasks (Priority 1).
* **State Machine:** Implements a formalized `BOOT`  `CALIBRATE`  `GENERATE` lifecycle.

### 2. The Gateway (ESP32 + Rust std)

* **Role:** The "Scribe."
* **Responsibility:** Buffers high-frequency telemetry and handles the TLS handshake with the VPS.
* **Isolation:** Connected via **UART DMA**. The ESP32 cannot "write" to the Governor's control variables, only request state changes via a sanitized mailbox.

### 3. The Cloud (Dioxus + SurrealDB)

* **Role:** The "Command Center."
* **Responsibility:** Authorization Server ("Permissive Action Link") and Live Dashboard.
* **Tech:** Rust Full-stack. The frontend and backend share the exact same data types as the firmware via the `shared` library.

---

##    Technology Stack

| Layer | Implementation | Rationale |
| --- | --- | --- |
| **Safety Core** | **Rust RTIC** on STM32H753 | Zero-cost abstractions, data-race freedom |
| **Telemetry** | **Rust std** on ESP32-C3 | Secure memory-safe networking stack |
| **Backend** | **Dioxus** + **SurrealDB** | Type-safe "Hardware-to-UI" pipeline |
| **Control** | PID + **Feed-Forward (Derivative Kick)** | Handling drastic Load Rejection events |
| **Safety** | Hardware Interrupt (EXTI) + **Watchdog** | SIL-2 Compliance (Fail-Safe) |

---

##    Milestone Road‑map (Jan – June 2026)

| Month | Deliverable | Competence Demonstrated |
| --- | --- | --- |
| **Jan** | **M-G Set & RTIC Boot** | Hardware wrapper abstraction, Monotonic timer |
| **Feb** | **"Apollo 9" Handshake** | Blocking Boot Logic, ESP32<->STM32 Serialization |
| **Mar** | **PID & Feed-Forward** | Stable 60Hz generation under static load |
| **Apr** | **Load Rejection** | **Critical:** Catching RPM spikes (100% Load Drop) |
| **May** | **Cloud Dashboard** | Dioxus/Grafana visualization of live telemetry |
| **Jun** | **Portfolio Release** | Full documentation, video demo, and CI/CD Freeze |

Detailed tracking available in **`docs/90_release_notes/roadmap.md`**.

---

##    Build & Flash

The workspace handles dependencies automatically.

```bash
# 1. Build the Shared Library & Firmware
$ cargo build --release --workspace

# 2. Flash the Governor (STM32)
# Uses OpenOCD via custom shell script
$ bash firmware/governor-stm32/scripts/flash.sh

# 3. Flash the Gateway (ESP32)
# Uses espflash
$ espflash flash firmware/gateway-esp32/target/riscv32imc.../release/gateway

# 4. Run the Backend (Local Dev)
$ cd backend && dx serve

```

---

##    Standards Referenced

* **IEC 61508 SIL‑2** – *Safety life‑cycle & diagnostics*
* **NERC CIP-003** – *Cyber Security — Security Management Controls*
* **IEEE 1547** – *Interconnection and Interoperability of Distributed Energy Resources*
* **MISRA C / Rust 2024** – *High-integrity coding guidelines*

© 2026 Juan Carlos Mancilla Jr  · MIT License
