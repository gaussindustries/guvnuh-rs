# Guv'nuh – Embedded Governor Development Platform

A laboratory‑scale governor that converts a 0.5 HP induction machine into a **programmable micro‑generator**, providing a complete test‑bed for turbine‑control firmware, safety engineering, and grid‑interface studies.

---

##   Project Purpose

> Deliver a fully documented, safety‑rated governor reference design that demonstrates the firmware, hardware, and workflow competencies required for entry‑level controls positions/ firmware engineer.

---

##   Functional Scope (Phase‑1)

| ID     | Requirement                         | Target Value                     |
| ------ | ----------------------------------- | -------------------------------- |
|  F‑01  | Closed‑loop speed regulation        | ±2 % droop, 0–1 kW range         |
|  F‑02  | Overspeed trip to safe torque‑off   | < 150 ms                         |
|  F‑03  | Dual, diverse current feedback      | Hall CT + shunt (mismatch ≤ 5 %) |
|  F‑04  | 24 V SELV control‑power segregation | EN 60204‑1 compliant             |
|  F‑05  | Modbus‑RTU supervision              | 115 200 baud half‑duplex         |

Phase‑2 will add EtherCAT/CiA‑402; Phase‑3 will replace the resistor load with a 48 V storage converter.

---

##   Repository Structure

```
├── docs/                    ↳ Requirements → architecture → wiring → reference
├── hardware/                ↳ KiCad sources, STEP files, BOMs, Gerbers
├── firmware/                ↳ Rust 2024 RTIC crates (stm32 & esp32)
├── software_tools/          ↳ Modbus CLI, data‑logger utilities
├── simulations/             ↳ Spice and MATLAB/Python control analysis
├── tests/                   ↳ Smoke, power, regression scripts
└── ci/                      ↳ GitHub Actions (PCB + firmware builds)
```

Detailed diagrams are located in **`docs/10_architecture/`**; frozen schematics and assemblies in **`hardware/pdf_plots/`**.

---

##   Technology Stack

| Layer             | Implementation                                       | Rationale                       |
| ----------------- | ---------------------------------------------------- | ------------------------------- |
| MCU firmware      | Rust 2024 + **RTIC** on STM32H753                    | Memory‑safe deterministic tasks |
| Fieldbus          | Modbus‑RTU (RS‑485) → EtherCAT (Phase‑2)             | Covers legacy and modern plants |
| Control algorithm | PI + droop, adaptive gain scheduler                  | Mirrors hydro governor practice |
| Safety            | Dual‑sensor voting, STO relay, MISRA static analysis | Aligns with IEC 61508 SIL‑2     |
| UX                | UART CLI + React/Next.js dashboard (Wi‑Fi co‑proc)   | Field & SCADA‑ready interfaces  |

---

##   Milestone Road‑map ( 26 weeks )

| Week | Deliverable                  | Competence Demonstrated             |
| ---- | ---------------------------- | ----------------------------------- |
|  3   | SPI DMA stream (ADS131M04)   | High‑rate data path, cache control  |
|  9   | Closed‑loop PI on VFD        | Robust control on real hardware     |
|  13  | Sensor PCB smoke‑test CI     | Hardware diversity & automation     |
|  19  | Overspeed trip demo          | Safety & fault handling             |
|  22  | MISRA static‑analysis report | Standards compliance                |
|  26  | Application packet submitted | Complete portfolio ready for review |

The full week‑by‑week table is maintained in **`docs/90_release_notes/roadmap_26wk.md`**.

---

##   Build & Flash

```bash
# clone and build
$ git clone https://github.com/gaussindustries/guvnuh-rs.git
$ cd guvnuh-rs
$ rustup target add thumbv7em-none-eabihf
$ cargo build -p stm32_firmware --release

# program via ST‑LINK
$ openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
          -c "program target/thumbv7em-none-eabihf/release/stm32_firmware.elf verify reset exit"
```

---

##   Standards Referenced

* IEC 61800‑5‑2 (STO)   *hardware interlock*
* IEC 61508 SIL‑2       *safety life‑cycle & diagnostics*
* ISA‑50 / Modbus‑RTU   *legacy plant communication*
* IEEE 519              *harmonic & ripple measurement*

---

© 2025 Juan Carlos Mancilla Jr  · MIT License
