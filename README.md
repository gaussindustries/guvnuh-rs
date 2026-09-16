# Guv'nuh — Micro-Grid Governor Control Platform

A Rust, bare-metal-to-browser control system for a laboratory micro-grid generator set. A hard-real-time STM32H753 governs the machine; an ESP32 bridges it to a Dioxus server and a desktop operator console. Built as the engineering centerpiece for [Gauss Industries](https://gaussindustri.es).

---

## What it does

Guv'nuh closes the loop on a self-excited induction generator (SEIG) driven by a variable-speed prime mover: it holds speed and frequency against changing electrical load, sheds load safely on fault, and streams a full electrical picture — per-phase voltage and current, real power, power factor, DC-bus state — to an operator in real time.

The architecture follows one principle: **the STM32 runs the machine; everything else observes and commands.** Safety-critical decisions (overspeed, load rejection, over-voltage/current) are computed and acted on locally, on-device, with no dependence on the network being up. The ESP32 relays; the server stores; the terminal analyzes. Comms can drop without the machine ever losing control.

---

## System architecture

```
  ┌─────────────┐   UART/COBS    ┌──────────┐    TCP/COBS    ┌──────────────┐   HTTP    ┌───────────────┐
  │  STM32H753  │◄──────────────►│  ESP32   │◄──────────────►│ Dioxus Server│◄─────────►│ Desktop Term. │
  │  (RTIC 2)   │  telemetry ▲   │ (Embassy)│  telemetry ▲   │   + SurrealDB│           │  (Dioxus)     │
  │  the machine│  commands  ▼   │  bridge  │  commands  ▼   │   store/API  │           │  operator UI  │
  └─────────────┘                └──────────┘                └──────────────┘           └───────────────┘
        │
        ├─ dual ADS131M04 (3-phase V/I + DC bus) — coherent sampling
        ├─ TIM1 PWM → prime-mover drive
        ├─ TIM2 QEI → speed encoder
        └─ ordered load bank (2-step resistive, series-ground)
```

**One wire format, end to end.** Every hop serializes the same `shared` crate types with `postcard` + COBS framing — no translation layers, no schema drift between tiers (the whole set rebuilds together when the wire types change).

---

## The STM32H753 firmware — the real-time core

Written against **RTIC 2**, priority-scheduled so the machine stays in tolerance regardless of what else is happening:

| Priority | Task | Rate | Job |
|---|---|---|---|
| 3 | ADC DRDY ISR | ~4 kSPS | Read both ADCs coherently, accumulate sum-of-squares |
| 3 | UART4 RX ISR | event | Drain FIFO, COBS-decode commands into a queue |
| 2 | Safety supervisor | 500 Hz | Overspeed trip + fast electrical protection + load-rejection detection |
| 1 | Control loop | 100 Hz | Governing, full ADC finalize, telemetry assembly |

**Three-phase electrical measurement.** Two ADS131M04 24-bit delta-sigma ADCs share one SPI bus with tied SYNC lines, so all eight channels sample the *same instant* — essential for real-power math (V×I per phase must be coherent). The STM32 computes only the control-critical scalars locally (RMS V/I, real power, PF, frequency, DC bus); harmonics and deep analysis are left to the terminal. Sample rate is a single tunable constant, sized against measured worst-case execution time.

**Load rejection — the fast local path.** A sudden real-power drop means the generator has been unloaded and will overspeed (the defining hazard for a turbine prime mover). The 500 Hz supervisor watches total real power and reacts in-loop — shedding load and faulting — with no network round-trip. This is why the electrical measurements live on the STM32 and not just in telemetry.

**Ordered load bank.** A two-step resistive bank whose second stage draws its ground return *through* the first, so the switching order is a hard electrical constraint. The firmware models this as a state machine that makes an invalid state unrepresentable: it refuses to engage step 2 without step 1, sheds gracefully (2 then 1) under normal operation, and sheds immediately (open 1, dropping both) on E-stop or fault.

---

## The ESP32 bridge

An **Embassy** async firmware that does one job well: shuttle framed bytes between the STM32's UART and a TCP socket to the server, intercepting the link-layer handshake locally so a reboot on either side re-syncs without disturbing the machine. Deliberately dumb — it holds no control logic, matching the "STM32 runs the machine" principle.

---

## Server & operator console

**Dioxus fullstack server** (axum + SurrealDB): ingests telemetry over TCP, stores trials, and exposes a hardware-control API. Telemetry x-axes are derived from a **server-side frame index**, not the device clock — eliminating boot-offset artifacts by construction.

**Desktop operator terminal** (Dioxus): live governing controls (manual / open-loop / closed-loop / scripted profiles), real-time charts (RPM, three-phase voltage and current on shared axes, power and phase-alignment error), a load-bank panel that mirrors the firmware's ordered state machine in the UI, and a trials browser that replays any recorded run with every electrical channel toggleable.

**Scripted profiles** let an operator define an RPM trajectory with load steps scheduled at specific breakpoints — "at t=5s apply half load, at t=10s go full, at t=15s shed" — executed on-device with the ordered switching still enforced.

---

## Tech stack

- **Firmware:** Rust, `no_std`, RTIC 2 (STM32H753), Embassy (ESP32)
- **Wire format:** `postcard` + COBS over UART and TCP; shared type crate
- **Server:** Rust, Dioxus fullstack, axum, SurrealDB (raw HTTP transport)
- **Terminal:** Rust, Dioxus desktop
- **Sensing:** dual TI ADS131M04, external 8.192 MHz coherent clock
- **Networking:** WireGuard-gated admin plane, nginx reverse proxy

---

## Status

Operational: full command + telemetry pipeline live across all four tiers; load bank commanding end-to-end with ordered-switching and E-stop shedding verified in hardware; governing loop running closed on the bench. Three-phase ADC front-end is wired into the firmware (flag-gated) and pending analog front-end bring-up.

**Roadmap:** phase-lock to a grid reference (the alignment view is built and waiting), EtherCAT fieldbus, a SIL-2 certification path, and a micro steam turbine prime mover.

---

## License

© Gauss Industries. All rights reserved.

*Integrity · Innovation · Invention*
