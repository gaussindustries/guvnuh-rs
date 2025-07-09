# 26‑Week Project Road‑map  (Guv’nuh ‑ Embedded Governor Platform)

Each row shows what **must be delivered in the repo that week**, the core **topic to master**, the **standard(s)** that anchor the work, and **why a utility firmware team would care**.

| Wk | Deliverable → Git commit / video evidence            | Study / Research Focus             | Standards touched               | Real‑world EMS relevance                          |
| -- | ---------------------------------------------------- | ---------------------------------- | ------------------------------- | ------------------------------------------------- |
| 1  | **System block diagram** PDF (docs/10\_architecture) | MCU clock tree • Isolation classes | IEC 60204‑1 colour codes        | Clear system decomposition used in plant FAT docs |
| 2  | 24 V SMPS & Nucleo blinky on bench video             | Brown‑out & boot pins              | UL 489 breaker sizing           | Basic power‑up checklist in any control cabinet   |
| 3  | SPI‑DMA stream from ADS131M04 → CSV                  | Cache maintenance & ΣΔ decimation  | IEEE 1057 ADC terminology       | High‑rate telemetry path like PMU recorders       |
| 4  | README GIF – MATLAB step response (open‑loop PI)     | Discrete PI design & Bode          | ISA 5.5 symbols                 | PID tuning is bread‑and‑butter for hydro units    |
| 5  | Hall CT + INA240 current capture demo                | CMRR, shunt layout                 | IEEE 519 harmonic metrics       | Grid codes demand current waveform analysis       |
| 6  | **CI badge** – GitHub Actions build & unit tests     | gcov coverage, Ceedling            | —                               | Professional firmware workflow gate               |
| 7  | RTIC state‑machine with overspeed **FAULT**          | Safe‑state philosophy              | IEC 61508 SIL‑2                 | Mandatory for governor replacement projects       |
| 8  | Modbus‑RTU master polling VFD (logic‑trace PNG)      | Half‑duplex timing, CRC16          | ISA‑50 / Modbus                 | Legacy comms still dominant in TVA hydro plants   |
| 9  | Closed‑loop PI on real VFD – bench video             | Droop control, fixed‑point sat     | IEEE 421.5 governor definitions | Mirrors Woodward hydro droop behaviour            |
| 10 | **Zephyr sample PR merged**                          | Device‑tree overlays               | Zephyr coding standard          | OSS contribution signals peer‑reviewed code       |
| 11 | InteractiveBOM HTML for sensor PCB                   | Component lifecycle, DNP lines     | IPC‑2581 meta data              | BOM clarity for procurement & spares              |
| 12 | Sensor PCB v0.1 smoke test pass                      | Reflow profiles, QFN rework        | J‑STD‑001 hand solder           | Credible board bring‑up skill                     |
| 13 | Python smoke‑test script in `tests/01_smoke`         | PySerial fixtures                  | IEEE 829 test docs              | Automated regression like plant factory tests     |
| 14 | Encoder RPM live over UART                           | Quadrature decoding, aliasing      | IEC 60034 generator speed terms | Encoder feed is core to turbine trip logic        |
| 15 | UART CLI for set‑point changes                       | CLI framing, SCPI vs custom        | NERC MOD‑025 set‑point audit    | Operator HMI hooks                                |
| 16 | Dump‑load PCB (Si8271 + MOSFET) routed               | Kelvin routing, θJC                | UL 1077 suppl. protect          | Dump‑load equals resistor banks on wind/hydro     |
| 17 | Wiring colour & fuse map (docs/30\_wiring)           | NEC ampacity tables                | NEC 310, EN 60204               | Field wiring doc for electricians                 |
| 18 | Dump‑load board assembled & heatsinked               | Thermal compound, torque spec      | IEC 60505 ageing                | Reliability of high‑dissipation parts             |
| 19 | Overspeed trip demo – logic analyser + video         | Miller plateau, TVS clamp          | IEC 61800‑5‑2 STO               | Safety case evidence for turbine overspeed        |
| 20 | Adaptive droop branch merged                         | Gain scheduling, anti‑windup       | IEEE 421.2 dynamics             | Maintains stability across load steps             |
| 21 | React/Next.js dashboard (ESP32 Wi‑Fi)                | WebSocket, CORS                    | ISA 101 HMI                     | Shows full‑stack SCADA interface capability       |
| 22 | MISRA / Cppcheck report ≤10 warn                     | Static analysis deviations         | MISRA C:2012                    | Compliance ticks HR checklists                    |
| 23 | Doxygen GitHub Pages live                            | API grouping                       | IEEE Std‑2001 doc               | In‑house firmware docs for maintenance            |
| 24 | **EEI MASS/POSS test pass** PDF                      | Aptitude prep                      | EEI plant testing req           | TVA prerequisite satisfied                        |
| 25 | 2‑min polished YouTube demo                          | Storyboard, audio mix              | —                               | Marketing portfolio to recruiters                 |

