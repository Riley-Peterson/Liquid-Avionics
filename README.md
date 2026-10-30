# Liquid-Avionics
Avionics system for liquid rocket

## Firmware

The firmware lives in `Software/Liquid_Avionics`, a Rust/embassy crate for the STM32F446RE. Flashing goes through an ST-Link with [probe-rs](https://probe.rs), which connects under reset, so the ST-Link's NRST must be wired to the board's debug header. Logs come back over RTT (defmt).

Run every command below from `Software/Liquid_Avionics`. The crate builds three binaries. Each is a complete firmware image, so flashing one replaces whatever was on the chip.

| Binary | Source | Run with | What it does |
|---|---|---|---|
| `flight` | `src/main.rs` | `cargo run` | Flight software. Currently a placeholder that logs PT1–4 once a second and every CAN frame it receives. |
| `diag` | `src/bin/diag.rs` | `cargo run --bin diag` | Board bring-up test. Checks the crystal, pin levels, ADC, W25Q flash, IMU, barometer, RFD900, CAN and the Camera 2 switch, prints a PASS/WARN/FAIL summary, then streams sensor data. **Overwrites the last 4 KiB sector of the external flash.** |
| `pt_can` | `src/bin/pt_can.rs` | `cargo run --release --bin pt_can` | Reads PT1–4 at 100 Hz and sends each as its own J1939 frame on CAN (PGNs 0xFF21–0xFF24, SA 0x40). Logs values and CAN TX stats once a second. |

Code shared by all three lives in the library (`src/lib.rs`): `board` (pin map, clocks, analog helpers), `can_map` (CAN IDs from the team CAN map) and `drivers` (MS5611, LSM6DSV16BX, W25Q).
