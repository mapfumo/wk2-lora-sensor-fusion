# Week 2: LoRa Sensor Fusion - Making Constraints Explicit

![Week 2](wk2_image.png)

**Building a reliable embedded system involves far more than getting code to compile.** Week 2 of the 4-month learning program focused on understanding where hardware limits, timing guarantees, and data representation quietly shape everything above them.

The result is a dual-node LoRa sensor fusion system built on the STM32F446RE using Rust and RTIC 1.1 — not as a showcase, but as a **foundation for reliability**.

## Series Navigation

- [Week 1: RTIC LoRa Basics](https://github.com/mapfumo/wk1_rtic_lora) | [Blog Post](https://www.mapfumo.net/posts/building-deterministic-iiot-systems-with-embedded-rust-and-rtic/)
- **Week 2: Sensor Fusion & Constraints** (You are here) | [Blog Post](https://www.mapfumo.net/posts/lora-sensor-fusion-week-2/)
- Week 3: Binary Protocols & CRC (Coming soon)

---

## Table of Contents

- [Overview](#overview)
- [Week 2 Focus: Making Constraints Explicit](#week-2-focus-making-constraints-explicit)
- [Hardware Configuration](#hardware-configuration)
- [Key Technical Lessons](#key-technical-lessons)
- [System Architecture](#system-architecture)
- [Building & Flashing](#building--flashing)
- [Design Philosophy](#design-philosophy)
- [Current Status](#current-status)

---

## Overview

This repository contains the firmware for a **dual-node LoRa sensor network** built using **Rust** and the **RTIC 1.1** framework for deterministic, interrupt-driven performance on the STM32F446RE.

The system consists of:

- **Node 1 (Remote Sensor Node)**: Multi-sensor data fusion with SHT31-D and BME680
- **Node 2 (Gateway/Receiver)**: LoRa-to-serial bridge with local BMP280 sensor and OLED display

### What Makes Week 2 Different

Week 2's goal was deliberately narrow:

- ✓ Build a two-node LoRa system
- ✓ Fuse data from multiple sensors on the transmitting node
- ✓ Display and verify the data locally
- ✓ Receive, parse, and observe it on a second node

**No cloud. No dashboards. No retries or binary framing yet.**

Just a system that behaves predictably end-to-end.

---

## Week 2 Focus: Making Constraints Explicit

Week 2 wasn't about adding features—it was about understanding the foundational constraints that make reliability possible:

| Constraint Type       | What We Learned                                  | Impact                                                  |
| --------------------- | ------------------------------------------------ | ------------------------------------------------------- |
| **Data Provenance**   | Every value has a clear, documented source       | No hidden averaging or sensor blending                  |
| **Timing Guarantees** | UART at 115,200 baud = 8.7 µs per byte           | Interrupt handlers must stay minimal                    |
| **Buffer Limits**     | Self-imposed (30 bytes) vs. hardware (240 bytes) | Understanding real vs. assumed constraints              |
| **Ownership Rules**   | Rust forces explicit I2C bus access control      | Honest architectural decisions enforced at compile time |

### Design Philosophy

> **"Reliability is not added later. It is built up, one unglamorous layer at a time."**

This week established that before moving to binary serialization, framing, and retries:

- ✓ Data has a clear origin
- ✓ Timing is intentional
- ✓ Constraints are understood
- ✓ Failures are diagnosable

---

## Hardware Configuration

### Node 1 - Remote Sensor Node (Main Binary)

| Component            | Specification                        | Notes                                    |
| -------------------- | ------------------------------------ | ---------------------------------------- |
| **MCU**              | STM32F446RET6 (Cortex-M4F @ 84 MHz)  | HSI clock, no external crystal           |
| **Board**            | NUCLEO-F446RE                        |                                          |
| **Radio**            | REYAX RYLR998 LoRa Module            | UART4 @ 115200 baud                      |
| **Primary Sensor**   | SHT31-D                              | Temperature (±0.3°C) & Humidity (±2% RH) |
| **Secondary Sensor** | BME680                               | Gas Resistance (VOC detection)           |
| **Display**          | SSD1306 OLED 128x64 I2C              | Always-on sanity check                   |
| **Power**            | Split-rail                           | Elegoo MB102 for LoRa, Nucleo for logic  |
| **Debug**            | LED on PA5                           | Status indicator (1 Hz toggle)           |
| **ST-Link Probe**    | `0483:374b:0671FF3833554B3043164817` | For dual-probe setup                     |

### Node 2 - Gateway/Receiver (node2 Binary)

| Component         | Specification                        | Notes                                    |
| ----------------- | ------------------------------------ | ---------------------------------------- |
| **MCU**           | STM32F446RET6 (Cortex-M4F @ 84 MHz)  | HSI clock                                |
| **Board**         | NUCLEO-F446RE                        |                                          |
| **Radio**         | REYAX RYLR998 LoRa Module            | UART4 @ 115200 baud                      |
| **Sensor**        | BMP280                               | Temperature & Pressure (local reference) |
| **Display**       | SSD1306 OLED 128x64 I2C              |                                          |
| **Power**         | USB-powered via ST-Link              |                                          |
| **Debug**         | LED on PA5                           | Status indicator                         |
| **ST-Link Probe** | `0483:374b:066DFF3833584B3043115433` | For dual-probe setup                     |

### Pin Configuration

| Peripheral | Protocol | Pin(s) | Function                          |
| ---------- | -------- | ------ | --------------------------------- |
| LED        | GPIO     | PA5    | Status indicator (toggles @ 1 Hz) |
| I2C1 SCL   | I2C      | PB8    | Sensor & Display bus clock        |
| I2C1 SDA   | I2C      | PB9    | Sensor & Display bus data         |
| UART4 TX   | UART     | PC10   | LoRa module transmit              |
| UART4 RX   | UART     | PC11   | LoRa module receive               |

---

## Key Technical Lessons

### Lesson 1: The 51-Byte Myth (Self-Imposed Limits)

**Symptom**: LoRa packets were being truncated. Only 28–30 bytes of each message arrived intact.

**Initial Hypothesis**: LoRa limitation. In LoRaWAN contexts, a 51-byte payload cap is often cited.

**Reality**: The RYLR998 is **not** LoRaWAN. It supports payloads up to **240 bytes**.

**Root Cause**: Both transmit and receive paths were using `heapless::String` buffers sized far too conservatively for full AT commands carrying fused sensor data.

**Solution**:

- Expanded buffers to 255 bytes
- Sized operations based on actual payload length, not assumptions

**Takeaway**: Embedded constraints don't just come from hardware — they also come from our own defensive choices.

---

### Lesson 2: UART Interrupt Timing (When Helpful Code Becomes Harmful)

**Symptom**: After fixing buffer sizes, packets arrived scrambled. Characters were missing, reordered, or corrupted.

**Root Cause**: At 115,200 baud, UART delivers a new byte roughly every **8.7 microseconds**. During debugging, extra diagnostic logic had crept into the UART interrupt handler. That code was slow enough to cause missed bytes.

**The RTIC Lesson**:

> **Interrupt handlers must stay fast and boring.**

**Solution**: Reduced UART handler's job to:

1. Read the byte
2. Buffer it
3. Detect message boundaries

Everything else — parsing, display updates, I2C traffic — was moved to slower, scheduled tasks.

**Result**: Once that separation was enforced, the data stream became clean and repeatable.

---

### Lesson 3: Sensor Fusion by Selection, Not Averaging

**Design Decision**: Use the best sensor for each measurement type, avoiding "blending."

| Measurement    | Sensor | Accuracy | Rationale                                    |
| -------------- | ------ | -------- | -------------------------------------------- |
| Temperature    | SHT31  | ±0.3°C   | Dedicated high-precision sensor              |
| Humidity       | SHT31  | ±2% RH   | Superior to BME680 (±3% RH), no 48hr burn-in |
| Gas Resistance | BME680 | N/A      | Unique VOC detection capability              |
| Pressure\*     | BME680 | ±1 hPa   | Available but not currently transmitted      |

**Rationale**: This keeps the system honest. Each value has a clear provenance, and no attempt is made to hide sensor limitations behind averaging or smoothing. **At this stage, clarity beats cleverness.**

---

### Lesson 4: Data Representation (Small Lies That Matter)

**The Problem**: The SHT31 reports data in non-obvious units:

- Temperature in **centidegrees Celsius** (2852 = 28.52°C)
- Humidity in **basis points** (5520 = 55.20% RH)

**Why This Matters**: A value like 2852 looks alarming until you remember it means 28.52°C. Small misunderstandings like this propagate quickly in embedded systems if left unchecked.

**Solution**: Explicit unit conversions with clear documentation. The OLED display serves as an always-on sanity check, not just a UI.

**Additional Discovery**: OLED rendering revealed that starting text at `y = 0` clips glyph ascenders. These aren't dramatic failures, but they accumulate.

---

### Lesson 5: Shared Hardware Forces Honest Design

**The Challenge**: All sensors and the OLED share a single I2C bus. In Rust, this is not something you can "just make work."

**Rust's Ownership Rules Force You to Decide**:

- Who owns the bus?
- When can it be accessed?
- Under what guarantees?

**Solution**: Uses the `shared-bus` crate with a Cortex-M mutex, providing safe, serialized access without runtime ambiguity.

**The RTIC Pattern**: Rust doesn't let you defer architectural decisions. It makes you confront them early, when the system is still small enough to reason about.

---

## System Architecture

### Framework & Dependencies

- **RTIC 1.1**: Real-Time Interrupt-driven Concurrency
- **HAL**: `stm32f4xx-hal` 0.23.0
- **Clock**: HSI @ 84 MHz (no external crystal required)

**Key Crates**:

- `cortex-m-rtic = "1.1"` - RTIC framework
- `stm32f4xx-hal = "0.23.0"` - Hardware abstraction layer
- `sht3x = "0.1.1"` - SHT31 sensor driver
- `bme680 = "0.6.0"` - BME680 sensor driver
- `ssd1306 = "0.8.4"` - OLED display driver
- `shared-bus = "0.3.1"` - I2C bus sharing with mutex
- `embedded-graphics = "0.8.1"` - Graphics primitives
- `heapless = "0.8"` - Stack-allocated data structures (255-byte buffers)
- `defmt-rtt = "0.4"` - Logging via RTT

See [Cargo.toml](Cargo.toml) for complete dependency list.

### Timer Configuration

| Timer    | Frequency | Purpose                            |
| -------- | --------- | ---------------------------------- |
| **TIM2** | 1 Hz      | Periodic sensor sampling interrupt |
| **TIM3** | 1 MHz     | BME680 delay provider              |
| **TIM5** | 1 MHz     | SHT31 delay provider               |

### Resource Management (RTIC)

**Shared Resources** (accessed via locks):

- `lora_uart` - UART4 for LoRa communication
- `display` - SSD1306 OLED
- `sht31` - Primary temperature/humidity sensor
- `bme680` - Gas resistance sensor

**Local Resources** (exclusive access):

- `led` - Status indicator
- `timer` - TIM2 for periodic tasks
- `bme_delay` - TIM3 delay provider
- `sht_delay` - TIM5 delay provider

---

## LoRa Protocol

### Transmission Format

Data is transmitted via UART to the LoRa module using AT commands:

```
AT+SEND=0,20,T:28.5H:55.2G:142\r\n
```

**Payload Format**: `T:{temp}H:{humidity}G:{gas_resistance}`

| Field | Unit | Source | Example            |
| ----- | ---- | ------ | ------------------ |
| `T:`  | °C   | SHT31  | `28.5`             |
| `H:`  | % RH | SHT31  | `55.2`             |
| `G:`  | Ω    | BME680 | `142` (kΩ implied) |

**Current Limit**: ASCII-based AT commands (Week 2). Binary serialization planned for Week 3.

---

## Display Output

### Node 1 (Sensor Node)

```
T:28.5C H:55%        (Line 1: SHT31 temperature & humidity)
Gas:142k             (Line 2: BME680 gas resistance)
ID:01 RSSI:--        (Line 3: LoRa metadata - placeholder)
SNR:-- #---          (Line 4: Signal stats - placeholder)
```

### Node 2 (Gateway)

```
T:28.5C H:55%        (Line 1: Received temperature & humidity)
Gas:142k             (Line 2: Received gas resistance)
RSSI:-22 SNR:13      (Line 3: Signal quality from LoRa module)
Pkts: 142            (Line 4: Packet counter)
```

**Layout**: Optimized 12-pixel line spacing with 4 lines of data, leaving 10 pixels free for future status indicators or 5th line.

---

## Measured Performance

After buffer and timing optimizations:

| Metric             | Value          | Notes                                                |
| ------------------ | -------------- | ---------------------------------------------------- |
| **RSSI**           | -20 to -22 dBm | Excellent signal strength at test range (~5m indoor) |
| **SNR**            | ~13 dB         | Clean reception, no interference                     |
| **Packet Loss**    | 0%             | After buffer/timing fixes                            |
| **Update Rate**    | 1 Hz           | Stable, deterministic                                |
| **UART Byte Time** | 8.7 µs         | @ 115,200 baud                                       |
| **Max Payload**    | 240 bytes      | RYLR998 actual limit (not 51 bytes)                  |

**Result**: The system's behavior became explainable. Nothing surprised me.

---

## Building & Flashing

### Prerequisites

```bash
# Install Rust embedded toolchain
rustup target add thumbv7em-none-eabihf

# Install probe-rs for flashing
cargo install probe-rs --features cli
```

### Build Commands

**Build Node 1 (Sensor Node)**:

```bash
cargo build --release
```

**Build Node 2 (Gateway)**:

```bash
cargo build --release --bin node2
```

---

## Dual-Probe Development Setup

This project supports **simultaneous development on both nodes** using a single computer with two ST-Link probes. Each NUCLEO board has a unique ST-Link serial number.

### Finding Your Probe Serial Numbers

```bash
probe-rs list
```

Output:

```
[0]: STLink V2-1 -- 0483:374b:0671FF3833554B3043164817 (ST-LINK)
[1]: STLink V2-1 -- 0483:374b:066DFF3833584B3043115433 (ST-LINK)
```

Use the `VID:PID:SerialNumber` format with the `--probe` flag.

### Flash Commands

**Flash Node 1**:

```bash
cargo build --release && probe-rs run \
  --probe 0483:374b:0671FF3833554B3043164817 \
  --chip STM32F446RETx \
  target/thumbv7em-none-eabihf/release/wk2-lora-sensor-fusion
```

**Flash Node 2**:

```bash
cargo build --release --bin node2 && probe-rs run \
  --probe 0483:374b:066DFF3833584B3043115433 \
  --chip STM32F446RETx \
  target/thumbv7em-none-eabihf/release/node2
```

### Shell Aliases (Recommended)

Add to your `~/.bashrc` or `~/.zshrc`:

```bash
# Node 1 (replace serial with your actual probe)
alias n1='cargo build --release && probe-rs run --probe 0483:374b:0671FF3833554B3043164817 --chip STM32F446RETx target/thumbv7em-none-eabihf/release/wk2-lora-sensor-fusion'

# Node 2 (replace serial with your actual probe)
alias n2='cargo build --release --bin node2 && probe-rs run --probe 0483:374b:066DFF3833584B3043115433 --chip STM32F446RETx target/thumbv7em-none-eabihf/release/node2'
```

Then simply:

```bash
n1  # Flash and monitor Node 1
n2  # Flash and monitor Node 2
```

---

## Project Structure

```
wk2-lora-sensor-fusion/
├── src/
│   ├── main.rs          # Node 1: Sensor node with multi-sensor fusion
│   └── bin/
│       └── node2.rs     # Node 2: Gateway/receiver with OLED
├── Cargo.toml           # Dependencies and build config
├── memory.x             # Linker script for STM32F446
├── README.md            # This file
├── NOTES.md             # Detailed learning notes and insights
└── TROUBLESHOOTING.md   # Issues and solutions
```

---

## Current Status

### Completed (Week 2)

- [x] UART4 LoRa Bring-up
- [x] I2C Multi-sensor Bus Integration (shared-bus crate)
- [x] OLED Display Integration (SSD1306)
- [x] Sensor Data Fusion (SHT31 primary, BME680 gas only)
- [x] Temperature Conversion Fixes (centidegrees → °C)
- [x] Humidity Conversion (basis points → %)
- [x] Buffer Sizing Lesson (30 bytes → 255 bytes)
- [x] UART ISR Timing Optimization (fast & boring)
- [x] Data Unit Representation (centidegrees, basis points)
- [x] Optimized Display Layout (4 lines, network info)
- [x] Packet Counter Implementation
- [x] LoRa Transmission Verified (Logic Analyzer)
- [x] Runtime LoRa Module Configuration
- [x] Node 2 Receiver Implementation
- [x] LoRa End-to-End Communication (N1 → N2)
- [x] RSSI/SNR Signal Quality Monitoring
- [x] System Behavior is Explainable

### Planned (Week 3+)

- [ ] Range Testing Documentation
- [ ] Binary Serialization (replacing AT commands)
- [ ] CRC Data Integrity
- [ ] ACK/NACK Acknowledgment Protocol
- [ ] Packet Retransmission
- [ ] Error Recovery Mechanisms

---

## Learning Resources

**For detailed technical notes and debugging insights**:

- [NOTES.md](NOTES.md) - Week 2 technical deep-dive
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - Issues and solutions

**Related Projects**:

- [Week 1: RTIC LoRa Basics](https://github.com/mapfumo/wk1_rtic_lora)
- [Week 1 Blog Post](https://www.mapfumo.net/posts/building-deterministic-iiot-systems-with-embedded-rust-and-rtic/)

---

## Why Week 2 Matters

Week 2 is not about performance or optimization.

It's about reaching a point where:

- Data has a clear origin
- Timing is intentional
- Constraints are understood
- Failures are diagnosable

**Only then does it make sense to move on to binary serialization, framing, CRCs, and retries.**

Reliability is not added later.  
It is built up, one unglamorous layer at a time.

**Week 2 laid that layer.**

---

## License

This project is part of a 4-month embedded systems learning program.

---

**Author**: Antony (Tony) Mapfumo  
**Part of**: 4-Month Embedded Rust Learning Roadmap  
**Week**: 2 of 16
