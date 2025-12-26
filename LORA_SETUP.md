# LoRa Module Configuration Guide

## RYLR998 Network Setup for Node 1 (Transmitter) and Node 2 (Receiver)

Before flashing the firmware, both LoRa modules must be configured with matching network parameters.

## Prerequisites

- Two RYLR998 LoRa modules
- USB-to-UART adapter (or use Nucleo's virtual COM port)
- Serial terminal (minicom, screen, or PuTTY)

## Network Parameters

For this project, we'll use:

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Network ID** | `18` | Private network identifier (0-255) |
| **Frequency** | `915` | ISM band in MHz (915 for US, 868 for EU, 433 for Asia) |
| **Bandwidth** | `9` | 500 kHz (9 = max throughput) |
| **Spreading Factor** | `7` | Fastest data rate (7-12, lower = faster) |
| **Coding Rate** | `1` | 4/5 error correction |
| **Transmit Power** | `15` | 15 dBm (default, max 20 dBm) |

### Address Assignment

| Node | Address | Role |
|------|---------|------|
| **Node 1 (N1)** | `1` | Transmitter (sensor data source) |
| **Node 2 (N2)** | `2` | Receiver (gateway/display) |

## Configuration Commands

### Step 1: Connect to LoRa Module

**Option A: Direct USB-UART Connection**
```bash
# Linux/macOS
screen /dev/ttyUSB0 115200

# Or with minicom
minicom -D /dev/ttyUSB0 -b 115200

# Windows (PuTTY)
# COM port, 115200 baud, 8N1
```

**Option B: Through Nucleo Board**
```bash
# Find Nucleo's virtual COM port
ls /dev/ttyACM*

# Connect (replace X with your port number)
screen /dev/ttyACMX 115200
```

### Step 2: Test Communication

Type the following command (should respond with `+OK`):
```
AT
```

Expected response:
```
+OK
```

### Step 3: Configure Node 1 (Transmitter)

Execute these commands one at a time:

```bash
# Set address to 1
AT+ADDRESS=1
# Response: +OK

# Set network ID to 18
AT+NETWORKID=18
# Response: +OK

# Set frequency to 915 MHz (adjust for your region)
AT+BAND=915000000
# Response: +OK

# Set bandwidth to 500 kHz (9 = 500kHz)
AT+PARAMETER=7,9,1,7
# Response: +OK
# Format: AT+PARAMETER=<SF>,<BW>,<CR>,<Preamble>
#   SF (Spreading Factor): 7
#   BW (Bandwidth): 9 (500 kHz)
#   CR (Coding Rate): 1 (4/5)
#   Preamble: 7

# Optional: Increase TX power to 20 dBm (max)
AT+CRFOP=20
# Response: +OK

# Verify configuration
AT+ADDRESS?
# Response: +ADDRESS=1

AT+NETWORKID?
# Response: +NETWORKID=18

AT+BAND?
# Response: +BAND=915000000

AT+PARAMETER?
# Response: +PARAMETER=7,9,1,7
```

### Step 4: Configure Node 2 (Receiver)

**Important**: Only change the address; all other parameters must match Node 1!

```bash
# Set address to 2 (different from Node 1)
AT+ADDRESS=2
# Response: +OK

# Set network ID to 18 (SAME as Node 1)
AT+NETWORKID=18
# Response: +OK

# Set frequency to 915 MHz (SAME as Node 1)
AT+BAND=915000000
# Response: +OK

# Set parameters (SAME as Node 1)
AT+PARAMETER=7,9,1,7
# Response: +OK

# Optional: Set RX power
AT+CRFOP=20
# Response: +OK

# Verify configuration
AT+ADDRESS?
# Response: +ADDRESS=2

AT+NETWORKID?
# Response: +NETWORKID=18
```

## Regional Frequency Bands

Choose the appropriate frequency for your region:

| Region | Frequency | AT Command |
|--------|-----------|------------|
| **North America** | 915 MHz | `AT+BAND=915000000` |
| **Europe** | 868 MHz | `AT+BAND=868000000` |
| **Asia/Africa** | 433 MHz | `AT+BAND=433000000` |

**Note**: Verify local regulations before transmitting. Some regions restrict LoRa usage.

## Firmware Updates Required

### Node 1 (Transmitter) - main.rs

The current code sends to address `0` (broadcast). Update to send to Node 2's address:

**Current** (line 247):
```rust
let _ = core::write!(pkt, "AT+SEND=0,25,T:{:.1}H:{:.1}G:{:.0}#{:04}\r\n",
```

**Should be**:
```rust
let _ = core::write!(pkt, "AT+SEND=2,25,T:{:.1}H:{:.1}G:{:.0}#{:04}\r\n",
```
Change `0` to `2` to target Node 2's address.

### Node 2 (Receiver) - node2.rs

No changes needed - receiver listens for all incoming messages on the network.

## Testing Communication

### Option 1: Manual Test (Before Firmware)

**On Node 1:**
```
AT+SEND=2,5,HELLO
```

**On Node 2 (should receive):**
```
+RCV=1,5,HELLO,-50,40
```
Format: `+RCV=<Sender Address>,<Length>,<Data>,<RSSI>,<SNR>`

### Option 2: With Firmware Running

1. Flash Node 1 with transmitter firmware
2. Flash Node 2 with receiver firmware
3. Power both nodes
4. Node 1's OLED should show: `N1 TX:AUTO #0001`
5. Node 2's OLED should display received sensor data with RSSI/SNR

## Expected Message Format

**Node 1 transmits (every 10 seconds or on button press):**
```
AT+SEND=2,25,T:29.8H:62.1G:52174#0308\r\n
```

**Node 2 receives:**
```
+RCV=1,25,T:29.8H:62.1G:52174#0308,-45,38
```
Where:
- `-45` = RSSI (Received Signal Strength Indicator) in dBm
- `38` = SNR (Signal-to-Noise Ratio) in dB

## Troubleshooting

### No Response to AT Commands

- Check baud rate (must be 115200)
- Verify TX/RX wiring (TX → RX, RX → TX)
- Ensure common ground
- Try pressing module reset button

### Node 2 Not Receiving

- Verify both modules have same Network ID (AT+NETWORKID?)
- Verify both modules have same frequency band (AT+BAND?)
- Verify both modules have same parameters (AT+PARAMETER?)
- Check addresses are different (Node 1 = 1, Node 2 = 2)
- Ensure antennas are connected
- Check power supply (LoRa modules need 3.3V, up to 500mA peak)

### Weak Signal (Low RSSI)

- Add/upgrade antenna (default wire antenna is poor)
- Increase TX power: `AT+CRFOP=20` (max 20 dBm)
- Reduce distance between nodes
- Avoid metal obstacles between nodes
- Check antenna connections

### Commands Don't Persist After Power Cycle

RYLR998 modules save settings to flash automatically. If settings reset:
- Module may be defective
- Check power supply stability (brownouts can corrupt flash)

## Quick Reference Card

### Essential AT Commands

| Command | Description | Example |
|---------|-------------|---------|
| `AT` | Test communication | `AT` → `+OK` |
| `AT+ADDRESS=<n>` | Set module address (0-255) | `AT+ADDRESS=1` |
| `AT+NETWORKID=<n>` | Set network ID (0-255) | `AT+NETWORKID=18` |
| `AT+BAND=<freq>` | Set frequency in Hz | `AT+BAND=915000000` |
| `AT+PARAMETER=<SF>,<BW>,<CR>,<P>` | Set RF parameters | `AT+PARAMETER=7,9,1,7` |
| `AT+CRFOP=<power>` | Set TX power (0-20 dBm) | `AT+CRFOP=20` |
| `AT+SEND=<addr>,<len>,<data>` | Send data | `AT+SEND=2,5,HELLO` |
| `AT+ADDRESS?` | Query address | Returns `+ADDRESS=1` |
| `AT+RESET` | Reset module | Reboots module |

### Parameter Values

**Bandwidth** (BW parameter):
- `7` = 125 kHz
- `8` = 250 kHz
- `9` = 500 kHz (fastest, shortest range)

**Spreading Factor** (SF parameter):
- `7` = Fastest data rate, shortest range
- `12` = Slowest data rate, longest range

**Coding Rate** (CR parameter):
- `1` = 4/5 (least overhead)
- `4` = 4/8 (most robust)

## Next Steps

After configuration:
1. Update Node 1 firmware to send to address `2` instead of `0`
2. Build and flash Node 1: `cargo build --release`
3. Build and flash Node 2: `cargo build --release --bin node2`
4. Power both nodes and verify communication
5. Test range with RSSI/SNR monitoring
6. Document packet loss rates at various distances

---

**Configuration Checklist:**

- [ ] Both modules respond to `AT` command
- [ ] Node 1 address set to `1`
- [ ] Node 2 address set to `2`
- [ ] Both modules on same Network ID (`18`)
- [ ] Both modules on same frequency (915/868/433 MHz)
- [ ] Both modules have matching parameters (7,9,1,7)
- [ ] Manual `AT+SEND` test successful
- [ ] Node 1 firmware updated to send to address `2`
- [ ] Both firmwares built and flashed
- [ ] End-to-end communication verified
