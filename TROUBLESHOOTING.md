# Troubleshooting Guide

This document records all issues encountered during development and their solutions. Useful for similar projects.

## Compilation Errors

### 1. `I2CDisplayInterface` not found in scope

**Error Message:**
```
error[E0425]: cannot find type `I2CDisplayInterface` in this scope
  --> src/main.rs:66:32
   |
66 |     type LoraDisplay = Ssd1306<I2CDisplayInterface, DisplaySize128x64, ...>;
   |                                ^^^^^^^^^^^^^^^^^^^ not found in this scope
```

**Cause**: The type name changed between versions of `display-interface-i2c`. The crate uses `I2CInterface` not `I2CDisplayInterface`.

**Solution**: Use `I2CInterface` with the correct generic parameter:
```rust
// Before (incorrect)
type LoraDisplay = Ssd1306<I2CDisplayInterface, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

// After (correct)
type LoraDisplay = Ssd1306<I2CInterface<I2cProxy>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;
```

**Location**: [main.rs:67](src/main.rs#L67)

**Fix Applied**: Changed type alias to use `I2CInterface<I2cProxy>`

---

### 2. Field `cfgr` is private

**Error Message:**
```
error[E0616]: field `cfgr` of struct `stm32f4xx_hal::pac::rcc::RegisterBlock` is private
  --> src/main.rs:91:26
   |
91 |         let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
   |                          ^^^^ private field
```

**Cause**: stm32f4xx-hal 0.23.0 changed the RCC configuration API completely. The old `constrain()` → `cfgr` → `freeze()` pattern no longer works.

**Solution**: Use the new `freeze()` API with `Config`:
```rust
// Old API (0.20.x) - DOESN'T WORK
let rcc = dp.RCC.constrain();
let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
let timer = Timer::new(dp.TIM2, &clocks);

// New API (0.23.0) - CORRECT
use stm32f4xx_hal::rcc::Config;

let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(84.MHz()));
let timer = dp.TIM2.counter_hz(&mut rcc);
```

Also update all peripheral initializations:
```rust
// Old: pass &clocks
Serial::new(dp.UART4, (tx, rx), config, &clocks)
I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &clocks)

// New: pass &mut rcc
Serial::new(dp.UART4, (tx, rx), config, &mut rcc)
I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc)
```

**Locations Affected**:
- [main.rs:89](src/main.rs#L89) - RCC configuration
- [main.rs:107-112](src/main.rs#L107-L112) - UART4 initialization
- [main.rs:118](src/main.rs#L118) - I2C1 initialization
- [main.rs:144](src/main.rs#L144) - Timer initialization

**References**:
- [stm32f4xx-hal 0.23.0 examples](https://github.com/stm32-rs/stm32f4xx-hal/tree/master/examples)
- [RCC documentation](https://docs.rs/stm32f4xx-hal/0.23.0/stm32f4xx_hal/rcc/index.html)

---

### 3. Type annotations needed for closure

**Error Message:**
```
error[E0282]: type annotations needed
   --> src/main.rs:167:49
    |
167 |                         cx.shared.display.lock(|disp| {
    |                                                 ^^^^
168 |                             let _ = disp.clear(BinaryColor::Off);
    |                                     ---- type must be known at this point
```

**Cause**: Rust compiler cannot infer the type of the closure parameter when using RTIC's resource locking. The type needs to be explicit.

**Solution**: Add explicit type annotation to the closure parameter:
```rust
// Before (compiler can't infer)
cx.shared.display.lock(|disp| {
    let _ = disp.clear(BinaryColor::Off);
});

// After (explicit type)
cx.shared.display.lock(|disp: &mut LoraDisplay| {
    let _ = disp.clear(BinaryColor::Off);
});
```

**Location**: [main.rs:170](src/main.rs#L170)

**Why This Happens**: RTIC's lock mechanism uses generic closures, and without explicit types, the compiler cannot always deduce the concrete type from usage alone.

---

### 4. Borrow of moved value: `delay`

**Error Message:**
```
error[E0382]: borrow of moved value: `delay`
   --> src/main.rs:120:58
    |
 97 |         let mut delay = dp.TIM3.delay_ms(&mut rcc);
    |             --------- move occurs because `delay` has type `Delay<...>`, which does not implement the `Copy` trait
...
119 |         let sht31 = SHT3x::new(bus.acquire_i2c(), delay, ShtAddress::Low);
    |                                                   ----- value moved here
120 |         let mut bme680 = Bme680::init(bus.acquire_i2c(), &mut delay, I2CAddress::Secondary).unwrap();
    |                                                          ^^^^^^^^^^ value borrowed here after move
```

**Cause**: The `SHT3x::new()` function takes ownership of the delay (moves it), so it cannot be used by `bme680` afterward. Different APIs have different ownership requirements:
- SHT3x takes ownership: `fn new(i2c: I2C, delay: D, ...)`
- BME680 borrows: `fn init(i2c: I2C, delay: &mut D, ...)`

**Solution**: Create separate delay instances using different hardware timers:
```rust
// Create separate delay instances
let sht_delay = dp.TIM5.delay_us(&mut rcc);  // For SHT31 (will be owned)
let mut bme_delay = dp.TIM3.delay_us(&mut rcc);  // For BME680 (will be borrowed)

// Use different delays
let sht31 = SHT3x::new(bus.acquire_i2c(), sht_delay, ShtAddress::Low);
let mut bme680 = Bme680::init(bus.acquire_i2c(), &mut bme_delay, I2CAddress::Secondary).unwrap();
```

Update type aliases to match:
```rust
type ShtDelay = Delay<pac::TIM5, 1000000>;
type BmeDelay = Delay<pac::TIM3, 1000000>;
```

**Locations**:
- [main.rs:100-102](src/main.rs#L100-L102) - Delay creation
- [main.rs:62-63](src/main.rs#L62-L63) - Type definitions
- [main.rs:73-74](src/main.rs#L73-L74) - Shared struct types

**Key Lesson**: Always check whether functions take ownership (`T`) or borrow (`&T` or `&mut T`). When in doubt, check the function signature in the documentation.

---

## Runtime Errors

### 5. TryFromIntError in delay_ms

**Error Message:**
```
[ERROR] panicked at stm32f4xx-hal-0.23.0/src/timer.rs:960:55:
called `Result::unwrap()` on an `Err` value: TryFromIntError(())

Frame 12: delay<...> @ 0x00000000080002fa inline
       stm32f4xx-hal-0.23.0/src/timer.rs:230:9
Frame 13: delay_ms<...> @ 0x00000000080002fa inline
       stm32f4xx-hal-0.23.0/src/timer.rs:212:14
```

**Cause**: Timer prescaler calculation fails when trying to convert the system clock frequency to the delay timer frequency. The `delay_ms()` function uses 1 kHz (1000 Hz) as the timer frequency, which causes integer overflow/underflow in the prescaler calculation when using an 84 MHz HSI clock.

**Root Cause Analysis**:
- System clock: 84 MHz (84,000,000 Hz)
- `delay_ms()` target: 1 kHz (1,000 Hz)
- Prescaler calculation: `(84_000_000 / 1_000) - 1`
- This large division causes TryFromIntError in the type conversion

**Solution**: Use `delay_us()` with 1 MHz frequency instead:
```rust
// Don't use this (causes TryFromIntError with HSI clock)
let delay = dp.TIM3.delay_ms(&mut rcc);  // 1000 Hz frequency
type MyDelay = Delay<pac::TIM3, 1000>;

// Use this instead (works reliably)
let delay = dp.TIM3.delay_us(&mut rcc);  // 1_000_000 Hz frequency
type MyDelay = Delay<pac::TIM3, 1000000>;
```

**Important**: The delay object still supports millisecond operations via `delay.delay_ms(200)`, but the internal timer runs at microsecond precision (1 MHz).

**Affected Lines**:
- [main.rs:100-102](src/main.rs#L100-L102) - Delay creation
- [main.rs:62-63](src/main.rs#L62-L63) - Type definitions

**Why This Works**: A 1 MHz timer frequency (1,000,000 Hz) results in a smaller, more manageable prescaler value that fits within the integer type bounds.

**General Rule**: For STM32 timers with stm32f4xx-hal 0.23.0:
- ✅ Use `delay_us()` for new code
- ❌ Avoid `delay_ms()` unless using very low clock speeds

---

## Data Interpretation Issues

### 6. Incorrect Temperature Readings (2852°C in Brisbane)

**Symptom**: Display shows unrealistic temperatures like 2852°C for indoor environment in Brisbane, Australia (expected ~25-30°C).

**Initial Investigation**:
```rust
// Code showing raw value
let _ = core::write!(buf, "SHT: {:.1}C", meas.temperature);
// Displayed: "SHT: 2852C"
```

**Cause**: The `sht3x` crate (v0.1.1) returns temperature as `i32` in **centidegrees Celsius** (°C × 100), not degrees Celsius.

**Deep Dive - Source Code Analysis**:
```rust
// From https://github.com/miek/sht3x-rs/blob/master/src/lib.rs
pub struct Measurement {
    pub temperature: i32,  // ← centidegrees (°C × 100)
    pub humidity: u16,     // ← basis points (0.01% RH)
}

// Conversion formula from raw sensor data
const fn convert_temperature(raw: u16) -> i32 {
    -4500 + (17500 * raw as i32) / 65535
    // Returns centidegrees: e.g., 2852 = 28.52°C
}
```

**Understanding the Math**:
- Sensor returns 16-bit raw value (0-65535)
- Formula scales to -45.00°C to +130.00°C range
- Result is in centidegrees for integer precision
- Example: raw value 2852 → 28.52°C actual temperature

**Solution**: Convert centidegrees to degrees Celsius:
```rust
// Display conversion
let _ = core::write!(buf, "SHT: {:.1}C", meas.temperature as f32 / 100.0);
// Now displays: "SHT: 28.5C" ✓

// LoRa packet conversion
let _ = core::write!(pkt, "AT+SEND=0,16,S:{:.1}B:{:.1}G:{:.0}\r\n",
    meas.temperature as f32 / 100.0,  // Convert to °C
    t_bme,  // Already in °C
    gas);
```

**Affected Lines**:
- [main.rs:179-180](src/main.rs#L179-L180) - Display output
- [main.rs:196-197](src/main.rs#L196-L197) - LoRa transmission

**Key Lesson**: Always verify sensor data representation in driver documentation AND source code. Common patterns:
- **Raw ADC values**: Need calibration formula
- **Fixed-point scaled integers**: millidegrees (÷1000), centidegrees (÷100), decidegrees (÷10)
- **Floating-point values**: Direct use (but check units!)
- **Basis points**: Percentages × 100 (e.g., 5000 = 50.00%)

**Verification Steps**:
1. Check crate documentation at docs.rs
2. Read the source code (especially conversion functions)
3. Test with known reference values (ice water = 0°C, body temp ≈ 37°C)
4. Compare with other sensors if available

**References**:
- [sht3x-rs repository](https://github.com/miek/sht3x-rs)
- [sht3x-rs source code](https://github.com/miek/sht3x-rs/blob/master/src/lib.rs)
- [SHT3x datasheet](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Datasheets/Sensirion_Humidity_Sensors_SHT3x_Datasheet_digital.pdf)

---

### 7. Display Text Clipping on OLED

**Symptom**: First line of text "T:28.5C H:55%" appears truncated or cut off at the top of the SSD1306 128x64 OLED display.

**User Report**: "T:28.5C H:55% being truncated on OLED. Probably an offset thing"

**Initial Investigation**:
```rust
// Original code with text starting at y=0
Text::new(&buf, Point::new(0, 0), style).draw(disp).ok();   // Line 1
Text::new(&buf, Point::new(0, 12), style).draw(disp).ok();  // Line 2
Text::new(&buf, Point::new(0, 24), style).draw(disp).ok();  // Line 3
Text::new(&buf, Point::new(0, 36), style).draw(disp).ok();  // Line 4
```

**Cause**: Font ascenders and descenders are clipped when text baseline is positioned at y=0. The FONT_6X10 font has:
- Total height: 10 pixels
- Character glyphs extend both above and below the baseline
- Starting at y=0 causes the top portions of characters to render off-screen

**Root Cause Analysis**:
- Display coordinate system: (0,0) is top-left corner
- Font rendering: Baseline is at the y-coordinate, but glyphs extend upward
- FONT_6X10 needs ~8 pixels above baseline for tall characters (capitals, ascenders)
- Starting at y=0 provides no room for these upper pixels

**Solution**: Add proper top margin and adjust line spacing:
```rust
// Corrected code with 8-pixel top margin
Text::new(&buf, Point::new(0, 8), style).draw(disp).ok();   // Line 1 (y=8)
Text::new(&buf, Point::new(0, 20), style).draw(disp).ok();  // Line 2 (y=8+12)
Text::new(&buf, Point::new(0, 32), style).draw(disp).ok();  // Line 3 (y=20+12)
Text::new(&buf, Point::new(0, 44), style).draw(disp).ok();  // Line 4 (y=32+12)
```

**Layout Calculation**:
- Top margin: 8 pixels (prevents clipping)
- Line spacing: 12 pixels (10px font height + 2px gap)
- Line 1 baseline: y = 8
- Line 2 baseline: y = 8 + 12 = 20
- Line 3 baseline: y = 20 + 12 = 32
- Line 4 baseline: y = 32 + 12 = 44
- Bottom of last line: y = 44 + 10 = 54
- Remaining space: 64 - 54 = 10 pixels for future use

**Affected Lines**:
- [main.rs:183](src/main.rs#L183) - Line 1 display position
- [main.rs:188](src/main.rs#L188) - Line 2 display position
- [main.rs:193](src/main.rs#L193) - Line 3 display position
- [main.rs:198](src/main.rs#L198) - Line 4 display position

**Key Lesson**: When working with bitmap fonts on small displays:
1. **Understand font metrics**: Know the baseline, ascent, and descent of your font
2. **Test with varied characters**: Some characters (like 'T', 'H', 'g', 'y') extend further
3. **Add margins**: Always leave space at top and bottom edges
4. **Calculate spacing**: Font height + gap between lines
5. **Verify on hardware**: Text rendering issues may not be obvious in documentation

**General Formula for Multi-Line Text**:
```
TOP_MARGIN = font_ascent (typically 6-8 pixels for 10px font)
LINE_SPACING = font_height + inter_line_gap
Line_N_Y = TOP_MARGIN + (N - 1) × LINE_SPACING
```

**Font-Specific Guidelines**:
- **FONT_6X10**: Use y=8 start, 12px spacing (tested and working)
- **FONT_6X12**: Use y=10 start, 14px spacing
- **FONT_6X13**: Use y=11 start, 15px spacing

**References**:
- [SSD1306 coordinate system](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [embedded-graphics text positioning](https://docs.rs/embedded-graphics/latest/embedded_graphics/text/index.html)
- [FONT_6X10 documentation](https://docs.rs/embedded-graphics/latest/embedded_graphics/mono_font/ascii/constant.FONT_6X10.html)

---

## Hardware Verification

### 8. Verifying LoRa Transmission with Logic Analyzer

**Context**: When implementing LoRa transmission, it's not enough to see the OLED display update - you need to verify that actual UART data is being sent to the LoRa module.

**Problem**: How do you prove that the STM32 is correctly transmitting AT commands to the RYLR998 LoRa module?

**Solution**: Use a logic analyzer to capture and decode the UART transmission.

**Hardware Setup**:
```
Logic Analyzer Connections:
  CH0 (or any channel) → PC10 (UART4 TX - STM32 to LoRa module)
  GND                  → GND (common ground)
  [Optional] CH1       → PC11 (UART4 RX - LoRa module responses)
```

**PulseView Configuration**:
1. **Sample Rate**: 1-4 MHz (for 115200 baud UART)
2. **Protocol Decoder**: UART (Async Serial)
3. **Decoder Settings**:
   - RX channel: CH0 (connected to TX pin)
   - Baud rate: `115200`
   - Data bits: `8`
   - Parity: `none`
   - Stop bits: `1`
   - Bit order: `lsb-first`
   - Format: `ascii`
   - RX polarity: `idle high`
4. **Trigger**: Falling edge on CH0 (optional, but helpful)

**Example Capture - Successful Transmission**:

```
Decoded UART Output:
AT+SEND=0,25,T:29.8H:62.1G:52174#0308\r\n

Raw Hex:
41 54 2B 53 45 4E 44 3D 30 2C 32 35 2C 54 3A 32
39 2E 38 48 3A 36 32 2E 31 47 3A 35 32 31 37 34
23 30 33 30 38 0D 0A
```

**Packet Breakdown**:
| Portion | Value | Meaning |
|---------|-------|---------|
| `AT+SEND=0,25,` | Command header | Send to address 0, payload 25 bytes |
| `T:29.8` | Temperature | 29.8°C from SHT31 sensor |
| `H:62.1` | Humidity | 62.1% RH from SHT31 sensor |
| `G:52174` | Gas resistance | 52,174Ω from BME680 sensor |
| `#0308` | Packet counter | Sequential packet number (308) |
| `\r\n` | Terminator | Carriage return + line feed (0x0D 0x0A) |

**Key Observations from Capture**:
- ✅ Baud rate is correct (115200) - characters decode cleanly
- ✅ AT command format is valid
- ✅ Sensor data matches OLED display values
- ✅ Packet counter increments with each transmission
- ✅ Proper line termination (`\r\n`)

**Common Frame Errors**:
If you see "Frame error" warnings in the decoder output:
- **At start of capture**: Usually harmless - occurred before trigger stabilized
- **During transmission**: Check connections, verify baud rate, ensure clean power
- **Intermittent**: May indicate noise, poor connections, or timing issues

**What This Proves**:
1. STM32 UART is functioning correctly
2. Correct baud rate configuration (115200)
3. Data is reaching the LoRa module's RX pin
4. Packet format is correct and complete
5. Button/timer triggers are working as expected

**Next Steps After Verification**:
- If you also capture PC11 (UART RX), you should see `+OK\r\n` responses from the LoRa module
- For end-to-end verification, set up a second node to receive and decode these LoRa transmissions
- For range testing, this packet counter allows you to calculate packet loss percentage

**Tools Used**:
- **Hardware**: DIGISHUO 24MHz 8CH Logic Analyzer (or similar)
- **Software**: PulseView (open-source logic analyzer GUI)
- **Protocol**: UART/Async Serial decoder

**Debugging Tip**: If you don't see clean UART data:
1. Verify connection to PC10 (not PC11)
2. Check GND is connected
3. Increase sample rate (try 4 MHz)
4. Adjust trigger settings
5. Verify baud rate in both firmware and decoder match exactly

**Code Location**: [main.rs:244-257](src/main.rs#L244-L257)

---

## LoRa Communication Issues

### 9. Node 2 Only Receives First Packet, Then Stops

**Context**: System uses RYLR998 LoRa modules with UART AT commands. Node 1 (transmitter) sends sensor data to Node 2 (receiver) every 10 seconds.

**Problem**: Node 2 successfully receives and displays the first packet, then receives NO subsequent packets even though Node 1 continues transmitting successfully.

**Symptoms**:
- Node 1 logs show continuous successful transmissions (packets #1, #2, #3, etc.)
- Node 2 logs show ONLY the first packet received
- Node 2 UART interrupts stop firing after processing first packet
- Hardware and wiring verified working (first packet proves this)
- Issue persists even with previously-working committed code after power cycle

**Root Cause**: UART Overrun Error (ORE) flag not properly cleared, blocking future interrupts.

RYLR998 LoRa modules are **command-response devices**, not fire-and-forget:
- After `AT+SEND`, module responds with `+OK\r\n` and `+SENT\r\n`
- These responses fill the UART RX buffer if not drained
- When buffer overruns, the ORE flag is set in the Status Register
- **If ORE is not cleared, UART stops triggering RX interrupts entirely**
- This causes both transmitter and receiver to stop working after first packet

**Failed Approaches** (what didn't work):
1. ❌ Moving display I2C operations to timer interrupt (architectural improvement, but didn't fix the core issue)
2. ❌ Changing from `if let Ok(byte)` to `while let Ok(byte)` for buffer draining (helped but incomplete)
3. ❌ Fixing AT command format to send `\r\n` separately (correct, but not the blocker)
4. ❌ Checking ORE flag BEFORE draining data (consumed valid data bytes)

**Correct Solution**: Drain data FIRST, then clear error flags AFTER

```rust
#[task(binds = UART4, shared = [lora_uart, last_packet, packets_received], local = [rx_buffer])]
fn uart4_handler(mut cx: uart4_handler::Context) {
    let mut should_process = false;
    let mut bytes_read = 0u16;

    cx.shared.lora_uart.lock(|uart| {
        // 1. FIRST: Drain all available data bytes
        while let Ok(byte) = uart.read() {
            bytes_read += 1;
            if cx.local.rx_buffer.len() < RX_BUFFER_SIZE {
                let _ = cx.local.rx_buffer.push(byte);
            }
            if byte == b'\n' {
                should_process = true;
            }
        }

        // 2. AFTER draining: Check and clear error flags
        // At this point, RXNE is clear, so reading DR won't consume data
        let uart_ptr = unsafe { &*pac::UART4::ptr() };
        let sr = uart_ptr.sr().read();

        if sr.ore().bit_is_set() || sr.nf().bit_is_set() || sr.fe().bit_is_set() {
            // Clear errors by reading DR (FIFO is empty now)
            let _ = uart_ptr.dr().read();
            defmt::warn!("UART4 errors cleared (ORE={} NF={} FE={})",
                sr.ore().bit_is_set(), sr.nf().bit_is_set(), sr.fe().bit_is_set());
        }
    });

    // 3. Process message OUTSIDE uart lock
    if should_process {
        // ... parse and handle message
        cx.local.rx_buffer.clear();
    }
}
```

**Why This Order Matters**:

1. **Reading SR then DR clears error flags** (per STM32 reference manual)
2. **If you read DR when RXNE is set, you consume a data byte**
3. **By draining data FIRST, then checking errors, DR is empty when we clear flags**
4. **This prevents data loss while still clearing ORE to re-enable interrupts**

**Critical STM32 UART Details**:

| Flag | Meaning | How to Clear | Impact if Not Cleared |
|------|---------|--------------|----------------------|
| ORE  | Overrun Error - new byte arrived while RXNE still set | Read SR, then read DR | **UART stops triggering interrupts** |
| NE   | Noise Error - noise detected on RX line | Read SR, then read DR | Corrupted data |
| FE   | Framing Error - stop bit not detected | Read SR, then read DR | Corrupted data |

**Additional Fixes Applied**:

1. **CRITICAL - Clear ORE flag during initialization**:
   ```rust
   // After AT command configuration, BEFORE enabling interrupt
   send_at_command(&mut lora_uart, "AT+PARAMETER=7,9,1,7");

   // Flush any pending responses
   while lora_uart.read().is_ok() {}

   // Explicitly clear any error flags - if ORE is set, interrupts won't fire!
   let uart_ptr = unsafe { &*pac::UART4::ptr() };
   let sr = uart_ptr.sr().read();
   if sr.ore().bit_is_set() || sr.nf().bit_is_set() || sr.fe().bit_is_set() {
       let _ = uart_ptr.dr().read();
       defmt::info!("INIT: Cleared error flags (ORE={} NF={} FE={})",
           sr.ore().bit_is_set(), sr.nf().bit_is_set(), sr.fe().bit_is_set());
   }

   // NOW it's safe to enable interrupt
   lora_uart.listen(SerialEvent::RxNotEmpty);
   ```

2. **Node 1 - Drain LoRa module responses**:
   ```rust
   // After AT+SEND, module responds with +OK and +SENT
   // Must drain these or buffer fills and ORE blocks future TX
   #[task(binds = UART4, shared = [lora_uart])]
   fn uart4_handler(mut cx: uart4_handler::Context) {
       cx.shared.lora_uart.lock(|uart| {
           let mut drained = 0;
           while uart.read().is_ok() {
               drained += 1;
           }
           // ... then check and clear error flags
       });
   }
   ```

3. **Node 2 - Separate UART and Display operations**:
   - UART interrupt: Fast receive and parse only (no I2C operations)
   - Timer interrupt: Update display with parsed data
   - This prevents slow I2C display flush (~10-50ms) from blocking UART interrupts

4. **AT Command Format**:
   ```rust
   // WRONG: Embedded \r\n gets sent byte-by-byte
   let _ = core::write!(pkt, "AT+SEND=2,25,...\r\n");
   for b in pkt.as_bytes() { uart.write(*b); }

   // CORRECT: Send \r\n separately after command
   let _ = core::write!(pkt, "AT+SEND=2,25,...");
   for b in pkt.as_bytes() { uart.write(*b); }
   uart.write(b'\r');
   uart.write(b'\n');
   ```

**Testing the Fix**:

Before fix:
```
Node 1: TX #1, #2, #3, #4...  (transmitting continuously)
Node 2: RX #1 ... (then nothing, UART interrupts stop)
```

After fix:
```
Node 1: TX #1, #2, #3, #4...  (transmitting continuously)
Node 2: RX #1, #2, #3, #4...  (receiving all packets)
```

**Key Learnings**:

1. **UART LoRa modules are NOT fire-and-forget** - they send responses that must be drained
2. **ORE flag MUST be cleared** or UART peripheral locks up entirely
3. **Order matters**: Drain data BEFORE clearing errors to avoid data loss
4. **UART interrupts arriving one-byte-at-a-time is suspicious** - indicates FIFO not filling or interrupts too slow
5. **Always check both TX and RX sides** - both can have buffer issues
6. **Power cycling can mask the issue** - makes it seem intermittent when it's actually systematic

**Code Locations**:
- Node 1 (Transmitter): [src/main.rs:323-348](src/main.rs#L323-L348)
- Node 2 (Receiver): [src/bin/node2.rs:275-342](src/bin/node2.rs#L275-L342)

**References**:
- [STM32F446 Reference Manual - USART section 30.6.1 (Status Register)](https://www.st.com/resource/en/reference_manual/dm00135183.pdf)
- [RYLR998 AT Command Guide](https://reyax.com/products/rylr998/)
- GitHub issues reporting similar UART ORE problems with STM32

---

### 10. LoRa Message Truncation - Buffer Size Not LoRa Protocol Limit

**Context**: After fixing the ORE flag issue (#9), Node 2 appeared to still have problems - not receiving packets completely or displaying data. Investigation initially focused on LoRa payload limits.

**Symptom**: Messages appeared to be truncated at ~28-30 bytes, leading to incomplete packet reception and `should_process` never becoming true (missing `\n` terminator).

**Initial Theory (INCORRECT)**: LoRa spreading factor (SF7) was limiting payload to ~30 bytes over the air.

**Actual Root Cause**: `heapless::String<32>` and `String<64>` buffers in Rust code were too small to hold complete AT commands with full sensor data payloads.

**Hardware Specifications - RYLR998 LoRa Module**:
- **Maximum Payload**: 240 bytes per message
- **AT Command Format**: `AT+SEND=<addr>,<len>,<data>`
- **Response Format**: `+RCV=<addr>,<len>,<data>,<rssi>,<snr>\r\n`
- **Protocol**: Not subject to LoRaWAN 51-byte limit (uses proprietary protocol)

**The Real Problem - Software Buffer Constraints**:

Node 1 (Transmitter):
```rust
// PROBLEM: Buffer too small for command string
let mut pkt: String<64> = String::new();
let _ = core::write!(pkt, "AT+SEND=2,25,T:{:.1}H:{:.1}G:{:.0}#{:04}",
    temp_c, humid_pct, gas, packet_counter);
// String too long → core::write! returns Err → packet never sent!
```

Payload: `T:28.8H:64.1G:104102#0001` (25 bytes)
Full command: `AT+SEND=2,25,T:28.8H:64.1G:104102#0001` (~38 bytes)
**Buffer capacity: 64 bytes** → Barely fits, no room for longer gas values!

Node 2 (Receiver):
```rust
const RX_BUFFER_SIZE: usize = 128;  // Too small!
```

Expected RX: `+RCV=1,25,T:28.8H:64.1G:104102#0001,-20,12\r\n` (~44 bytes)
**Buffer capacity: 128 bytes** → Appears OK but doesn't leave margin for full 240-byte payloads

**How Buffer Overflow Manifests**:
1. **Silent failure**: `core::write!()` returns `Err` but code doesn't check result
2. **Partial strings**: Buffer fills to capacity, rest is dropped
3. **Missing terminators**: If `\n` doesn't fit in buffer, message never processes
4. **Inconsistent behavior**: Works with short test payloads, fails with real sensor data

**Solution**: Increase buffer sizes to support full RYLR998 capability (240 bytes)

Node 1 (Transmitter):
```rust
// Build payload separately
let mut payload: String<128> = String::new();
let _ = core::write!(payload, "T:{:.1}H:{:.1}G:{:.0}#{:04}",
    temp_c, humid_pct, gas, packet_counter);

// Build AT command with dynamic length
let mut cmd: String<255> = String::new();
let _ = core::write!(cmd, "AT+SEND=2,{},{}",
    payload.len(), payload.as_str());
```

Node 2 (Receiver):
```rust
const RX_BUFFER_SIZE: usize = 255;  // Support full 240-byte payload + overhead
```

**Why This Wasn't Obvious**:
1. Test payload `TEST#0001` was only 10 bytes - fit easily in small buffers
2. No error checking on `core::write!()` results
3. UART appeared to work correctly (it was!)
4. Focus on LoRa protocol limits instead of application buffers
5. Misleading search results about LoRaWAN 51-byte limit (doesn't apply to RYLR998)

**Key Differences - LoRaWAN vs RYLR998**:

| Feature | LoRaWAN | RYLR998 (Reyax) |
|---------|---------|-----------------|
| Max Payload | 51-222 bytes (SF-dependent) | **240 bytes** (hardware limit) |
| Protocol | LoRaWAN standard | Proprietary AT commands |
| Network | Infrastructure mode | Peer-to-peer |
| Duty Cycle | EU 868: 1% limit | User-managed |
| Addressing | DevEUI/AppEUI | Simple 0-65535 addresses |

**Diagnostic Observations That Led to Discovery**:

From external analysis provided by user:
1. "drained 1 bytes" logs → UART working correctly, reading `+OK\r\n` responses
2. Node 2 showing `has_packet=false` → Never finding message terminator
3. AT command hardcoded length (`AT+SEND=2,10,...`) → Should be dynamic
4. String<32> buffers throughout init code → Pattern of undersized buffers

**Testing After Fix**:
```
Node 1 Log:
LoRa TX [AUTO]: AT+SEND=2,25,T:28.8H:64.1G:104102#0001 (payload: 25 bytes)
Transmission complete - packet #1

Node 2 Log:
N2 UART interrupt fired
N2 UART: drained 44 bytes, should_process=true, buf_len=44
LoRa RX: +RCV=1,25,T:28.8H:64.1G:104102#0001,-20,12
Parsed - T:28.8 H:64.1 G:104102 Pkt:1 RSSI:-20 SNR:12
```

**General Lessons**:

1. **Always check buffer sizes against maximum payload** - don't assume "reasonable" values
2. **Hardware limits ≠ Software limits** - your buffers can be the real constraint
3. **Check `core::write!()` return values** - silent failures are hard to debug
4. **Use dynamic length calculations** - don't hardcode payload sizes
5. **Research the ACTUAL hardware** - don't assume all LoRa modules follow LoRaWAN specs
6. **Test with maximum-size payloads** - short test data won't reveal buffer issues
7. **Embedded Rust: `heapless` sizes are HARD LIMITS** - no dynamic growth like std::String

**Code Locations**:
- Node 1: [src/main.rs:307-330](src/main.rs#L307-L330) - Payload and command building
- Node 2: [src/bin/node2.rs:33](src/bin/node2.rs#L33) - RX_BUFFER_SIZE constant

**External Resources That Helped**:
- RYLR998 datasheet confirming 240-byte limit
- User analysis identifying String<32> as bottleneck
- Understanding that RYLR998 is NOT LoRaWAN (different protocol stack entirely)

---

## Best Practices Learned

1. **Check HAL version compatibility**: API changes between minor versions can be significant
2. **Read driver source code**: Documentation may not specify data representation clearly
3. **One timer per delay consumer**: Avoid ownership conflicts by allocating separate timers
4. **Use microsecond timers**: Better compatibility with prescaler calculations (use `delay_us()`)
5. **Test with real hardware early**: Compilation success ≠ correct behavior
6. **Verify units immediately**: Don't assume sensor values are in expected units
7. **Keep type aliases updated**: They document the system architecture
8. **Use defmt for debugging**: Much more efficient than ITM/semihosting
9. **Understand sensor characteristics**: Different sensors excel at different measurements (sensor fusion)
10. **Account for font metrics**: Add proper margins for text rendering to prevent clipping
11. **Plan display layouts**: Calculate spacing and verify sufficient room for future expansion

## Quick Reference

### Common stm32f4xx-hal 0.23.0 Patterns

```rust
// Must import Config
use stm32f4xx_hal::rcc::Config;

// RCC setup with HSI
let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(84.MHz()));

// GPIO split (requires &mut rcc)
let gpioa = dp.GPIOA.split(&mut rcc);

// Delays (use delay_us, not delay_ms!)
let delay = dp.TIM3.delay_us(&mut rcc);
type MyDelay = Delay<pac::TIM3, 1000000>;  // 1 MHz

// I2C initialization
let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);

// Serial/UART initialization
let uart = Serial::new(
    dp.UART4,
    (tx, rx),
    SerialConfig::default().baudrate(115200.bps()),
    &mut rcc
).unwrap();

// Timer/Counter initialization
let timer = dp.TIM2.counter_hz(&mut rcc);
timer.start(1.Hz()).unwrap();
timer.listen(Event::Update);
```

### Debugging Steps

1. **Compilation error**:
   - Check stm32f4xx-hal version and API docs
   - Search GitHub issues for the HAL crate
   - Look at official examples in the HAL repository

2. **Runtime panic**:
   - Enable defmt logging with RTT
   - Check stack trace for panic location
   - Verify clock configuration
   - Check timer/prescaler settings

3. **Wrong sensor values**:
   - Verify data type and conversion in driver source code
   - Test with known reference (ice water, room temp)
   - Compare multiple sensors
   - Check sensor datasheet for value ranges

4. **I2C issues**:
   - Verify pull-up resistors (typically 4.7kΩ)
   - Check bus speed (100 kHz is safest)
   - Verify device addresses with i2cdetect
   - Ensure proper power supply

5. **Timer issues**:
   - Use `delay_us()` instead of `delay_ms()`
   - Verify prescaler calculations
   - Check clock source configuration

### Common Gotchas

| Issue | Symptom | Solution |
|-------|---------|----------|
| Moved value | Borrow after move error | Use separate instances or clone |
| Private field | Can't access `.cfgr` | Update to new API with `Config` |
| TryFromIntError | Panic in timer setup | Use `delay_us()` not `delay_ms()` |
| Wrong sensor values | Unrealistic readings | Check data scaling (÷100, ÷1000, etc) |
| Type inference | Closure type needed | Add explicit type annotation |
| I2C bus sharing | Ownership conflicts | Use `shared-bus` crate |
| Display text clipping | Top line cut off | Start at y=8 with 12px spacing for FONT_6X10 |

---

## Version Information

This troubleshooting guide was created for:
- **stm32f4xx-hal**: 0.23.0
- **RTIC**: 1.1
- **Rust Edition**: 2021
- **Target**: thumbv7em-none-eabihf

API changes in future versions may require updates to these solutions.
