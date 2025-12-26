# Project Notes - Week 2: LoRa Sensor Fusion

## Learning Objectives

This week focused on:
1. Multi-sensor I2C bus management
2. RTIC (Real-Time Interrupt-driven Concurrency) framework
3. Hardware abstraction layer patterns in embedded Rust
4. Sensor data fusion and display
5. Serial communication with LoRa modules

## Key Concepts

### 1. I2C Bus Sharing

**Problem**: Multiple devices (SHT31, BME680, SSD1306) need to share a single I2C bus, but Rust's ownership system prevents multiple mutable references.

**Solution**: `shared-bus` crate with Cortex-M mutex
- Creates a static bus manager
- Provides `I2cProxy` instances for each device
- Thread-safe access through `CortexMMutex`

```rust
type BusManager = shared_bus::BusManager<CortexMMutex<I2cCompat<MyI2c>>>;
let bus: &'static BusManager = shared_bus::new_cortexm!(I2cCompat<MyI2c> = i2c_compat).unwrap();

// Each device gets its own proxy
let sht31 = SHT3x::new(bus.acquire_i2c(), sht_delay, ShtAddress::Low);
let bme680 = Bme680::init(bus.acquire_i2c(), &mut bme_delay, I2CAddress::Secondary).unwrap();
let interface = I2CInterface::new(bus.acquire_i2c(), 0x3C, 0x40);
```

### 2. Embedded HAL Compatibility Bridge

**Issue**: `stm32f4xx-hal` 0.23.0 implements `embedded-hal` 1.0, but many sensor drivers still use `embedded-hal` 0.2.7.

**Solution**: Create a compatibility wrapper that implements the old traits on the new types:

```rust
pub struct I2cCompat<I2C>(pub I2C);

impl<I2C> embedded_hal_0_2::blocking::i2c::Write for I2cCompat<I2C>
where I2C: embedded_hal::i2c::I2c {
    type Error = I2C::Error;
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.write(addr, bytes)
    }
}
```

This pattern allows legacy drivers to work with modern HAL implementations.

### 3. RTIC Resource Management

RTIC provides compile-time guaranteed safe resource sharing:

**Shared Resources** (can be accessed by multiple tasks with locking):
```rust
#[shared]
struct Shared {
    lora_uart: Serial<pac::UART4>,
    display: LoraDisplay,
    sht31: SHT3x<I2cProxy, ShtDelay>,
    bme680: Bme680<I2cProxy, BmeDelay>,
}
```

**Local Resources** (exclusive to one task):
```rust
#[local]
struct Local {
    led: Pin<'A', 5, Output>,
    timer: CounterHz<pac::TIM2>,
    bme_delay: BmeDelay,
}
```

Access pattern:
```rust
cx.shared.display.lock(|disp| {
    // Safe mutable access within this closure
});
```

### 4. Timer Delay Providers

Different sensors need their own delay implementations:
- SHT31 driver takes ownership of its delay (cannot share)
- BME680 driver borrows delay (can be shared via Local)

Solution: Allocate separate hardware timers:
- TIM3 @ 1 MHz for BME680 (stored in Local)
- TIM5 @ 1 MHz for SHT31 (owned by sensor)

Generic type: `Delay<TIMx, FREQ>` where FREQ = 1_000_000 for microsecond precision.

### 5. Sensor Data Types

**Important**: Sensor drivers may use scaled integer representations instead of floats:

- **SHT3x**: Temperature is `i32` in **centidegrees Celsius** (°C × 100)
  - Raw value: 2852 → Actual: 28.52°C
  - Convert: `temperature as f32 / 100.0`
  - Source code shows: `const fn convert_temperature(raw: u16) -> i32 { -4500 + (17500 * raw as i32) / 65535 }`

- **SHT3x**: Humidity is `u16` in **basis points** (% RH × 100)
  - Raw value: 5520 → Actual: 55.20% RH
  - Convert: `humidity as f32 / 100.0`
  - Provides 0.01% resolution for precise humidity measurements

- **BME680**: Uses `f32` directly via `.temperature_celsius()` method

Always check the driver documentation or source code for data representation!

### 6. stm32f4xx-hal 0.23.0 API Changes

The HAL underwent significant API changes from 0.20.x to 0.23.0:

**Old API** (0.20.x):
```rust
let rcc = dp.RCC.constrain();
let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
let timer = Timer::new(dp.TIM2, &clocks);
```

**New API** (0.23.0):
```rust
let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(84.MHz()));
let timer = dp.TIM2.counter_hz(&mut rcc);
```

Key changes:
- `freeze()` now takes a `Config` and returns mutable `Rcc`
- Peripherals initialized with `&mut rcc` instead of `&clocks`
- Direct peripheral methods instead of wrapper constructors
- Must import `rcc::Config`

## Code Patterns

### Heapless String Formatting

Embedded systems can't use `std::String`. Use `heapless::String` instead:

```rust
use heapless::String;
use core::fmt::Write;

let mut buf: String<64> = String::new();  // 64-byte stack buffer
let _ = core::write!(buf, "Temp: {:.1}C", temperature);
```

### Error Handling in `no_std`

Without `std`, we can't use `?` with `Box<dyn Error>`. Common patterns:

```rust
// Ignore errors (when failure is acceptable)
let _ = sensor.read();

// Propagate with Result
if let Ok(data) = sensor.read() {
    // Handle data
}

// Unwrap in init (panic on failure is acceptable during setup)
let sensor = Sensor::init(i2c).unwrap();
```

### Non-blocking UART Writes

`nb::block!` macro handles the non-blocking HAL pattern:

```rust
for byte in data.as_bytes() {
    let _ = nb::block!(uart.write(*byte));
}
```

## Design Decisions

1. **1 Hz Sampling Rate**: Slow enough for debugging, fast enough for environmental monitoring
2. **Separate Timers**: Avoids ownership conflicts between sensors needing delay providers
3. **BME680 in Forced Mode**: Lower power, explicit control over measurements
4. **Buffered Display Mode**: Faster updates, single flush operation
5. **Inline Error Handling**: `let _ = ...` pattern for non-critical operations
6. **HSI Clock Source**: No external crystal required, simplifies hardware

### 7. Sensor Fusion Strategy

**Design Decision**: Use the best sensor for each measurement type rather than averaging or preferring a single device.

| Measurement | Sensor | Accuracy | Rationale |
|-------------|--------|----------|-----------|
| Temperature | SHT31 | ±0.3°C | Dedicated high-precision temperature sensor |
| Humidity | SHT31 | ±2% RH | Superior to BME680 (±3% RH), no 48-hour burn-in required |
| Gas Resistance | BME680 | N/A | Unique VOC detection capability |
| Pressure | BME680 | ±1 hPa | Not currently transmitted (future expansion) |

**BME680 Humidity Reliability Issues**:
- Requires 48-hour burn-in period for accurate humidity readings
- Gas heater operation affects humidity sensor performance
- Baseline calibration needs settling time
- SHT31 provides immediate accurate readings without burn-in

**Implementation**:
```rust
// SHT31 provides primary environmental data
if let Ok(meas) = sht.measure(Repeatability::High) {
    let temp_c = meas.temperature as f32 / 100.0;    // Centidegrees → °C
    let humid_pct = meas.humidity as f32 / 100.0;    // Basis points → %

    // BME680 provides gas resistance only
    let gas = data.gas_resistance_ohm();

    // Combine best-of-breed measurements
    let _ = core::write!(pkt, "T:{:.1}H:{:.1}G:{:.0}", temp_c, humid_pct, gas);
}
```

### 8. Display Layout Optimization

**Challenge**: Fit critical sensor data and future LoRa metadata on 128x64 OLED.

**FONT_6X10 Characteristics**:
- Character width: 6 pixels
- Character height: 10 pixels (includes ascenders/descenders)
- Requires baseline positioning to prevent clipping

**Optimized 4-Line Layout**:
```
y=8:  T:28.5C H:55%     (Temperature & Humidity from SHT31)
y=20: Gas:142k          (Gas resistance from BME680)
y=32: ID:01 RSSI:--     (LoRa node ID and signal strength - placeholder)
y=44: SNR:-- #---       (Signal-to-noise ratio and packet count - placeholder)
```

**Key Spacing Decisions**:
- **12-pixel line spacing**: Font height (10px) + 2px inter-line gap
- **Start at y=8**: Prevents font ascenders from clipping at top edge (needs 8px margin)
- **Ends at y=54**: Leaves 10 pixels free at bottom for future status indicators
- **Compact formatting**: Combines related measurements on single lines to save vertical space

**Text Positioning Math**:
- Line 1: y = 8 (top margin)
- Line 2: y = 8 + 12 = 20
- Line 3: y = 20 + 12 = 32
- Line 4: y = 32 + 12 = 44
- Bottom edge: y = 44 + 10 = 54 (10 pixels remaining: 64 - 54)

**Future Expansion Space**:
- Could add a 5th line at y=56 (8px top margin available)
- Alternatively use bottom 10 pixels for status bar/icons
- Room for scrolling or paging if more data needed

## Performance Considerations

- **I2C Speed**: 100 kHz (standard mode) - reliable for multiple devices
- **Display Updates**: Buffered mode reduces I2C traffic
- **Delay Precision**: 1 MHz timer frequency provides microsecond accuracy
- **Interrupt Priority**: Default RTIC priorities ensure predictable scheduling
- **Stack Usage**: All strings use heapless with fixed 64-byte buffers

## Embedded HAL Ecosystem

### Version Compatibility

The embedded-hal ecosystem is transitioning from 0.2.x to 1.0:
- Modern HALs implement embedded-hal 1.0
- Many sensor drivers still expect embedded-hal 0.2.7
- Solution: Compatibility wrappers (newtype pattern)

### Common Pattern

```rust
// Dependency in Cargo.toml
embedded-hal = "1.0"
embedded-hal-0-2 = { package = "embedded-hal", version = "0.2.7" }

// Wrapper implementation
pub struct I2cCompat<I2C>(pub I2C);

impl<I2C> embedded_hal_0_2::blocking::i2c::Write for I2cCompat<I2C>
where I2C: embedded_hal::i2c::I2c {
    // Adapt new API to old API
}
```

## RTIC Patterns

### Task Definition

```rust
#[task(binds = TIM2, shared = [display, sensors], local = [led, timer])]
fn periodic_task(mut cx: periodic_task::Context) {
    // Access local resources directly
    cx.local.led.toggle();

    // Access shared resources with locks
    cx.shared.display.lock(|disp| {
        // Use disp here
    });
}
```

### Resource Initialization

```rust
#[init]
fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
    // Setup peripherals
    (
        Shared { /* shared resources */ },
        Local { /* local resources */ },
        init::Monotonics()
    )
}
```

## Future Improvements

1. Add error handling and recovery for sensor failures
2. Implement circular buffer for sensor history
3. ~~Add humidity and pressure display~~ ✅ Humidity now displayed (Week 2)
4. Implement LoRa acknowledgment handling
5. Power optimization (sleep modes between samples)
6. Calibration routines for sensor drift
7. Add timestamp to LoRa packets
8. Implement sensor health monitoring
9. Add watchdog timer for fault recovery
10. ~~Optimize display refresh rate~~ ✅ Display layout optimized (Week 2)
11. Populate real LoRa metadata (RSSI, SNR, packet count) from gateway responses
12. Add BME680 pressure data to LoRa packets (currently read but not transmitted)
13. Implement display paging or scrolling for additional metrics

## References

- [RTIC Book](https://rtic.rs/)
- [embedded-hal Compatibility](https://docs.rust-embedded.org/book/design-patterns/hal.html)
- [stm32f4xx-hal Documentation](https://docs.rs/stm32f4xx-hal/0.23.0/)
- [SHT3x Datasheet](https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-various-applications/)
- [BME680 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)
- [SSD1306 OLED Documentation](https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf)
- [shared-bus crate](https://docs.rs/shared-bus/)

## Key Takeaways

1. **Ownership is enforced at compile-time**: Use shared-bus or static allocations for shared peripherals
2. **Type safety prevents runtime errors**: Different timer types cannot be mixed
3. **HAL versions matter**: API changes between minor versions can be significant
4. **Always verify data representation**: Don't assume sensor values are in expected units
5. **RTIC provides zero-cost abstractions**: No runtime overhead for resource management
6. **defmt is powerful**: Use it extensively for debugging embedded systems
7. **Read the source code**: When documentation is unclear, the source code is truth
8. **Sensor fusion requires understanding sensor characteristics**: Different sensors excel at different measurements - use each sensor's strengths
9. **Font rendering needs proper margins**: Text positioned at y=0 will clip; always account for ascenders/descenders
10. **Display space is limited**: Combine related data on single lines; plan layout for future expansion
