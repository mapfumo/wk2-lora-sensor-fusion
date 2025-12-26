#![no_std]
#![no_main]

use panic_probe as _;
use defmt_rtt as _;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    use stm32f4xx_hal::{
        prelude::*,
        gpio::{Output, Pin},
        pac,
        timer::{CounterHz, Event, Delay},
        serial::{Serial, Config as SerialConfig, Event as SerialEvent},
        i2c::I2c,
        rcc::Config,
    };

    use shared_bus::CortexMMutex;
    use ssd1306::{prelude::*, Ssd1306, mode::BufferedGraphicsMode};
    use display_interface_i2c::I2CInterface;
    use embedded_graphics::{
        mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
        pixelcolor::BinaryColor,
        prelude::*,
        text::Text,
    };
    use heapless::String;
    use core::fmt::Write as _;

    use sht3x::{SHT3x, Repeatability, Address as ShtAddress};
    use bme680::{Bme680, I2CAddress, IIRFilterSize, OversamplingSetting, SettingsBuilder, PowerMode};
    use core::time::Duration;

    // --- Configuration Constants ---
    const NODE_ID: &str = "N1";              // Node identifier for display
    const AUTO_TX_INTERVAL_SECS: u32 = 10;  // Auto-transmit every 10 seconds
    const NETWORK_ID: u8 = 18;               // LoRa network ID
    const LORA_FREQ: u32 = 915;              // LoRa frequency in MHz (915 for US)

    // --- Bridge for embedded-hal 1.0 -> 0.2.7 ---
    pub struct I2cCompat<I2C>(pub I2C);

    impl<I2C> embedded_hal_0_2::blocking::i2c::Write for I2cCompat<I2C>
    where I2C: embedded_hal::i2c::I2c {
        type Error = I2C::Error;
        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.0.write(addr, bytes)
        }
    }

    impl<I2C> embedded_hal_0_2::blocking::i2c::Read for I2cCompat<I2C>
    where I2C: embedded_hal::i2c::I2c {
        type Error = I2C::Error;
        fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.0.read(addr, buffer)
        }
    }

    impl<I2C> embedded_hal_0_2::blocking::i2c::WriteRead for I2cCompat<I2C>
    where I2C: embedded_hal::i2c::I2c {
        type Error = I2C::Error;
        fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
            self.0.write_read(addr, bytes, buffer)
        }
    }

    type MyI2c = I2c<pac::I2C1>;
    type ShtDelay = Delay<pac::TIM5, 1000000>;
    type BmeDelay = Delay<pac::TIM3, 1000000>;
    type BusManager = shared_bus::BusManager<CortexMMutex<I2cCompat<MyI2c>>>;
    type I2cProxy = shared_bus::I2cProxy<'static, CortexMMutex<I2cCompat<MyI2c>>>;
    
    type LoraDisplay = Ssd1306<I2CInterface<I2cProxy>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

    #[shared]
    struct Shared {
        lora_uart: Serial<pac::UART4>,
        display: LoraDisplay,
        sht31: SHT3x<I2cProxy, ShtDelay>,
        bme680: Bme680<I2cProxy, BmeDelay>,
    }

    #[local]
    struct Local {
        led: Pin<'A', 5, Output>,
        button: Pin<'C', 13>,  // Blue button on Nucleo (PC13)
        timer: CounterHz<pac::TIM2>,
        bme_delay: BmeDelay,
        packet_counter: u32,   // Counts packets sent
        tx_countdown: u32,     // Seconds until next auto-transmit
    }

    // Helper function to send AT command and wait for response
    fn send_at_command(uart: &mut Serial<pac::UART4>, cmd: &str) {
        defmt::info!("Sending AT command: {}", cmd);

        // Send command
        for byte in cmd.as_bytes() {
            let _ = nb::block!(uart.write(*byte));
        }

        // Send \r\n
        let _ = nb::block!(uart.write(b'\r'));
        let _ = nb::block!(uart.write(b'\n'));

        // Wait a bit for module to process
        cortex_m::asm::delay(8_400_000); // ~100ms at 84 MHz
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let dp = cx.device;
        
        // 1. Configure RCC clocks (0.23.0 API uses freeze with Config)
        let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(84.MHz()));

        // 2. Split GPIOs (requires &mut rcc in 0.23.0)
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);

        let led = gpioa.pa5.into_push_pull_output();
        let button = gpioc.pc13;  // Blue button (has built-in pull-up, active-low)

        // Create delay instances for SHT31 and BME680
        // SHT31 takes ownership of its delay (TIM5)
        let sht_delay = dp.TIM5.delay_us(&mut rcc);
        // BME680 delay (TIM3) will be moved to Local for use in handler
        let mut bme_delay = dp.TIM3.delay_us(&mut rcc);

        // --- UART4 ---
        let tx = gpioc.pc10.into_alternate();
        let rx = gpioc.pc11.into_alternate();
        let mut lora_uart = Serial::new(
            dp.UART4,
            (tx, rx),
            SerialConfig::default().baudrate(115200.bps()),
            &mut rcc
        ).unwrap();

        // Configure LoRa module before enabling RX interrupt
        defmt::info!("Configuring LoRa module (Node 1)...");
        send_at_command(&mut lora_uart, "AT");
        send_at_command(&mut lora_uart, "AT+ADDRESS=1");

        let mut cmd_buf: String<32> = String::new();
        let _ = core::write!(cmd_buf, "AT+NETWORKID={}", NETWORK_ID);
        send_at_command(&mut lora_uart, cmd_buf.as_str());

        cmd_buf.clear();
        let _ = core::write!(cmd_buf, "AT+BAND={}000000", LORA_FREQ);
        send_at_command(&mut lora_uart, cmd_buf.as_str());

        send_at_command(&mut lora_uart, "AT+PARAMETER=7,9,1,7");

        // Flush any pending responses from configuration
        while lora_uart.read().is_ok() {}

        // Explicitly clear any error flags (especially ORE) before enabling interrupt
        let uart_ptr = unsafe { &*pac::UART4::ptr() };
        let sr = uart_ptr.sr().read();
        if sr.ore().bit_is_set() || sr.nf().bit_is_set() || sr.fe().bit_is_set() {
            let _ = uart_ptr.dr().read();
            defmt::info!("N1 INIT: Cleared error flags (ORE={} NF={} FE={})",
                sr.ore().bit_is_set(), sr.nf().bit_is_set(), sr.fe().bit_is_set());
        }

        defmt::info!("LoRa module configured");

        lora_uart.listen(SerialEvent::RxNotEmpty);

        // --- I2C1 ---
        let scl = gpiob.pb8.into_alternate_open_drain();
        let sda = gpiob.pb9.into_alternate_open_drain();
        let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);
        
        let i2c_compat = I2cCompat(i2c);
        let bus: &'static BusManager = shared_bus::new_cortexm!(I2cCompat<MyI2c> = i2c_compat).unwrap();

        // --- Sensors ---
        let sht31 = SHT3x::new(bus.acquire_i2c(), sht_delay, ShtAddress::Low);
        let mut bme680 = Bme680::init(bus.acquire_i2c(), &mut bme_delay, I2CAddress::Secondary).unwrap();
        
        let settings = SettingsBuilder::new()
            .with_humidity_oversampling(OversamplingSetting::OS2x)
            .with_pressure_oversampling(OversamplingSetting::OS4x)
            .with_temperature_oversampling(OversamplingSetting::OS2x)
            .with_temperature_filter(IIRFilterSize::Size3)
            .with_gas_measurement(Duration::from_millis(150), 300, 25)
            .with_run_gas(true)
            .build();
        let _ = bme680.set_sensor_settings(&mut bme_delay, settings);

        // --- Display ---
        let interface = I2CInterface::new(bus.acquire_i2c(), 0x3C, 0x40);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        // --- Timer ---
        let mut timer = dp.TIM2.counter_hz(&mut rcc);
        timer.start(1.Hz()).unwrap();  // Still ticks at 1 Hz for countdown
        timer.listen(Event::Update);

        (
            Shared { lora_uart, display, sht31, bme680 },
            Local {
                led,
                button,
                timer,
                bme_delay,
                packet_counter: 0,                    // Start at packet #0
                tx_countdown: AUTO_TX_INTERVAL_SECS,  // First TX in 10 seconds
            },
            init::Monotonics()
        )
    }

    #[task(binds = TIM2, shared = [sht31, bme680, display, lora_uart], local = [led, button, timer, bme_delay, packet_counter, tx_countdown])]
    fn tim2_handler(mut cx: tim2_handler::Context) {
        cx.local.timer.clear_flags(stm32f4xx_hal::timer::Flag::Update);
        cx.local.led.toggle();

        // Determine if we should transmit this cycle
        let mut should_transmit = false;
        let mut trigger_source = "AUTO";

        // Check button (active-low: pressed = low)
        if cx.local.button.is_low() {
            defmt::info!("Button pressed - triggering immediate transmission");
            should_transmit = true;
            trigger_source = "BTN";
            *cx.local.tx_countdown = AUTO_TX_INTERVAL_SECS;  // Reset countdown
        } else {
            // Auto-transmit countdown
            if *cx.local.tx_countdown > 0 {
                *cx.local.tx_countdown -= 1;
            }

            if *cx.local.tx_countdown == 0 {
                defmt::info!("Auto-transmit countdown reached 0");
                should_transmit = true;
                *cx.local.tx_countdown = AUTO_TX_INTERVAL_SECS;  // Reset countdown
            }
        }

        // Only read sensors and transmit if triggered
        if should_transmit {
            let delay = cx.local.bme_delay;

            cx.shared.bme680.lock(|bme| {
                let _ = bme.set_sensor_mode(delay, PowerMode::ForcedMode);
            });

            delay.delay_ms(200u32);

            cx.shared.bme680.lock(|bme| {
                if let Ok((data, _state)) = bme.get_sensor_data(delay) {
                    // BME680 used only for gas resistance (SHT31 is more accurate for temp/humidity)
                    let gas = data.gas_resistance_ohm();

                    cx.shared.sht31.lock(|sht| {
                        if let Ok(meas) = sht.measure(Repeatability::High) {
                            let temp_c = meas.temperature as f32 / 100.0;
                            let humid_pct = meas.humidity as f32 / 100.0;

                            // Increment packet counter
                            *cx.local.packet_counter += 1;

                            cx.shared.display.lock(|disp: &mut LoraDisplay| {
                                let _ = disp.clear(BinaryColor::Off);
                                let style = MonoTextStyleBuilder::new()
                                    .font(&FONT_6X10)
                                    .text_color(BinaryColor::On)
                                    .build();

                                let mut buf: String<64> = String::new();
                                // Line 1: Temp & Humidity (compact)
                                let _ = core::write!(buf, "T:{:.1}C H:{:.0}%", temp_c, humid_pct);
                                Text::new(&buf, Point::new(0, 8), style).draw(disp).ok();

                                buf.clear();
                                // Line 2: Gas resistance
                                let _ = core::write!(buf, "Gas:{:.0}k", gas as f32 / 1000.0);
                                Text::new(&buf, Point::new(0, 20), style).draw(disp).ok();

                                buf.clear();
                                // Line 3: Node ID and TX status with packet counter
                                let _ = core::write!(buf, "{} TX:{} #{:04}", NODE_ID, trigger_source, *cx.local.packet_counter);
                                Text::new(&buf, Point::new(0, 32), style).draw(disp).ok();

                                buf.clear();
                                // Line 4: Network ID and frequency
                                let _ = core::write!(buf, "Net:{} {}MHz", NETWORK_ID, LORA_FREQ);
                                Text::new(&buf, Point::new(0, 44), style).draw(disp).ok();

                                buf.clear();
                                // Line 5: Countdown to next auto-TX
                                let _ = core::write!(buf, "Next:{}s", *cx.local.tx_countdown);
                                Text::new(&buf, Point::new(0, 56), style).draw(disp).ok();

                                let _ = disp.flush();
                            });

                            cx.shared.lora_uart.lock(|uart| {
                                // Build payload with sensor data
                                // Format: T:<temp>H:<humidity>G:<gas>#<packet_num>
                                let mut payload: String<128> = String::new();
                                let _ = core::write!(payload, "T:{:.1}H:{:.1}G:{:.0}#{:04}",
                                    temp_c, humid_pct, gas, *cx.local.packet_counter);

                                // Build AT command with dynamic length calculation
                                // RYLR998 supports up to 240 bytes payload (not LoRaWAN's 51-byte limit!)
                                // String<255> accommodates: "AT+SEND=2," (12) + length digits (3) + "," (1) + payload (128) = ~144 bytes
                                let mut cmd: String<255> = String::new();
                                let _ = core::write!(cmd, "AT+SEND=2,{},{}",
                                    payload.len(), payload.as_str());

                                defmt::info!("LoRa TX [{}]: {} (payload: {} bytes)",
                                    trigger_source, cmd.as_str(), payload.len());

                                // Send command bytes to LoRa module
                                for b in cmd.as_bytes() {
                                    let _ = nb::block!(uart.write(*b));
                                }

                                // Send \r\n terminator AFTER the command (required by RYLR998 AT protocol)
                                let _ = nb::block!(uart.write(b'\r'));
                                let _ = nb::block!(uart.write(b'\n'));

                                defmt::info!("Transmission complete - packet #{}", *cx.local.packet_counter);
                            });
                        }
                    });
                }
            });
        }
    }

    // UART interrupt: Drain all incoming bytes AND clear error flags
    // This must drain the buffer to prevent it from filling up and blocking transmissions
    // CRITICAL: Must clear Overrun Error (ORE) or UART stops working entirely
    #[task(binds = UART4, shared = [lora_uart])]
    fn uart4_handler(mut cx: uart4_handler::Context) {
        cx.shared.lora_uart.lock(|uart| {
            // Drain ALL available bytes from UART buffer
            let mut drained = 0;
            while uart.read().is_ok() {
                drained += 1;
            }

            if drained > 0 {
                defmt::info!("N1 UART: drained {} bytes", drained);
            }

            // AFTER draining data, check and clear error flags
            // At this point, RXNE should be clear, so reading DR won't consume data
            let uart_ptr = unsafe { &*pac::UART4::ptr() };
            let sr = uart_ptr.sr().read();

            if sr.ore().bit_is_set() || sr.nf().bit_is_set() || sr.fe().bit_is_set() {
                // Clear errors by reading DR (should be empty now)
                let _ = uart_ptr.dr().read();
                defmt::warn!("N1 UART4 errors cleared (ORE={} NF={} FE={})",
                    sr.ore().bit_is_set(), sr.nf().bit_is_set(), sr.fe().bit_is_set());
            }
        });
    }
}