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
        timer::{CounterHz, Event},
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
    use heapless::{String, Vec};
    use core::fmt::Write as _;

    // --- Configuration Constants ---
    const NODE_ID: &str = "N2";              // Node identifier for display
    const RX_BUFFER_SIZE: usize = 255;       // UART RX buffer size (RYLR998 max payload is 240 bytes)
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
    type BusManager = shared_bus::BusManager<CortexMMutex<I2cCompat<MyI2c>>>;
    type I2cProxy = shared_bus::I2cProxy<'static, CortexMMutex<I2cCompat<MyI2c>>>;

    type LoraDisplay = Ssd1306<I2CInterface<I2cProxy>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

    #[derive(Debug, Clone, Copy)]
    pub struct SensorData {
        pub temperature: f32,
        pub humidity: f32,
        pub gas_resistance: u32,
        pub packet_num: u16,
    }

    #[shared]
    struct Shared {
        lora_uart: Serial<pac::UART4>,
        display: LoraDisplay,
        last_packet: Option<ParsedMessage>,
        packets_received: u32,
    }

    #[local]
    struct Local {
        led: Pin<'A', 5, Output>,
        timer: CounterHz<pac::TIM2>,
        rx_buffer: Vec<u8, RX_BUFFER_SIZE>,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct ParsedMessage {
        pub sensor_data: SensorData,
        pub rssi: i16,
        pub snr: i16,
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

        // 1. Configure RCC clocks
        let mut rcc = dp.RCC.freeze(Config::hsi().sysclk(84.MHz()));

        // 2. Split GPIOs
        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);

        let led = gpioa.pa5.into_push_pull_output();

        // --- UART4 for LoRa ---
        let tx = gpioc.pc10.into_alternate();
        let rx = gpioc.pc11.into_alternate();
        let mut lora_uart = Serial::new(
            dp.UART4,
            (tx, rx),
            SerialConfig::default().baudrate(115200.bps()),
            &mut rcc
        ).unwrap();

        // Configure LoRa module before enabling RX interrupt
        defmt::info!("Configuring LoRa module (Node 2)...");
        send_at_command(&mut lora_uart, "AT");
        send_at_command(&mut lora_uart, "AT+ADDRESS=2");

        let mut cmd_buf: String<32> = String::new();
        let _ = core::write!(cmd_buf, "AT+NETWORKID={}", NETWORK_ID);
        send_at_command(&mut lora_uart, cmd_buf.as_str());

        cmd_buf.clear();
        let _ = core::write!(cmd_buf, "AT+BAND={}000000", LORA_FREQ);
        send_at_command(&mut lora_uart, cmd_buf.as_str());

        send_at_command(&mut lora_uart, "AT+PARAMETER=7,9,1,7");

        // Flush any pending responses from configuration BEFORE enabling interrupt
        while lora_uart.read().is_ok() {}

        // Explicitly clear any error flags (especially ORE) before enabling interrupt
        let uart_ptr = unsafe { &*pac::UART4::ptr() };
        let sr = uart_ptr.sr().read();
        if sr.ore().bit_is_set() || sr.nf().bit_is_set() || sr.fe().bit_is_set() {
            let _ = uart_ptr.dr().read();
            defmt::info!("N2 INIT: Cleared error flags (ORE={} NF={} FE={})",
                sr.ore().bit_is_set(), sr.nf().bit_is_set(), sr.fe().bit_is_set());
        }

        defmt::info!("LoRa module configured");
        lora_uart.listen(SerialEvent::RxNotEmpty);

        // --- I2C1 for Display ---
        let scl = gpiob.pb8.into_alternate_open_drain();
        let sda = gpiob.pb9.into_alternate_open_drain();
        let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &mut rcc);

        let i2c_compat = I2cCompat(i2c);
        let bus: &'static BusManager = shared_bus::new_cortexm!(I2cCompat<MyI2c> = i2c_compat).unwrap();

        // --- Display ---
        let interface = I2CInterface::new(bus.acquire_i2c(), 0x3C, 0x40);
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        // Initial display message
        let style = MonoTextStyleBuilder::new()
            .font(&FONT_6X10)
            .text_color(BinaryColor::On)
            .build();
        let _ = display.clear(BinaryColor::Off);
        Text::new("N2 RECEIVER", Point::new(0, 8), style).draw(&mut display).ok();

        let mut init_buf: String<32> = String::new();
        let _ = core::write!(init_buf, "Net:{} {}MHz", NETWORK_ID, LORA_FREQ);
        Text::new(&init_buf, Point::new(0, 20), style).draw(&mut display).ok();

        Text::new("Waiting...", Point::new(0, 32), style).draw(&mut display).ok();
        let _ = display.flush();

        // --- Timer for LED blinking ---
        let mut timer = dp.TIM2.counter_hz(&mut rcc);
        timer.start(2.Hz()).unwrap();  // 2 Hz for heartbeat
        timer.listen(Event::Update);

        (
            Shared {
                lora_uart,
                display,
                last_packet: None,
                packets_received: 0,
            },
            Local {
                led,
                timer,
                rx_buffer: Vec::new(),
            },
            init::Monotonics()
        )
    }

    #[task(binds = TIM2, shared = [display, last_packet, packets_received], local = [led, timer])]
    fn tim2_handler(mut cx: tim2_handler::Context) {
        cx.local.timer.clear_flags(stm32f4xx_hal::timer::Flag::Update);
        cx.local.led.toggle();

        // Copy packet data quickly while holding lock
        let packet_copy = cx.shared.last_packet.lock(|pkt_opt| *pkt_opt);
        let total_count = cx.shared.packets_received.lock(|count| *count);

        defmt::info!("N2 Timer: total_count={}, has_packet={}", total_count, packet_copy.is_some());

        // Update display OUTSIDE locks (slow I2C is OK here in timer context)
        if let Some(parsed) = packet_copy {
            cx.shared.display.lock(|disp| {
                let _ = disp.clear(BinaryColor::Off);
                let style = MonoTextStyleBuilder::new()
                    .font(&FONT_6X10)
                    .text_color(BinaryColor::On)
                    .build();

                let mut buf: String<64> = String::new();

                // Line 1: Temperature & Humidity
                let _ = core::write!(buf, "T:{:.1}C H:{:.0}%",
                    parsed.sensor_data.temperature, parsed.sensor_data.humidity);
                Text::new(&buf, Point::new(0, 8), style).draw(disp).ok();

                buf.clear();
                // Line 2: Gas resistance
                let _ = core::write!(buf, "Gas:{:.0}k",
                    parsed.sensor_data.gas_resistance as f32 / 1000.0);
                Text::new(&buf, Point::new(0, 20), style).draw(disp).ok();

                buf.clear();
                // Line 3: Node ID and packet info
                let _ = core::write!(buf, "{} RX #{:04}",
                    NODE_ID, parsed.sensor_data.packet_num);
                Text::new(&buf, Point::new(0, 32), style).draw(disp).ok();

                buf.clear();
                // Line 4: Network ID and frequency
                let _ = core::write!(buf, "Net:{} {}MHz",
                    NETWORK_ID, LORA_FREQ);
                Text::new(&buf, Point::new(0, 44), style).draw(disp).ok();

                buf.clear();
                // Line 5: RSSI and SNR with total count
                let _ = core::write!(buf, "RSSI:{} SNR:{} #{}",
                    parsed.rssi, parsed.snr, total_count);
                Text::new(&buf, Point::new(0, 56), style).draw(disp).ok();

                let _ = disp.flush();  // Slow I2C flush is safe here
            });
        }
    }

    // UART interrupt: Fast receive and parse only - NO display updates
    // This must complete quickly to allow subsequent interrupts to fire
    #[task(binds = UART4, shared = [lora_uart, last_packet, packets_received], local = [rx_buffer])]
    fn uart4_handler(mut cx: uart4_handler::Context) {
        // Read ALL available bytes from UART in one interrupt
        let mut should_process = false;
        let mut bytes_read = 0u16;

        cx.shared.lora_uart.lock(|uart| {
            // Drain all available bytes from UART buffer
            while let Ok(byte) = uart.read() {
                bytes_read += 1;
                // Add byte to buffer
                if cx.local.rx_buffer.len() < RX_BUFFER_SIZE {
                    let _ = cx.local.rx_buffer.push(byte);
                }
                // Check for complete message (ends with \n)
                if byte == b'\n' {
                    should_process = true;
                }
            }
        });

        // Log that we got UART interrupt and how many bytes
        if bytes_read > 0 {
            defmt::info!("UART INT: {} bytes, complete={}", bytes_read, should_process);
        }

        // Process message OUTSIDE uart lock to allow new interrupts
        if should_process {
            if let Ok(msg) = core::str::from_utf8(cx.local.rx_buffer.as_slice()) {
                defmt::info!("LoRa RX: {}", msg);

                // Parse +RCV message format: +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>
                if msg.starts_with("+RCV=") {
                    if let Some(parsed) = parse_lora_message(msg) {
                        defmt::info!("Parsed - T:{} H:{} G:{} Pkt:{} RSSI:{} SNR:{}",
                            parsed.sensor_data.temperature, parsed.sensor_data.humidity,
                            parsed.sensor_data.gas_resistance, parsed.sensor_data.packet_num,
                            parsed.rssi, parsed.snr);

                        // Store parsed data for timer interrupt to display
                        // This is fast - just copying data, no I2C operations
                        cx.shared.last_packet.lock(|last_pkt| {
                            *last_pkt = Some(parsed);
                        });

                        cx.shared.packets_received.lock(|count| {
                            *count += 1;
                        });
                    }
                }
            }

            // Clear buffer for next message
            cx.local.rx_buffer.clear();
        }
    }

    fn parse_lora_message(msg: &str) -> Option<ParsedMessage> {
        // Expected format: +RCV=<Address>,<Length>,<Data>,<RSSI>,<SNR>
        // Data format: T:29.8H:62.1G:52174#0308

        let parts: Vec<&str, 8> = msg.trim().split(',').collect();
        if parts.len() < 5 {
            return None;
        }

        // Extract data payload (index 2)
        let data = parts[2];

        // Parse RSSI and SNR
        let rssi: i16 = parts[3].parse().ok()?;
        let snr: i16 = parts[4].trim().parse().ok()?;

        // Parse sensor data: T:XX.XH:XX.XG:XXXXX#XXXX
        let mut temp: Option<f32> = None;
        let mut humidity: Option<f32> = None;
        let mut gas: Option<u32> = None;
        let mut packet_num: Option<u16> = None;

        // Split by '#' to separate packet number
        let data_parts: Vec<&str, 2> = data.split('#').collect();
        if data_parts.len() == 2 {
            packet_num = data_parts[1].parse().ok();
        }

        // Parse T:, H:, G: fields from format: T:29.8H:62.1G:52174
        let sensor_part = data_parts.get(0)?;

        // Find T: field
        if let Some(t_idx) = sensor_part.find("T:") {
            let after_t = &sensor_part[t_idx + 2..];
            if let Some(h_idx) = after_t.find('H') {
                temp = after_t[..h_idx].parse().ok();
            }
        }

        // Find H: field
        if let Some(h_idx) = sensor_part.find("H:") {
            let after_h = &sensor_part[h_idx + 2..];
            if let Some(g_idx) = after_h.find('G') {
                humidity = after_h[..g_idx].parse().ok();
            }
        }

        // Find G: field
        if let Some(g_idx) = sensor_part.find("G:") {
            let after_g = &sensor_part[g_idx + 2..];
            gas = after_g.parse().ok();
        }

        Some(ParsedMessage {
            sensor_data: SensorData {
                temperature: temp?,
                humidity: humidity?,
                gas_resistance: gas?,
                packet_num: packet_num?,
            },
            rssi,
            snr,
        })
    }
}
