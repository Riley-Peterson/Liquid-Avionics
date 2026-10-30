// Import statements that bring in features from Crates we want
#![no_std]
#![no_main]
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peripherals};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

// Initial configuration copied from Cube MX.
// This assumes we have a 16 MHz clock and gives
// the highest possible resulting clock speed
fn init_peripherals() -> Peripherals {
	let mut config = Config::default();
	config.rcc.hse = Some(Hse {
		freq: Hertz(16_000_000),                // Input Frequency
		mode: HseMode::Oscillator,
	});
	config.rcc.pll_src = PllSource::HSE;
	config.rcc.pll = Some(Pll {
		prediv: PllPreDiv::DIV8,				// "PLLM"
		mul: PllMul::MUL72,						// "PLLN"
		divp: Some(PllPDiv::DIV2),				// "PLLP"
		divq: Some(PllQDiv::DIV3),				// "PLLQ"
		divr: None,
	});
	config.rcc.sys = Sysclk::PLL1_P;
	config.rcc.ahb_pre = AHBPrescaler::DIV1;	// HCLK
	config.rcc.apb1_pre = APBPrescaler::DIV2;	// PCLK1
	config.rcc.apb2_pre = APBPrescaler::DIV1;	// PCLK2

	return embassy_stm32::init(config);
}

// Tell Embassy this is the program's entry point
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    
    // Run the initialization code defined above
    let p = init_peripherals();

    // Initialize PA5 as an Output pin, defaulting to logical high, with a slow slew rate.
    // The object must be mutable so we can toggle its state
    let mut led = Output::new(p.PA5, Level::High, Speed::Low);

    loop {
        // When debugging, the info! macro prints additional information
		info!("high");	
        led.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}