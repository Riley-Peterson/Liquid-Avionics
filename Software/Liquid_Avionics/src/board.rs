// Board-level constants and clock configuration for the Liquid Avionics stack.
use embassy_stm32::Config;
use embassy_stm32::gpio::Pull;
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;

/// I2C address of the MS5611. CSB is tied low on the Peripherals board -> 0x77.
pub const MS5611_ADDR: u8 = 0x77;

/// SEL mode jumper (PB2, J10 pin 4). No external pull on the board, so use
/// the internal pull-up: open = HIGH, grounded = LOW.
pub const SEL_PULL: Pull = Pull::Up;

/// Pressure transducer input divider (R_top = 10k, R_bottom = 15k).
/// V_in = V_adc * (10k + 15k) / 15k
pub const PT_DIVIDER_NUM: u32 = 25;
pub const PT_DIVIDER_DEN: u32 = 15;

/// Clock tree copied from Cube MX. Both sources give SYSCLK = 72 MHz,
/// PCLK1 = 36 MHz, PCLK2 = 72 MHz, PLLQ = 48 MHz.
/// `use_hse = false` falls back to the internal 16 MHz RC so the board still
/// boots (and can report the failure) if the crystal is not oscillating.
pub fn clock_config(use_hse: bool) -> Config {
	let mut config = Config::default();
	if use_hse {
		config.rcc.hse = Some(Hse {
			freq: Hertz(16_000_000),            // Input Frequency
			mode: HseMode::Oscillator,
		});
		config.rcc.pll_src = PllSource::HSE;
	} else {
		config.rcc.hsi = true;
		config.rcc.pll_src = PllSource::HSI;
	}
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
	config
}

/// Try to start the 16 MHz HSE crystal directly through the RCC registers.
/// Must be called before `embassy_stm32::init`, because embassy waits forever
/// for HSERDY. Returns true if the crystal came up within the timeout.
pub fn probe_hse() -> bool {
	use embassy_stm32::pac::RCC;

	RCC.cr().modify(|w| w.set_hseon(true));
	// Running on the 16 MHz HSI here, so ~4M iterations is well over 100 ms.
	// The NX2016SA datasheet start-up time is a few ms.
	let mut ready = false;
	for _ in 0..4_000_000u32 {
		if RCC.cr().read().hserdy() {
			ready = true;
			break;
		}
		cortex_m::asm::nop();
	}
	RCC.cr().modify(|w| w.set_hseon(false));
	ready
}

/// Keeps the executor from ever sleeping (WFE) so the ST-Link can attach to
/// running firmware. With the core idle in WFE, a fresh SWD connect fails with
/// DP WAIT even though embassy sets DBGMCU_CR (DBG_SLEEP/STOP/STANDBY); a
/// connection made while the core is busy survives later sleeps. Spawn this in
/// any binary you want to attach to after a cold power-up.
#[embassy_executor::task]
pub async fn keep_awake() {
	loop {
		// Wake ourselves and return Pending once, so the executor always has
		// work queued and never reaches its WFE.
		let mut yielded = false;
		core::future::poll_fn(|cx| {
			if yielded {
				core::task::Poll::Ready(())
			} else {
				yielded = true;
				cx.waker().wake_by_ref();
				core::task::Poll::Pending
			}
		})
		.await;
	}
}
