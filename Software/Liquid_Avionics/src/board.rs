// Board-level constants, pin map and clock configuration for the Liquid Avionics stack.
use embassy_stm32::gpio::Pull;
use embassy_stm32::peripherals as p;
use embassy_stm32::rcc::*;
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, Peri, Peripherals};

/// I2C address of the MS5611. CSB is tied low on the Peripherals board -> 0x77.
pub const MS5611_ADDR: u8 = 0x77;

/// SEL mode jumper (PB2, J10 pin 4). No external pull on the board, so use
/// the internal pull-up: open = HIGH, grounded = LOW.
pub const SEL_PULL: Pull = Pull::Up;

/// Pressure transducer input divider (R_top = 10k, R_bottom = 15k).
/// V_in = V_adc * (10k + 15k) / 15k
pub const PT_DIVIDER_NUM: u32 = 25;
pub const PT_DIVIDER_DEN: u32 = 15;

// ---------------------------------------------------------------- pin map
// Every MCU resource the board wires to something, grouped by subsystem.
// DMA streams are listed TX first, RX second, matching embassy's constructors.

pub struct Board {
	pub adc: Peri<'static, p::ADC1>,
	pub pt: PtPins,
	pub cameras: CameraPins,
	pub imu: ImuPins,
	pub baro: BaroPins,
	pub flash: FlashPins,
	pub radio: RadioPins,
	pub can: CanPins,
	/// SEL mode jumper (J10 pin 4), see `SEL_PULL`.
	pub sel: Peri<'static, p::PB2>,
}

/// Pressure transducer inputs, through the 10k/15k divider. ADC1.
pub struct PtPins {
	pub pt1: Peri<'static, p::PC5>,
	pub pt2: Peri<'static, p::PC4>,
	pub pt3: Peri<'static, p::PB1>,
	pub pt4: Peri<'static, p::PB0>,
}

/// PROFET high-side switches. Camera 1's PROFET is currently desoldered.
pub struct CameraPins {
	pub cam1_out: Peri<'static, p::PC13>,
	pub cam1_is: Peri<'static, p::PA2>,
	pub cam2_out: Peri<'static, p::PA0>,
	pub cam2_is: Peri<'static, p::PA1>,
}

/// LSM6DSV16BX on SPI1. With CS high the IMU is in I2C mode on SCK/MOSI.
pub struct ImuPins {
	pub spi: Peri<'static, p::SPI1>,
	pub sck: Peri<'static, p::PA5>,
	pub mosi: Peri<'static, p::PA7>,
	pub miso: Peri<'static, p::PA6>,
	pub cs: Peri<'static, p::PC10>,
	pub int1: Peri<'static, p::PA15>,
	pub int1_exti: Peri<'static, p::EXTI15>,
	pub int2: Peri<'static, p::PC11>,
	pub int2_exti: Peri<'static, p::EXTI11>,
	pub tx_dma: Peri<'static, p::DMA2_CH3>,
	pub rx_dma: Peri<'static, p::DMA2_CH0>,
}

/// MS5611 on I2C1 (address `MS5611_ADDR`), external pull-ups R1/R2.
pub struct BaroPins {
	pub i2c: Peri<'static, p::I2C1>,
	pub scl: Peri<'static, p::PB6>,
	pub sda: Peri<'static, p::PB7>,
	pub tx_dma: Peri<'static, p::DMA1_CH6>,
	pub rx_dma: Peri<'static, p::DMA1_CH0>,
}

/// W25Q40 NOR flash on SPI2.
pub struct FlashPins {
	pub spi: Peri<'static, p::SPI2>,
	pub sck: Peri<'static, p::PB10>,
	pub mosi: Peri<'static, p::PC1>,
	pub miso: Peri<'static, p::PC2>,
	pub cs: Peri<'static, p::PC3>,
	pub tx_dma: Peri<'static, p::DMA1_CH4>,
	pub rx_dma: Peri<'static, p::DMA1_CH3>,
}

/// RFD900 radio on USART6.
pub struct RadioPins {
	pub uart: Peri<'static, p::USART6>,
	pub rx: Peri<'static, p::PC7>,
	pub tx: Peri<'static, p::PC6>,
	pub tx_dma: Peri<'static, p::DMA2_CH6>,
	pub rx_dma: Peri<'static, p::DMA2_CH1>,
}

/// CAN1 through the SN65HVD230 transceiver.
pub struct CanPins {
	pub can: Peri<'static, p::CAN1>,
	pub rx: Peri<'static, p::PA11>,
	pub tx: Peri<'static, p::PA12>,
}

/// Splits the HAL's peripherals into the board's subsystem groups.
pub fn split(p: Peripherals) -> Board {
	Board {
		adc: p.ADC1,
		pt: PtPins { pt1: p.PC5, pt2: p.PC4, pt3: p.PB1, pt4: p.PB0 },
		cameras: CameraPins { cam1_out: p.PC13, cam1_is: p.PA2, cam2_out: p.PA0, cam2_is: p.PA1 },
		imu: ImuPins {
			spi: p.SPI1,
			sck: p.PA5,
			mosi: p.PA7,
			miso: p.PA6,
			cs: p.PC10,
			int1: p.PA15,
			int1_exti: p.EXTI15,
			int2: p.PC11,
			int2_exti: p.EXTI11,
			tx_dma: p.DMA2_CH3,
			rx_dma: p.DMA2_CH0,
		},
		baro: BaroPins { i2c: p.I2C1, scl: p.PB6, sda: p.PB7, tx_dma: p.DMA1_CH6, rx_dma: p.DMA1_CH0 },
		flash: FlashPins {
			spi: p.SPI2,
			sck: p.PB10,
			mosi: p.PC1,
			miso: p.PC2,
			cs: p.PC3,
			tx_dma: p.DMA1_CH4,
			rx_dma: p.DMA1_CH3,
		},
		radio: RadioPins { uart: p.USART6, rx: p.PC7, tx: p.PC6, tx_dma: p.DMA2_CH6, rx_dma: p.DMA2_CH1 },
		can: CanPins { can: p.CAN1, rx: p.PA11, tx: p.PA12 },
		sel: p.PB2,
	}
}

// ---------------------------------------------------------------- analog

/// Factory VREFINT reading taken at VDDA = 3.3 V (RM0390).
pub fn vrefint_cal() -> u32 {
	unsafe { core::ptr::read_volatile(0x1FFF_7A2A as *const u16) as u32 }
}

/// VDDA in mV from the sum of `samples` VREFINT readings. 0 if the sum is 0.
pub fn vdda_mv(vrefint_sum: u32, samples: u32) -> u32 {
	if vrefint_sum == 0 {
		return 0;
	}
	3300 * vrefint_cal() * samples / vrefint_sum
}

/// Voltage at a PT connector in mV from the sum of `samples` ADC readings,
/// undoing the input divider. Max 8 * 4095 * 3400 * 25 = 2.8e9 fits in u32,
/// so keep `samples` <= 8.
pub fn pt_input_mv(raw_sum: u32, samples: u32, vdda_mv: u32) -> u32 {
	raw_sum * vdda_mv * PT_DIVIDER_NUM / (4095 * samples * PT_DIVIDER_DEN)
}

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
