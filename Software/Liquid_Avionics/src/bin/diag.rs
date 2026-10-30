// Board bring-up diagnostic for the Liquid Avionics stack.
//
// Run with:   cargo run --bin diag
// Output comes back over RTT (defmt) through the ST-Link.
//
// Tests, in order:
//   1. HSE crystal start-up
//   2. Idle pin levels (pull-ups present, transceiver/radio driving lines, SEL mode jumper)
//   3. ADC: VDDA via VREFINT, MCU temp, PT1-4, camera IS lines
//   4. W25Q40 flash on SPI2: JEDEC/unique ID, erase/program/verify of the LAST sector
//   5. LSM6DSV16BX on SPI1: WHO_AM_I, reset, gravity check, INT1/INT2 data-ready rates
//   6. MS5611 on I2C1: bus scan, PROM CRC, pressure + temperature
//   7. RFD900 on USART6: AT command mode
//   8. CAN1 + SN65HVD230: bus sync and one-shot transmit, decodes the error code
//   9. Camera 2 PROFET: IS line with output off vs on
// Then it streams live sensor data once a second.
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, SampleTime};
use embassy_stm32::can::{self, Can, Frame};
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Flex, Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::{bind_interrupts, dma, interrupt, pac, peripherals};
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use embedded_hal_bus::spi::ExclusiveDevice;
use liquid_avionics::{board, can_map};
use liquid_avionics::drivers::lsm6dsv16bx::{self, AccelScale, GyroScale, Lsm6dsv16bx, Odr};
use liquid_avionics::drivers::ms5611::{Ms5611, Osr};
use liquid_avionics::drivers::w25q::{self, W25q};
use {defmt_rtt as _, panic_probe as _};

/// Set false to skip switching the Camera 2 PROFET on during the test.
const TEST_CAMERA2_OUTPUT: bool = true;
/// Flash test region: last 4 KiB sector of the W25Q40.
const FLASH_TEST_ADDR: u32 = 512 * 1024 - w25q::SECTOR_SIZE;

bind_interrupts!(struct Irqs {
	I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
	I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
	USART6 => usart::InterruptHandler<peripherals::USART6>;
	CAN1_TX => can::TxInterruptHandler<peripherals::CAN1>;
	CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN1>;
	CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN1>;
	CAN1_SCE => can::SceInterruptHandler<peripherals::CAN1>;
	EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
	DMA1_STREAM0 => dma::InterruptHandler<peripherals::DMA1_CH0>;	// I2C1 RX
	DMA1_STREAM6 => dma::InterruptHandler<peripherals::DMA1_CH6>;	// I2C1 TX
	DMA1_STREAM3 => dma::InterruptHandler<peripherals::DMA1_CH3>;	// SPI2 RX
	DMA1_STREAM4 => dma::InterruptHandler<peripherals::DMA1_CH4>;	// SPI2 TX
	DMA2_STREAM0 => dma::InterruptHandler<peripherals::DMA2_CH0>;	// SPI1 RX
	DMA2_STREAM3 => dma::InterruptHandler<peripherals::DMA2_CH3>;	// SPI1 TX
	DMA2_STREAM1 => dma::InterruptHandler<peripherals::DMA2_CH1>;	// USART6 RX
	DMA2_STREAM6 => dma::InterruptHandler<peripherals::DMA2_CH6>;	// USART6 TX
});

// ---------------------------------------------------------------- reporting

#[derive(Clone, Copy, PartialEq, Eq, Format)]
enum Status {
	Pass,
	Warn,
	Fail,
}

struct Report {
	items: [(&'static str, Status); 32],
	len: usize,
}

impl Report {
	const fn new() -> Self {
		Self { items: [("", Status::Pass); 32], len: 0 }
	}

	fn record(&mut self, name: &'static str, status: Status) {
		match status {
			Status::Pass => info!("[PASS] {}", name),
			Status::Warn => warn!("[WARN] {}", name),
			Status::Fail => error!("[FAIL] {}", name),
		}
		if self.len < self.items.len() {
			self.items[self.len] = (name, status);
			self.len += 1;
		}
	}

	fn check(&mut self, name: &'static str, ok: bool) -> bool {
		self.record(name, if ok { Status::Pass } else { Status::Fail });
		ok
	}

	fn summary(&self) {
		let count = |s| self.items[..self.len].iter().filter(|(_, st)| *st == s).count();
		info!("==================== SUMMARY ====================");
		for (name, st) in &self.items[..self.len] {
			if *st != Status::Pass {
				info!("  {} : {}", st, name);
			}
		}
		info!("  {} pass, {} warn, {} fail", count(Status::Pass), count(Status::Warn), count(Status::Fail));
		info!("=================================================");
	}
}

fn section(name: &str) {
	info!("");
	info!("---------- {} ----------", name);
}

/// Minimal open-drain I2C master on two GPIOs (~10 kHz, internal pull-ups).
/// Write `reg`, repeated START, read one byte. None on any missing ACK.
fn bitbang_i2c_read_reg(scl: &mut Flex<'_>, sda: &mut Flex<'_>, addr: u8, reg: u8) -> Option<u8> {
	let half = || embassy_time::block_for(Duration::from_micros(50));
	// "High" = released to the pull-up, "low" = actively driven.
	let release = |pin: &mut Flex<'_>| pin.set_as_input(Pull::Up);
	let drive_low = |pin: &mut Flex<'_>| {
		pin.set_low();
		pin.set_as_output(Speed::Low);
	};
	let write_byte = |scl: &mut Flex<'_>, sda: &mut Flex<'_>, byte: u8| -> bool {
		for i in (0..8).rev() {
			if byte & (1 << i) != 0 { release(sda) } else { drive_low(sda) }
			half();
			release(scl);
			half();
			drive_low(scl);
		}
		release(sda); // ACK bit
		half();
		release(scl);
		half();
		let ack = sda.is_low();
		drive_low(scl);
		ack
	};

	release(scl);
	release(sda);
	half();
	// START
	drive_low(sda);
	half();
	drive_low(scl);
	let mut ok = write_byte(scl, sda, addr << 1) && write_byte(scl, sda, reg);
	let mut value = 0u8;
	if ok {
		// Repeated START
		release(sda);
		half();
		release(scl);
		half();
		drive_low(sda);
		half();
		drive_low(scl);
		ok = write_byte(scl, sda, (addr << 1) | 1);
		if ok {
			release(sda);
			for _ in 0..8 {
				half();
				release(scl);
				half();
				value = (value << 1) | sda.is_high() as u8;
				drive_low(scl);
			}
			// NACK the single byte
			release(sda);
			half();
			release(scl);
			half();
			drive_low(scl);
		}
	}
	// STOP
	drive_low(sda);
	half();
	release(scl);
	half();
	release(sda);
	half();
	scl.set_as_input(Pull::None);
	sda.set_as_input(Pull::None);
	ok.then_some(value)
}

/// Read a pin with the internal pull-up and then pull-down.
/// Returns (level_with_pullup, level_with_pulldown).
async fn probe_pin(pin: &mut Flex<'_>) -> (bool, bool) {
	pin.set_as_input(Pull::Up);
	Timer::after_micros(200).await;
	let up = pin.is_high();
	pin.set_as_input(Pull::Down);
	Timer::after_micros(200).await;
	let down = pin.is_high();
	pin.set_as_input(Pull::None);
	(up, down)
}

// ---------------------------------------------------------------- main

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
	let hse_ok = board::probe_hse();
	let mut b = board::split(embassy_stm32::init(board::clock_config(hse_ok)));
	let mut report = Report::new();
	// Give probe-rs time to attach to RTT so the report header isn't dropped
	// from the 1 KB log buffer.
	Timer::after_secs(1).await;

	info!("=================================================");
	info!(" Liquid Avionics board diagnostic");
	info!("=================================================");

	// ------------------------------------------------------------ MCU
	section("MCU");
	let idcode = pac::DBGMCU.idcode().read();
	let uid = embassy_stm32::uid::uid();
	let flash_kb = unsafe { core::ptr::read_volatile(0x1FFF_7A22 as *const u16) };
	info!("DEV_ID=0x{:03x} REV_ID=0x{:04x} flash={}KB UID={:02x}", idcode.dev_id(), idcode.rev_id(), flash_kb, uid);
	report.check("MCU is STM32F446 (DEV_ID 0x421)", idcode.dev_id() == 0x421);
	if hse_ok {
		report.record("HSE 16 MHz crystal started, SYSCLK = 72 MHz from PLL(HSE)", Status::Pass);
	} else {
		report.record("HSE crystal did NOT start - running on HSI fallback (check Y1, C12, C13)", Status::Fail);
	}

	// ------------------------------------------------------------ pin levels
	section("Idle pin levels");
	{
		// External pull-ups win against the ~40k internal pull-down.
		let (_, scl) = probe_pin(&mut Flex::new(b.baro.scl.reborrow())).await;
		let (_, sda) = probe_pin(&mut Flex::new(b.baro.sda.reborrow())).await;
		info!("I2C1 SCL(PB6) w/ pulldown={} SDA(PB7) w/ pulldown={}", scl, sda);
		report.check("I2C1 SCL pull-up (R1) present", scl);
		report.check("I2C1 SDA pull-up (R2) present", sda);

		let (_, imu_cs) = probe_pin(&mut Flex::new(b.imu.cs.reborrow())).await;
		report.check("IMU_CS pull-up (R19) present", imu_cs);

		// IMU MISO (PA6) with the MCU pull-up, CS deasserted vs asserted.
		// SDO is high-Z while CS is high, so a low reading there means MISO is
		// held low by something other than an active, selected IMU.
		{
			let mut miso = Flex::new(b.imu.miso.reborrow());
			let mut cs = Output::new(b.imu.cs.reborrow(), Level::High, Speed::Low);
			miso.set_as_input(Pull::Up);
			Timer::after_micros(200).await;
			let cs_high = miso.is_high();
			cs.set_low();
			Timer::after_micros(200).await;
			let cs_low = miso.is_high();
			cs.set_high();
			info!("IMU MISO (PA6) w/ pull-up: CS high -> {}, CS low -> {}", cs_high, cs_low);
			match (cs_high, cs_low) {
				(true, true) => report.record("IMU MISO floats when idle and selected (SDO not driving)", Status::Warn),
				(true, false) => report.record("IMU MISO high-Z when idle, driven when selected (IMU alive)", Status::Pass),
				(false, _) => report.record("IMU MISO held LOW with CS high - short to GND or unpowered IMU clamping", Status::Fail),
			}
		}

		// With CS high the IMU is in I2C mode on its SCL(PA5)/SDA(PA7) pins.
		// Bit-bang a WHO_AM_I read there: if it answers, the chip and the
		// SCK/MOSI pads are good and any SPI failure points at CS or SDO.
		// (Only valid from power-up: a successful SPI init disables I2C.)
		{
			// Self-test the bit-banger on the known-good MS5611 (PROM C1 high byte)
			let mut bscl = Flex::new(b.baro.scl.reborrow());
			let mut bsda = Flex::new(b.baro.sda.reborrow());
			match bitbang_i2c_read_reg(&mut bscl, &mut bsda, board::MS5611_ADDR, 0xA2) {
				Some(v) => info!("bit-bang I2C self-test on MS5611: ACK, PROM C1 high byte = 0x{:02x}", v),
				None => warn!("bit-bang I2C self-test on MS5611: no ACK (IMU result below is not trustworthy)"),
			}

			let _cs = Output::new(b.imu.cs.reborrow(), Level::High, Speed::Low);
			let mut scl = Flex::new(b.imu.sck.reborrow());
			let mut sda = Flex::new(b.imu.mosi.reborrow());
			let mut found = None;
			for addr in [0x6Bu8, 0x6A] {
				if let Some(id) = bitbang_i2c_read_reg(&mut scl, &mut sda, addr, 0x0F) {
					found = Some((addr, id));
					break;
				}
			}
			match found {
				Some((addr, id)) => {
					info!("IMU over bit-banged I2C: addr 0x{:02x} WHO_AM_I = 0x{:02x}", addr, id);
					report.check("IMU answers in I2C mode (chip alive, SCK/MOSI pads OK)", id == lsm6dsv16bx::WHO_AM_I_VALUE);
				}
				None => info!("IMU did not ACK over bit-banged I2C at 0x6A/0x6B"),
			}
		}

		// The CAN transceiver R pin drives high when the bus is recessive.
		let (_, can_rx) = probe_pin(&mut Flex::new(b.can.rx.reborrow())).await;
		report.check("CAN1_RX driven high by SN65HVD230 (transceiver powered, bus recessive)", can_rx);

		// UART idle is high; only true if the RFD900 is powered and its TX reaches PC7.
		let (_, uart_rx) = probe_pin(&mut Flex::new(b.radio.rx.reborrow())).await;
		if uart_rx {
			report.record("USART6_RX (PC7) idles high - RFD900 TX line present", Status::Pass);
		} else {
			report.record("USART6_RX (PC7) not driven - RFD900 unpowered/unplugged or TX/RX swapped", Status::Warn);
		}

		// IMU INT pins are push-pull, driven low after power-on.
		let (int1_up, _) = probe_pin(&mut Flex::new(b.imu.int1.reborrow())).await;
		let (int2_up, _) = probe_pin(&mut Flex::new(b.imu.int2.reborrow())).await;
		report.check("IMU_Int1 (PA15) driven low by IMU", !int1_up);
		report.check("IMU_Int2 (PC11) driven low by IMU", !int2_up);

		let sel = Input::new(b.sel.reborrow(), board::SEL_PULL);
		Timer::after_micros(200).await;
		if sel.is_high() {
			report.record("SEL (PB2) open -> reads HIGH via internal pull-up", Status::Pass);
		} else {
			report.record("SEL (PB2) grounded -> reads LOW (alternate mode)", Status::Pass);
		}
	}

	// Hold camera 1 output low (PROFET desoldered) and camera 2 off.
	let _cam1 = Output::new(b.cameras.cam1_out, Level::Low, Speed::Low);
	let mut cam2 = Output::new(b.cameras.cam2_out, Level::Low, Speed::Low);

	// ------------------------------------------------------------ ADC
	section("ADC");
	let mut adc = Adc::new(b.adc);
	let mut vrefint = adc.enable_vrefint();
	let mut temp_ch = adc.enable_temperature();
	Timer::after_micros(20).await;
	let mut pt = [
		("PT1", b.pt.pt1.degrade_adc()),
		("PT2", b.pt.pt2.degrade_adc()),
		("PT3", b.pt.pt3.degrade_adc()),
		("PT4", b.pt.pt4.degrade_adc()),
	];
	let mut cam1_is = b.cameras.cam1_is.degrade_adc();
	let mut cam2_is = b.cameras.cam2_is.degrade_adc();
	let st = SampleTime::CYCLES480;

	// Factory temp sensor calibration (RM0390 / DS10693), measured at VDDA = 3.3 V
	let ts_cal1 = unsafe { core::ptr::read_volatile(0x1FFF_7A2C as *const u16) } as f32;
	let ts_cal2 = unsafe { core::ptr::read_volatile(0x1FFF_7A2E as *const u16) } as f32;

	let vref_raw = adc.blocking_read(&mut vrefint, st) as u32;
	let vdda_mv = board::vdda_mv(vref_raw, 1);
	info!("VREFINT raw={} cal={} -> VDDA = {} mV", vref_raw, board::vrefint_cal(), vdda_mv);
	report.check("VDDA within 3.2-3.4 V (3.3V LDO)", (3200..=3400).contains(&vdda_mv));

	let to_mv = |raw: u16| raw as u32 * vdda_mv / 4095;
	// Temp sensor cal is referenced to 3.3 V, so rescale the reading to that.
	let t_raw = adc.blocking_read(&mut temp_ch, st) as f32 * vdda_mv as f32 / 3300.0;
	let mcu_temp = 30.0 + (t_raw - ts_cal1) * (110.0 - 30.0) / (ts_cal2 - ts_cal1);
	info!("MCU die temperature = {} C", mcu_temp);
	report.check("MCU die temperature plausible (0-70 C)", (0.0..70.0).contains(&mcu_temp));

	for (name, ch) in pt.iter_mut() {
		let raw = adc.blocking_read(ch, st);
		let pin_mv = to_mv(raw);
		let in_mv = board::pt_input_mv(raw as u32, 1, vdda_mv);
		info!("{}: raw={} pin={} mV -> input={} mV (assuming 10k/15k divider)", name, raw, pin_mv, in_mv);
	}
	let cam1_raw = adc.blocking_read(&mut cam1_is, st);
	let cam2_off_raw = adc.blocking_read(&mut cam2_is, st);
	info!("CAM1_IS={} mV (PROFET removed)  CAM2_IS (off)={} mV", to_mv(cam1_raw), to_mv(cam2_off_raw));

	// ------------------------------------------------------------ Flash
	section("W25Q40 flash (SPI2)");
	{
		let mut cfg = spi::Config::default();
		cfg.frequency = Hertz(8_000_000);
		cfg.mode = spi::MODE_0;
		let bus = Spi::new(b.flash.spi, b.flash.sck, b.flash.mosi, b.flash.miso, b.flash.tx_dma, b.flash.rx_dma, Irqs, cfg);
		let cs = Output::new(b.flash.cs, Level::High, Speed::VeryHigh);
		let dev = ExclusiveDevice::new(bus, cs, Delay).unwrap();
		let mut flash = W25q::new(dev, 512 * 1024);
		flash_test(&mut flash, &mut report).await;
	}

	// ------------------------------------------------------------ IMU
	section("LSM6DSV16BX IMU (SPI1)");
	let mut spi1_cfg = spi::Config::default();
	spi1_cfg.frequency = Hertz(8_000_000);
	spi1_cfg.mode = spi::MODE_3;
	let spi1 = Spi::new(b.imu.spi, b.imu.sck, b.imu.mosi, b.imu.miso, b.imu.tx_dma, b.imu.rx_dma, Irqs, spi1_cfg);
	let imu_cs = Output::new(b.imu.cs, Level::High, Speed::VeryHigh);
	let mut imu_dev = ExclusiveDevice::new(spi1, imu_cs, Delay).unwrap();
	imu_spi_probe(&mut imu_dev, &spi1_cfg).await;
	let mut imu = Lsm6dsv16bx::new(imu_dev);
	let mut int1 = ExtiInput::new(b.imu.int1, b.imu.int1_exti, Pull::Down, Irqs);
	let mut int2 = ExtiInput::new(b.imu.int2, b.imu.int2_exti, Pull::Down, Irqs);
	let imu_ok = imu_test(&mut imu, &mut int1, &mut int2, &mut report).await;

	// ------------------------------------------------------------ Baro
	section("MS5611 barometer (I2C1)");
	let mut i2c_cfg = i2c::Config::default();
	i2c_cfg.frequency = Hertz(400_000);
	i2c_cfg.timeout = Duration::from_millis(20);
	// No bus scan: embassy 0.6's I2C v1 times out on address-only probes, and
	// the MS5611 is the only device on this bus anyway.
	let i2c = I2c::new(b.baro.i2c, b.baro.scl, b.baro.sda, b.baro.tx_dma, b.baro.rx_dma, Irqs, i2c_cfg);
	let mut baro = Ms5611::new(i2c, board::MS5611_ADDR);
	let baro_ok = baro_test(&mut baro, &mut report).await;

	// ------------------------------------------------------------ Radio
	section("RFD900 (USART6)");
	{
		let mut cfg = usart::Config::default();
		cfg.baudrate = 57_600;
		match Uart::new(b.radio.uart, b.radio.rx, b.radio.tx, b.radio.tx_dma, b.radio.rx_dma, Irqs, cfg) {
			Ok(mut uart) => radio_test(&mut uart, &mut report).await,
			Err(e) => {
				error!("USART6 config error: {}", e);
				report.record("USART6 init", Status::Fail);
			}
		}
	}

	// ------------------------------------------------------------ CAN
	section("CAN1 + SN65HVD230");
	{
		let mut can = Can::new(b.can.can, b.can.rx, b.can.tx, Irqs);
		can_test(&mut can, &mut report).await;
	}

	// ------------------------------------------------------------ Camera 2
	section("Camera 2 PROFET");
	if TEST_CAMERA2_OUTPUT {
		cam2.set_high();
		Timer::after_millis(500).await;
		let on_raw = adc.blocking_read(&mut cam2_is, st);
		cam2.set_low();
		info!("CAM2_IS off={} mV  on={} mV", to_mv(cam2_off_raw), to_mv(on_raw));
		info!("(a change in IS shows the PROFET switched; with no camera load IS stays near 0)");
		report.record("Camera 2 switched for 500 ms - check IS values above / camera power LED", Status::Pass);
	} else {
		info!("skipped (TEST_CAMERA2_OUTPUT = false)");
	}

	report.summary();

	// ------------------------------------------------------------ live data
	info!("Streaming live data every 1 s (Ctrl+C to stop)");
	loop {
		if imu_ok {
			match imu.read().await {
				Ok(s) => info!(
					"IMU a=[{=f32}, {=f32}, {=f32}] g  w=[{=f32}, {=f32}, {=f32}] dps  T={=f32} C",
					s.accel_g[0], s.accel_g[1], s.accel_g[2], s.gyro_dps[0], s.gyro_dps[1], s.gyro_dps[2], s.temp_c
				),
				Err(e) => warn!("IMU read error: {}", Debug2Format(&e)),
			}
		}
		if baro_ok {
			match baro.measure(Osr::X4096).await {
				Ok(m) => info!("BARO P={} Pa  T={} cC", m.pressure_pa, m.temp_centi_c),
				Err(e) => warn!("Baro read error: {}", e),
			}
		}
		let mut line = [0u32; 4];
		for (i, (_, ch)) in pt.iter_mut().enumerate() {
			line[i] = board::pt_input_mv(adc.blocking_read(ch, st) as u32, 1, vdda_mv);
		}
		info!("PT input mV = {}", line);
		Timer::after_secs(1).await;
	}
}

// ---------------------------------------------------------------- tests

async fn flash_test<S: embedded_hal_async::spi::SpiDevice>(flash: &mut W25q<S>, report: &mut Report)
where
	S::Error: core::fmt::Debug,
{
	if let Err(e) = flash.init().await {
		error!("flash init: {}", Debug2Format(&e));
	}
	let id = match flash.jedec_id().await {
		Ok(id) => id,
		Err(e) => {
			error!("JEDEC read failed: {}", Debug2Format(&e));
			report.record("Flash JEDEC ID read", Status::Fail);
			return;
		}
	};
	info!("JEDEC ID: mfr=0x{:02x} type=0x{:02x} cap=0x{:02x} ({} bytes)", id.manufacturer, id.memory_type, id.capacity, id.capacity_bytes());
	if id.manufacturer == 0x00 || id.manufacturer == 0xFF {
		report.record("Flash not responding (ID all 0x00/0xFF) - check CS/SCK/MISO/MOSI, VCC", Status::Fail);
		return;
	}
	report.check("Flash JEDEC ID = Winbond W25Q40 (EF 40 13)", id == w25q::JedecId { manufacturer: 0xEF, memory_type: 0x40, capacity: 0x13 });

	if let Ok(uid) = flash.unique_id().await {
		info!("Flash unique ID: {:016x}", uid);
	}
	if let Ok(sr) = flash.status().await {
		info!("SR1=0x{:02x} SR2=0x{:02x} SR3=0x{:02x}", sr[0], sr[1], sr[2]);
		// Block-protect bits BP0-2 / TB / SEC in SR1 bits 2..6
		report.check("Flash not write-protected (SR1 BP bits clear)", sr[0] & 0x7C == 0);
	}

	// Erase -> blank check -> program -> verify -> erase, on the last sector only.
	let mut buf = [0u8; w25q::PAGE_SIZE];
	let t0 = Instant::now();
	let erase = flash.erase_sector(FLASH_TEST_ADDR).await;
	let erase_ms = t0.elapsed().as_millis();
	if let Err(e) = erase {
		error!("sector erase: {}", Debug2Format(&e));
		report.record("Flash sector erase", Status::Fail);
		return;
	}
	info!("sector erase took {} ms", erase_ms);
	let blank = flash.read(FLASH_TEST_ADDR, &mut buf).await.is_ok() && buf.iter().all(|&b| b == 0xFF);
	report.check("Flash sector erases to 0xFF", blank);

	let mut pattern = [0u8; w25q::PAGE_SIZE];
	for (i, b) in pattern.iter_mut().enumerate() {
		*b = (i as u8).wrapping_mul(37) ^ 0xA5;
	}
	let t0 = Instant::now();
	let prog = flash.program_page(FLASH_TEST_ADDR, &pattern).await;
	let prog_us = t0.elapsed().as_micros();
	if let Err(e) = prog {
		error!("page program: {}", Debug2Format(&e));
		report.record("Flash page program", Status::Fail);
		return;
	}
	info!("page program took {} us", prog_us);
	buf.fill(0);
	let verify = flash.read(FLASH_TEST_ADDR, &mut buf).await.is_ok() && buf == pattern;
	if !verify {
		let bad = buf.iter().zip(pattern.iter()).filter(|(a, b)| a != b).count();
		error!("{} of 256 bytes mismatched; first bytes read {:02x}", bad, buf[..16]);
	}
	report.check("Flash program + read-back verify (256 B)", verify);
	let _ = flash.erase_sector(FLASH_TEST_ADDR).await;
}

/// Raw WHO_AM_I read. If it fails at the normal settings, retry slower in both
/// SPI modes to rule out a timing/mode problem before blaming hardware.
async fn imu_spi_probe<'a>(
	dev: &mut ExclusiveDevice<Spi<'a, embassy_stm32::mode::Async, spi::mode::Master>, Output<'a>, Delay>,
	normal: &spi::Config,
) {
	use embedded_hal_async::spi::{Operation, SpiDevice};
	async fn who<D: SpiDevice>(d: &mut D) -> u8 {
		let mut b = [0u8; 1];
		match d.transaction(&mut [Operation::Write(&[0x0F | 0x80]), Operation::Read(&mut b)]).await {
			Ok(()) => b[0],
			Err(_) => 0xEE,
		}
	}
	if who(dev).await == lsm6dsv16bx::WHO_AM_I_VALUE {
		return;
	}
	warn!("IMU WHO_AM_I wrong at normal settings; retrying at 1 MHz in both SPI modes:");
	for (mname, mode) in [("MODE0", spi::MODE_0), ("MODE3", spi::MODE_3)] {
		let mut cfg = *normal;
		cfg.frequency = Hertz(1_000_000);
		cfg.mode = mode;
		let _ = dev.bus_mut().set_config(&cfg);
		Timer::after_micros(100).await;
		info!("  {}: WHO_AM_I = 0x{:02x}", mname, who(dev).await);
	}
	let _ = dev.bus_mut().set_config(normal);
}

async fn imu_test<S: embedded_hal_async::spi::SpiDevice>(
	imu: &mut Lsm6dsv16bx<S>,
	int1: &mut ExtiInput<'_, embassy_stm32::mode::Async>,
	int2: &mut ExtiInput<'_, embassy_stm32::mode::Async>,
	report: &mut Report,
) -> bool
where
	S::Error: core::fmt::Debug,
{
	let id = match imu.who_am_i().await {
		Ok(id) => id,
		Err(e) => {
			error!("SPI error: {}", Debug2Format(&e));
			report.record("IMU WHO_AM_I read", Status::Fail);
			return false;
		}
	};
	info!("WHO_AM_I = 0x{:02x} (expect 0x71)", id);
	if !report.check("IMU WHO_AM_I = 0x71", id == lsm6dsv16bx::WHO_AM_I_VALUE) {
		return false;
	}
	if let Err(e) = imu.init().await {
		error!("IMU init: {}", Debug2Format(&e));
		report.record("IMU software reset", Status::Fail);
		return false;
	}
	let cfg = lsm6dsv16bx::Config {
		accel_odr: Odr::Hz120,
		accel_scale: AccelScale::G4,
		gyro_odr: Odr::Hz120,
		gyro_scale: GyroScale::Dps250,
	};
	if imu.configure(cfg).await.is_err() || imu.enable_drdy_interrupts(true, true).await.is_err() {
		report.record("IMU configure", Status::Fail);
		return false;
	}
	Timer::after_millis(100).await; // let the filters settle

	// Average 32 samples at rest
	let mut acc = [0f32; 3];
	let mut gyr = [0f32; 3];
	let mut temp = 0f32;
	let n = 32;
	for _ in 0..n {
		Timer::after_millis(10).await;
		match imu.read().await {
			Ok(s) => {
				for i in 0..3 {
					acc[i] += s.accel_g[i] / n as f32;
					gyr[i] += s.gyro_dps[i] / n as f32;
				}
				temp += s.temp_c / n as f32;
			}
			Err(e) => {
				error!("IMU read: {}", Debug2Format(&e));
				report.record("IMU data read", Status::Fail);
				return false;
			}
		}
	}
	let mag2 = acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2];
	info!("accel avg = [{=f32}, {=f32}, {=f32}] g  |a|^2 = {=f32}", acc[0], acc[1], acc[2], mag2);
	info!("gyro  avg = [{=f32}, {=f32}, {=f32}] dps", gyr[0], gyr[1], gyr[2]);
	info!("IMU temp  = {=f32} C", temp);
	report.check("IMU at rest reads ~1 g (0.9-1.1 g)", (0.81..1.21).contains(&mag2));
	let gyro_ok = gyr.iter().all(|w| w.abs() < 5.0);
	report.check("IMU gyro at rest < 5 dps on all axes", gyro_ok);

	// Data-ready interrupt check, 120 Hz ODR -> expect ~60 in 500 ms.
	let n1 = count_drdy(imu, int1).await;
	info!("INT1 accel DRDY (PA15): {} edges in 500 ms (~{} Hz)", n1, n1 * 2);
	report.check("IMU_Int1 toggles at accel ODR (~120 Hz)", (48..=72).contains(&n1));
	let n2 = count_drdy(imu, int2).await;
	info!("INT2 gyro DRDY (PC11): {} edges in 500 ms (~{} Hz)", n2, n2 * 2);
	report.check("IMU_Int2 toggles at gyro ODR (~120 Hz)", (48..=72).contains(&n2));
	true
}

/// Each read clears the latched DRDY; count how many arrive in 500 ms.
async fn count_drdy<S: embedded_hal_async::spi::SpiDevice>(
	imu: &mut Lsm6dsv16bx<S>,
	pin: &mut ExtiInput<'_, embassy_stm32::mode::Async>,
) -> u32 {
	let _ = imu.read_raw().await; // clear any pending DRDY
	let start = Instant::now();
	let mut count = 0u32;
	while start.elapsed() < Duration::from_millis(500) {
		if with_timeout(Duration::from_millis(50), pin.wait_for_high()).await.is_err() {
			break;
		}
		count += 1;
		let _ = imu.read_raw().await;
	}
	count
}

async fn baro_test<I: embedded_hal_async::i2c::I2c>(baro: &mut Ms5611<I>, report: &mut Report) -> bool
where
	I::Error: Format,
{
	match baro.init().await {
		Ok(cal) => {
			info!("PROM: {}", cal.prom);
			report.record("MS5611 reset + PROM CRC4 OK", Status::Pass);
		}
		Err(e) => {
			error!("MS5611 init: {}", e);
			report.record("MS5611 responds at 0x77 with valid PROM", Status::Fail);
			return false;
		}
	}
	match baro.measure(Osr::X4096).await {
		Ok(m) => {
			info!("Pressure = {} Pa ({} mbar)  Temp = {} cC", m.pressure_pa, m.pressure_pa / 100, m.temp_centi_c);
			// College Station ~ 100 kPa; allow anywhere a human could be standing
			report.check("MS5611 pressure plausible (70-110 kPa)", (70_000..110_000).contains(&m.pressure_pa));
			report.check("MS5611 temperature plausible (0-60 C)", (0..6000).contains(&m.temp_centi_c));
			true
		}
		Err(e) => {
			error!("MS5611 measure: {}", e);
			report.record("MS5611 conversion", Status::Fail);
			false
		}
	}
}

async fn radio_test(uart: &mut Uart<'_, embassy_stm32::mode::Async>, report: &mut Report) {
	let mut buf = [0u8; 128];
	for baud in [57_600u32, 115_200] {
		if uart.set_baudrate(baud).is_err() {
			continue;
		}
		info!("trying {} baud: '+++' (needs 1 s guard time either side)", baud);
		Timer::after_millis(1100).await;
		if uart.write(b"+++").await.is_err() {
			continue;
		}
		let n = read_for(uart, &mut buf, Duration::from_millis(2000)).await;
		let resp = &buf[..n];
		info!("  response: {=[u8]:a}", resp);
		if !contains(resp, b"OK") {
			continue;
		}
		report.record("RFD900 entered AT command mode", Status::Pass);

		let _ = uart.write(b"ATI\r\n").await;
		let n = read_for(uart, &mut buf, Duration::from_millis(500)).await;
		info!("  ATI  -> {=[u8]:a}", &buf[..n]);
		let _ = uart.write(b"ATI5\r\n").await;
		let n = read_for(uart, &mut buf, Duration::from_millis(500)).await;
		info!("  ATI5 -> {=[u8]:a}", &buf[..n]);
		let _ = uart.write(b"ATO\r\n").await;
		let _ = read_for(uart, &mut buf, Duration::from_millis(200)).await;
		info!("RFD900 found at {} baud, returned to data mode", baud);
		return;
	}
	report.record("RFD900 no AT response at 57600/115200 - check power, TX/RX swap, baud", Status::Fail);
}

/// Collect bytes until `window` elapses.
async fn read_for(uart: &mut Uart<'_, embassy_stm32::mode::Async>, buf: &mut [u8], window: Duration) -> usize {
	let deadline = Instant::now() + window;
	let mut n = 0;
	while n < buf.len() {
		let now = Instant::now();
		if now >= deadline {
			break;
		}
		match with_timeout(deadline - now, uart.read_until_idle(&mut buf[n..])).await {
			Ok(Ok(k)) => n += k,
			_ => break,
		}
	}
	n
}

fn contains(hay: &[u8], needle: &[u8]) -> bool {
	hay.windows(needle.len()).any(|w| w == needle)
}

async fn can_test(can: &mut Can<'_>, report: &mut Report) {
	can.modify_config()
		.set_bitrate(can_map::BITRATE)
		.set_loopback(false)
		.set_silent(false);
	// One-shot transmit. Written directly because embassy-stm32 0.6.0's
	// set_automatic_retransmit(enabled) writes `enabled` straight into NART
	// ("No Automatic ReTransmission"), i.e. the meaning is inverted.
	pac::CAN1.mcr().modify(|w| w.set_nart(true));

	// Enabling waits for 11 recessive bits. A stuck-dominant RX never syncs.
	if with_timeout(Duration::from_millis(200), can.enable()).await.is_err() {
		report.record("CAN never saw an idle bus - RX stuck low (transceiver unpowered / CANH-CANL short?)", Status::Fail);
		return;
	}
	report.record("CAN controller synchronized to idle bus", Status::Pass);

	// Set LEC to the software code (7) so we know the next value came from this frame.
	pac::CAN1.esr().modify(|w| w.set_lec(pac::can::vals::Lec::CUSTOM));
	let frame = Frame::new_standard(0x123, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
	let status = can.write(&frame).await;
	if with_timeout(Duration::from_millis(50), can.flush(status.mailbox())).await.is_err() {
		can.abort(status.mailbox());
	}
	Timer::after_millis(5).await;

	let esr = pac::CAN1.esr().read();
	info!("ESR: LEC={} TEC={} REC={} BOFF={}", esr.lec().to_bits(), esr.tec(), esr.rec(), esr.boff());
	use pac::can::vals::Lec;
	match esr.lec() {
		Lec::NO_ERROR => report.record("CAN frame ACKed by another node on the bus", Status::Pass),
		Lec::ACK => {
			info!("ACK error = our own bits came back correctly through the transceiver,");
			info!("but no other node acknowledged. Expected with nothing else on the bus.");
			report.record("CAN TX->transceiver->RX loop OK (no other node to ACK)", Status::Pass);
		}
		Lec::BIT_RECESSIVE | Lec::BIT_DOMINANT => {
			report.record("CAN bit error - TX/RX path or transceiver wiring problem", Status::Fail)
		}
		Lec::STUFF | Lec::FORM | Lec::CRC => {
			report.record("CAN stuff/form/CRC error - bit-rate mismatch with another node or noisy bus", Status::Warn)
		}
		Lec::CUSTOM => report.record("CAN frame never went out (LEC unchanged)", Status::Fail),
	}
}
