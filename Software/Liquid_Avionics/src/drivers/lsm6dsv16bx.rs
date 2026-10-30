// LSM6DSV16BX 6-axis IMU (+/-16 g accel, +/-4000 dps gyro), SPI interface.
// Register map from ST's lsm6dsv16bx-pid driver (lsm6dsv16bx_reg.h).
// Note: this part outputs accelerometer data in Z, Y, X register order.
use embassy_time::Timer;
use embedded_hal_async::spi::{Operation, SpiDevice};

pub const WHO_AM_I_VALUE: u8 = 0x71;

mod reg {
	pub const IF_CFG: u8 = 0x03;
	pub const INT1_CTRL: u8 = 0x0D;
	pub const INT2_CTRL: u8 = 0x0E;
	pub const WHO_AM_I: u8 = 0x0F;
	pub const CTRL1: u8 = 0x10; // accel ODR + mode
	pub const CTRL2: u8 = 0x11; // gyro ODR + mode
	pub const CTRL3: u8 = 0x12;
	pub const CTRL6: u8 = 0x15; // gyro full scale
	pub const CTRL8: u8 = 0x17; // accel full scale
	pub const STATUS_REG: u8 = 0x1E;
	pub const OUT_TEMP_L: u8 = 0x20;
}

const SPI_READ: u8 = 0x80;

#[derive(Debug, defmt::Format)]
pub enum Error<E> {
	Bus(E),
	/// WHO_AM_I returned something other than 0x71
	WrongId(u8),
	/// Software reset bit never self-cleared
	ResetTimeout,
}

impl<E> From<E> for Error<E> {
	fn from(e: E) -> Self {
		Error::Bus(e)
	}
}

/// Output data rate, shared encoding for accel (CTRL1) and gyro (CTRL2).
/// (Accel also supports 1.875 Hz; omitted since the gyro does not.)
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Odr {
	PowerDown = 0x0,
	Hz7_5 = 0x2,
	Hz15 = 0x3,
	Hz30 = 0x4,
	Hz60 = 0x5,
	Hz120 = 0x6,
	Hz240 = 0x7,
	Hz480 = 0x8,
	Hz960 = 0x9,
	Hz1920 = 0xA,
	Hz3840 = 0xB,
	Hz7680 = 0xC,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum AccelScale {
	G2 = 0x0,
	G4 = 0x1,
	G8 = 0x2,
	G16 = 0x3,
}

impl AccelScale {
	/// mg per LSB
	pub fn sensitivity_mg(self) -> f32 {
		match self {
			AccelScale::G2 => 0.061,
			AccelScale::G4 => 0.122,
			AccelScale::G8 => 0.244,
			AccelScale::G16 => 0.488,
		}
	}
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum GyroScale {
	Dps125 = 0x0,
	Dps250 = 0x1,
	Dps500 = 0x2,
	Dps1000 = 0x3,
	Dps2000 = 0x4,
	Dps4000 = 0xC,
}

impl GyroScale {
	/// mdps per LSB
	pub fn sensitivity_mdps(self) -> f32 {
		match self {
			GyroScale::Dps125 => 4.375,
			GyroScale::Dps250 => 8.75,
			GyroScale::Dps500 => 17.5,
			GyroScale::Dps1000 => 35.0,
			GyroScale::Dps2000 => 70.0,
			GyroScale::Dps4000 => 140.0,
		}
	}
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Config {
	pub accel_odr: Odr,
	pub accel_scale: AccelScale,
	pub gyro_odr: Odr,
	pub gyro_scale: GyroScale,
}

/// Raw counts, axes already reordered to X, Y, Z.
#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct RawSample {
	pub temp: i16,
	pub gyro: [i16; 3],
	pub accel: [i16; 3],
}

#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct Sample {
	pub temp_c: f32,
	pub gyro_dps: [f32; 3],
	pub accel_g: [f32; 3],
}

pub struct Lsm6dsv16bx<S> {
	spi: S,
	config: Config,
}

impl<S: SpiDevice> Lsm6dsv16bx<S> {
	pub fn new(spi: S) -> Self {
		Self {
			spi,
			config: Config {
				accel_odr: Odr::PowerDown,
				accel_scale: AccelScale::G2,
				gyro_odr: Odr::PowerDown,
				gyro_scale: GyroScale::Dps125,
			},
		}
	}

	pub async fn read_reg(&mut self, addr: u8) -> Result<u8, Error<S::Error>> {
		let mut buf = [0u8; 1];
		self.read_regs(addr, &mut buf).await?;
		Ok(buf[0])
	}

	/// Burst read; relies on IF_INC (on by default) to auto-increment.
	pub async fn read_regs(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Error<S::Error>> {
		self.spi
			.transaction(&mut [Operation::Write(&[addr | SPI_READ]), Operation::Read(buf)])
			.await?;
		Ok(())
	}

	pub async fn write_reg(&mut self, addr: u8, value: u8) -> Result<(), Error<S::Error>> {
		self.spi.write(&[addr & !SPI_READ, value]).await?;
		Ok(())
	}

	pub async fn who_am_i(&mut self) -> Result<u8, Error<S::Error>> {
		self.read_reg(reg::WHO_AM_I).await
	}

	/// Check ID, software reset, and put the part in SPI-only mode with
	/// block data update. Sensors are left powered down.
	pub async fn init(&mut self) -> Result<(), Error<S::Error>> {
		let id = self.who_am_i().await?;
		if id != WHO_AM_I_VALUE {
			return Err(Error::WrongId(id));
		}

		self.write_reg(reg::CTRL3, 0x01).await?; // SW_RESET
		let mut reset_done = false;
		for _ in 0..10 {
			Timer::after_millis(1).await;
			if self.read_reg(reg::CTRL3).await? & 0x01 == 0 {
				reset_done = true;
				break;
			}
		}
		if !reset_done {
			return Err(Error::ResetTimeout);
		}

		self.write_reg(reg::CTRL3, 0x44).await?; // BDU | IF_INC
		self.write_reg(reg::IF_CFG, 0x01).await?; // I2C_I3C_disable (SPI only)
		Ok(())
	}

	/// Both sensors run in high-performance mode (OP_MODE = 000).
	pub async fn configure(&mut self, config: Config) -> Result<(), Error<S::Error>> {
		self.write_reg(reg::CTRL8, config.accel_scale as u8).await?;
		self.write_reg(reg::CTRL6, config.gyro_scale as u8).await?;
		self.write_reg(reg::CTRL1, config.accel_odr as u8).await?;
		self.write_reg(reg::CTRL2, config.gyro_odr as u8).await?;
		self.config = config;
		Ok(())
	}

	/// Route data-ready signals to the INT pins.
	/// INT1 = accel data ready, INT2 = gyro data ready (latched until read).
	pub async fn enable_drdy_interrupts(&mut self, int1_accel: bool, int2_gyro: bool) -> Result<(), Error<S::Error>> {
		self.write_reg(reg::INT1_CTRL, int1_accel as u8).await?; // INT1_DRDY_XL
		self.write_reg(reg::INT2_CTRL, (int2_gyro as u8) << 1).await?; // INT2_DRDY_G
		Ok(())
	}

	/// STATUS_REG: bit0 XLDA, bit1 GDA, bit2 TDA
	pub async fn status(&mut self) -> Result<u8, Error<S::Error>> {
		self.read_reg(reg::STATUS_REG).await
	}

	/// One 14-byte burst: temp, gyro XYZ, accel ZYX.
	pub async fn read_raw(&mut self) -> Result<RawSample, Error<S::Error>> {
		let mut b = [0u8; 14];
		self.read_regs(reg::OUT_TEMP_L, &mut b).await?;
		let w = |i: usize| i16::from_le_bytes([b[i], b[i + 1]]);
		Ok(RawSample {
			temp: w(0),
			gyro: [w(2), w(4), w(6)],
			accel: [w(12), w(10), w(8)],
		})
	}

	pub async fn read(&mut self) -> Result<Sample, Error<S::Error>> {
		let raw = self.read_raw().await?;
		Ok(self.scale(&raw))
	}

	pub fn scale(&self, raw: &RawSample) -> Sample {
		let a = self.config.accel_scale.sensitivity_mg() / 1000.0;
		let g = self.config.gyro_scale.sensitivity_mdps() / 1000.0;
		Sample {
			temp_c: 25.0 + raw.temp as f32 / 256.0,
			gyro_dps: raw.gyro.map(|v| v as f32 * g),
			accel_g: raw.accel.map(|v| v as f32 * a),
		}
	}
}
