// MS5611-01BA03 barometric pressure sensor, I2C interface.
// Reference: MS5611-01BA03 datasheet + TE AN520 (PROM CRC).
use embassy_time::Timer;
use embedded_hal_async::i2c::I2c;

const CMD_RESET: u8 = 0x1E;
const CMD_CONVERT_D1: u8 = 0x40; // pressure
const CMD_CONVERT_D2: u8 = 0x50; // temperature
const CMD_ADC_READ: u8 = 0x00;
const CMD_PROM_READ: u8 = 0xA0;

#[derive(Debug, defmt::Format)]
pub enum Error<E> {
	Bus(E),
	/// PROM CRC4 mismatch: (computed, stored)
	Crc(u8, u8),
	/// ADC read returned 0, meaning the conversion was not finished or was
	/// interrupted by another command.
	NotReady,
}

impl<E> From<E> for Error<E> {
	fn from(e: E) -> Self {
		Error::Bus(e)
	}
}

/// Oversampling ratio. Higher = lower noise, longer conversion.
#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum Osr {
	X256 = 0x00,
	X512 = 0x02,
	X1024 = 0x04,
	X2048 = 0x06,
	X4096 = 0x08,
}

impl Osr {
	/// Max conversion time from the datasheet, rounded up.
	pub fn conversion_time_us(self) -> u64 {
		match self {
			Osr::X256 => 600,
			Osr::X512 => 1_170,
			Osr::X1024 => 2_280,
			Osr::X2048 => 4_540,
			Osr::X4096 => 9_040,
		}
	}
}

/// Factory calibration coefficients C1..C6 (PROM words 1..6).
#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Calibration {
	pub prom: [u16; 8],
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct Measurement {
	/// Pressure in Pa (== 0.01 mbar)
	pub pressure_pa: i32,
	/// Temperature in 0.01 degC
	pub temp_centi_c: i32,
}

pub struct Ms5611<I> {
	i2c: I,
	addr: u8,
	cal: Option<Calibration>,
}

impl<I: I2c> Ms5611<I> {
	pub fn new(i2c: I, addr: u8) -> Self {
		Self { i2c, addr, cal: None }
	}

	pub fn calibration(&self) -> Option<Calibration> {
		self.cal
	}

	/// Reset and load + verify calibration PROM. Call once at startup.
	pub async fn init(&mut self) -> Result<Calibration, Error<I::Error>> {
		self.reset().await?;
		let cal = self.read_prom().await?;
		self.cal = Some(cal);
		Ok(cal)
	}

	pub async fn reset(&mut self) -> Result<(), Error<I::Error>> {
		self.i2c.write(self.addr, &[CMD_RESET]).await?;
		Timer::after_millis(3).await; // 2.8 ms reload time
		Ok(())
	}

	pub async fn read_prom(&mut self) -> Result<Calibration, Error<I::Error>> {
		let mut prom = [0u16; 8];
		for (i, word) in prom.iter_mut().enumerate() {
			let mut buf = [0u8; 2];
			self.i2c.write_read(self.addr, &[CMD_PROM_READ + 2 * i as u8], &mut buf).await?;
			*word = u16::from_be_bytes(buf);
		}
		let computed = crc4(&prom);
		let stored = (prom[7] & 0x000F) as u8;
		if computed != stored {
			return Err(Error::Crc(computed, stored));
		}
		Ok(Calibration { prom })
	}

	/// Start a pressure (D1) conversion. Read it with `read_adc` after
	/// `osr.conversion_time_us()`. Split out so the flight loop can do other
	/// work during the conversion instead of awaiting `measure`.
	pub async fn start_pressure(&mut self, osr: Osr) -> Result<(), Error<I::Error>> {
		self.i2c.write(self.addr, &[CMD_CONVERT_D1 | osr as u8]).await?;
		Ok(())
	}

	/// Start a temperature (D2) conversion.
	pub async fn start_temperature(&mut self, osr: Osr) -> Result<(), Error<I::Error>> {
		self.i2c.write(self.addr, &[CMD_CONVERT_D2 | osr as u8]).await?;
		Ok(())
	}

	/// Read the 24-bit result of the last conversion.
	pub async fn read_adc(&mut self) -> Result<u32, Error<I::Error>> {
		let mut buf = [0u8; 3];
		self.i2c.write_read(self.addr, &[CMD_ADC_READ], &mut buf).await?;
		let v = u32::from_be_bytes([0, buf[0], buf[1], buf[2]]);
		if v == 0 {
			return Err(Error::NotReady);
		}
		Ok(v)
	}

	/// Convenience: full D1 + D2 cycle and compensation.
	pub async fn measure(&mut self, osr: Osr) -> Result<Measurement, Error<I::Error>> {
		let cal = match self.cal {
			Some(c) => c,
			None => self.init().await?,
		};
		self.start_temperature(osr).await?;
		Timer::after_micros(osr.conversion_time_us()).await;
		let d2 = self.read_adc().await?;

		self.start_pressure(osr).await?;
		Timer::after_micros(osr.conversion_time_us()).await;
		let d1 = self.read_adc().await?;

		Ok(compensate(&cal, d1, d2))
	}
}

/// First + second order temperature compensation (datasheet p.7-8).
pub fn compensate(cal: &Calibration, d1: u32, d2: u32) -> Measurement {
	let c = |i: usize| cal.prom[i] as i64;
	let d1 = d1 as i64;
	let d2 = d2 as i64;

	let dt = d2 - (c(5) << 8);
	let mut temp = 2000 + ((dt * c(6)) >> 23);
	let mut off = (c(2) << 16) + ((c(4) * dt) >> 7);
	let mut sens = (c(1) << 15) + ((c(3) * dt) >> 8);

	if temp < 2000 {
		let t2 = (dt * dt) >> 31;
		let low = (temp - 2000) * (temp - 2000);
		let mut off2 = 5 * low / 2;
		let mut sens2 = 5 * low / 4;
		if temp < -1500 {
			let very_low = (temp + 1500) * (temp + 1500);
			off2 += 7 * very_low;
			sens2 += 11 * very_low / 2;
		}
		temp -= t2;
		off -= off2;
		sens -= sens2;
	}

	let p = (((d1 * sens) >> 21) - off) >> 15;
	Measurement { pressure_pa: p as i32, temp_centi_c: temp as i32 }
}

/// CRC4 over the 8 PROM words, per TE AN520.
pub fn crc4(prom: &[u16; 8]) -> u8 {
	let mut words = *prom;
	words[7] &= 0xFF00; // CRC lives in the low nibble of word 7
	let mut rem: u16 = 0;
	for cnt in 0..16 {
		if cnt % 2 == 1 {
			rem ^= words[cnt >> 1] & 0x00FF;
		} else {
			rem ^= words[cnt >> 1] >> 8;
		}
		for _ in 0..8 {
			rem = if rem & 0x8000 != 0 { (rem << 1) ^ 0x3000 } else { rem << 1 };
		}
	}
	((rem >> 12) & 0xF) as u8
}
