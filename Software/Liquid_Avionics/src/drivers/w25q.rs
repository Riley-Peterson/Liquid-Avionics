// Winbond W25Qxx SPI NOR flash (board has a W25Q40, 4 Mbit / 512 KiB).
// Reference: Winbond W25Q40 datasheet. Standard SPI only (no dual/quad).
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::{Operation, SpiDevice};

pub const PAGE_SIZE: usize = 256;
pub const SECTOR_SIZE: u32 = 4096;

mod cmd {
	pub const WRITE_ENABLE: u8 = 0x06;
	pub const READ_SR1: u8 = 0x05;
	pub const READ_SR2: u8 = 0x35;
	pub const READ_SR3: u8 = 0x15;
	pub const READ_DATA: u8 = 0x03;
	pub const PAGE_PROGRAM: u8 = 0x02;
	pub const SECTOR_ERASE: u8 = 0x20;
	pub const CHIP_ERASE: u8 = 0xC7;
	pub const RELEASE_POWER_DOWN: u8 = 0xAB;
	pub const JEDEC_ID: u8 = 0x9F;
	pub const UNIQUE_ID: u8 = 0x4B;
	pub const ENABLE_RESET: u8 = 0x66;
	pub const RESET: u8 = 0x99;
}

const SR1_BUSY: u8 = 0x01;

#[derive(Debug, defmt::Format)]
pub enum Error<E> {
	Bus(E),
	/// Address/length outside the chip or a page-program crossing a page
	OutOfBounds,
	/// BUSY never cleared
	Timeout,
}

impl<E> From<E> for Error<E> {
	fn from(e: E) -> Self {
		Error::Bus(e)
	}
}

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub struct JedecId {
	pub manufacturer: u8,
	pub memory_type: u8,
	pub capacity: u8,
}

impl JedecId {
	/// Capacity in bytes from the JEDEC capacity code (2^code)
	pub fn capacity_bytes(&self) -> u32 {
		1u32.checked_shl(self.capacity as u32).unwrap_or(0)
	}
}

pub struct W25q<S> {
	spi: S,
	capacity: u32,
}

impl<S: SpiDevice> W25q<S> {
	/// `capacity` in bytes, used for bounds checks.
	pub fn new(spi: S, capacity: u32) -> Self {
		Self { spi, capacity }
	}

	pub fn capacity(&self) -> u32 {
		self.capacity
	}

	/// Wake from power-down and software-reset the device.
	pub async fn init(&mut self) -> Result<(), Error<S::Error>> {
		self.spi.write(&[cmd::RELEASE_POWER_DOWN]).await?;
		Timer::after_micros(30).await; // tRES1 = 3 us
		self.spi.write(&[cmd::ENABLE_RESET]).await?;
		self.spi.write(&[cmd::RESET]).await?;
		Timer::after_micros(50).await; // tRST = 30 us
		Ok(())
	}

	pub async fn jedec_id(&mut self) -> Result<JedecId, Error<S::Error>> {
		let mut id = [0u8; 3];
		self.spi
			.transaction(&mut [Operation::Write(&[cmd::JEDEC_ID]), Operation::Read(&mut id)])
			.await?;
		Ok(JedecId { manufacturer: id[0], memory_type: id[1], capacity: id[2] })
	}

	pub async fn unique_id(&mut self) -> Result<u64, Error<S::Error>> {
		let mut id = [0u8; 8];
		self.spi
			.transaction(&mut [
				Operation::Write(&[cmd::UNIQUE_ID, 0, 0, 0, 0]), // 4 dummy bytes
				Operation::Read(&mut id),
			])
			.await?;
		Ok(u64::from_be_bytes(id))
	}

	/// (SR1, SR2, SR3)
	pub async fn status(&mut self) -> Result<[u8; 3], Error<S::Error>> {
		Ok([
			self.read_status(cmd::READ_SR1).await?,
			self.read_status(cmd::READ_SR2).await?,
			self.read_status(cmd::READ_SR3).await?,
		])
	}

	async fn read_status(&mut self, which: u8) -> Result<u8, Error<S::Error>> {
		let mut sr = [0u8; 1];
		self.spi.transaction(&mut [Operation::Write(&[which]), Operation::Read(&mut sr)]).await?;
		Ok(sr[0])
	}

	pub async fn is_busy(&mut self) -> Result<bool, Error<S::Error>> {
		Ok(self.read_status(cmd::READ_SR1).await? & SR1_BUSY != 0)
	}

	async fn wait_ready(&mut self, timeout: Duration, poll: Duration) -> Result<(), Error<S::Error>> {
		let deadline = Instant::now() + timeout;
		while self.is_busy().await? {
			if Instant::now() > deadline {
				return Err(Error::Timeout);
			}
			Timer::after(poll).await;
		}
		Ok(())
	}

	async fn write_enable(&mut self) -> Result<(), Error<S::Error>> {
		self.spi.write(&[cmd::WRITE_ENABLE]).await?;
		Ok(())
	}

	fn check(&self, addr: u32, len: usize) -> Result<(), Error<S::Error>> {
		match addr.checked_add(len as u32) {
			Some(end) if end <= self.capacity => Ok(()),
			_ => Err(Error::OutOfBounds),
		}
	}

	fn addr_cmd(op: u8, addr: u32) -> [u8; 4] {
		[op, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8]
	}

	pub async fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Error<S::Error>> {
		self.check(addr, buf.len())?;
		self.spi
			.transaction(&mut [Operation::Write(&Self::addr_cmd(cmd::READ_DATA, addr)), Operation::Read(buf)])
			.await?;
		Ok(())
	}

	/// Program up to one page. Must not cross a 256-byte page boundary.
	pub async fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), Error<S::Error>> {
		self.check(addr, data.len())?;
		if (addr as usize % PAGE_SIZE) + data.len() > PAGE_SIZE {
			return Err(Error::OutOfBounds);
		}
		self.write_enable().await?;
		self.spi
			.transaction(&mut [Operation::Write(&Self::addr_cmd(cmd::PAGE_PROGRAM, addr)), Operation::Write(data)])
			.await?;
		// tPP max 3 ms
		self.wait_ready(Duration::from_millis(10), Duration::from_micros(100)).await
	}

	/// Program an arbitrary range, splitting on page boundaries.
	pub async fn write(&mut self, mut addr: u32, mut data: &[u8]) -> Result<(), Error<S::Error>> {
		self.check(addr, data.len())?;
		while !data.is_empty() {
			let room = PAGE_SIZE - (addr as usize % PAGE_SIZE);
			let n = room.min(data.len());
			self.program_page(addr, &data[..n]).await?;
			addr += n as u32;
			data = &data[n..];
		}
		Ok(())
	}

	/// Erase the 4 KiB sector containing `addr`.
	pub async fn erase_sector(&mut self, addr: u32) -> Result<(), Error<S::Error>> {
		self.check(addr, 1)?;
		let addr = addr & !(SECTOR_SIZE - 1);
		self.write_enable().await?;
		self.spi.write(&Self::addr_cmd(cmd::SECTOR_ERASE, addr)).await?;
		// tSE max 400 ms
		self.wait_ready(Duration::from_millis(500), Duration::from_millis(1)).await
	}

	pub async fn erase_chip(&mut self) -> Result<(), Error<S::Error>> {
		self.write_enable().await?;
		self.spi.write(&[cmd::CHIP_ERASE]).await?;
		// tCE max 10 s for W25Q40
		self.wait_ready(Duration::from_secs(15), Duration::from_millis(10)).await
	}
}
