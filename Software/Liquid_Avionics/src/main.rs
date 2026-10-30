// Flight software for the Liquid Avionics stack.
//
// Placeholder for now: logs the PT readings once a second and every CAN frame
// it receives, over RTT.
//
// Run with:   cargo run
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, SampleTime};
use embassy_stm32::can::filter::Mask32;
use embassy_stm32::can::{self, Can, Fifo, Frame, Id};
use embassy_stm32::peripherals::{self, ADC1};
use embassy_stm32::bind_interrupts;
use embassy_time::{Duration, Ticker};
use liquid_avionics::board::{self, PtPins};
use liquid_avionics::can_map;
use {defmt_rtt as _, panic_probe as _};

/// ADC samples averaged per PT per reading.
const OVERSAMPLE: u32 = 8;
const ST: SampleTime = SampleTime::CYCLES480;

bind_interrupts!(struct Irqs {
	CAN1_TX => can::TxInterruptHandler<peripherals::CAN1>;
	CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN1>;
	CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN1>;
	CAN1_SCE => can::SceInterruptHandler<peripherals::CAN1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
	let hse_ok = board::probe_hse();
	let b = board::split(embassy_stm32::init(board::clock_config(hse_ok)));
	info!("Liquid Avionics flight software (placeholder)");
	if !hse_ok {
		warn!("HSE did not start, running on HSI");
	}

	spawner.spawn(pt_task(Adc::new(b.adc), b.pt).unwrap());
	spawner.spawn(can_task(Can::new(b.can.can, b.can.rx, b.can.tx, Irqs)).unwrap());
}

/// Logs PT1-PT4 at the connector, in mV, once a second.
#[embassy_executor::task]
async fn pt_task(mut adc: Adc<'static, ADC1>, pt: PtPins) {
	let mut vrefint = adc.enable_vrefint();
	let mut chans = [
		pt.pt1.degrade_adc(),
		pt.pt2.degrade_adc(),
		pt.pt3.degrade_adc(),
		pt.pt4.degrade_adc(),
	];

	let mut ticker = Ticker::every(Duration::from_secs(1));
	loop {
		ticker.next().await;
		let vdda_mv = board::vdda_mv(read_sum(&mut adc, &mut vrefint), OVERSAMPLE);
		let mut mv = [0u32; 4];
		for (out, ch) in mv.iter_mut().zip(chans.iter_mut()) {
			*out = board::pt_input_mv(read_sum(&mut adc, ch), OVERSAMPLE, vdda_mv);
		}
		info!("PT mV={} VDDA={} mV", mv, vdda_mv);
	}
}

/// Sum of OVERSAMPLE raw readings.
fn read_sum(adc: &mut Adc<'_, ADC1>, ch: &mut impl AdcChannel<ADC1>) -> u32 {
	(0..OVERSAMPLE).map(|_| adc.blocking_read(ch, ST) as u32).sum()
}

/// Brings CAN1 up and logs every frame received.
#[embassy_executor::task]
async fn can_task(mut can: Can<'static>) {
	can.modify_filters().enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
	can.modify_config().set_bitrate(can_map::BITRATE).set_loopback(false).set_silent(false);
	// Waits for 11 recessive bits, so this hangs (only this task) if RX is stuck low.
	can.enable().await;
	info!("CAN up at {} bit/s", can_map::BITRATE);

	loop {
		match can.read().await {
			Ok(envelope) => log_frame(&envelope.frame),
			Err(e) => warn!("CAN bus error: {}", e),
		}
	}
}

fn log_frame(frame: &Frame) {
	match frame.id() {
		// Team frames are J1939: priority, PGN, source address (see can_map).
		Id::Extended(id) => {
			let raw = id.as_raw();
			info!(
				"CAN rx prio={} pgn=0x{:04x} sa=0x{:02x} data={:02x}",
				raw >> 26,
				(raw >> 8) & 0x3FFFF,
				raw & 0xFF,
				frame.data()
			);
		}
		Id::Standard(id) => info!("CAN rx std id=0x{:03x} data={:02x}", id.as_raw(), frame.data()),
	}
}
