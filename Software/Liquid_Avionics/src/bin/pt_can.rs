// Reads PT1-PT4 and streams each one over CAN as its own frame, per the
// "Liquid AV CAN Map" doc (PGNs 0xFF21-0xFF24, SA 0x40, priority 5).
//
// Run with:   cargo run --release --bin pt_can
// RTT prints the PT values and CAN TX stats once a second.
#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, AdcChannel, SampleTime};
use embassy_stm32::can::{self, Can, Frame};
use embassy_stm32::peripherals::ADC1;
use embassy_stm32::{bind_interrupts, pac, peripherals};
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use liquid_avionics::board;
use liquid_avionics::can_map::{self, PGN_PT, PRIO_SENSOR, SA_LIQUID_AV};
use {defmt_rtt as _, panic_probe as _};

/// Frames per second for each PT.
const PT_RATE_HZ: u64 = 100;
/// ADC samples averaged per PT per frame.
const OVERSAMPLE: u32 = 8;
/// Time allowed each tick to queue the four frames. With no other node on the
/// bus nothing is ACKed, the mailboxes fill and the rest of the tick's frames
/// are dropped instead of stalling the loop.
const TX_BUDGET: Duration = Duration::from_millis(5);
const CAN_BITRATE: u32 = 250_000;

bind_interrupts!(struct Irqs {
	CAN1_TX => can::TxInterruptHandler<peripherals::CAN1>;
	CAN1_RX0 => can::Rx0InterruptHandler<peripherals::CAN1>;
	CAN1_RX1 => can::Rx1InterruptHandler<peripherals::CAN1>;
	CAN1_SCE => can::SceInterruptHandler<peripherals::CAN1>;
});

const ST: SampleTime = SampleTime::CYCLES480;

/// Sum of OVERSAMPLE raw readings.
fn read_sum(adc: &mut Adc<'_, ADC1>, ch: &mut impl AdcChannel<ADC1>) -> u32 {
	(0..OVERSAMPLE).map(|_| adc.blocking_read(ch, ST) as u32).sum()
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
	let hse_ok = board::probe_hse();
	let p = embassy_stm32::init(board::clock_config(hse_ok));
	// Without this, probe-rs can't attach after a cold power-up (DP WAIT).
	spawner.spawn(board::keep_awake().unwrap());
	if !hse_ok {
		warn!("HSE did not start, running on HSI");
	}

	let mut adc = Adc::new(p.ADC1);
	let mut vrefint = adc.enable_vrefint();
	let mut pts = [
		p.PC5.degrade_adc(), // PT1
		p.PC4.degrade_adc(), // PT2
		p.PB1.degrade_adc(), // PT3
		p.PB0.degrade_adc(), // PT4
	];
	// Factory VREFINT reading at VDDA = 3.3 V (RM0390), used to measure VDDA.
	let vrefint_cal = unsafe { core::ptr::read_volatile(0x1FFF_7A2A as *const u16) } as u32;

	let mut can = Can::new(p.CAN1, p.PA11, p.PA12, Irqs);
	// Automatic retransmit stays on (reset default). Don't call
	// set_automatic_retransmit: embassy-stm32 0.6.0 writes it inverted.
	can.modify_config().set_bitrate(CAN_BITRATE).set_loopback(false).set_silent(false);
	// Send in queue order. In priority mode a full queue aborts the highest-ID
	// pending frame, which would starve PT4.
	can.set_tx_fifo_scheduling(true);
	can.enable().await;

	let ids = PGN_PT.map(|pgn| can_map::j1939_id(PRIO_SENSOR, pgn, SA_LIQUID_AV));
	info!("Streaming PT1-4 at {} Hz, IDs {:08x}", PT_RATE_HZ, ids);

	let mut ticker = Ticker::every(Duration::from_hz(PT_RATE_HZ));
	let mut seq: u8 = 0;
	let mut sent: u32 = 0;
	let mut dropped: u32 = 0;
	let mut last_log = Instant::now();
	loop {
		ticker.next().await;

		let vref_sum = read_sum(&mut adc, &mut vrefint);
		// VDDA in mV = 3300 * cal / raw; skipped as 0 if the read failed.
		let vdda_mv = if vref_sum > 0 { 3300 * vrefint_cal * OVERSAMPLE / vref_sum } else { 0 };

		let mut mv = [can_map::NA_U16; 4];
		for (out, ch) in mv.iter_mut().zip(pts.iter_mut()) {
			if vdda_mv == 0 {
				continue;
			}
			// pin mV = sum * VDDA / (4095 * N), then undo the input divider.
			// Max 8 * 4095 * 3400 * 25 = 2.8e9, fits in u32.
			let input_mv = read_sum(&mut adc, ch) * vdda_mv * board::PT_DIVIDER_NUM
				/ (4095 * OVERSAMPLE * board::PT_DIVIDER_DEN);
			*out = input_mv.min(0xFFFE) as u16;
		}

		let mut queued = 0u32;
		let _ = with_timeout(TX_BUDGET, async {
			for (id, &v) in ids.iter().zip(mv.iter()) {
				let frame = Frame::new_extended(*id, &can_map::pt_payload(v, seq)).unwrap();
				can.write(&frame).await;
				queued += 1;
			}
		})
		.await;
		sent += queued;
		dropped += 4 - queued;
		seq = seq.wrapping_add(1);

		if last_log.elapsed() >= Duration::from_secs(1) {
			last_log = Instant::now();
			let esr = pac::CAN1.esr().read();
			info!(
				"PT mV={} VDDA={} | queued={} dropped={} TEC={} REC={} passive={} busoff={}",
				mv,
				vdda_mv,
				sent,
				dropped,
				esr.tec(),
				esr.rec(),
				esr.epvf(),
				esr.boff()
			);
		}
	}
}
