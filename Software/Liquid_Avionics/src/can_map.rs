// CAN IDs for the GSE link, from the "Liquid AV CAN Map" knowledge-base doc.
// Team J1939 conventions: 250 kbit/s, 29-bit IDs, PDU2 broadcast PGNs in the
// Proprietary B range (0xFFxx), ID = (priority << 26) | (PGN << 8) | SA.
// Payloads are little-endian, DLC 8, reserved bytes 0xFF.

/// This board's source address (proposed in the doc, not yet confirmed).
pub const SA_LIQUID_AV: u8 = 0x40;
/// GSE source address. Commands from any other SA are ignored.
pub const SA_GSE: u8 = 0xFA;

pub const PRIO_COMMAND: u8 = 3;
pub const PRIO_SENSOR: u8 = 5;
pub const PRIO_STATUS: u8 = 6;

// GSE -> AV
pub const PGN_CAMERA_1_POWER: u16 = 0xFF05;
pub const PGN_CAMERA_2_POWER: u16 = 0xFF06;
pub const PGN_LAUNCH_DETECT: u16 = 0xFF07;
pub const PGN_RADIO_POWER_MODE: u16 = 0xFF08;
pub const PGN_RADIO_SET_PARAM: u16 = 0xFF09;
pub const PGN_RADIO_GET_PARAM: u16 = 0xFF0A;
pub const PGN_TIME_SYNC: u16 = 0xFF0B;

// Both directions (existing team PGN)
pub const PGN_HEARTBEAT: u16 = 0xFF10;

// AV -> GSE. 0xFF15 (the old packed PT_DATA) is retired; don't reuse it.
pub const PGN_TEMPERATURES: u16 = 0xFF16;
pub const PGN_BARO: u16 = 0xFF17;
pub const PGN_ACCEL: u16 = 0xFF18;
pub const PGN_GYRO: u16 = 0xFF19;
pub const PGN_CAMERA_STATUS: u16 = 0xFF1A;
pub const PGN_RADIO_STATUS: u16 = 0xFF1B;
pub const PGN_RADIO_PARAM: u16 = 0xFF1C;
pub const PGN_COMMAND_ACK: u16 = 0xFF1D;
pub const PGN_BOARD_INFO: u16 = 0xFF1E;
/// One PGN per pressure transducer, PT1..PT4. Last hex digit = PT number.
pub const PGN_PT: [u16; 4] = [0xFF21, 0xFF22, 0xFF23, 0xFF24];

/// "Not available" for unsigned telemetry fields.
pub const NA_U16: u16 = 0xFFFF;

/// Builds a 29-bit J1939 CAN ID.
pub const fn j1939_id(priority: u8, pgn: u16, sa: u8) -> u32 {
	((priority as u32) << 26) | ((pgn as u32) << 8) | sa as u32
}

/// PT1..PT4 frame payload: bytes 0-1 mV at the transducer (u16 LE, 0xFFFF =
/// not available), byte 2 sample counter shared by all four PTs from the same
/// sample, bytes 3-7 reserved (0xFF).
pub fn pt_payload(mv: u16, seq: u8) -> [u8; 8] {
	let [lo, hi] = mv.to_le_bytes();
	[lo, hi, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
}
