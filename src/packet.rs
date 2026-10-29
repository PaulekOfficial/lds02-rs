//! LDS02RR wire format.
//!
//! ```text
//! offset  size  field
//!      0     1  start byte, always 0xFA
//!      1     1  index, 0xA0..=0xF9 (packet n covers angles 4n..4n+3)
//!      2     2  speed, little endian, RPM * 64
//!      4    16  4 readings of 4 bytes each:
//!                 byte 0      distance [7:0]
//!                 byte 1      bit 7 invalid, bit 6 strength warning,
//!                             bits 5..0 distance [13:8]
//!                 bytes 2..3  signal strength, little endian
//!     20     2  checksum over bytes 0..20, little endian
//! ```

/// UART baud rate of the sensor (8N1).
pub const BAUD_RATE: u32 = 115_200;
/// Length of one packet in bytes.
pub const PACKET_LEN: usize = 22;
/// Packets per full revolution.
pub const PACKETS_PER_REV: usize = 90;
/// Readings (one per degree) in each packet.
pub const READINGS_PER_PACKET: usize = 4;

const START_BYTE: u8 = 0xFA;
const INDEX_BASE: u8 = 0xA0;

/// Reason a packet was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PacketError {
    /// First byte is not `0xFA`.
    InvalidStart(u8),
    /// Index byte is outside `0xA0..=0xF9`.
    InvalidIndex(u8),
    /// Checksum in the packet does not match its contents.
    Checksum {
        /// Checksum computed from the packet contents.
        expected: u16,
        /// Checksum stored in the packet.
        received: u16,
    },
}

impl core::fmt::Display for PacketError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidStart(b) => write!(f, "invalid start byte {b:#04x}"),
            Self::InvalidIndex(b) => write!(f, "invalid index byte {b:#04x}"),
            Self::Checksum { expected, received } => {
                write!(f, "checksum mismatch: expected {expected:#06x}, received {received:#06x}")
            }
        }
    }
}

impl core::error::Error for PacketError {}

/// One distance reading.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Reading {
    /// Distance in millimeters. When `invalid` is set this holds the sensor's
    /// error code instead.
    pub distance_mm: u16,
    /// Signal strength of the reflection.
    pub strength: u16,
    /// The sensor could not measure this angle.
    pub invalid: bool,
    /// Signal strength is lower than expected for this distance.
    pub strength_warning: bool,
}

impl Reading {
    /// Placeholder for an angle whose packet was never received.
    pub const MISSING: Self = Self {
        distance_mm: 0,
        strength: 0,
        invalid: true,
        strength_warning: false,
    };

    /// Distance in millimeters, or `None` if the reading is invalid.
    pub fn distance(&self) -> Option<u16> {
        (!self.invalid).then_some(self.distance_mm)
    }

    fn parse(b: &[u8]) -> Self {
        Self {
            distance_mm: u16::from_le_bytes([b[0], b[1] & 0x3F]),
            strength: u16::from_le_bytes([b[2], b[3]]),
            invalid: b[1] & 0x80 != 0,
            strength_warning: b[1] & 0x40 != 0,
        }
    }
}

/// One decoded packet: four consecutive degrees of a revolution.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet {
    /// Packet number in the revolution, `0..90`.
    pub index: u8,
    /// Raw rotation speed, RPM * 64.
    pub speed: u16,
    /// Readings for angles `first_angle()..first_angle() + 4`.
    pub readings: [Reading; READINGS_PER_PACKET],
}

impl Packet {
    /// Decodes and validates one raw packet.
    pub fn parse(bytes: &[u8; PACKET_LEN]) -> Result<Self, PacketError> {
        if bytes[0] != START_BYTE {
            return Err(PacketError::InvalidStart(bytes[0]));
        }
        if !is_index(bytes[1]) {
            return Err(PacketError::InvalidIndex(bytes[1]));
        }
        let expected = checksum(&bytes[..20]);
        let received = u16::from_le_bytes([bytes[20], bytes[21]]);
        if expected != received {
            return Err(PacketError::Checksum { expected, received });
        }
        Ok(Self {
            index: bytes[1] - INDEX_BASE,
            speed: u16::from_le_bytes([bytes[2], bytes[3]]),
            readings: core::array::from_fn(|i| Reading::parse(&bytes[4 + 4 * i..8 + 4 * i])),
        })
    }

    /// Rotation speed in revolutions per minute.
    pub fn rpm(&self) -> f32 {
        f32::from(self.speed) / 64.0
    }

    /// Angle in degrees of the first reading.
    pub fn first_angle(&self) -> u16 {
        u16::from(self.index) * READINGS_PER_PACKET as u16
    }

    /// `(angle in degrees, reading)` pairs of this packet.
    pub fn angles(&self) -> impl Iterator<Item = (u16, Reading)> + '_ {
        (self.first_angle()..).zip(self.readings.iter().copied())
    }
}

fn is_index(byte: u8) -> bool {
    (INDEX_BASE..INDEX_BASE + PACKETS_PER_REV as u8).contains(&byte)
}

/// LDS02RR checksum over the first 20 bytes of a packet.
///
/// The data is read as little endian 16-bit words, folded into a 32-bit
/// accumulator with a shift per word, then reduced to 15 bits.
pub fn checksum(data: &[u8]) -> u16 {
    let acc = data.chunks_exact(2).fold(0u32, |acc, w| {
        (acc << 1) + u32::from(u16::from_le_bytes([w[0], w[1]]))
    });
    (((acc & 0x7FFF) + (acc >> 15)) & 0x7FFF) as u16
}

/// Byte-wise packet parser that finds packet boundaries in a raw stream.
///
/// Feed every received byte to [`push`](Self::push). Bytes before a valid
/// start/index pair are skipped, so the parser syncs on its own after power
/// up or lost bytes.
#[derive(Debug, Clone)]
pub struct PacketParser {
    buf: [u8; PACKET_LEN],
    len: usize,
}

impl Default for PacketParser {
    fn default() -> Self {
        Self::new()
    }
}

impl PacketParser {
    /// Creates a parser waiting for a start byte.
    pub const fn new() -> Self {
        Self {
            buf: [0; PACKET_LEN],
            len: 0,
        }
    }

    /// Adds one byte. Returns `Some` when a full packet has been received,
    /// with an error if its checksum is wrong.
    pub fn push(&mut self, byte: u8) -> Option<Result<Packet, PacketError>> {
        match self.len {
            0 if byte != START_BYTE => return None,
            1 if !is_index(byte) => {
                // A repeated start byte may itself begin the real packet.
                self.len = usize::from(byte == START_BYTE);
                return None;
            }
            _ => {}
        }
        self.buf[self.len] = byte;
        self.len += 1;
        if self.len < PACKET_LEN {
            return None;
        }
        let result = Packet::parse(&self.buf);
        match result {
            Ok(_) => self.len = 0,
            Err(_) => self.resync(),
        }
        Some(result)
    }

    /// Drops any partial packet.
    pub fn reset(&mut self) {
        self.len = 0;
    }

    /// Restarts from the next start byte inside a rejected frame, so a false
    /// start does not swallow the real packet that overlaps it.
    fn resync(&mut self) {
        match self.buf[1..].iter().position(|&b| b == START_BYTE) {
            Some(pos) => {
                let start = pos + 1;
                self.buf.copy_within(start.., 0);
                self.len = PACKET_LEN - start;
            }
            None => self.len = 0,
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Builds a valid raw packet with the given index (0..90), RPM * 64 and
    /// four `(distance, strength, flags)` readings.
    pub(crate) fn raw_packet(index: u8, speed: u16, readings: [(u16, u16, u8); 4]) -> [u8; 22] {
        let mut p = [0u8; PACKET_LEN];
        p[0] = START_BYTE;
        p[1] = INDEX_BASE + index;
        p[2..4].copy_from_slice(&speed.to_le_bytes());
        for (i, (dist, strength, flags)) in readings.into_iter().enumerate() {
            let o = 4 + 4 * i;
            p[o] = dist as u8;
            p[o + 1] = (dist >> 8) as u8 & 0x3F | flags;
            p[o + 2..o + 4].copy_from_slice(&strength.to_le_bytes());
        }
        let chk = checksum(&p[..20]);
        p[20..].copy_from_slice(&chk.to_le_bytes());
        p
    }

    #[test]
    fn checksum_matches_reference_algorithm() {
        // Reference implementation from the XV11 hacking wiki.
        let data: [u8; 20] = core::array::from_fn(|i| (i as u8).wrapping_mul(37) ^ 0xA5);
        let mut chk32: u32 = 0;
        for t in 0..10 {
            chk32 = (chk32 << 1) + data[2 * t] as u32 + ((data[2 * t + 1] as u32) << 8);
        }
        let reference = ((chk32 & 0x7FFF) + (chk32 >> 15)) & 0x7FFF;
        assert_eq!(checksum(&data), reference as u16);
    }

    #[test]
    fn parses_fields() {
        let raw = raw_packet(
            10,
            300 * 64,
            [(1234, 77, 0), (0x3FFF, 1, 0x40), (2, 0, 0x80), (0, 9, 0)],
        );
        let p = Packet::parse(&raw).unwrap();
        assert_eq!(p.index, 10);
        assert_eq!(p.first_angle(), 40);
        assert_eq!(p.rpm(), 300.0);
        assert_eq!(p.readings[0].distance(), Some(1234));
        assert_eq!(p.readings[0].strength, 77);
        assert_eq!(p.readings[1].distance_mm, 0x3FFF);
        assert!(p.readings[1].strength_warning);
        assert!(p.readings[2].invalid);
        assert_eq!(p.readings[2].distance(), None);
        let angles: Vec<u16> = p.angles().map(|(a, _)| a).collect();
        assert_eq!(angles, [40, 41, 42, 43]);
    }

    #[test]
    fn rejects_bad_packets() {
        let mut raw = raw_packet(0, 0, [(0, 0, 0); 4]);
        raw[5] ^= 1;
        assert!(matches!(Packet::parse(&raw), Err(PacketError::Checksum { .. })));
        raw[1] = 0x10;
        assert_eq!(Packet::parse(&raw), Err(PacketError::InvalidIndex(0x10)));
        raw[0] = 0;
        assert_eq!(Packet::parse(&raw), Err(PacketError::InvalidStart(0)));
    }

    fn parse_stream(bytes: &[u8]) -> Vec<Result<Packet, PacketError>> {
        let mut parser = PacketParser::new();
        bytes.iter().filter_map(|&b| parser.push(b)).collect()
    }

    #[test]
    fn syncs_after_garbage() {
        let mut stream = vec![0x00, 0xFA, 0x12, 0xFA, 0xFA];
        stream.extend(raw_packet(1, 64, [(5, 5, 0); 4]));
        stream.extend(raw_packet(2, 64, [(6, 6, 0); 4]));
        let out = parse_stream(&stream);
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].unwrap().index, 1);
        assert_eq!(out[1].unwrap().index, 2);
    }

    #[test]
    fn resyncs_inside_corrupt_frame() {
        // Truncated packet directly followed by a good one: the frame fails
        // its checksum, and the good packet must still be found inside it.
        let mut stream = raw_packet(3, 64, [(1, 1, 0); 4])[..10].to_vec();
        stream.extend(raw_packet(4, 64, [(2, 2, 0); 4]));
        let out = parse_stream(&stream);
        assert!(out[0].is_err());
        assert_eq!(out.last().unwrap().unwrap().index, 4);
    }
}
