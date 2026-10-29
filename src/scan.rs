//! Assembly of packets into full revolutions.

use crate::packet::{PACKETS_PER_REV, Packet, READINGS_PER_PACKET, Reading};

const DEGREES: usize = PACKETS_PER_REV * READINGS_PER_PACKET;

/// One full 360° revolution, one reading per degree.
#[derive(Debug, Clone)]
pub struct Scan {
    readings: [Reading; DEGREES],
    packets: u8,
    rpm_sum: f32,
}

impl Scan {
    const fn new() -> Self {
        Self {
            readings: [Reading::MISSING; DEGREES],
            packets: 0,
            rpm_sum: 0.0,
        }
    }

    /// Reading at `angle` degrees (taken modulo 360).
    pub fn reading(&self, angle: u16) -> Reading {
        self.readings[usize::from(angle) % DEGREES]
    }

    /// All 360 readings, indexed by angle in degrees. Angles whose packet was
    /// lost hold [`Reading::MISSING`].
    pub fn readings(&self) -> &[Reading; DEGREES] {
        &self.readings
    }

    /// Number of packets received for this revolution, at most 90.
    pub fn packets(&self) -> u8 {
        self.packets
    }

    /// `true` if no packet of this revolution was lost.
    pub fn is_complete(&self) -> bool {
        usize::from(self.packets) == PACKETS_PER_REV
    }

    /// Mean rotation speed over the received packets.
    pub fn rpm(&self) -> f32 {
        if self.packets == 0 {
            0.0
        } else {
            self.rpm_sum / f32::from(self.packets)
        }
    }

    fn insert(&mut self, packet: &Packet) {
        let first = usize::from(packet.first_angle());
        self.readings[first..first + READINGS_PER_PACKET].copy_from_slice(&packet.readings);
        self.packets += 1;
        self.rpm_sum += packet.rpm();
    }
}

/// Collects packets into [`Scan`]s.
///
/// A revolution ends with packet 89, or earlier when the packet index wraps
/// around because packet 89 was lost.
#[derive(Debug, Clone)]
pub struct ScanBuilder {
    scan: Scan,
    last_index: Option<u8>,
    /// First packet of the next revolution, held back while the finished
    /// scan is lent to the caller.
    pending: Option<Packet>,
    finished: bool,
}

impl Default for ScanBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl ScanBuilder {
    /// Creates a builder with an empty scan.
    pub const fn new() -> Self {
        Self {
            scan: Scan::new(),
            last_index: None,
            pending: None,
            finished: false,
        }
    }

    /// Adds a packet. Returns the finished scan when this packet completes a
    /// revolution. The scan stays valid until the next call.
    pub fn push(&mut self, packet: Packet) -> Option<&Scan> {
        if self.finished {
            self.clear();
        }
        if let Some(pending) = self.pending.take() {
            self.insert(&pending);
        }
        if self.last_index.is_some_and(|last| packet.index <= last) {
            self.pending = Some(packet);
            self.finished = true;
            return Some(&self.scan);
        }
        self.insert(&packet);
        if usize::from(packet.index) == PACKETS_PER_REV - 1 {
            self.finished = true;
            return Some(&self.scan);
        }
        None
    }

    /// Discards the partial revolution, e.g. after the motor stopped.
    pub fn reset(&mut self) {
        self.clear();
        self.pending = None;
    }

    fn clear(&mut self) {
        self.scan = Scan::new();
        self.last_index = None;
        self.finished = false;
    }

    fn insert(&mut self, packet: &Packet) {
        self.scan.insert(packet);
        self.last_index = Some(packet.index);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::tests::raw_packet;

    fn packet(index: u8, dist: u16) -> Packet {
        Packet::parse(&raw_packet(index, 300 * 64, [(dist, 1, 0); 4])).unwrap()
    }

    #[test]
    fn completes_on_last_packet() {
        let mut builder = ScanBuilder::new();
        for i in 0..89 {
            assert!(builder.push(packet(i, i.into())).is_none());
        }
        let scan = builder.push(packet(89, 89)).unwrap();
        assert!(scan.is_complete());
        assert_eq!(scan.rpm(), 300.0);
        assert_eq!(scan.reading(0).distance(), Some(0));
        assert_eq!(scan.reading(359).distance(), Some(89));
        assert_eq!(scan.reading(361).distance(), Some(0));
    }

    #[test]
    fn completes_on_wrap_when_last_packet_lost() {
        let mut builder = ScanBuilder::new();
        for i in 0..89 {
            builder.push(packet(i, 1));
        }
        let scan = builder.push(packet(0, 2)).unwrap();
        assert_eq!(scan.packets(), 89);
        assert_eq!(scan.reading(359), Reading::MISSING);

        // The wrapping packet starts the next revolution.
        for i in 1..89 {
            assert!(builder.push(packet(i, 2)).is_none());
        }
        let scan = builder.push(packet(89, 2)).unwrap();
        assert!(scan.is_complete());
        assert_eq!(scan.reading(0).distance(), Some(2));
    }
}
