//! Packet reader on top of `embedded-io` / `embedded-io-async`.

use crate::packet::{Packet, PacketError, PacketParser};

/// Error while reading packets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// The underlying reader failed (UART overrun, framing error, ...).
    Io(E),
    /// A packet was received but rejected. Reading can simply continue.
    Packet(PacketError),
    /// The reader reported end of stream.
    Eof,
}

impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "I/O error: {e:?}"),
            Self::Packet(e) => write!(f, "bad packet: {e}"),
            Self::Eof => f.write_str("end of stream"),
        }
    }
}

impl<E: core::fmt::Debug> core::error::Error for Error<E> {}

/// LDS02RR reader over a serial RX half configured for 115200 baud 8N1.
///
/// Use [`read_packet`](Self::read_packet) with an async reader (embassy,
/// esp-hal `Uart<'_, Async>`) or [`read_packet_blocking`](Self::read_packet_blocking)
/// with a blocking one.
#[derive(Debug)]
pub struct Lds02rr<R> {
    reader: R,
    parser: PacketParser,
    buf: [u8; 64],
    pos: usize,
    len: usize,
}

impl<R> Lds02rr<R> {
    /// Wraps a serial reader already configured for [`BAUD_RATE`](crate::BAUD_RATE) 8N1.
    pub const fn new(reader: R) -> Self {
        Self {
            reader,
            parser: PacketParser::new(),
            buf: [0; 64],
            pos: 0,
            len: 0,
        }
    }

    /// Returns the reader. Buffered, unparsed bytes are dropped.
    pub fn release(self) -> R {
        self.reader
    }

    /// Parses buffered bytes until a packet completes or the buffer is empty.
    fn next_buffered(&mut self) -> Option<Result<Packet, PacketError>> {
        while self.pos < self.len {
            let byte = self.buf[self.pos];
            self.pos += 1;
            if let Some(result) = self.parser.push(byte) {
                return Some(result);
            }
        }
        None
    }

    fn filled<E>(&mut self, read: Result<usize, E>) -> Result<(), Error<E>> {
        let read = read.map_err(|e| {
            // Bytes were lost, so a partial packet would only be completed
            // with bytes from the next one and fail its checksum.
            self.parser.reset();
            Error::Io(e)
        });
        match read? {
            0 => Err(Error::Eof),
            n => {
                self.pos = 0;
                self.len = n;
                Ok(())
            }
        }
    }
}

impl<R: embedded_io::Read> Lds02rr<R> {
    /// Blocks until the next packet is received.
    pub fn read_packet_blocking(&mut self) -> Result<Packet, Error<R::Error>> {
        loop {
            if let Some(result) = self.next_buffered() {
                return result.map_err(Error::Packet);
            }
            let read = self.reader.read(&mut self.buf);
            self.filled(read)?;
        }
    }
}

#[cfg(feature = "async")]
#[cfg_attr(docsrs, doc(cfg(feature = "async")))]
impl<R: embedded_io_async::Read> Lds02rr<R> {
    /// Waits for the next packet.
    pub async fn read_packet(&mut self) -> Result<Packet, Error<R::Error>> {
        loop {
            if let Some(result) = self.next_buffered() {
                return result.map_err(Error::Packet);
            }
            let read = self.reader.read(&mut self.buf).await;
            self.filled(read)?;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::tests::raw_packet;

    /// Reader that returns its data in small chunks.
    struct Chunked<'a>(&'a [u8]);

    impl embedded_io::ErrorType for Chunked<'_> {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Read for Chunked<'_> {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = self.0.len().min(buf.len()).min(5);
            buf[..n].copy_from_slice(&self.0[..n]);
            self.0 = &self.0[n..];
            Ok(n)
        }
    }

    #[test]
    fn reads_packets_across_chunks() {
        let mut stream = vec![1, 2, 3];
        for i in 0..4 {
            stream.extend(raw_packet(i, 64 * 300, [(100, 1, 0); 4]));
        }
        let mut lidar = Lds02rr::new(Chunked(&stream));
        for i in 0..4 {
            assert_eq!(lidar.read_packet_blocking().unwrap().index, i);
        }
        assert_eq!(lidar.read_packet_blocking(), Err(Error::Eof));
    }

    /// Reader that returns one scripted result per call.
    struct Scripted<'a>(&'a [Result<&'a [u8], ()>]);

    impl embedded_io::ErrorType for Scripted<'_> {
        type Error = embedded_io::ErrorKind;
    }

    impl embedded_io::Read for Scripted<'_> {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let Some((next, rest)) = self.0.split_first() else {
                return Ok(0);
            };
            self.0 = rest;
            let data = next.map_err(|()| embedded_io::ErrorKind::Other)?;
            buf[..data.len()].copy_from_slice(data);
            Ok(data.len())
        }
    }

    #[test]
    fn drops_partial_packet_after_read_error() {
        let first = raw_packet(0, 64 * 300, [(100, 1, 0); 4]);
        let second = raw_packet(1, 64 * 300, [(100, 1, 0); 4]);
        let script = [Ok(&first[..10]), Err(()), Ok(&second[..])];
        let mut lidar = Lds02rr::new(Scripted(&script));
        assert_eq!(
            lidar.read_packet_blocking(),
            Err(Error::Io(embedded_io::ErrorKind::Other))
        );
        assert_eq!(lidar.read_packet_blocking().unwrap().index, 1);
    }
}
