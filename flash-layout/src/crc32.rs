//! CRC-32/ISO-HDLC, the checksum an over-the-air update carries for its
//! image: reflected polynomial `0xEDB8_8320`, initial value and final XOR
//! `0xFFFF_FFFF` (the zlib and Ethernet CRC).  The OTA receiver checks a
//! staged image with it, the bootloader checks it again before swapping the
//! image in, and the basestation's sender computes it for the offer.
//!
//! Table driven: the bootloader runs it over up to 112 KB at 4 MHz before a
//! swap, where the bit-at-a-time loop costs about four times as long.

/// One table entry per byte value, built at compile time.
const TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut n = 0;
    while n < 256 {
        let mut c = n as u32;
        let mut bit = 0;
        while bit < 8 {
            c = if c & 1 != 0 {
                (c >> 1) ^ 0xEDB8_8320
            } else {
                c >> 1
            };
            bit += 1;
        }
        table[n] = c;
        n += 1;
    }
    table
};

/// A CRC-32 fed in pieces.
#[derive(Clone, Copy, Debug)]
pub struct Crc32(u32);

impl Crc32 {
    /// A CRC over no bytes yet.
    pub const fn new() -> Self {
        Self(0xFFFF_FFFF)
    }

    /// Feed `data` into the CRC.
    pub fn update(&mut self, data: &[u8]) {
        let mut c = self.0;
        for &b in data {
            c = TABLE[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
        }
        self.0 = c;
    }

    /// The CRC of every byte fed so far.
    pub const fn finish(&self) -> u32 {
        !self.0
    }
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

/// CRC-32 of `data` in one call.
pub fn crc32(data: &[u8]) -> u32 {
    let mut c = Crc32::new();
    c.update(data);
    c.finish()
}

#[cfg(test)]
mod tests {
    use super::*;

    // The bit-at-a-time form the OTA code used before this module existed;
    // images already in the field were checked with it.
    fn bitwise(data: &[u8]) -> u32 {
        let mut crc: u32 = 0xFFFF_FFFF;
        for &byte in data {
            crc ^= byte as u32;
            for _ in 0..8 {
                crc = if crc & 1 != 0 {
                    (crc >> 1) ^ 0xEDB8_8320
                } else {
                    crc >> 1
                };
            }
        }
        !crc
    }

    #[test]
    fn check_value() {
        // The catalogued check value of CRC-32/ISO-HDLC.
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn matches_the_bitwise_form_in_any_split() {
        let mut data = [0u8; 1000];
        let mut s: u32 = 0x1234_5678;
        for b in data.iter_mut() {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            *b = s as u8;
        }
        let whole = bitwise(&data);
        assert_eq!(crc32(&data), whole);
        for split in [0, 1, 7, 8, 499, 999, 1000] {
            let mut c = Crc32::new();
            c.update(&data[..split]);
            c.update(&data[split..]);
            assert_eq!(c.finish(), whole, "split at {split}");
        }
    }
}
