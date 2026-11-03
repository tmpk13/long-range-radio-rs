//! Fixed-size byte FIFO for the UART side of the bridge.

pub struct Ring<const N: usize> {
    buf: [u8; N],
    head: usize,
    len: usize,
}

impl<const N: usize> Default for Ring<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Ring<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            head: 0,
            len: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn free(&self) -> usize {
        N - self.len
    }

    /// Queue one byte; false (and the byte dropped) when full.
    pub fn push(&mut self, byte: u8) -> bool {
        if self.len == N {
            return false;
        }
        self.buf[(self.head + self.len) % N] = byte;
        self.len += 1;
        true
    }

    pub fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.buf[self.head];
        self.head = (self.head + 1) % N;
        self.len -= 1;
        Some(byte)
    }

    /// Queue as much of `data` as fits; returns the count queued.
    pub fn push_slice(&mut self, data: &[u8]) -> usize {
        let n = data.len().min(self.free());
        for &byte in &data[..n] {
            self.push(byte);
        }
        n
    }

    /// Move up to `out.len()` of the oldest bytes into `out`; returns the
    /// count moved.
    pub fn pop_slice(&mut self, out: &mut [u8]) -> usize {
        let n = out.len().min(self.len);
        for slot in &mut out[..n] {
            *slot = self.pop().unwrap_or(0);
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fifo_order_across_the_wrap() {
        let mut r: Ring<4> = Ring::new();
        assert_eq!(r.push_slice(&[1, 2, 3]), 3);
        assert_eq!(r.pop(), Some(1));
        assert_eq!(r.pop(), Some(2));
        // head is at 2: these wrap around the end of the array.
        assert_eq!(r.push_slice(&[4, 5, 6]), 3);
        let mut out = [0u8; 8];
        assert_eq!(r.pop_slice(&mut out), 4);
        assert_eq!(&out[..4], &[3, 4, 5, 6]);
        assert!(r.is_empty());
    }

    #[test]
    fn full_ring_refuses_and_keeps_old_bytes() {
        let mut r: Ring<2> = Ring::new();
        assert!(r.push(7));
        assert!(r.push(8));
        assert!(!r.push(9));
        assert_eq!(r.push_slice(&[10]), 0);
        assert_eq!(r.free(), 0);
        assert_eq!(r.pop(), Some(7));
        assert_eq!(r.pop(), Some(8));
        assert_eq!(r.pop(), None);
    }
}
