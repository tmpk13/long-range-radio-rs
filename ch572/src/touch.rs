//! The "1200 baud touch": the host opens the CDC port at 1200 baud and
//! closes it again.  Closing drops DTR, and DTR falling while the port is
//! set to 1200 baud asks the bridge to restart into the CH572's ROM
//! bootloader, so the firmware can be replaced over USB without a button
//! (the board has none for the CH572D).

pub const TOUCH_BAUD: u32 = 1_200;

#[derive(Default)]
pub struct Touch {
    dtr: bool,
}

impl Touch {
    pub const fn new() -> Self {
        Self { dtr: false }
    }

    /// Feed the current line state; true when a touch just completed.
    pub fn update(&mut self, baud: u32, dtr: bool) -> bool {
        let fell = self.dtr && !dtr;
        self.dtr = dtr;
        fell && baud == TOUCH_BAUD
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fires_only_on_dtr_falling_at_1200() {
        let mut t = Touch::new();
        // Normal session at 115200: open and close.
        assert!(!t.update(115_200, true));
        assert!(!t.update(115_200, false));
        // stty -F /dev/ttyACM0 1200: open, set the rate, close.
        assert!(!t.update(115_200, true));
        assert!(!t.update(1_200, true));
        assert!(t.update(1_200, false));
        // Staying closed does not fire again.
        assert!(!t.update(1_200, false));
    }
}
