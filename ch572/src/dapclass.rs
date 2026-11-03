//! CMSIS-DAP v2 USB interface: a vendor-class interface with one bulk OUT
//! endpoint for commands and one bulk IN endpoint for responses.
//!
//! Hosts find it by its interface string: probe-rs and OpenOCD accept any
//! vendor interface whose string contains "CMSIS-DAP".  dap-rs ships a v2
//! class too, but it adds a third endpoint for SWO trace, and the Wio-E5's
//! SWO pin (PB3) is the SD card clock on this board.

use usb_device::class_prelude::*;

/// Full-speed bulk packet size, which is also the DAP packet size.
pub const PACKET_SIZE: u16 = 64;

pub struct CmsisDapV2<'a, B: UsbBus> {
    interface: InterfaceNumber,
    name: StringIndex,
    read_ep: EndpointOut<'a, B>,
    write_ep: EndpointIn<'a, B>,
}

impl<'a, B: UsbBus> CmsisDapV2<'a, B> {
    pub fn new(alloc: &'a UsbBusAllocator<B>) -> Self {
        Self {
            interface: alloc.interface(),
            name: alloc.string(),
            read_ep: alloc.bulk(PACKET_SIZE),
            write_ep: alloc.bulk(PACKET_SIZE),
        }
    }

    /// The next command packet, if one has arrived.
    pub fn read_packet(&mut self, buf: &mut [u8; PACKET_SIZE as usize]) -> Option<usize> {
        self.read_ep.read(buf).ok()
    }

    /// Queue a response packet; `WouldBlock` while the previous one is
    /// still waiting for the host.
    pub fn write_packet(&mut self, data: &[u8]) -> usb_device::Result<usize> {
        self.write_ep.write(data)
    }
}

impl<B: UsbBus> UsbClass<B> for CmsisDapV2<'_, B> {
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.interface_alt(self.interface, 0, 0xFF, 0, 0, Some(self.name))?;
        writer.endpoint(&self.read_ep)?;
        writer.endpoint(&self.write_ep)?;
        Ok(())
    }

    fn get_string(&self, index: StringIndex, _lang_id: LangID) -> Option<&str> {
        (index == self.name).then_some("CMSIS-DAP v2 Interface")
    }

    fn set_alt_setting(&mut self, interface: InterfaceNumber, alternative: u8) -> bool {
        if interface != self.interface || alternative != 0 {
            return false;
        }
        // SET_INTERFACE resets the interface's data toggles to DATA0;
        // unstalling is how the bus driver is told to do that.
        self.read_ep.unstall();
        self.write_ep.unstall();
        true
    }

    fn get_alt_setting(&mut self, interface: InterfaceNumber) -> Option<u8> {
        (interface == self.interface).then_some(0)
    }
}
