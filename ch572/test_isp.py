"""Host tests for isp.py against a fake CH572 bootloader."""

import struct

import pytest

import isp

UID = bytes.fromhex("407ed2647f9c917f")  # the first board's chip
CFG = bytes.fromhex("ffffffffffffffff40003a40")


def elf32(segments):
    """A minimal ELF32 with one PT_LOAD per (vaddr, paddr, data, memsz)."""
    phoff, phentsize = 52, 32
    data_off = phoff + phentsize * len(segments)
    hdr = bytearray(52)
    hdr[:6] = b"\x7fELF\x01\x01"
    struct.pack_into("<I", hdr, 0x1C, phoff)
    struct.pack_into("<HH", hdr, 0x2A, phentsize, len(segments))
    phdrs, blobs = b"", b""
    for vaddr, paddr, data, memsz in segments:
        phdrs += struct.pack("<8I", 1, data_off + len(blobs), vaddr, paddr,
                             len(data), memsz, 5, 4)
        blobs += data
    return bytes(hdr) + phdrs + blobs


def test_elf_image_places_segments_at_load_addresses():
    raw = elf32([
        (0x0, 0x0, b"\x01" * 10, 10),
        (0x20000000, 0x10, b"\x02" * 3, 3),  # RAM code stored after .text
        (0x20000100, 0x20000100, b"", 0x40),  # .bss: nothing to load
    ])
    base, img = isp.elf_image(raw)
    assert base == 0
    assert img == b"\x01" * 10 + b"\xff" * 6 + b"\x02" * 3 + b"\xff" * 5
    assert len(img) % 8 == 0


def test_elf_image_rejects_other_files():
    with pytest.raises(isp.IspError):
        isp.elf_image(b"\x7fELF\x02\x01" + bytes(60))


def test_xor_key_matches_the_bootloaders_checksum():
    # The first board's bootloader answered the key exchange with 0x6a.
    key = isp.xor_key(UID, 0x72)
    assert key == bytes([0x1F] * 7 + [0x91])
    assert sum(key) & 0xFF == 0x6A
    assert isp.xor(isp.xor(b"firmware", key), key) == b"firmware"


class FakeBootloader(isp.Bootloader):
    """Answers like the CH572's bootloader: program and verify fail with
    0xfe until the config has been written in the session."""

    def __init__(self, chip=(0x72, 0x13)):  # no USB device behind it
        self.chip = chip
        self.cfg_written = False
        self.flash = {}
        self.key = isp.xor_key(UID, chip[0])
        self.log = []

    def xfer(self, cmd, payload):
        self.log.append(cmd)
        if cmd == isp.IDENTIFY:
            return 0, bytes(self.chip)
        if cmd == isp.READ_CFG:
            return 0, bytes([payload[0], 0]) + CFG + bytes.fromhex("00020300") + UID
        if cmd == isp.WRITE_CFG:
            self.cfg_written = True
            return 0, b"\x00\x00"
        if cmd == isp.ISP_KEY:
            return 0, bytes([sum(self.key) & 0xFF, 0])
        if cmd == isp.ERASE:
            return 0, b"\x00\x00"
        if cmd in (isp.PROGRAM, isp.VERIFY):
            if not self.cfg_written:
                return 0, b"\xfe\x00"
            addr, = struct.unpack_from("<I", payload)
            data = isp.xor(payload[5:], self.key)
            if cmd == isp.PROGRAM:
                self.flash[addr] = data
                return 0, b"\x00\x00"
            return 0, bytes([0 if self.flash.get(addr) == data else 0xFE, 0])
        if cmd == isp.ISP_END:
            return 0, b"\x00\x00"
        raise AssertionError("unexpected command 0x%02x" % cmd)


def test_flash_writes_config_first_and_checks_every_reply():
    bl = FakeBootloader()
    img = bytes(range(256)) * 2
    isp.flash(bl, 0, img, log=lambda _: None)
    assert bl.log.index(isp.WRITE_CFG) < bl.log.index(isp.PROGRAM)
    assert b"".join(bl.flash[a] for a in sorted(bl.flash)) == img
    assert bl.log[-1] == isp.ISP_END


def test_flash_reports_a_rejected_program():
    bl = FakeBootloader()
    bl.xfer = lambda cmd, payload, _real=bl.xfer: (
        (0, b"\xfe\x00") if cmd == isp.PROGRAM else _real(cmd, payload))
    with pytest.raises(isp.IspError, match="program 0x00000"):
        isp.flash(bl, 0, bytes(64), log=lambda _: None)


def test_flash_refuses_another_chip():
    with pytest.raises(isp.IspError, match="not a CH572"):
        isp.flash(FakeBootloader(chip=(0x82, 0x16)), 0, bytes(8), log=lambda _: None)


def test_parse_reply_rejects_a_reply_to_another_command():
    assert isp.parse_reply(0xA6, bytes.fromhex("a6000200fe00")) == (0, b"\xfe\x00")
    with pytest.raises(isp.IspError):
        isp.parse_reply(0xA6, bytes.fromhex("e0000200fe00"))
