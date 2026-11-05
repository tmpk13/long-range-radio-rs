"""Flash an ELF to the CH572's ROM bootloader over USB.

wchisp cannot flash this chip: the bootloader (BTVER 02.30) answered
every program and verify command with result 0xFE until the config
registers had been written, and wchisp never writes them on a flash and
ignores the result byte of program replies, so it reports success with
nothing written. This follows WCH's own order instead:
identify, config write, key exchange, erase, program, key exchange,
verify, end. Every reply's result byte is checked.

The config write sends back the 12 bytes just read. On the first board
that still changed CFG_ROM_READ from 0x3A to 0xFF, the datasheet's debug
read-out protection; USB flashing is not affected.

Usage: isp.py ELF [--wait SECONDS]
"""

import argparse
import struct
import sys
import time

CHUNK = 56  # largest payload that keeps a command in one 64-byte packet
EP_OUT, EP_IN = 0x02, 0x82
IDS = ((0x1A86, 0x55E0), (0x4348, 0x55E0))

IDENTIFY, ISP_END, ISP_KEY, ERASE, PROGRAM, VERIFY, READ_CFG, WRITE_CFG = (
    0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8)
CFG_ALL, CFG_REGS = 0x1F, 0x07  # read masks: everything, or the 12 config bytes
CFG_BOOT_EN = 1 << 6


class IspError(Exception):
    pass


def elf_image(raw):
    """Load segments of an ELF32 image at their load addresses.

    Returns (base, bytes), padded with 0xFF to a multiple of 8.
    """
    if raw[:4] != b"\x7fELF" or raw[4] != 1 or raw[5] != 1:
        raise IspError("not a little-endian ELF32 file")
    phoff, = struct.unpack_from("<I", raw, 0x1C)
    phentsize, phnum = struct.unpack_from("<HH", raw, 0x2A)
    segs = []
    for i in range(phnum):
        p_type, p_offset, _, p_paddr, p_filesz = struct.unpack_from(
            "<IIIII", raw, phoff + i * phentsize)
        if p_type == 1 and p_filesz:  # PT_LOAD with contents
            segs.append((p_paddr, raw[p_offset:p_offset + p_filesz]))
    if not segs:
        raise IspError("no loadable segments")
    base = min(a for a, _ in segs)
    end = max(a + len(d) for a, d in segs)
    img = bytearray(b"\xff" * (end - base))
    for a, d in segs:
        img[a - base:a - base + len(d)] = d
    img += b"\xff" * (-len(img) % 8)
    return base, bytes(img)


def xor_key(uid, chip_id):
    """The data key for an all-zero key seed: the UID's byte sum eight
    times, with the chip ID added to the last byte."""
    s = sum(uid) & 0xFF
    return bytes([s] * 7 + [(s + chip_id) & 0xFF])


def xor(data, key):
    return bytes(b ^ key[i % 8] for i, b in enumerate(data))


def frame(cmd, payload):
    return bytes([cmd]) + struct.pack("<H", len(payload)) + payload


def parse_reply(cmd, raw):
    """Return (status, payload) of a bootloader reply to cmd."""
    if len(raw) < 4 or raw[0] != cmd:
        raise IspError("reply to 0x%02x was %s" % (cmd, raw.hex()))
    n, = struct.unpack_from("<H", raw, 2)
    return raw[1], raw[4:4 + n]


class Bootloader:
    def __init__(self, dev):
        import usb.util
        self.dev = dev
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
        usb.util.claim_interface(dev, 0)

    def xfer(self, cmd, payload, timeout_ms=5000):
        self.dev.write(EP_OUT, frame(cmd, payload), timeout_ms)
        return parse_reply(cmd, bytes(self.dev.read(EP_IN, 64, timeout_ms)))

    def ok(self, cmd, payload, what):
        """Send a command whose reply carries a result byte; 0 is success."""
        status, p = self.xfer(cmd, payload)
        if status != 0 or not p or p[0] != 0:
            raise IspError("%s: status %02x result %s" % (what, status, p.hex()))


def wait_device(seconds):
    import usb.core
    deadline = time.monotonic() + seconds
    while True:
        for vid, pid in IDS:
            dev = usb.core.find(idVendor=vid, idProduct=pid)
            if dev is not None:
                return dev
        if time.monotonic() > deadline:
            raise IspError("no WCH bootloader (1a86:55e0 or 4348:55e0) within %d s" % seconds)
        time.sleep(0.2)


def flash(bl, base, img, log=print):
    _, p = bl.xfer(IDENTIFY, bytes(2) + b"MCU ISP & WCH.CN")
    chip_id, chip_type = p[0], p[1]
    if (chip_id, chip_type) != (0x72, 0x13):
        raise IspError("chip 0x%02x type 0x%02x is not a CH572" % (chip_id, chip_type))
    bl.xfer(IDENTIFY, bytes([chip_id, chip_type]) + b"MCU ISP & WCH.CN")

    _, p = bl.xfer(READ_CFG, bytes([CFG_ALL, 0]))
    btver, uid = p[14:18], p[18:26]
    _, p = bl.xfer(READ_CFG, bytes([CFG_REGS, 0]))
    cfg = p[2:14]
    user, = struct.unpack_from("<I", cfg, 8)
    log("CH572, bootloader %x%x.%x%x, UID %s, USER_CFG 0x%08x"
        % (*btver, uid.hex(), user))
    if not user & CFG_BOOT_EN or user >> 28 != 0b0100:
        raise IspError("USER_CFG 0x%08x: bootloader off or not valid; not writing it" % user)
    bl.ok(WRITE_CFG, bytes([CFG_REGS, 0]) + cfg, "config write")

    key = xor_key(uid, chip_id)
    _, p = bl.xfer(ISP_KEY, bytes(30))
    if p[0] != sum(key) & 0xFF:
        raise IspError("key checksum: bootloader %02x, expected %02x" % (p[0], sum(key) & 0xFF))
    bl.ok(ERASE, struct.pack("<I", max(8, -(-len(img) // 1024))), "erase")
    for off in range(0, len(img), CHUNK):
        bl.ok(PROGRAM, struct.pack("<I", base + off) + b"\x00"
              + xor(img[off:off + CHUNK], key), "program 0x%05x" % (base + off))
    log("programmed %d bytes" % len(img))

    _, p = bl.xfer(ISP_KEY, bytes(30))
    for off in range(0, len(img), CHUNK):
        bl.ok(VERIFY, struct.pack("<I", base + off) + b"\x00"
              + xor(img[off:off + CHUNK], key), "verify 0x%05x" % (base + off))
    log("verified")
    try:
        bl.xfer(ISP_END, bytes([1]))  # 1: reset into the new firmware
    except Exception:
        pass  # the chip may reset before it answers


def main():
    ap = argparse.ArgumentParser(description="Flash an ELF to the CH572 ROM bootloader")
    ap.add_argument("elf")
    ap.add_argument("--wait", type=int, default=30,
                    help="seconds to wait for the bootloader (default 30)")
    args = ap.parse_args()
    base, img = elf_image(open(args.elf, "rb").read())
    print("waiting %d s for the bootloader: plug in a blank chip, or run mise run ch572-boot"
          % args.wait, flush=True)
    try:
        flash(Bootloader(wait_device(args.wait)), base, img)
    except IspError as e:
        sys.exit("error: %s" % e)
    print("done; the bridge should come up as 1209:0001")


if __name__ == "__main__":
    main()
