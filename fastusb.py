# fastusb.py -- fast MCU -> PC writes over the ESP32-C3 USB-Serial-JTAG port (MicroPython)
#
# Bypasses MicroPython's stdout driver (which measured ~12 KiB/s on v1.27) and writes
# straight into the USB-Serial-JTAG TX FIFO. Measured on an ESP32-C3 SuperMini + Windows:
# ~700-745 KiB/s.
#
# Usage:
#     import fastusb
#     n = fastusb.write(data)      # data: bytes / bytearray / memoryview, any length
#     # n == len(data) on success; smaller if the host stopped reading (~0.2 s timeout)
#
# Notes:
# - Every call ends with the packet flushed, so short tails are delivered immediately.
# - Don't call this from an interrupt handler or a second thread while print()/stdout is
#   also writing; mixing is fine from the main thread (each call leaves the FIFO flushed).
# - Registers: ESP32-C3 USB_SERIAL_JTAG base 0x60043000, +0x0 EP1 (TX FIFO byte),
#   +0x4 EP1_CONF (bit0 WR_DONE, bit1 IN_EP_DATA_FREE).
import micropython

SPIN_LIMIT = 3_000_000  # busy-wait iterations before giving up (~0.2 s at 160 MHz)


@micropython.viper
def _send(buf, n: int, spin_limit: int) -> int:
    p = ptr8(buf)
    r = ptr32(0x60043000)
    i = 0
    while i < n:
        spin = 0
        while not (r[1] & 2):          # wait until FIFO is writable
            spin += 1
            if spin > spin_limit:
                return i               # host isn't reading
        m = n - i
        if m > 64:
            m = 64
        j = 0
        while j < m:
            r[0] = p[i + j]            # push one byte into the TX FIFO
            j += 1
        r[1] = 1                       # WR_DONE: send this packet now
        i += m
    return n


def write(data, spin_limit=SPIN_LIMIT):
    return _send(data, len(data), spin_limit)

