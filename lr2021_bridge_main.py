# lr2021_bridge_main.py  --  ESP32-C3 + LR2021 (FLRC), MicroPython. Same file on BOTH boards.
# Copy to the board as main.py, together with fastusb.py (unchanged).
#
# FIXED radio setup: 868.000 MHz, FLRC 325 kbps / 357 kHz, CR 3/4, Gaussian BT 0.5,
# 32-bit syncword, CRC16, fixed 255-byte packets.
#
# Over-the-air frame (always 255 bytes):  [n][web-app frame, n bytes][zero padding]
# PC -> board (stdin):   [0xA5][type][n][payload]     type 1 = TX, type 2 = CONFIG (ignored)
# board -> PC (fastusb): [0xAA][0x55][type][n][payload]
#   type 1 = RX     : [rssi int8 = -128..0][frame bytes...]
#   type 2 = STATS  : '<IIIII6sB'
#   type 3 = TXDONE : '<H' running count
#
# Do NOT print() while this runs.

import sys
import time
import struct
import select
import micropython
import machine
from machine import SPI, Pin
import fastusb


time.sleep(5)

micropython.kbd_intr(-1)

# ---------------- pins ----------------
spi = SPI(1, baudrate=16000000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
          sck=Pin(4), mosi=Pin(6), miso=Pin(5))
CS = Pin(7, Pin.OUT, value=1)
IRQ = Pin(10, Pin.IN)       # DIO9 of LR2021
NRST = Pin(2, Pin.OUT, value=1)
BUSY = Pin(3, Pin.IN)

# ---------------- fixed radio configuration ----------------
FREQ = 868000000
SYNCWORD = b'\x54\x69\x6E\x79'
BITRATE_BW = 0x06
CR = 0x01
PULSE_SHAPE = 0x05
AGC_PBL_LEN = 0x03
SYNC_LEN = 0x02
CRC_LEN = 0x01
PKT_DYNAMIC = 0
FRAME_LEN = 255
MAX_PL = 250

CALIB_MASK = 0x6F           # Table 6-28: LF_RC, HF_RC, PLL, AAF(bit 3), MU, PA_OFF

# Lowest verified entry of Table 7-19 (10.0 dBm). The datasheet has no 2 dBm entry for the LF PA.
PA_RAW, PA_DUTY, PA_SLICES = 16, 1, 2
PA_RAMP = 0x04

IRQ_RX_DONE = 1 << 18
IRQ_TX_DONE = 1 << 19
IRQ_TIMEOUT = 1 << 21
IRQ_CRC = 1 << 22
IRQ_LEN = 1 << 23
IRQ_MASK = IRQ_RX_DONE | IRQ_TX_DONE | IRQ_TIMEOUT | IRQ_CRC | IRQ_LEN

USB_SPIN = 150000
MUTE_MS = 20
STATS_MS = 500
TX_TIMEOUT_MS = 1500


# ---------------- low level SPI ----------------
def busy_wait():
    n = 0
    while BUSY.value():
        n += 1
        if n > 300000:
            return


def _cmd(buf):
    busy_wait()
    CS.value(0)
    spi.write(buf)
    CS.value(1)


def wr(hi, lo, args=()):
    busy_wait()
    CS.value(0)
    spi.write(bytes([hi, lo]) + bytes(args))
    CS.value(1)


def rd(hi, lo, n):
    busy_wait()
    CS.value(0)
    spi.write(bytes([hi, lo]))
    CS.value(1)
    busy_wait()
    CS.value(0)
    r = bytearray(n)
    spi.readinto(r, 0x00)
    CS.value(1)
    return r


def u32(v):
    return [(v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF]


# ---------------- prebuilt command buffers ----------------
_SETTX        = b'\x02\x0D\x00\x00\x00'
_CLRIRQ       = b'\x01\x16\xFF\xFF\xFF\xFF'
_STDBY_XOSC   = b'\x01\x28\x01'
_SETRX_CONT   = b'\x02\x0C\xFF\xFF\xFF'
_CLEAR_RXFIFO = b'\x01\x1E'

# ClearIrq with a variable mask (only the bits we have actually handled)
_clr = bytearray(b'\x01\x16\x00\x00\x00\x00')

_ftx        = bytearray(2 + FRAME_LEN)
_ftx[1]     = 0x02
_ftx_mv     = memoryview(_ftx)
_ZERO       = bytes(FRAME_LEN)

_ftx_rx     = bytearray(2 + FRAME_LEN)
_ftx_rx[1]  = 0x01
_ftx_rx_mv  = memoryview(_ftx_rx)
_frx        = bytearray(2 + FRAME_LEN)
_frx_mv     = memoryview(_frx)


def fifo_read(n):
    busy_wait()
    CS.value(0)
    spi.write_readinto(_ftx_rx_mv[:2 + n], _frx_mv[:2 + n])
    CS.value(1)                              # data is in _frx[2:2+n]


def fifo_write_and_tx(pl):
    n = len(pl)
    _ftx[2] = n
    _ftx_mv[3:3 + n] = pl
    _ftx_mv[3 + n:] = _ZERO[:FRAME_LEN - 1 - n]
    _cmd(_ftx)
    _cmd(_SETTX)


def get_irq():
    r = rd(0x01, 0x00, 6)                    # GetStatus
    return (r[2] << 24) | (r[3] << 16) | (r[4] << 8) | r[5]


def clear_irq():
    _cmd(_CLRIRQ)


def clear_irq_bits(bits):
    # Clear only the flags we have read and are about to handle, so a flag that is
    # raised in between (e.g. a second RX_DONE) is not silently wiped.
    bits &= IRQ_MASK
    if bits:
        struct.pack_into('>I', _clr, 2, bits)
        _cmd(_clr)


def get_flrc_status():
    """Read GetFlrcPacketStatus (opcode 0x024B), Table 18-9 (verified against datasheet).
      byte 2..3 : pkt_len(15:0)
      byte 4    : rssi_avg(8:1)
      byte 5    : rssi_sync(8:1)
      byte 6    : sw_num(7:4), rfu(3), rssi_avg(0) = bit 2, rfu(1), rssi_sync(0) = bit 0
      byte 7..9 : freq_offset(23:0)
    Signal power = -rssi_avg / 2 dBm.  Returns (pkt_len, rssi_dbm_int).
    """
    r = rd(0x02, 0x4B, 10)
    pkt_len = (r[2] << 8) | r[3]
    rssi_avg = (r[4] << 1) | ((r[6] >> 2) & 0x01)
    rssi_dbm = -(rssi_avg >> 1)
    if rssi_dbm > 0:
        rssi_dbm = 0
    if rssi_dbm < -127:
        rssi_dbm = -127                      # -128 is reserved as "unknown"
    return pkt_len, rssi_dbm


def set_power():
    wr(0x02, 0x02, [0x00, ((PA_DUTY & 0x0F) << 4) | (PA_SLICES & 0x0F), 16])
    wr(0x02, 0x03, [PA_RAW & 0xFF, PA_RAMP])


# ---------------- radio init (once) ----------------
def radio_init():
    NRST.value(0)
    time.sleep_ms(1)
    NRST.value(1)
    time.sleep_ms(5)
    busy_wait()
    time.sleep_ms(2)

    wr(0x01, 0x28, [0x00])              # STDBY_RC
    wr(0x01, 0x12, [9, 0x10])           # SetDioFunction(9, IRQ): datasheet 5.1.1 -> Standby RC only
    wr(0x01, 0x11)                      # ClearErrors
    wr(0x01, 0x21, [0x02])              # SetRegMode: SIMO DC-DC
    wr(0x01, 0x28, [0x01])              # STDBY_XOSC
    time.sleep_ms(10)

    wr(0x01, 0x22, [CALIB_MASK])        # Calibrate
    busy_wait()
    time.sleep_ms(5)

    code = (FREQ // 4000000) & 0x7FFF
    wr(0x01, 0x23, [(code >> 8) & 0xFF, code & 0xFF, 0, 0, 0, 0])   # CalibFE
    busy_wait()
    time.sleep_ms(5)

    wr(0x01, 0x28, [0x01])
    wr(0x02, 0x00, u32(FREQ))           # SetRfFrequency
    clear_irq()
    wr(0x01, 0x11)

    wr(0x02, 0x07, [0x05])              # SetPacketType FLRC
    wr(0x02, 0x06, [0x02])              # SetRxTxFallbackMode = STDBY_XOSC
    wr(0x02, 0x48, [BITRATE_BW, ((CR & 0x0F) << 4) | (PULSE_SHAPE & 0x0F)])
    byte2 = ((AGC_PBL_LEN & 0x0F) << 2) | (SYNC_LEN & 0x03)
    byte3 = (1 << 6) | (0x01 << 3) | ((PKT_DYNAMIC & 0x01) << 2) | (CRC_LEN & 0x03)
    wr(0x02, 0x49, [byte2, byte3, (FRAME_LEN >> 8) & 0xFF, FRAME_LEN & 0xFF])
    wr(0x02, 0x4C, [1] + list(SYNCWORD))
    wr(0x02, 0x01, [0x00, 0x00])        # SetRxPath: LF path, no boost
    wr(0x01, 0x15, [9] + u32(IRQ_MASK)) # SetDioIrqConfig
    set_power()


def enter_rx():
    _cmd(_STDBY_XOSC)
    clear_irq()
    _cmd(_CLEAR_RXFIFO)
    _cmd(_SETRX_CONT)


# ---------------- state ----------------
tx_q = []
txing = False
need_rx = False
tx_t0 = 0
rx_fwd = rx_drops = tx_ok = tx_fail = tx_cnt = 0
mute_until = time.ticks_ms()
t0 = time.ticks_ms()
last_stats = t0
ID = (machine.unique_id() + b'\x00' * 6)[:6]

rd_in = sys.stdin.buffer
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)
inb = bytearray(256)
inmv = memoryview(inb)

rxf = bytearray(4 + 1 + 250)
rxf_mv = memoryview(rxf)
rxf[0] = 0xAA; rxf[1] = 0x55; rxf[2] = 1

stf = bytearray(4 + 27)
stf[0] = 0xAA; stf[1] = 0x55; stf[2] = 2; stf[3] = 27

tdf = bytearray(6)
tdf[0] = 0xAA; tdf[1] = 0x55; tdf[2] = 3; tdf[3] = 2


def usb_send(mv):
    global mute_until
    now = time.ticks_ms()
    if time.ticks_diff(now, mute_until) < 0:
        return
    if fastusb.write(mv, USB_SPIN) != len(mv):
        mute_until = time.ticks_add(now, MUTE_MS)


# ---------------- TX / RX ----------------
def start_tx(pl):
    global txing, tx_t0
    _cmd(_STDBY_XOSC)
    fifo_write_and_tx(pl)
    txing = True
    tx_t0 = time.ticks_ms()


def finish_tx(ok):
    global txing, tx_ok, tx_fail, tx_cnt, need_rx
    txing = False
    if ok:
        tx_ok += 1
    else:
        tx_fail += 1
    tx_cnt = (tx_cnt + 1) & 0xFFFF
    struct.pack_into('<H', tdf, 4, tx_cnt)
    usb_send(tdf)
    need_rx = True


def recover():
    global txing, need_rx
    txing = False
    need_rx = False
    try:
        enter_rx()
    except Exception:
        pass


def rx_done(irq):
    global rx_fwd, rx_drops
    # A CRC-error packet may be flagged WITHOUT RX_DONE. Never trust its contents, and
    # always flush the FIFO (datasheet 5.3.1: bad-CRC data is still written to the FIFO).
    if irq & IRQ_CRC:
        _cmd(_CLEAR_RXFIFO)
        rx_drops += 1
        return
    # Read packet status (length + RSSI) before touching the FIFO.
    n, rssi_dbm = get_flrc_status()
    if n != FRAME_LEN:
        _cmd(_CLEAR_RXFIFO)
        rx_drops += 1
        return
    fifo_read(n)
    m = _frx[2]
    if m < 1 or m > MAX_PL:
        rx_drops += 1
        return
    rxf[3] = m + 1
    rxf[4] = rssi_dbm & 0xFF                 # signed int8, -128 = "unknown"
    rxf[5:5 + m] = _frx_mv[3:3 + m]
    rx_fwd += 1
    usb_send(rxf_mv[:5 + m])


def radio_poll():
    global rx_drops, need_rx
    if txing:
        if IRQ.value():
            irq = get_irq()
            # clear only what we read (keeps a pending RX flag intact)
            clear_irq_bits(irq)
            if irq & (IRQ_TX_DONE | IRQ_TIMEOUT):
                finish_tx(bool(irq & IRQ_TX_DONE))
                return
        if time.ticks_diff(time.ticks_ms(), tx_t0) > TX_TIMEOUT_MS:
            finish_tx(False)
        return

    if tx_q:
        start_tx(tx_q.pop(0))
        return

    if need_rx:
        enter_rx()
        need_rx = False

    if IRQ.value():
        irq = get_irq()
        clear_irq_bits(irq)
        if irq & (IRQ_RX_DONE | IRQ_CRC):     # CRC alone also handled
            rx_done(irq)
        elif irq & IRQ_LEN:
            _cmd(_CLEAR_RXFIFO)
            rx_drops += 1


# ---------------- USB in ----------------
def read_exact(n):
    got = 0
    while got < n:
        k = rd_in.readinto(inmv[got:n])
        if k:
            got += k


def handle_usb():
    read_exact(1)
    if inb[0] != 0xA5:
        return
    read_exact(2)
    t = inb[0]
    n = inb[1]
    if n:
        read_exact(n)
    if t == 1 and 0 < n <= MAX_PL:
        if len(tx_q) >= 24:
            tx_q.pop(0)
        tx_q.append(bytes(inmv[:n]))


def send_stats(now):
    struct.pack_into('<IIIII6sB', stf, 4, time.ticks_diff(now, t0) & 0xFFFFFFFF,
                     rx_fwd & 0xFFFFFFFF, rx_drops & 0xFFFFFFFF,
                     tx_ok & 0xFFFFFFFF, tx_fail & 0xFFFFFFFF,
                     ID, 1)
    usb_send(stf)


# ---------------- run ----------------
radio_init()
enter_rx()
need_rx = False

while True:
    try:
        k = 0
        while k < 4 and poll.poll(0):
            handle_usb()
            k += 1
        radio_poll()
    except (OSError, ValueError):
        recover()

    now = time.ticks_ms()
    if time.ticks_diff(now, last_stats) >= STATS_MS:
        last_stats = now
        send_stats(now)
