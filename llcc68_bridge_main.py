# llcc68_bridge_main.py  --  ESP32-C3 + LLCC68 (GFSK), MicroPython. Same file on BOTH boards.
# Copy to the board as main.py, together with fastusb.py (unchanged).
#
# Drop-in replacement for espnow_bridge_main.py. Same USB framing, so the web app keeps working.
#
# PC -> board frame (stdin):   [0xA5][type][n][payload n bytes]
#   type 1 = TX     : payload (1..250 bytes) is transmitted as one GFSK packet
#   type 2 = CONFIG : payload = [phy][txpower][channel]   (0 = leave unchanged)
#                     phy: 1 = 9.6k, 2 = 48k, 3 = 100k, 4 = 250k  (see PRESETS)
#                     txpower: dBm (<= 22), channel: 1..13 -> FREQ_BASE + (ch-1)*FREQ_STEP
# board -> PC frame (fastusb): [0xAA][0x55][type][n][payload]
#   type 1 = RX     : [rssi int8][received packet bytes...]
#   type 2 = STATS  : '<IIIII6sB'  uptime_ms, rx_fwd, rx_drops(CRC err), tx_ok, tx_fail, id, channel
#   type 3 = TXDONE : '<H' running count (mod 65536) of TX frames finished ON AIR
#
# Do NOT print() while this runs: stdout shares the USB port with the binary stream.

import sys
import time
import struct
import select
import micropython
import machine
from machine import SPI, Pin
import fastusb

micropython.kbd_intr(-1)

# ---------------- pins (same as your TX script) ----------------
spi = SPI(1, baudrate=8000000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
          sck=Pin(4), mosi=Pin(6), miso=Pin(5))
cs = Pin(7, Pin.OUT, value=1)
RESET = Pin(2, Pin.OUT, value=1)
BUSY = Pin(3, Pin.IN)
DIO1 = Pin(10, Pin.IN)
TXEN = Pin(1, Pin.OUT, value=0)
RXEN = Pin(0, Pin.OUT, value=0)

# ---------------- radio settings ----------------
FREQ_BASE = 868000000       # channel 1. CHECK YOUR LOCAL REGULATIONS / DUTY CYCLE
FREQ_STEP = 100000
PREAMBLE_BITS = 32          # TX preamble length in BITS (detector is 16 bits)
SYNC_WORD = b'\x12\xAD'
MAX_PL = 250
# code: (bitrate bps, fdev Hz, RX_BW code).  RX BW >= 2*fdev + bitrate (+ crystal margin)
PRESETS = {
    1: (9600,   10000, 0x1B),   # 78.2 kHz
    2: (48000,  25000, 0x1A),   # 156.2 kHz
    3: (100000, 50000, 0x0A),   # 234.3 kHz
    4: (250000, 62500, 0x09),   # 467 kHz
}
DEF_PHY = 4
DEF_PWR = 2
DEF_CH = 1

IRQ_TX = 0x0001
IRQ_RX = 0x0002
IRQ_CRC = 0x0040
IRQ_MASK = 0x0243           # TxDone | RxDone | CrcErr | Timeout

USB_SPIN = 150000
MUTE_MS = 100
STATS_MS = 500
TX_TIMEOUT_MS = 1500


# ---------------- low level SPI ----------------
def busy_wait():
    n = 0
    while BUSY.value():
        n += 1
        if n > 300000:
            return


def cmd(b):
    busy_wait()
    cs.value(0)
    spi.write(b)
    cs.value(1)


def cmd_read(b, out):
    busy_wait()
    cs.value(0)
    spi.write_readinto(b, out)
    cs.value(1)


def write_reg(addr, data):
    cmd(bytes([0x0D, (addr >> 8) & 0xFF, addr & 0xFF]) + data)


_irq_tx = bytes([0x12, 0, 0, 0])
_irq_rx = bytearray(4)
_bs_tx = bytes([0x13, 0, 0, 0])
_bs_rx = bytearray(4)
_ps_tx = bytes([0x14, 0, 0, 0, 0])
_ps_rx = bytearray(5)
_rd_tx = bytearray(3 + 255)
_rd_tx[0] = 0x1E
_rd_tx_mv = memoryview(_rd_tx)
_rd_rx = bytearray(3 + 255)
_rd_rx_mv = memoryview(_rd_rx)


def get_irq():
    cmd_read(_irq_tx, _irq_rx)
    return (_irq_rx[2] << 8) | _irq_rx[3]


def clear_irq():
    cmd(b'\x02\xff\xff')


pp_len = -1


def set_pp(plen):
    # variable-length GFSK packet. TX: plen = actual length. RX: plen = max length (255).
    global pp_len
    if plen == pp_len:
        return
    pp_len = plen
    cmd(bytes([0x8C,
               (PREAMBLE_BITS >> 8) & 0xFF, PREAMBLE_BITS & 0xFF,
               0x05,       # preamble detector 16 bits
               0x10,       # sync word 16 bits
               0x00,       # no address filter
               0x01,       # variable length
               plen,
               0x06,       # CRC 2 byte inverted
               0x01]))     # whitening ON (keeps the spectrum balanced for files full of 0x00/0xFF)


def set_mod(phy):
    br, fdev, bw = PRESETS[phy]
    brr = (32 * 32000000) // br
    fr = (fdev << 25) // 32000000
    cmd(bytes([0x8B,
               (brr >> 16) & 0xFF, (brr >> 8) & 0xFF, brr & 0xFF,
               0x09,                       # Gaussian BT 0.5
               bw,
               (fr >> 16) & 0xFF, (fr >> 8) & 0xFF, fr & 0xFF]))


def set_freq(hz):
    r = (hz << 20) // 1000000          # hz * 2^25 / 32e6
    cmd(bytes([0x86, (r >> 24) & 0xFF, (r >> 16) & 0xFF, (r >> 8) & 0xFF, r & 0xFF]))


def set_power(dbm):
    if dbm > 22:
        dbm = 22
    if dbm <= 14:
        cmd(b'\x95\x02\x02\x00\x01')   # +14 dBm PA config
    else:
        cmd(b'\x95\x04\x07\x00\x01')   # +22 dBm PA config
    cmd(bytes([0x8E, dbm & 0xFF, 0x04]))


def radio_init():
    RESET.value(0)
    time.sleep_us(300)
    RESET.value(1)
    time.sleep_ms(10)
    cmd(b'\x80\x00')                    # STDBY_RC
    cmd(b'\x8a\x00')                    # packet type GFSK
    set_freq(FREQ_BASE + (cur_ch - 1) * FREQ_STEP)
    cmd(b'\x98\xd7\xdb')                # image calibration 863-870 MHz
    cmd(b'\x8f\x00\x00')                # TX/RX buffer base = 0
    set_mod(cur_phy)
    set_power(cur_pwr)
    write_reg(0x06BC, b'\x1D\x0F')      # CRC init (CCITT)
    write_reg(0x06BE, b'\x10\x21')      # CRC poly
    write_reg(0x06C0, SYNC_WORD)
    cmd(bytes([0x08, 0x02, 0x43, 0x02, 0x43, 0, 0, 0, 0]))   # IRQ -> DIO1


def enter_rx():
    TXEN.value(0)
    RXEN.value(1)
    clear_irq()
    set_pp(255)
    cmd(b'\x82\xff\xff\xff')            # RX continuous


# ---------------- state ----------------
cur_phy = DEF_PHY
cur_pwr = DEF_PWR
cur_ch = DEF_CH
pend_cfg = None
tx_q = []
txing = False
tx_t0 = 0
rx_fwd = rx_drops = tx_ok = tx_fail = tx_cnt = 0
mute_until = time.ticks_ms()
t0 = time.ticks_ms()
last_stats = t0
ID = (machine.unique_id() + b'\x00' * 6)[:6]

rd = sys.stdin.buffer
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
    cmd(b'\x80\x00')                    # standby
    RXEN.value(0)
    TXEN.value(1)
    set_pp(len(pl))
    busy_wait()
    cs.value(0)
    spi.write(b'\x0e\x00')
    spi.write(pl)
    cs.value(1)
    clear_irq()
    cmd(b'\x83\x00\x00\x00')            # TX, no timeout
    txing = True
    tx_t0 = time.ticks_ms()


def finish_tx(ok):
    global txing, tx_ok, tx_fail, tx_cnt
    txing = False
    if ok:
        tx_ok += 1
    else:
        tx_fail += 1
        cmd(b'\x80\x00')
    enter_rx()                          # chip is in standby here: TXEN off first, then RXEN on
    tx_cnt = (tx_cnt + 1) & 0xFFFF
    struct.pack_into('<H', tdf, 4, tx_cnt)
    usb_send(tdf)                       # may spin up to ~10 ms; must not delay the switch back


def recover():
    # After any SPI/IO error: PA off first (standby), then TXEN off, then back to RX.
    global txing
    txing = False
    try:
        cmd(b'\x80\x00')
    except Exception:
        pass
    TXEN.value(0)
    try:
        enter_rx()
    except Exception:
        pass


def rx_done():
    global rx_fwd, rx_drops
    cmd_read(_bs_tx, _bs_rx)
    n = _bs_rx[2]
    off = _bs_rx[3]
    cmd_read(_ps_tx, _ps_rx)
    rssi = -(_ps_rx[3] >> 1)            # RssiSync / 2, dBm
    if n == 0 or n > MAX_PL:
        rx_drops += 1
        return
    _rd_tx[1] = off
    cmd_read(_rd_tx_mv[:3 + n], _rd_rx_mv[:3 + n])
    rxf[3] = n + 1
    rxf[4] = rssi & 0xFF
    rxf[5:5 + n] = _rd_rx_mv[3:3 + n]
    rx_fwd += 1
    usb_send(rxf_mv[:5 + n])


def apply_cfg_now(phy, pwr, ch):
    global cur_phy, cur_pwr, cur_ch
    cmd(b'\x80\x00')
    if ch and 1 <= ch <= 13:
        cur_ch = ch
        set_freq(FREQ_BASE + (ch - 1) * FREQ_STEP)
    if phy in PRESETS:
        cur_phy = phy
        set_mod(phy)
    if pwr:
        cur_pwr = pwr
        set_power(pwr)
    enter_rx()


def radio_poll():
    global pend_cfg, rx_drops
    if txing:
        if DIO1.value():
            irq = get_irq()
            clear_irq()
            if irq & IRQ_TX:
                finish_tx(True)
                return
        if time.ticks_diff(time.ticks_ms(), tx_t0) > TX_TIMEOUT_MS:
            finish_tx(False)
        return
    if DIO1.value():
        irq = get_irq()
        clear_irq()
        if irq & IRQ_RX:
            if irq & IRQ_CRC:
                rx_drops += 1
            else:
                rx_done()
    if pend_cfg is not None:
        c = pend_cfg
        pend_cfg = None
        apply_cfg_now(c[0], c[1], c[2])
    elif tx_q:
        start_tx(tx_q.pop(0))


# ---------------- USB in ----------------
def read_exact(n):
    got = 0
    while got < n:
        k = rd.readinto(inmv[got:n])
        if k:
            got += k


def handle_usb():
    global pend_cfg
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
    elif t == 2 and n >= 3:
        pend_cfg = (inb[0], inb[1], inb[2])


def send_stats(now):
    struct.pack_into('<IIIII6sB', stf, 4, time.ticks_diff(now, t0) & 0xFFFFFFFF,
                     rx_fwd & 0xFFFFFFFF, rx_drops & 0xFFFFFFFF,
                     tx_ok & 0xFFFFFFFF, tx_fail & 0xFFFFFFFF,
                     ID, cur_ch & 0xFF)
    usb_send(stf)


# ---------------- run ----------------
radio_init()
enter_rx()

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
