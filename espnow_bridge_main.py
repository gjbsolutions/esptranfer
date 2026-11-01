# espnow_bridge_main.py  --  ESP32-C3, MicroPython. Same file on BOTH boards.
# Copy to the board as main.py, together with fastusb.py.
#
# Transparent bridge between the PC web app (index.html) and ESP-NOW:
#   PC -> board : frames to broadcast over ESP-NOW (and radio settings)
#   board -> PC : every received ESP-NOW packet (+RSSI), TX-done markers, stats
# All file-transfer logic (chunking, ACK/retransmit, CRC) lives in the web app.
#
# Do NOT print() while this runs: stdout shares the USB port with the binary stream.
#
# PC -> board frame (read from stdin):   [0xA5][type][n][payload n bytes]
#   type 1 = TX     : payload (1..250 bytes) is broadcast via ESP-NOW
#   type 2 = CONFIG : payload = [phy][txpower][channel]   (0 = leave unchanged)
#                     phy: 1 = 1M, 2 = 2M, 3 = 11M, 4 = LR500K
# board -> PC frame (fastusb):           [0xAA][0x55][type][n][payload]
#   type 1 = RX     : [rssi int8][received packet bytes...]
#   type 2 = STATS  : '<IIIII6sB'  uptime_ms, rx_fwd, rx_drops, tx_ok, tx_fail, mac, channel
#   type 3 = TXDONE : '<H' running count (mod 65536) of TX frames processed

import sys
import time
import struct
import select
import micropython
import network
import espnow
import fastusb

# 5 s window to interrupt with Ctrl+C before kbd_intr() disables it below.
time.sleep(5)

micropython.kbd_intr(-1)    # binary data may contain 0x03; never raise KeyboardInterrupt

BCAST = b'\xff' * 6
USB_SPIN = 150000           # ~10 ms of busy-wait per USB write before giving up
MUTE_MS = 100               # after a failed USB write (no host), skip USB this long
STATS_MS = 500
READ_TIMEOUT_MS = 200       # max wait for the rest of a USB frame
PHY_CODES = {1: 0x00, 2: 0x01, 3: 0x03, 4: 0x2A}   # wifi_phy_rate_t values

sta = network.WLAN(network.STA_IF)
sta.active(True)
try:
    sta.config(protocol=network.MODE_11B | network.MODE_11G |
                        network.MODE_11N | network.MODE_LR)
except Exception:
    pass

e = espnow.ESPNow()
e.config(rxbuf=16384)       # lower (e.g. 8192) if you get MemoryError
e.active(True)
try:
    e.add_peer(BCAST)
except Exception:
    pass

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

rx_fwd = tx_ok = tx_fail = tx_cnt = 0
mute_until = time.ticks_ms()
t0 = time.ticks_ms()
last_stats = t0


def usb_send(mv):
    global mute_until
    now = time.ticks_ms()
    if time.ticks_diff(now, mute_until) < 0:
        return
    if fastusb.write(mv, USB_SPIN) != len(mv):
        mute_until = time.ticks_add(now, MUTE_MS)


def forward(mac, msg):
    global rx_fwd
    n = len(msg)
    if n > 250:
        return
    try:
        r = e.peers_table[bytes(mac)][0]
    except Exception:
        r = -128
    rxf[3] = n + 1
    rxf[4] = r & 0xFF
    rxf[5:5 + n] = msg
    rx_fwd += 1
    usb_send(rxf_mv[:5 + n])


def read_exact(n):
    # False if the host stops mid-frame, so a truncated frame cannot stall the main loop.
    got = 0
    t = time.ticks_ms()
    while got < n:
        k = rd.readinto(inmv[got:n])
        if k:
            got += k
        elif time.ticks_diff(time.ticks_ms(), t) > READ_TIMEOUT_MS:
            return False
    return True


def apply_cfg(phy, pwr, ch):
    try:
        if ch:
            sta.config(channel=ch)
        if pwr:
            sta.config(txpower=pwr)
        if phy in PHY_CODES:
            e.config(rate=PHY_CODES[phy])
    except Exception:
        pass


def send_radio(mv):
    global tx_ok, tx_fail, tx_cnt
    try:
        ok = e.send(BCAST, mv)
    except OSError:
        ok = False
    if ok:
        tx_ok += 1
    else:
        tx_fail += 1
    tx_cnt = (tx_cnt + 1) & 0xFFFF
    struct.pack_into('<H', tdf, 4, tx_cnt)
    usb_send(tdf)


def handle_usb():
    if not read_exact(1) or inb[0] != 0xA5:
        return                      # resync: drop byte
    if not read_exact(2):
        return
    t = inb[0]
    n = inb[1]
    if n and not read_exact(n):
        return
    if t == 1 and 0 < n <= 250:
        send_radio(inmv[:n])
    elif t == 2 and n >= 3:
        apply_cfg(inb[0], inb[1], inb[2])


def send_stats(now):
    try:
        drops = e.stats()[4]
    except Exception:
        drops = 0
    try:
        ch = sta.config('channel')
    except Exception:
        ch = 0
    struct.pack_into('<IIIII6sB', stf, 4, time.ticks_diff(now, t0) & 0xFFFFFFFF,
                     rx_fwd & 0xFFFFFFFF, drops & 0xFFFFFFFF,
                     tx_ok & 0xFFFFFFFF, tx_fail & 0xFFFFFFFF,
                     sta.config('mac'), ch & 0xFF)
    usb_send(stf)


while True:
    try:
        n = 0
        while n < 8:
            mac, msg = e.irecv(0)
            if not msg:
                break
            forward(mac, msg)
            n += 1
        if poll.poll(0):
            handle_usb()
        elif n == 0:
            mac, msg = e.irecv(1)
            if msg:
                forward(mac, msg)
    except Exception:
        pass                        # never let the bridge die; keep forwarding

    now = time.ticks_ms()
    if time.ticks_diff(now, last_stats) >= STATS_MS:
        last_stats = now
        send_stats(now)
