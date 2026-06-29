"""
damiao_can.py — DaMiao USB-to-CAN adapter transport.

The DaMiao USB-to-CAN debugger (HDSC HC32, enumerates as a USB-CDC serial port,
e.g. /dev/ttyACM0) does NOT speak SocketCAN. It uses DaMiao's own framing:

  TX (host → adapter), 30 bytes, one standard 8-byte CAN frame:
    [0]=0x55 [1]=0xAA  header        [2]=0x1e (len=30)   [3]=0x03 cmd
    [13]=id&0xff  [14]=(id>>8)&0xff  standard CAN id
    [18]=0x08  data length           [21..28]=8 CAN data bytes
  RX (adapter → host), 16 bytes, one received CAN frame:
    [0]=0xAA header  [15]=0x55 tail
    [3..6]=CAN id (little-endian 32-bit)   [7..14]=8 data bytes

Byte layout taken from DaMiao's official DM_CAN SDK.
"""
import threading
import serial

TX_LEN = 30
RX_LEN = 16
RX_HEADER = 0xAA
RX_TAIL = 0x55
DEFAULT_BAUD = 921600   # CDC-ACM ignores baud, but the DaMiao tool uses this


class DamiaoCAN:
    def __init__(self, port="/dev/ttyACM0", baudrate=DEFAULT_BAUD, on_frame=None):
        """on_frame(can_id:int, data:bytes[8]) is called for every RX frame."""
        self.port = port
        self.baudrate = baudrate
        self.on_frame = on_frame
        self._ser = None
        self._rx_thread = None
        self._running = False

    # ── lifecycle ──────────────────────────────────────────────────────────────
    def open(self):
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self._ser.reset_input_buffer()
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ── TX ──────────────────────────────────────────────────────────────────────
    def send(self, can_id, data8):
        """Send one standard CAN frame (8 data bytes) to `can_id`."""
        f = bytearray(TX_LEN)
        f[0] = 0x55
        f[1] = 0xAA
        f[2] = 0x1E
        f[3] = 0x03
        f[4] = 0x01
        f[8] = 0x0A
        f[13] = can_id & 0xFF
        f[14] = (can_id >> 8) & 0xFF
        f[18] = 0x08
        f[21:29] = bytes(data8[:8])
        self._ser.write(f)

    # ── RX ──────────────────────────────────────────────────────────────────────
    def _rx_loop(self):
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(256)
            except Exception:
                continue
            if chunk:
                buf.extend(chunk)
                i = 0
                while len(buf) - i >= RX_LEN:
                    if buf[i] != RX_HEADER or buf[i + RX_LEN - 1] != RX_TAIL:
                        i += 1            # resync
                        continue
                    p = buf[i:i + RX_LEN]
                    can_id = p[3] | (p[4] << 8) | (p[5] << 16) | (p[6] << 24)
                    data = bytes(p[7:15])
                    if self.on_frame:
                        self.on_frame(can_id, data)
                    i += RX_LEN
                del buf[:i]
                if len(buf) > 4096:       # never let an unsynced buffer grow
                    del buf[:-RX_LEN]
