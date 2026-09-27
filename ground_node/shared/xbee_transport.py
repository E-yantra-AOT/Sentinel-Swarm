"""
xbee_transport.py
=================
Lightweight XBee transparent-mode serial transport.

Each message is a single JSON object terminated by a newline.
Works with XBee modules configured in Transparent (AP=0) mode,
PAN ID 3333, baud 9600.

Usage on Robot A (sender):
    xbee = XBeeTransport('/dev/ttyUSB0')
    xbee.send({'state': 'TRACKING', 'cx': 0.52, 'bearing': -3.2})

Usage on Robot B (receiver):
    xbee = XBeeTransport('/dev/ttyUSB0')
    msg = xbee.recv()   # returns dict or None (non-blocking)
    if msg:
        print(msg['state'], msg['cx'])
"""

import serial
import json
import time
import threading
import logging

logger = logging.getLogger(__name__)

# ─── Protocol Constants ───────────────────────────────────────────────────────
XBEE_PORT      = '/dev/ttyUSB0'
XBEE_BAUD      = 9600
XBEE_TIMEOUT   = 0.05   # Serial read timeout in seconds (non-blocking)

# Safety: if no message received for this many seconds, treat link as dead
LINK_TIMEOUT_S = 2.0


class XBeeTransport:
    """
    Thread-safe XBee transparent-mode JSON transport.
    Sends and receives newline-delimited JSON messages over serial.
    """

    def __init__(self, port=XBEE_PORT, baud=XBEE_BAUD):
        self.port = port
        self.baud = baud
        self._ser = None
        self._rx_buffer = b""
        self._last_rx_time = None
        self._lock = threading.Lock()
        self._connect()

    def _connect(self):
        try:
            self._ser = serial.Serial(
                self.port, self.baud,
                timeout=XBEE_TIMEOUT,
                write_timeout=1.0
            )
            time.sleep(0.5)
            logger.info(f"[XBee] Connected on {self.port} @ {self.baud} baud")
        except serial.SerialException as e:
            logger.error(f"[XBee] Failed to open {self.port}: {e}")
            self._ser = None

    @property
    def is_connected(self):
        return self._ser is not None and self._ser.is_open

    def send(self, data: dict) -> bool:
        """
        Serialize dict to JSON and send over XBee.
        Returns True on success, False on failure.
        """
        if not self.is_connected:
            logger.warning("[XBee] Cannot send — not connected.")
            return False
        try:
            payload = json.dumps(data, separators=(',', ':')) + '\n'
            with self._lock:
                self._ser.write(payload.encode('ascii'))
            return True
        except Exception as e:
            logger.error(f"[XBee] Send error: {e}")
            return False

    def recv(self) -> dict | None:
        """
        Non-blocking read. Returns a parsed dict if a complete message
        is available, otherwise returns None.
        """
        if not self.is_connected:
            return None
        try:
            with self._lock:
                chunk = self._ser.read(256)
            if chunk:
                self._rx_buffer += chunk
                # Extract complete newline-terminated messages
                while b'\n' in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line.decode('ascii'))
                        self._last_rx_time = time.time()
                        return msg
                    except (json.JSONDecodeError, UnicodeDecodeError) as e:
                        logger.warning(f"[XBee] Bad packet: {line!r} → {e}")
                        self._rx_buffer = b""  # Clear buffer to prevent cascading corruption
        except Exception as e:
            logger.error(f"[XBee] Recv error: {e}")
        return None

    def link_alive(self) -> bool:
        """
        Returns True if a message was received within LINK_TIMEOUT_S seconds.
        Use this as a safety watchdog on Robot B.
        """
        if self._last_rx_time is None:
            return False
        return (time.time() - self._last_rx_time) < LINK_TIMEOUT_S

    def close(self):
        if self._ser and self._ser.is_open:
            self._ser.close()
            logger.info("[XBee] Port closed.")


# ─── Quick self-test (run directly to verify serial link) ─────────────────────
if __name__ == '__main__':
    import sys
    logging.basicConfig(level=logging.INFO, format='%(message)s')

    mode = sys.argv[1] if len(sys.argv) > 1 else 'recv'
    port = sys.argv[2] if len(sys.argv) > 2 else XBEE_PORT

    xbee = XBeeTransport(port)

    if not xbee.is_connected:
        print(f"ERROR: Could not open {port}. Check if XBee is plugged in.")
        sys.exit(1)

    if mode == 'send':
        print(f"Sending test packets on {port} every 1s. Press Ctrl+C to stop.")
        seq = 0
        try:
            while True:
                msg = {'robot': 'A', 'seq': seq, 'state': 'TRACKING',
                       'cx': round(0.5 + seq * 0.01, 3), 'bearing': round(-3.2 + seq, 2)}
                ok = xbee.send(msg)
                print(f"TX {'OK' if ok else 'FAIL'}: {msg}")
                seq += 1
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass

    else:  # recv mode
        print(f"Listening on {port}. Press Ctrl+C to stop.")
        try:
            while True:
                msg = xbee.recv()
                if msg:
                    print(f"RX: {msg}")
                    print(f"    Link alive: {xbee.link_alive()}")
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass

    xbee.close()
    print("Done.")
