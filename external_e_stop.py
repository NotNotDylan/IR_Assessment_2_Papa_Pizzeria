# external_e_stop.py
# Cross-platform Arduino e-stop listener with latch-friendly semantics for your runner.

import threading
import time
from typing import Optional

try:
    import serial
    from serial.tools import list_ports
except Exception:  # serial may not exist on all dev machines
    serial = None
    list_ports = None


def autodetect_arduino_port() -> Optional[str]:
    """
    Try to find an Arduino-like serial port. Returns port string or None.
    You can hardcode COM3 / /dev/ttyACM0 if you prefer.
    """
    if list_ports is None:
        return None
    ports = list(list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "arduino" in desc or "wch" in desc or "usb serial" in desc or "ch340" in hwid:
            return p.device
    # Fall back to first ttyACM/ttyUSB on *nix
    for p in ports:
        if "ttyacm" in (p.device or "").lower() or "ttyusb" in (p.device or "").lower():
            return p.device
    return None


class ExternalEStop:
    """
    Reads newline-terminated messages from Arduino:
      - 'ESTOP' -> pressed
      - 'CLEAR' -> released
      - 'HB'    -> heartbeat (keeps link alive)
    If no heartbeat is seen within hb_timeout_s, we fail-safe to pressed=True.
    """

    def __init__(self, port: Optional[str] = None, baud: int = 115200, hb_timeout_s: float = 3.0, ex_estop_connected: bool = False):
        self.port = port or autodetect_arduino_port()
        self.baud = baud
        self.hb_timeout_s = hb_timeout_s
        self.ex_estop_connected = ex_estop_connected

        self._ser = None
        self._rx_thread = None
        self._stop = threading.Event()

        self.is_pressed = False
        self._last_hb = time.monotonic()  # updated on any valid line, not just HB

        self._connect()
        self._start_reader()

    # ---- public API ----
    def check_stop(self) -> bool:
        """Return True if hardware e-stop is currently asserted, or if link timed out."""
        if self.ex_estop_connected:
            if (time.monotonic() - self._last_hb) > self.hb_timeout_s:
                # link stale -> fail safe
                return True
        return bool(self.is_pressed)

    def close(self):
        """Stop background thread and close serial."""
        self._stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    # ---- internals ----
    def _connect(self):
        if self.ex_estop_connected:
            if serial is None:
                print("[ExternalEStop] pyserial not available; running in NO-SERIAL mode.")
                return
            if not self.port:
                print("[ExternalEStop] No serial port found (autodetect failed). Set port explicitly.")
                return
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
                print(f"[ExternalEStop] Connected on {self.port} @ {self.baud}")
            except Exception as e:
                print(f"[ExternalEStop] Could not open {self.port}: {e}")
                self._ser = None

    def _start_reader(self):
        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._rx_thread.start()

    def _reader_loop(self):
        buf = b""
        while not self._stop.is_set():
            if self._ser is None:
                # try reconnect every second
                time.sleep(1.0)
                self._connect()
                continue

            try:
                chunk = self._ser.read(128)
            except Exception:
                # read error -> drop connection and retry
                self._ser = None
                continue

            if not chunk:
                # idle: still check heartbeat timeout in check_stop()
                continue

            buf += chunk
            # process newline-terminated lines
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                msg = line.decode(errors="ignore").strip().upper()
                if not msg:
                    continue

                self._last_hb = time.monotonic()
                if msg == "ESTOP":
                    self.is_pressed = True
                    # print("[ExternalEStop] ESTOP")
                elif msg == "CLEAR":
                    self.is_pressed = False
                    # print("[ExternalEStop] CLEAR")
                elif msg == "HB" or msg == "HELLO":
                    # heartbeat / hello
                    pass
                else:
                    # Unknown line; ignore
                    pass