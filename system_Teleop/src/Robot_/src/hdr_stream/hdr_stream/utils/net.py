# utils/net.py
import socket
import threading
from typing import Callable, Optional

class NetClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port))

        # Nagle OFF (low latency)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

        # TCP keepalive
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass

        self.sock.settimeout(1.0)
        self._running = True
        print(f"[net] connected to {self.host}:{self.port}")

    def close(self) -> None:
        self._running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        print("[net] connection closed")

    def send_line(self, line: str) -> None:
        if not self.sock:
            raise RuntimeError("socket not connected")
        self.sock.sendall((line + "\n").encode("utf-8"))
        # print(f"[tx] {line}")

    def start_recv_loop(self, on_bytes: Callable[[bytes], None]) -> None:
        if not self.sock:
            raise RuntimeError("socket not connected")

        def loop():
            while self._running:
                try:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        break
                    on_bytes(chunk)
                except socket.timeout:
                    continue
                except OSError:
                    break

        self._rx_thread = threading.Thread(target=loop, daemon=True)
        self._rx_thread.start()