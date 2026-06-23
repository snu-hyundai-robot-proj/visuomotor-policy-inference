# utils/parser.py
import json
from typing import Callable

class NDJSONParser:
    def __init__(self):
        self._buffer = b""

    def feed(self, data: bytes, on_message: Callable[[dict], None]) -> None:
        self._buffer += data

        while b"\n" in self._buffer:
            line, self._buffer = self._buffer.split(b"\n", 1)
            if not line:
                continue

            try:
                msg = json.loads(line.decode("utf-8"))
                on_message(msg)
            except json.JSONDecodeError as e:
                print(f"[parser] json decode error: {e}")