# utils/dispatcher.py
from typing import Callable, Dict, Optional


class Dispatcher:
    def __init__(self):
        self.on_type: Dict[str, Callable[[dict], None]] = {}
        self.on_error: Optional[Callable[[dict], None]] = None

    def dispatch(self, msg: dict) -> None:
        if "error" in msg:
            if self.on_error:
                self.on_error(msg)
            else:
                print(f"[error] {msg}")
            return
        msg_type = msg.get('type')
        # print(msg)
        # print(f"type : {msg_type}")

        if msg_type and msg_type in self.on_type:
            if msg_type == 'handshake_ack' or msg_type == 'data':
                cb = self.on_type.get(msg_type)
                cb(msg)
            else:
                self.on_type[msg_type]#/view/doc-open-stream/ko/5-examples/msg)
        else:
            print(f"[event] {msg}")
