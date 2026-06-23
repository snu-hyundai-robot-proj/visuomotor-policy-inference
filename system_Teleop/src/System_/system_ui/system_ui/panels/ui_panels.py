from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QPushButton, QLineEdit
from PyQt5.QtCore import pyqtSignal


class SimpleGroupPanel(QGroupBox):
    """
    재사용 가능한 GroupBox + (Optional) LineEdit + Buttons 패널
    """

    sig_clicked = pyqtSignal(str, str, str)

    def __init__(
        self,
        title: str,
        button_names: list,
        use_button_size = False,
        button_size=(100,100),
        parent=None,
        resize=(300, 200),
        use_line_edit: bool = True,  # 🔹 ON/OFF 옵션
        line_text="0.0.0.0"
    ):
        super().__init__(title, parent)

        self.setFixedSize(resize[0], resize[1])

        self._use_line_edit = use_line_edit
        self._edit = None  # 기본 None

        # (1) 레이아웃
        main_layout = QVBoxLayout()

        # (2) 입력창 조건부 생성
        if self._use_line_edit:
            self._edit = QLineEdit()
            self._edit.setText(line_text)
            main_layout.addWidget(self._edit)

        # (3) 버튼 생성
        self._buttons = {}
        btn_layout = QHBoxLayout()

        for name in button_names:
            btn = QPushButton(name)
            if use_button_size:btn.setFixedSize(button_size[0],button_size[1])
            btn.clicked.connect(lambda _, n=name: self._on_button(n))
            self._buttons[name] = btn
            btn_layout.addWidget(btn)

        main_layout.addLayout(btn_layout)
        self.setLayout(main_layout)

    # --------------------------
    # 내부 버튼 클릭 처리
    # --------------------------
    def _on_button(self, name: str):
        text = ""

        if self._use_line_edit and self._edit is not None:
            text = self._edit.text()

        self.sig_clicked.emit(self.title(), name, text)

    # --------------------------
    # 외부 API
    # --------------------------
    def text(self) -> str:
        if self._use_line_edit and self._edit is not None:
            return self._edit.text()
        return ""

    def set_text(self, value: str):
        if self._use_line_edit and self._edit is not None:
            self._edit.setText(value)

    def set_button_enabled(self, name: str, enabled: bool):
        if name in self._buttons:
            self._buttons[name].setEnabled(enabled)