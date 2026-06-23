from dataclasses import dataclass
from typing import Dict, Optional, List, Tuple

from PyQt5.QtCore import QObject, pyqtSignal

from system_ui.spawn.process_spawner import RosProcessSpawner


@dataclass
class ProcSpec:
    name: str
    cmd: str
    ros_setup: str = "/opt/ros/humble/setup.bash"
    ws_setup: Optional[str] = None


class ProcessManager(QObject):
    sig_log = pyqtSignal(str, str)              # (name, text)
    sig_state = pyqtSignal(str, bool)           # (name, running)
    sig_exited = pyqtSignal(str, int, int)      # (name, exitCode, exitStatus)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._specs: Dict[str, ProcSpec] = {}                  # name -> spec 저장
        self._spawners: Dict[str, RosProcessSpawner] = {}      # name -> spawner 저장

    # --------------------------------------------------
    # Register / query
    # --------------------------------------------------
    def register(self, spec: ProcSpec) -> None:
        self._specs[spec.name] = spec                          # spec 등록

        if spec.name not in self._spawners:                    # spawner 없으면 생성
            self._spawners[spec.name] = self._create_spawner(spec)

    def unregister(self, name: str) -> None:
        self._specs.pop(name, None)                            # spec 제거
        sp = self._spawners.pop(name, None)                    # spawner 제거
        if sp is not None:
            sp.deleteLater()                                   # Qt 객체 안전 삭제 예약

    def names(self) -> List[str]:
        return sorted(self._specs.keys())                      # 등록 이름 목록 반환

    def get_spec(self, name: str) -> Optional[ProcSpec]:
        return self._specs.get(name)                           # spec 조회

    def is_running(self, name: str) -> bool:
        sp = self._spawners.get(name)                          # 해당 spawner 조회
        return bool(sp and sp.is_running())                    # 실행 여부 반환

    def list_status(self) -> List[Tuple[str, bool]]:
        out: List[Tuple[str, bool]] = []                       # 상태 결과 리스트
        for n in self.names():                                 # 전체 name 순회
            out.append((n, self.is_running(n)))                # (이름, 실행중여부) 추가
        return out

    # --------------------------------------------------
    # Control
    # --------------------------------------------------
    def start(self, name: str) -> bool:
        spec = self._specs.get(name)                           # 실행 spec 가져오기
        if spec is None:
            self.sig_log.emit(name, "[ERROR] Not registered.\n")
            return False

        sp = self._spawners.get(name)                          # spawner 조회
        if sp is None:
            sp = self._create_spawner(spec)                    # 없으면 생성
            self._spawners[name] = sp

        if sp.is_running():
            self.sig_log.emit(name, "[WARN] Already running.\n")
            return False

        self.sig_log.emit(name, f"[MANAGER] start: {spec.cmd}\n")
        sp.start_cmd(spec.cmd)                                 # 실제 프로세스 시작
        return True

    def stop(self, name: str) -> bool:
        sp = self._spawners.get(name)                          # spawner 조회
        if sp is None:
            self.sig_log.emit(name, "[WARN] No spawner.\n")
            return False

        if not sp.is_running():
            self.sig_log.emit(name, "[WARN] Not running.\n")
            return False

        self.sig_log.emit(name, "[MANAGER] stop (SIGINT)\n")
        sp.stop()                                              # graceful stop 요청
        return True

    def kill(self, name: str) -> bool:
        sp = self._spawners.get(name)                          # spawner 조회
        if sp is None:
            self.sig_log.emit(name, "[WARN] No spawner.\n")
            return False

        if not sp.is_running():
            self.sig_log.emit(name, "[WARN] Not running.\n")
            return False

        self.sig_log.emit(name, "[MANAGER] kill (SIGKILL)\n")
        sp.kill()                                              # 강제 종료 요청
        return True

    def toggle(self, name: str) -> bool:
        if self.is_running(name):                              # 이미 실행중이면
            return self.stop(name)                             # stop
        return self.start(name)                                # 아니면 start

    def restart(self, name: str) -> None:
        spec = self._specs.get(name)                           # spec 조회
        sp = self._spawners.get(name)                          # spawner 조회

        if spec is None:
            self.sig_log.emit(name, "[ERROR] Not registered.\n")
            return

        if sp is None:
            self.start(name)                                   # spawner 없으면 그냥 start
            return

        if not sp.is_running():
            self.start(name)                                   # 안 돌고 있으면 그냥 start
            return

        self.sig_log.emit(name, "[MANAGER] restart requested\n")

        def _on_exit_once(exit_code: int, exit_status: int):
            try:
                sp.sig_exited.disconnect(_on_exit_once)        # 1회성 연결 해제
            except Exception:
                pass
            self.sig_log.emit(name, "[MANAGER] restarting now...\n")
            self.start(name)                                   # 종료 후 재시작

        sp.sig_exited.connect(_on_exit_once)                   # 종료 후 재시작 연결
        sp.stop()                                              # 종료 요청

    def stop_all(self) -> None:
        for n in self.names():                                 # 모든 등록 프로세스 순회
            if self.is_running(n):
                self.stop(n)                                   # 실행 중이면 stop

    def kill_all(self) -> None:
        for n in self.names():                                 # 모든 등록 프로세스 순회
            if self.is_running(n):
                self.kill(n)                                   # 실행 중이면 kill

    def shutdown_all(self) -> None:
        self.stop_all()                                        # 우선 전체 graceful stop

    # --------------------------------------------------
    # Internal helpers
    # --------------------------------------------------
    def _create_spawner(self, spec: ProcSpec) -> RosProcessSpawner:
        sp = RosProcessSpawner(
            ros_setup=spec.ros_setup,                          # ROS setup 경로 전달
            ws_setup=spec.ws_setup,                            # workspace setup 경로 전달
            parent=self                                        # Qt parent 설정
        )

        sp.sig_log.connect(self._make_log_forwarder(spec.name))       # 로그 forwarding
        sp.sig_running.connect(self._make_state_forwarder(spec.name)) # running state forwarding
        sp.sig_exited.connect(self._make_exit_forwarder(spec.name))   # exit forwarding
        return sp

    def _make_log_forwarder(self, name: str):
        def _fwd(text: str):
            self.sig_log.emit(name, text)                      # 이름 붙여 UI로 로그 전송
        return _fwd

    def _make_state_forwarder(self, name: str):
        def _fwd(running: bool):
            self.sig_state.emit(name, running)                 # running 상태 전달
        return _fwd

    def _make_exit_forwarder(self, name: str):
        def _fwd(exit_code: int, exit_status: int):
            self.sig_exited.emit(name, exit_code, exit_status) # 종료 코드 전달
        return _fwd