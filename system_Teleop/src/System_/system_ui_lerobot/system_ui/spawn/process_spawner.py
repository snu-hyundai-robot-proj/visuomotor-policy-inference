import os
import shlex
import signal

from PyQt5.QtCore import QObject, pyqtSignal, QProcess, QProcessEnvironment, QTimer


class RosProcessSpawner(QObject):
    sig_log = pyqtSignal(str)                  # 로그 문자열 전달
    sig_running = pyqtSignal(bool)             # 실행 상태 전달
    sig_exited = pyqtSignal(int, int)          # exitCode, exitStatus 전달

    def __init__(self, ros_setup="/opt/ros/humble/setup.bash", ws_setup=None, parent=None):
        super().__init__(parent)

        self._p = QProcess(self)                               # 실제 child process 실행 객체
        self._ros_setup = ros_setup                            # ROS setup.bash 경로
        self._ws_setup = ws_setup                              # workspace setup.bash 경로

        self._p.setProcessChannelMode(QProcess.SeparateChannels)  # stdout/stderr 분리 수신

        self._pending_log_chunks = []                          # UI 배치 전송용 로그 버퍼
        self._stop_in_progress = False                         # stop 진행 중 여부
        self._stop_pgid = None                                 # stop 대상 process group id

        self._p.readyReadStandardOutput.connect(self._on_stdout)  # stdout 수신
        self._p.readyReadStandardError.connect(self._on_stderr)   # stderr 수신
        self._p.started.connect(self._on_started)                 # started 이벤트
        self._p.errorOccurred.connect(self._on_error)             # 에러 이벤트
        self._p.finished.connect(self._on_finished)               # 종료 이벤트

        self._flush_timer = QTimer(self)                       # 로그 UI 전송 주기 타이머
        self._flush_timer.setInterval(10)                      # 10ms 주기
        self._flush_timer.timeout.connect(self._flush_pending_logs)
        self._flush_timer.start()

        self._sigint_timer = QTimer(self)                      # SIGINT 후 대기 타이머
        self._sigint_timer.setSingleShot(True)
        self._sigint_timer.timeout.connect(self._after_sigint_timeout)

        self._term_timer = QTimer(self)                        # terminate 후 대기 타이머
        self._term_timer.setSingleShot(True)
        self._term_timer.timeout.connect(self._after_terminate_timeout)

    def is_running(self) -> bool:
        return self._p.state() != QProcess.NotRunning          # 프로세스 상태 확인

    def start_cmd(self, cmd: str):
        if self.is_running():                                  # 이미 실행 중이면 중복 시작 방지
            self.sig_log.emit("[WARN] Already running.\n")
            return

        if not os.path.isfile(self._ros_setup):                # ROS setup 파일 존재 확인
            self.sig_log.emit(f"[ERROR] ROS setup not found: {self._ros_setup}\n")
            return

        if self._ws_setup and not os.path.isfile(self._ws_setup):  # WS setup 존재 확인
            self.sig_log.emit(f"[ERROR] WS setup not found: {self._ws_setup}\n")
            return

        self._pending_log_chunks.clear()                       # 이전 로그 초기화
        self._stop_in_progress = False                         # stop 상태 초기화
        self._stop_pgid = None                                 # pgid 초기화
        self._sigint_timer.stop()                              # stop 단계 타이머 정지
        self._term_timer.stop()

        shell_lines = []
        shell_lines.append(f"source {shlex.quote(self._ros_setup)}")  # ROS source
        if self._ws_setup:
            shell_lines.append(f"source {shlex.quote(self._ws_setup)}")  # WS source

        # stdbuf 로 가능한 범위에서 line buffering 유도
        shell_lines.append(f"exec stdbuf -oL -eL {cmd}")

        full_cmd = " && ".join(shell_lines)                    # 하나의 shell 명령으로 결합

        env = QProcessEnvironment.systemEnvironment()          # 현재 시스템 환경 복사
        env.insert("PYTHONUNBUFFERED", "1")                   # Python print 버퍼링 해제
        env.insert("RCUTILS_LOGGING_USE_STDOUT", "1")         # ROS 로그 stdout 사용
        env.insert("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")  # line buffering 유도
        env.insert("RCUTILS_LOGGING_BUFFERED_STREAM", "0")    # rcutils 버퍼링 비활성

        if env.value("QT_QPA_PLATFORM") == "":
            env.insert("QT_QPA_PLATFORM", "xcb")              # Qt 플랫폼 기본값 설정

        self._p.setProcessEnvironment(env)                     # 환경 반영

        self._append_pending(f"[START] setsid bash -lc {full_cmd!r}\n")
        self._flush_pending_logs()

        # setsid 로 새 process group 생성
        self._p.start("setsid", ["bash", "-lc", full_cmd])

    def stop(self):
        if not self.is_running():                              # 실행 중이 아니면 무시
            return

        if self._stop_in_progress:                             # 이미 정지 진행 중이면 중복 방지
            self._append_pending("[STOP] already in progress\n")
            self._flush_pending_logs()
            return

        pid = int(self._p.processId())                         # child pid 획득
        if pid <= 0:
            self._append_pending("[STOP] invalid pid\n")
            self._flush_pending_logs()
            return

        try:
            self._stop_pgid = os.getpgid(pid)                  # process group id 획득
        except Exception as e:
            self._append_pending(f"[ERROR] failed to get pgid: {e}\n")
            self._flush_pending_logs()
            self._p.terminate()                                # fallback terminate
            return

        self._stop_in_progress = True                          # stop 상태 설정
        self._append_pending("[STOP] SIGINT to process group\n")
        self._flush_pending_logs()

        try:
            os.killpg(self._stop_pgid, signal.SIGINT)          # 전체 process group에 SIGINT
        except Exception as e:
            self._append_pending(f"[ERROR] SIGINT failed: {e}\n")
            self._flush_pending_logs()
            self._p.terminate()                                # 실패 시 terminate
            self._sigint_timer.start(1500)
            return

        self._sigint_timer.start(1500)                         # 1.5초 후 종료 여부 재확인

    def kill(self):
        if not self.is_running():                              # 실행 중이 아니면 무시
            return

        pid = int(self._p.processId())                         # child pid 획득
        if pid <= 0:
            return

        self._append_pending("[KILL] SIGKILL to process group\n")
        self._flush_pending_logs()

        try:
            pgid = os.getpgid(pid)                             # process group 조회
            os.killpg(pgid, signal.SIGKILL)                    # 전체 group 강제 종료
        except Exception as e:
            self._append_pending(f"[ERROR] kill failed: {e}\n")
            self._flush_pending_logs()
            self._p.kill()                                     # fallback kill

    # --------------------------------------------------
    # Internal helpers
    # --------------------------------------------------
    def _append_pending(self, text: str):
        if text:
            self._pending_log_chunks.append(text)              # 로그 버퍼에 추가

    def _flush_pending_logs(self):
        if not self._pending_log_chunks:
            return

        merged = "".join(self._pending_log_chunks)             # 로그 문자열 병합
        self._pending_log_chunks.clear()                       # 버퍼 비우기
        self.sig_log.emit(merged)                              # 상위로 로그 emit

    def _drain_all_available_output(self):
        out = bytes(self._p.readAllStandardOutput()).decode(errors="replace")  # stdout 모두 읽기
        err = bytes(self._p.readAllStandardError()).decode(errors="replace")   # stderr 모두 읽기

        if out:
            self._append_pending(out)                          # stdout 버퍼 추가
        if err:
            self._append_pending(err)                          # stderr 버퍼 추가

    # --------------------------------------------------
    # QProcess output handlers
    # --------------------------------------------------
    def _on_stdout(self):
        data = bytes(self._p.readAllStandardOutput()).decode(errors="replace")  # stdout 읽기
        if data:
            self._append_pending(data)                          # 버퍼에 추가
            self._flush_pending_logs()                          # 즉시 flush

    def _on_stderr(self):
        data = bytes(self._p.readAllStandardError()).decode(errors="replace")   # stderr 읽기
        if data:
            self._append_pending(data)                          # 버퍼에 추가
            self._flush_pending_logs()                          # 즉시 flush

    # --------------------------------------------------
    # QProcess lifecycle handlers
    # --------------------------------------------------
    def _on_started(self):
        self._append_pending("[INFO] Process started.\n")       # 시작 로그
        self._flush_pending_logs()                              # 바로 출력
        self.sig_running.emit(True)                             # 실행 상태 True

    def _on_error(self, err):
        self._append_pending(f"[ERROR] QProcess errorOccurred: {int(err)}\n")  # 에러 로그
        self._flush_pending_logs()                              # 바로 출력

    def _on_finished(self, exit_code: int, exit_status: int):
        self._drain_all_available_output()                      # 남아 있는 stdout/stderr 회수
        self._append_pending(f"\n[EXIT] code={exit_code}, status={exit_status}\n")
        self._flush_pending_logs()                              # 종료 로그 출력

        self._sigint_timer.stop()                               # stop 관련 타이머 정지
        self._term_timer.stop()
        self._stop_in_progress = False                          # 내부 상태 초기화
        self._stop_pgid = None

        self.sig_running.emit(False)                            # 실행 상태 False
        self.sig_exited.emit(exit_code, exit_status)            # 종료 시그널 전송

    # --------------------------------------------------
    # Async stop stages
    # --------------------------------------------------
    def _after_sigint_timeout(self):
        if not self.is_running():                               # 이미 꺼졌으면 종료
            return

        self._drain_all_available_output()                      # 남은 로그 회수
        self._append_pending("[STOP] SIGINT timeout -> terminate()\n")
        self._flush_pending_logs()

        try:
            self._p.terminate()                                 # graceful terminate 시도
        except Exception as e:
            self._append_pending(f"[ERROR] terminate failed: {e}\n")
            self._flush_pending_logs()

        self._term_timer.start(1500)                            # 이후 kill 단계 대기

    def _after_terminate_timeout(self):
        if not self.is_running():                               # 이미 종료됐으면 리턴
            return

        self._drain_all_available_output()                      # 마지막 로그 회수
        self._append_pending("[STOP] terminate timeout -> SIGKILL\n")
        self._flush_pending_logs()

        try:
            if self._stop_pgid is not None:
                os.killpg(self._stop_pgid, signal.SIGKILL)      # process group kill
            else:
                self._p.kill()                                  # fallback kill
        except Exception as e:
            self._append_pending(f"[ERROR] SIGKILL failed: {e}\n")
            self._flush_pending_logs()
            self._p.kill()                                      # 예외 시 직접 kill