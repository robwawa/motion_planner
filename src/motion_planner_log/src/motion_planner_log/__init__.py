"""Python logging adapter matching the motion_planner C++ log format."""

import logging
import os
import time
from datetime import datetime
from pathlib import Path

import fcntl


def _env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in ("1", "true", "yes", "on")


def _log_root():
    configured = os.environ.get("MOTION_PLANNER_LOG_DIR")
    if configured:
        return Path(configured)
    project_root = os.environ.get("MOTION_PLANNER_PROJECT_ROOT")
    if project_root:
        return Path(project_root) / "log"
    # In source space parents[4] is the workspace root. In an installed
    # Catkin package parents[4] is install/, so step out once more to keep
    # runtime logs outside the install space.
    package_path = Path(__file__).resolve()
    install_root = package_path.parents[4]
    if install_root.name == "install" and len(package_path.parents) > 5:
        return package_path.parents[5] / "log"
    return install_root / "log"


class _MotionPlannerLogger(logging.Logger):
    """Small compatibility layer for rospy's throttled logging calls."""

    def _throttle(self, level, period, message, args):
        now = time.monotonic()
        last = getattr(self, "_motion_planner_last_{}".format(level), None)
        if last is None or now - last >= period:
            setattr(self, "_motion_planner_last_{}".format(level), now)
            self.log(level, message, *args)

    def info_throttle(self, period, message, *args):
        self._throttle(logging.INFO, period, message, args)

    def warning_throttle(self, period, message, *args):
        self._throttle(logging.WARNING, period, message, args)

    def error_throttle(self, period, message, *args):
        self._throttle(logging.ERROR, period, message, args)


class _DailyFileHandler(logging.Handler):
    def __init__(self, directory):
        super().__init__()
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        try:
            self._max_bytes = int(os.environ.get("MOTION_PLANNER_LOG_MAX_MB", "0")) * 1024 * 1024
        except ValueError:
            self._max_bytes = 0

    def emit(self, record):
        try:
            line = (self.format(record) + "\n").encode("utf-8")
            date = datetime.now().strftime("%Y%m%d")
            stream_path = self.directory / (date + ".log")
            lock_path = self.directory / ".lock"
            with open(lock_path, "a+") as lock:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
                if self._max_bytes and stream_path.exists() and stream_path.stat().st_size >= self._max_bytes:
                    rotated = stream_path.with_name(stream_path.name + ".1")
                    if rotated.exists():
                        rotated.unlink()
                    stream_path.replace(rotated)
                fd = os.open(str(stream_path), os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
                try:
                    offset = 0
                    while offset < len(line):
                        offset += os.write(fd, line[offset:])
                finally:
                    os.close(fd)
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        except Exception:
            self.handleError(record)

    def close(self):
        super().close()


class _ConsoleFormatter(logging.Formatter):
    def format(self, record):
        timestamp = datetime.fromtimestamp(record.created).strftime("%Y-%m-%d %H:%M:%S")
        micros = int((record.created - int(record.created)) * 1000000)
        level = "WARN" if record.levelno == logging.WARNING else record.levelname
        return (
            "[{timestamp}.{micros:06d}]  [{level}] [{module}] "
            "{file}({function}):{line} {message}"
        ).format(
            timestamp=timestamp,
            micros=micros,
            level=level,
            module=getattr(record, "motion_module", record.name),
            file=record.filename,
            function=record.funcName,
            line=record.lineno,
            message=record.getMessage(),
        )


def configure(package_name, module_name=None):
    """Configure and return a module logger. Safe to call more than once."""
    if module_name is None:
        module_name = package_name
    logging.setLoggerClass(_MotionPlannerLogger)
    logger = logging.getLogger(module_name)
    if getattr(logger, "_motion_planner_configured", False):
        return logger

    level_name = os.environ.get("MOTION_PLANNER_LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    logger.setLevel(level)
    logger.propagate = False
    formatter = _ConsoleFormatter()

    try:
        file_handler = _DailyFileHandler(_log_root() / package_name)
    except OSError:
        file_handler = _DailyFileHandler(Path.home() / ".ros" / "log" / "motion_planner" / package_name)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    if _env_bool("MOTION_PLANNER_LOG_TO_CONSOLE", False):
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    logger._motion_planner_configured = True
    return logger
