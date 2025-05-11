import logging
from logging.handlers import TimedRotatingFileHandler
import os
import sys
from datetime import datetime


class MicrosecondFormatter(logging.Formatter):
    
    def formatTime(self, record, datefmt=None):
        ct = datetime.fromtimestamp(record.created)
        if datefmt:
            return ct.strftime(datefmt)
        return super().formatTime(record, datefmt)


def init_logger(
    name: str = "", 
    log_file: str = "logs/plc.log",
    level: int = logging.INFO,
    console: bool = True
) -> logging.Logger:
    """
    Initialize a logger that logs to both file and console.

    Parameters
    ----------
    name : str
        Name of the logger (empty string refers to the root logger).
    log_file : str
        Path to the log file. Parent directories will be created if needed.
    level : int
        Logging level (e.g., logging.INFO or logging.DEBUG).
    console : bool
        If True, log to sys.stdout. If False, only log to file.
    """
    os.makedirs(os.path.dirname(log_file), exist_ok=True)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False  # prevents duplicate log lines if already propagated

    if not logger.handlers:
        formatter = MicrosecondFormatter(
            fmt="%(asctime)s [%(name)s] [%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S.%f"
        )

        file_handler = TimedRotatingFileHandler(
            log_file, when="midnight", backupCount=7, encoding="utf-8"
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        if console:
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            logger.addHandler(console_handler)

    return logger
