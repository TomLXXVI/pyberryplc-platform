import logging
import logging.handlers
import datetime


def init_logger(logging_level: str = "info"):
    """Configures the root logger by adding a `TimedRotatingFileHandler`
    connected to the log file `unipi.log` and a `StreamHandler` that displays
    the log messages directly on screen. A new log file is created daily.
    """
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)

    match logging_level:
        case "debug":
            level = logging.DEBUG
        case "info":
            level = logging.INFO
        case "warning":
            level = logging.WARNING
        case "error":
            level = logging.ERROR
        case "critical":
            level = logging.CRITICAL
        case _:
            level = logging.DEBUG

    # noinspection PyTypeChecker
    file_handler = logging.handlers.TimedRotatingFileHandler(
        filename='rpi-plc.log',
        when='midnight',
        atTime=datetime.time(16, 0, 0)
    )
    file_handler.setLevel(level)
    file_formatter = logging.Formatter(
        fmt="[%(name)s %(asctime)s | %(levelname)s] %(message)s",
        datefmt="%d-%m-%Y %H:%M:%S"
    )
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(level)
    stream_formatter = logging.Formatter(
        fmt="[%(name)s %(asctime)s | %(levelname)s] %(message)s",
        datefmt="%d-%m-%Y %H:%M:%S"
    )
    stream_handler.setFormatter(stream_formatter)
    logger.addHandler(stream_handler)
