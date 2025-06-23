from pyberryplc.utils.tmc2208_utils import RMSCurrentLogger


current_logger = RMSCurrentLogger(
    None,
    filename="rms_current.csv"
)
current_logger.plot()
