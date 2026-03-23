from services.calibration_service import CalibrationService
from utils.configuration import load_config

# Instantiate
calibration_service = CalibrationService()
config = load_config("startup/startup.conf")
calibration_service.setup(influxdb_config=config["influxdb"])

# Get estimated motion duration
times, positions = calibration_service.get_motion_data()
_, _, durr = calibration_service.get_motion_duration(times, positions)
print(f"Estimated motion duration: {durr:.3f} s")