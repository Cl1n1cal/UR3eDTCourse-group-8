from services.calibration_service import CalibrationService
from utils.configuration import load_config

# Instantiate
calibration_service = CalibrationService()
config = load_config("startup/startup.conf")
calibration_service.setup(calibration_config=config["calibration_service"])

# Get estimated motion duration
data = calibration_service.get_mockup_values()
print("data len:", len(data))
print("Data[0]:", data[0])