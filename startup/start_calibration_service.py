from services.calibration_service import CalibrationService
from utils.configuration import load_config

"""
This module starts the simulation service foud in ../services/simulation_service.py in a new process.
"""

def start_calibration_service(ok_queue=None):
    calibration_service = CalibrationService()
    config = load_config("startup/startup.conf")
    calibration_service.setup(calibration_config=config["calibration_service"])
  
    if ok_queue is not None:
        ok_queue.put("OK")

    calibration_service.start_serving()
    
