from services.fault_correction_service import FaultCorrectionService
from utils.configuration import load_config

"""
This module starts the fault correction service found in ../services/fault_correction_service.py in a new process.
"""

def start_fault_correction_service(ok_queue=None):
    config = load_config("startup/startup.conf")
    fault_correction_config = config["digital_twin"]["fault_correction_service"]
    fault_correction_service = FaultCorrectionService(fault_correction_config)
    fault_correction_service.setup()

    if ok_queue is not None:
        ok_queue.put("OK")

    fault_correction_service.start_serving()
    