from services.inverse_kinematics_service import InverseKinematicsService
from utils.configuration import load_config

"""
This module starts the inverse kinematics service found in ../services/inverse_kinematics_service.py in a new process.
"""

def start_inverse_kinematics_service(ok_queue=None):
    config = load_config("startup/startup.conf")
    inverse_kinematics_service = InverseKinematicsService(inverse_kinematics_config=config["digital_twin"]["inverse_kinematics_service"])
    inverse_kinematics_service.setup()
  
    if ok_queue is not None:
        ok_queue.put("OK")

    inverse_kinematics_service.start_serving()
    
