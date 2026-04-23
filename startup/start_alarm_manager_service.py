from services.alarm_manager_service import AlarmManagerService
from utils.configuration import load_config

"""
This module starts the alarm manager service found in ../services/alarm_manager_service.py in a new process.
"""

def start_alarm_manager_service(ok_queue=None):
    config = load_config("startup/startup.conf")
    alarm_manager_service = AlarmManagerService(config["digital_twin"]["alarm_manager_service"])
    alarm_manager_service.setup()

    if ok_queue is not None:
        ok_queue.put("OK")

    alarm_manager_service.start_serving()
    