from services.monitoring_service import MonitoringService
from utils.configuration import load_config

"""
This module starts the monitoring service found in ../services/monitoring_service.py in a new process.
"""

def start_monitoring_service(ok_queue=None):
    monitoring_service = MonitoringService()
    config = load_config("startup/startup.conf")
    monitoring_service.setup(monitoring_config=config["digital_twin"]["monitoring_service"])

    if ok_queue is not None:
        ok_queue.put("OK")

    monitoring_service.start_monitoring()
    
