from services.electricity_service import ElectricityService
from utils.configuration import load_config

def start_electricity_service(ok_queue=None):
    service = ElectricityService()
    config = load_config("startup/startup.conf")
    service.setup(electricity_config=config["digital_twin"]["electricity_service"])

    if ok_queue is not None:
        ok_queue.put("OK")

    service.start_serving()
