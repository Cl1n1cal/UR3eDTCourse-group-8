from services.wear_prediction_service import WearPredictionService
from utils.configuration import load_config

def start_wear_prediction_service(ok_queue=None):
    service = WearPredictionService()
    config = load_config("startup/startup.conf")
    service.setup(config=config["digital_twin"]["wear_prediction_service"])

    if ok_queue is not None:
        ok_queue.put("OK")

    service.start_serving()
