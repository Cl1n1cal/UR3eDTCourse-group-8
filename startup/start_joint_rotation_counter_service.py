from services.joint_rotation_counter_service import JointRotationCounterService
from utils.configuration import load_config

def start_joint_rotation_counter_service(ok_queue=None):
    service = JointRotationCounterService()
    config = load_config("startup/startup.conf")
    service.setup(config=config["digital_twin"]["joint_rotation_counter_service"])

    if ok_queue is not None:
        ok_queue.put("OK")

    service.start_serving()
