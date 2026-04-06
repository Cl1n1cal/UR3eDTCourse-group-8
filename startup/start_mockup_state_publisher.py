from services.mockup_state_publisher import MockupStatePublisher
from utils.configuration import load_config; config = load_config('startup/startup.conf')

def start_mockup_state_publisher(ok_queue=None):
    publisher = MockupStatePublisher(
        use_local_mockup_time=config['digital_twin']['mockup_state_publisher']['use_local_mockup_time'],
        dead_mockup_time_threshold=config['digital_twin']['mockup_state_publisher']['dead_threshold']
    )
    publisher.setup()

    if ok_queue is not None:
        ok_queue.put("OK")

    publisher.start_serving()

