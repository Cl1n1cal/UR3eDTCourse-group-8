from services.mockup_state_publisher import MockupStatePublisher

def start_mockup_state_publisher(ok_queue=None):

    publisher = MockupStatePublisher()
    publisher.setup()

    if ok_queue is not None:
        ok_queue.put("OK")

    publisher.start_serving()

