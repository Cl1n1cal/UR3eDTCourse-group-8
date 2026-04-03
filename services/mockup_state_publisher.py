from communication.protocol import ROUTING_KEY_STATE, ROUTING_KEY_RECORDER, RobotArmStateKeys
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
from communication.protocol import unroll_list
from datetime import datetime, timezone
import time
import logging
from startup.utils.config import load_config_w_setuptools; c=load_config_w_setuptools('startup.conf');

class MockupStatePublisher:

    def __init__(self, dead_mockup_time_threshold):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.start_time = 0.0
        self.dead_mockup_threshold = dead_mockup_time_threshold
        self.dead_mockup_time = 0.0
        self.last_msg_time = None
        self.last_msg_delay = 0.0
        self._l = create_service_logger("mockup_state_publisher")
        self.is_first_message = True
    
    def setup(self):
        self._l.info("Setting up mockup publisher")
        self.rabbitmq.connect_to_server()
        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_STATE,
                                on_message_callback=self.forward_state_message)
    
    def validate_state_message(self, message: dict) -> dict | None:
        required_keys = [
            RobotArmStateKeys.ROBOT_MODE,
            RobotArmStateKeys.Q_ACTUAL,
            RobotArmStateKeys.QD_ACTUAL,
            RobotArmStateKeys.Q_TARGET,
            RobotArmStateKeys.TIMESTAMP,
            RobotArmStateKeys.JOINT_MAX_SPEED,
            RobotArmStateKeys.JOINT_MAX_ACCELERATION,
            RobotArmStateKeys.TCP_POSE,
        ]
        
        missing = [key for key in required_keys if key not in message or message[key] is None]
        if missing:
            self._l.warning(f"Missing required fields: {missing}")
            return None
        
        return message
    
    def format_recorder_state_message(self, data: dict) -> dict:
        time = self.start_time + data[RobotArmStateKeys.TIMESTAMP] + self.dead_mockup_time
        timestamp = datetime.fromtimestamp(time, timezone.utc).isoformat()
        fields = {}
        fields[RobotArmStateKeys.ROBOT_MODE] = data[RobotArmStateKeys.ROBOT_MODE]
        fields[RobotArmStateKeys.JOINT_MAX_SPEED] = data[RobotArmStateKeys.JOINT_MAX_SPEED]
        fields[RobotArmStateKeys.JOINT_MAX_ACCELERATION] = data[RobotArmStateKeys.JOINT_MAX_ACCELERATION]

        fields.update(unroll_list(RobotArmStateKeys.Q_ACTUAL, data[RobotArmStateKeys.Q_ACTUAL]))
        fields.update(unroll_list(RobotArmStateKeys.QD_ACTUAL, data[RobotArmStateKeys.QD_ACTUAL]))
        fields.update(unroll_list(RobotArmStateKeys.Q_TARGET, data[RobotArmStateKeys.Q_TARGET]))
        fields.update(unroll_list(RobotArmStateKeys.TCP_POSE, data[RobotArmStateKeys.TCP_POSE]))

        rdata = {
            "measurement": "mockup_state",
            "time": timestamp,
            "tags": {
                "source": "mockup_state_publisher"
            },
            "fields": fields,
        }

        return rdata
    
    def forward_state_message(self, ch, method, properties, message: dict):
        self._l.debug(f"Received mockup state message: {message}")

        data = self.validate_state_message(message)
        if not data:
            return
        
        if self.is_first_message:
            self.set_start_time(message)
            self.is_first_message = False

        self.check_for_dead_mockup(message)

        msg = self.format_recorder_state_message(data)
        
        try:
            self.rabbitmq.send_message(ROUTING_KEY_RECORDER, msg)
            self._l.debug(f"State message forwarded to recorder: {msg}")
        except Exception as e:
            self._l.error(f"Failed to forward state message to recorder: {e}")

    def check_for_dead_mockup(self, message):
            curr_time = time.time()
            curr_delay = curr_time - (self.start_time + message[RobotArmStateKeys.TIMESTAMP])
            if curr_delay - self.last_msg_delay > self.dead_mockup_threshold and self.last_msg_time:
                self._l.warning("Delay difference past the threshold! Assuming mockup was dead and ajusting timestamps.")
                self.dead_mockup_time += curr_time - self.last_msg_time
            self.last_msg_delay = curr_delay
            self.last_msg_time = curr_time

    def set_start_time(self, first_msg: dict):
        self.start_time = time.time() - first_msg[RobotArmStateKeys.TIMESTAMP]

    def start_serving(self):
        self.rabbitmq.start_consuming()
