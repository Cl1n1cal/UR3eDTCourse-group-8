from communication.protocol import ROUTING_KEY_STATE, ROUTING_KEY_RECORDER, RobotArmStateKeys
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
from communication.protocol import unroll_list

class MockupStatePublisher:

    def __init__(self):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("mockup_state_publisher")
    
    def setup(self):
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
        
        return {key: message[key] for key in required_keys}
    
    def format_recorder_state_message(self, data: dict) -> dict:
        timestamp = data[RobotArmStateKeys.TIMESTAMP]

        fields = {}
        fields[RobotArmStateKeys.ROBOT_MODE] = data[RobotArmStateKeys.ROBOT_MODE],
        fields[RobotArmStateKeys.JOINT_MAX_SPEED] = data[RobotArmStateKeys.JOINT_MAX_SPEED],
        fields[RobotArmStateKeys.JOINT_MAX_ACCELERATION] = data[RobotArmStateKeys.JOINT_MAX_ACCELERATION],

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
        
        msg = self.format_recorder_state_message(data)
        
        try:
            self.rabbitmq.send_message(ROUTING_KEY_RECORDER, msg)
            self._l.debug(f"State message forwarded to recorder: {msg}")
        except Exception as e:
            self._l.error(f"Failed to forward state message to recorder: {e}")

    def start_serving(self):
        self.rabbitmq.start_consuming()
