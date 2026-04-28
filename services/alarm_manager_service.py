from enum import Enum
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
from communication.protocol import ROUTING_KEY_MONITORING, MonitoringMsgTypes, MonitoringMsgKeys, ROUTING_KEY_RECORDER
import time
from datetime import datetime, timezone

class AlarmSeverity(Enum):
    NONE = 0
    LOW = 1
    MEDIUM =2
    HIGH = 3

class AlarmManagerService:
    def __init__(self, config):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("alarm_manager_service")
        self.current_status = {}
        self.tcp_mismatch_high_threshold = config["tcp_mismatch_high_threshold"]
        self.tcp_mismatch_medium_threshold = config["tcp_mismatch_medium_threshold"]
        self.q_mismatch_high_threshold = config["q_mismatch_high_threshold"]
        self.q_mismatch_medium_threshold = config["q_mismatch_medium_threshold"]
        self.max_velocity_exceeded_high_threshold = config["max_velocity_exceeded_high_threshold"]
        self.max_velocity_exceeded_medium_threshold = config["max_velocity_exceeded_medium_threshold"]
        self.max_acceleration_exceeded_high_threshold = config["max_acceleration_exceeded_high_threshold"]
        self.max_acceleration_exceeded_medium_threshold = config["max_acceleration_exceeded_medium_threshold"]

    def setup(self):
        self._l.info("Alarm Manager Service setup")
        self.rabbitmq.connect_to_server()
        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_MONITORING,
                        on_message_callback=self.handle_monitoring_message)
        self.rabbitmq.send_message(routing_key=ROUTING_KEY_RECORDER, message=self.create_alarm_msg())

    def handle_monitoring_message(self, ch, method, properties, msg):
        self._l.debug(f"Received monitoring message: {msg}")
        if not isinstance(msg, list):
            self._l.warning(f"Received monitoring message with invalid format: {msg}")
            return
        # Check monitor type
        for rob in msg:
            # Evaluate the new status based on the robustness value and the thresholds defined for each monitor type
            self.evaluate_status(rob)
        self.rabbitmq.send_message(routing_key=ROUTING_KEY_RECORDER, message=self.create_alarm_msg())
    
    def evaluate_status(self, rob):
        monitor_type = rob.get(MonitoringMsgKeys.TYPE)
        robustness_value = get_robustness(rob)
        if robustness_value is None:
            self._l.info(f"Received monitoring message with non-conclusive bounds: {rob}")
        match monitor_type:
                case MonitoringMsgTypes.STUCK_JOINT_0 | MonitoringMsgTypes.STUCK_JOINT_1 | MonitoringMsgTypes.STUCK_JOINT_2 | MonitoringMsgTypes.STUCK_JOINT_3 | MonitoringMsgTypes.STUCK_JOINT_4 | MonitoringMsgTypes.STUCK_JOINT_5:
                    self.current_status[monitor_type] = self.evaluate_stuck_joint(robustness_value)

                case MonitoringMsgTypes.WEAR_JOINT_0 | MonitoringMsgTypes.WEAR_JOINT_1 | MonitoringMsgTypes.WEAR_JOINT_2 | MonitoringMsgTypes.WEAR_JOINT_3 | MonitoringMsgTypes.WEAR_JOINT_4 | MonitoringMsgTypes.WEAR_JOINT_5:
                    pass

                case MonitoringMsgTypes.TCP_MISSMATCH:
                    self.current_status[monitor_type] = self.evaluate_tcp_mismatch(robustness_value)

                case MonitoringMsgTypes.MAX_VELOCITY_EXCEEDED:
                    self.current_status[monitor_type] = self.evaluate_max_velocity_exceeded(robustness_value)

                case MonitoringMsgTypes.MAX_ACCELERATION_EXCEEDED:
                    self.current_status[monitor_type] = self.evaluate_max_acceleration_exceeded(robustness_value)

                case MonitoringMsgTypes.SIMULATION_OFFLINE:
                    self.current_status[monitor_type] = self.evaluate_simulation_offline(robustness_value)

                case MonitoringMsgTypes.MOCKUP_OFFLINE:
                    self.current_status[monitor_type] = self.evaluate_mockup_offline(robustness_value)
                case MonitoringMsgTypes.Q_MISSMATCH:
                    self.current_status[monitor_type] = self.evaluate_q_mismatch(robustness_value)
                case _:
                    self._l.warning(f"Received monitoring message with unknown type: {monitor_type}")

    def evaluate_q_mismatch(self, robustness_value):
        if robustness_value < self.q_mismatch_high_threshold:
            return AlarmSeverity.HIGH
        elif robustness_value < self.q_mismatch_medium_threshold:
            return AlarmSeverity.MEDIUM
        elif robustness_value < 0:
            return AlarmSeverity.LOW
        else:
            return AlarmSeverity.NONE

    def evaluate_stuck_joint(self, robustness_value):
        if robustness_value >= 0.0:
            return AlarmSeverity.HIGH
        else:
            return AlarmSeverity.NONE
    
    def evaluate_tcp_mismatch(self, robustness_value):
        if robustness_value < self.tcp_mismatch_high_threshold:
            return AlarmSeverity.HIGH
        elif robustness_value < self.tcp_mismatch_medium_threshold:
            return AlarmSeverity.MEDIUM
        elif robustness_value < 0:
            return AlarmSeverity.LOW
        else:
            return AlarmSeverity.NONE
    
    def evaluate_max_velocity_exceeded(self, robustness_value):
        if robustness_value < self.max_velocity_exceeded_high_threshold:
            return AlarmSeverity.HIGH
        elif robustness_value < self.max_velocity_exceeded_medium_threshold:
            return AlarmSeverity.MEDIUM
        elif robustness_value < 0:
            return AlarmSeverity.LOW
        else:
            return AlarmSeverity.NONE
    
    def evaluate_max_acceleration_exceeded(self, robustness_value):
        if robustness_value < self.max_acceleration_exceeded_high_threshold:
            return AlarmSeverity.HIGH
        elif robustness_value < self.max_acceleration_exceeded_medium_threshold:
            return AlarmSeverity.MEDIUM
        elif robustness_value < 0:
            return AlarmSeverity.LOW
        else:
            return AlarmSeverity.NONE
    
    def evaluate_simulation_offline(self, robustness_value):
        if robustness_value < 0.0:
            return AlarmSeverity.HIGH
        else:
            return AlarmSeverity.NONE
    
    def evaluate_mockup_offline(self, robustness_value):
        if robustness_value < 0.0:
            return AlarmSeverity.HIGH
        else:
            return AlarmSeverity.NONE

    def create_alarm_msg(self):
        timestamp = datetime.fromtimestamp(time.time(), timezone.utc).isoformat()

        fields = {}
        #create fields for all alarm types with their severity as value
        for monitor_type in self.current_status:
            fields[monitor_type] = self.current_status[monitor_type].value

        rdata = {
            "measurement": "alarm_status",
            "time": timestamp,
            "tags": {
                "source": "alarm_manager_service"
            },
            "fields": fields,
        }
        self._l.debug(f"Created alarm message: {rdata}")
        return rdata

    def start_serving(self):
        if self.rabbitmq == None:
            raise RuntimeError("RabbitMQ client is not initialized. Call setup() first.")
        try:
            self.rabbitmq.start_consuming()
        except KeyboardInterrupt:
            self._l.info("Alarm Manager Service stopped by user.")

def get_robustness(msg):
    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
    if robustness_value is not None:
        return robustness_value
    
    robustness_lower = msg.get(MonitoringMsgKeys.ROBUSTNESS_LOWER_BOUND)
    robustness_upper = msg.get(MonitoringMsgKeys.ROBUSTNESS_UPPER_BOUND)

    if robustness_lower is None and robustness_upper is None:
        raise ValueError("Monitoring message does not contain robustness bounds.")
    
    if robustness_lower == robustness_upper:
        robustness_value = robustness_lower
    elif robustness_lower >= 0.0:
        robustness_value = robustness_lower
    elif robustness_upper <= 0.0:
        robustness_value = robustness_upper
    else:
        robustness_value = None

    return robustness_value