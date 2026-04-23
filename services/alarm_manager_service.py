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
        self.current_status = {
            MonitoringMsgTypes.STUCK_JOINT_0: AlarmSeverity.NONE,
            MonitoringMsgTypes.STUCK_JOINT_1: AlarmSeverity.NONE,
            MonitoringMsgTypes.STUCK_JOINT_2: AlarmSeverity.NONE,
            MonitoringMsgTypes.STUCK_JOINT_3: AlarmSeverity.NONE,
            MonitoringMsgTypes.STUCK_JOINT_4: AlarmSeverity.NONE,
            MonitoringMsgTypes.STUCK_JOINT_5: AlarmSeverity.NONE,

            MonitoringMsgTypes.WEAR_JOINT_0: AlarmSeverity.NONE,
            MonitoringMsgTypes.WEAR_JOINT_1: AlarmSeverity.NONE,
            MonitoringMsgTypes.WEAR_JOINT_2: AlarmSeverity.NONE,
            MonitoringMsgTypes.WEAR_JOINT_3: AlarmSeverity.NONE,
            MonitoringMsgTypes.WEAR_JOINT_4: AlarmSeverity.NONE,
            MonitoringMsgTypes.WEAR_JOINT_5: AlarmSeverity.NONE,

            MonitoringMsgTypes.TCP_MISSMATCH: AlarmSeverity.NONE,
            MonitoringMsgTypes.MAX_VELOCITY_EXCEEDED: AlarmSeverity.NONE,
            MonitoringMsgTypes.MAX_ACCELERATION_EXCEEDED: AlarmSeverity.NONE,
            MonitoringMsgTypes.SIMULATION_OFFLINE: AlarmSeverity.NONE,
            MonitoringMsgTypes.MOCKUP_OFFLINE: AlarmSeverity.NONE,
        }
        self.tcp_mismatch_high_threshold = config["tcp_mismatch_high_threshold"]
        self.tcp_mismatch_medium_threshold = config["tcp_mismatch_medium_threshold"]
        self.max_velocity_exceeded_high_threshold = config["max_velocity_exceeded_high_threshold"]
        self.max_velocity_exceeded_medium_threshold = config["max_velocity_exceeded_medium_threshold"]
        self.max_acceleration_exceeded_high_threshold = config["max_acceleration_exceeded_high_threshold"]
        self.max_acceleration_exceeded_medium_threshold = config["max_acceleration_exceeded_medium_threshold"]

    def setup(self):
        self._l.info("Alarm Manager Service setup")
        self.rabbitmq.connect_to_server()
        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_MONITORING,
                        on_message_callback=self.handle_monitoring_message)

    def handle_monitoring_message(self, ch, method, properties, msg):
        self._l.debug("Received monitoring message: ", msg)
        # Check monitor type
        monitor_type = msg.get(MonitoringMsgKeys.TYPE)
        if not monitor_type in self.current_status:
            self._l.warning("Received monitoring message with unknown type: ", monitor_type)
            return
        next_status = self.current_status
        # depending on the type, read robustness value and set severity of alarm
        next_status = self.evaluate_status(monitor_type, msg)
        # if any status has changed, send alarm message to rabbitmq with new status and severity
        if next_status != self.current_status:
            self._l.info("Alarm status changed: ", next_status)
            self.current_status = next_status
            self.rabbitmq.send_message(routing_key=ROUTING_KEY_RECORDER, message=self.create_alarm_msg())
    
    def evaluate_status(self, monitor_type, msg):
        next_status = self.current_status
        match monitor_type:
                case MonitoringMsgTypes.STUCK_JOINT_0 | MonitoringMsgTypes.STUCK_JOINT_1 | MonitoringMsgTypes.STUCK_JOINT_2 | MonitoringMsgTypes.STUCK_JOINT_3 | MonitoringMsgTypes.STUCK_JOINT_4 | MonitoringMsgTypes.STUCK_JOINT_5:
                    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
                    next_status[monitor_type] = self.evaluate_stuck_joint(robustness_value)

                case MonitoringMsgTypes.WEAR_JOINT_0 | MonitoringMsgTypes.WEAR_JOINT_1 | MonitoringMsgTypes.WEAR_JOINT_2 | MonitoringMsgTypes.WEAR_JOINT_3 | MonitoringMsgTypes.WEAR_JOINT_4 | MonitoringMsgTypes.WEAR_JOINT_5:
                    pass

                case MonitoringMsgTypes.TCP_MISSMATCH:
                    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
                    next_status[monitor_type] = self.evaluate_tcp_mismatch(robustness_value)

                case MonitoringMsgTypes.MAX_VELOCITY_EXCEEDED:
                    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
                    next_status[monitor_type] = self.evaluate_max_velocity_exceeded(robustness_value)

                case MonitoringMsgTypes.MAX_ACCELERATION_EXCEEDED:
                    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
                    next_status[monitor_type] = self.evaluate_max_acceleration_exceeded(robustness_value)

                case MonitoringMsgTypes.SIMULATION_OFFLINE:
                    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
                    next_status[monitor_type] = self.evaluate_simulation_offline(robustness_value)

                case MonitoringMsgTypes.MOCKUP_OFFLINE:
                    robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
                    next_status[monitor_type] = self.evaluate_mockup_offline(robustness_value)

        return next_status

    def evaluate_stuck_joint(self, robustness_value):
        if robustness_value < 0.0:
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
        for alarm_type, severity in self.current_status.items():
            fields[alarm_type] = severity.value

        rdata = {
            "measurement": "alarm_status",
            "time": timestamp,
            "tags": {
                "source": "alarm_manager_service"
            },
            "fields": fields,
        }

        return rdata

    def start_serving(self):
        if self.rabbitmq == None:
            raise RuntimeError("RabbitMQ client is not initialized. Call setup() first.")
        try:
            self.rabbitmq.start_consuming()
        except KeyboardInterrupt:
            self._l.info("Alarm Manager Service stopped by user.")