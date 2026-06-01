from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
from communication.protocol import (
    ROUTING_KEY_MONITORING, ROUTING_KEY_CTRL, MonitoringMsgTypes,
    MonitoringMsgKeys, CtrlMsgFields, CtrlMsgKeys, FaultTypes
)
import time
from startup.utils.start_as_daemon import start_as_daemon
from startup.start_sim_service import start_sim_service

class FaultCorrectionService:
    def __init__(self, config):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("fault_correction_service")
        self.last_correction_time = {}
        self.cooldown_period = config.get("cooldown_period", 5.0) # seconds

    def setup(self):
        self._l.info("Fault Correction Service setup")
        self.rabbitmq.connect_to_server()
        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_MONITORING,
                        on_message_callback=self.handle_monitoring_message)

    def handle_monitoring_message(self, ch, method, properties, msg):
        self._l.debug(f"Received monitoring message: {msg}")
        if not isinstance(msg, list):
            self._l.warning(f"Received monitoring message with invalid format: {msg}")
            return
        # Check monitor type
        for rob in msg:
            self.evaluate_status(rob)
            
    def get_robustness(self, msg):
        robustness_value = msg.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
        if robustness_value is not None:
            return robustness_value
        
        robustness_lower = msg.get(MonitoringMsgKeys.ROBUSTNESS_LOWER_BOUND)
        robustness_upper = msg.get(MonitoringMsgKeys.ROBUSTNESS_UPPER_BOUND)

        if robustness_lower is None and robustness_upper is None:
            return None
        
        if robustness_lower == robustness_upper:
            return robustness_lower
        elif robustness_lower >= 0.0:
            return robustness_lower
        elif robustness_upper <= 0.0:
            return robustness_upper
        else:
            return None

    def evaluate_status(self, rob):
        monitor_type = rob.get(MonitoringMsgKeys.TYPE)
        robustness_value = self.get_robustness(rob)
        if robustness_value is None:
            return
            
        current_time = time.time()
        # Ensure we don't flood
        last_time = self.last_correction_time.get(monitor_type, 0)
        if (current_time - last_time) < self.cooldown_period:
            return

        match monitor_type:
                case MonitoringMsgTypes.STUCK_JOINT_0:
                    self.unstuck_joint(0, robustness_value, monitor_type)
                case MonitoringMsgTypes.STUCK_JOINT_1:
                    self.unstuck_joint(1, robustness_value, monitor_type)
                case MonitoringMsgTypes.STUCK_JOINT_2:
                    self.unstuck_joint(2, robustness_value, monitor_type)
                case MonitoringMsgTypes.STUCK_JOINT_3:
                    self.unstuck_joint(3, robustness_value, monitor_type)
                case MonitoringMsgTypes.STUCK_JOINT_4:
                    self.unstuck_joint(4, robustness_value, monitor_type)
                case MonitoringMsgTypes.STUCK_JOINT_5:
                    self.unstuck_joint(5, robustness_value, monitor_type)

                case MonitoringMsgTypes.SIMULATION_OFFLINE:
                    if robustness_value < 0.0:
                        self._l.info("Simulation is offline! Restarting simulation service.")
                        self.last_correction_time[monitor_type] = time.time()
                        start_as_daemon(start_sim_service)

                case MonitoringMsgTypes.WEAR_PREDICTION:
                    if robustness_value < 0.0:
                        self._l.info("Wear prediction exceeded threshold: issuing reset command.")
                        self.handle_wear_reset(monitor_type)

                case _:
                    pass

    def unstuck_joint(self, joint_idx, robustness_value, monitor_type):
        if robustness_value < 0.0:
            self._l.info(f"Joint {joint_idx} is stuck! Sending unstuck command.")
            msg = {
                CtrlMsgKeys.TYPE: CtrlMsgFields.UNSTUCK_JOINT,
                CtrlMsgKeys.JOINTS: [joint_idx]
            }
            self.rabbitmq.send_message(routing_key=ROUTING_KEY_CTRL, message=msg)
            self.last_correction_time[monitor_type] = time.time()

    def handle_wear_reset(self, monitor_type: str, wear_level: float = 0.0, duration: float = 5.0):
        """Send a wear command to reset wear to `wear_level` for `duration` seconds."""
        msg = {
            CtrlMsgKeys.TYPE: CtrlMsgFields.INJECT_FAULT,
            CtrlMsgKeys.FAULT_TYPE: FaultTypes.WEAR,
            CtrlMsgKeys.FAULT_VALUE: wear_level,
            CtrlMsgKeys.DURATION: duration,
            CtrlMsgKeys.JOINTS: list(range(6)),
        }
        self.rabbitmq.send_message(routing_key=ROUTING_KEY_CTRL, message=msg)
        self.last_correction_time[monitor_type] = time.time()

    def start_serving(self):
        if self.rabbitmq == None:
            raise RuntimeError("RabbitMQ client is not initialized. Call setup() first.")
        try:
            self.rabbitmq.start_consuming()
        except KeyboardInterrupt:
            self._l.info("Fault Correction Service stopped by user.")
