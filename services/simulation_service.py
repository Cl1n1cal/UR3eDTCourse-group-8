import logging
import numpy as np
import time
from datetime import datetime, timezone
import math
import threading
from utils.calculation_functions import se3_to_pos_rpy
from models.robot_model import RobotModel
from communication.rabbitmq import Rabbitmq
from communication.factory import RabbitMQFactory, ROUTING_KEY_MODEL_STATE, ROUTING_KEY_CTRL, RobotArmStateKeys, CtrlMsgFields, CtrlMsgKeys, ROUTING_KEY_CALIBRATION
from communication.protocol import unroll_list
from startup.utils.logging_config import create_service_logger

class SimulationService:
    def __init__(self, step_size: float = 0.01, publish_period: float = 0.05, start_time: float = time.time(), dh_params: dict = {'d': [0.0], 'a': [0.0], 'alpha': [0.0]}):
        self.step_size = step_size
        self.publish_period = publish_period
        self.robot_model = RobotModel(step_size=self.step_size, d=dh_params['d'], a=dh_params['a'], alpha=dh_params['alpha'])
        self.consumer: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.publisher: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.time = start_time
        
        self._l = create_service_logger("simulation_service")
    
    def cleanup(self):
        self.consumer.close()
        self.publisher.close()

    def upload_state(self):
        self._l.debug("Uploading state to RabbitMQ.")

        rdata = self.create_recorder_state_msg()
        mdata = self.create_state_msg()

        self.publisher.send_message("robotarm.recorder.arm_state", rdata)
        self.publisher.send_message(ROUTING_KEY_MODEL_STATE, mdata)
        
        
    def load_program(self, q_end: np.ndarray, max_velocity: float, acceleration: float) -> None:
        # Set the values in the robot model
        self.robot_model.load_program(q_end, max_velocity, acceleration)

    def read_control_message(self, ch, method, properties, message: dict):
        self._l.info(f"Received control message: {message}")
        msg_type = message.get(CtrlMsgKeys.TYPE)
        
        match msg_type:
            case CtrlMsgFields.LOAD_PROGRAM:
                q_end = np.array(message.get(CtrlMsgKeys.JOINT_POSITIONS, [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])[0], dtype=float)
                max_velocity = math.radians(message.get(CtrlMsgKeys.MAX_VELOCITY, 0.0))
                acceleration = math.radians(message.get(CtrlMsgKeys.ACCELERATION, 0.0))
                self.load_program(q_end, max_velocity, acceleration)
            case CtrlMsgFields.PLAY:
                self.robot_model.play()
            case CtrlMsgFields.PAUSE:
                self.robot_model.pause()
            case CtrlMsgFields.STOP:
                self.robot_model.stop()
            case _:
                self._l.warning(f"Unknown control message type: {msg_type}")

    def read_calibration_message(self, ch, method, properties, message: dict):
        self._l.info(f"Received calibration message, updating model's DH parameters: {message}")
        self.robot_model.update_dh_parameters(d = message['d'], a =message['a'], alpha = message['alpha'])

    def step_simulation(self):
        self.time += self.step_size
        self.robot_model.step()
    
    def setup(self, initial_q = [0.0,0.0,0.0,0.0,0.0,0.0], max_velocity = 0, max_acceleration = 0):
        self._l.info("Setting up simulation service.")
        self.robot_model.setup_initial_state(np.array(initial_q), max_velocity, max_acceleration)
        self.publisher.connect_to_server()
        self.consumer.connect_to_server()
        self.consumer.subscribe(routing_key=ROUTING_KEY_CTRL,
                                on_message_callback=self.read_control_message)
        self.consumer.subscribe(routing_key=ROUTING_KEY_CALIBRATION,
                                on_message_callback=self.read_calibration_message)
    
    def start_serving(self):
        stop_event = threading.Event()

        def _sim_loop():
            last_publish_time = time.time()
            while not stop_event.is_set():
                curr_time = time.time()
                if curr_time - self.time >= self.step_size:
                    self.step_simulation()

                if curr_time - last_publish_time >= self.publish_period:
                    self.upload_state()
                    last_publish_time = curr_time

        sim_thread = threading.Thread(target=_sim_loop, daemon=True)
        sim_thread.start()

        try:
            self.consumer.start_consuming()
        except KeyboardInterrupt:
            self._l.info("Simulation stopped by user.")
        finally:
            stop_event.set()
            self.cleanup()
    
    def create_recorder_state_msg(self):
        timestamp = datetime.fromtimestamp(self.time, timezone.utc).isoformat()

        fields = {
            RobotArmStateKeys.ROBOT_MODE: self.robot_model.state,
            RobotArmStateKeys.JOINT_MAX_SPEED: math.degrees(self.robot_model.max_velocity),
            RobotArmStateKeys.JOINT_MAX_ACCELERATION: math.degrees(self.robot_model.max_acceleration),
        }

        fields.update(unroll_list(RobotArmStateKeys.Q_ACTUAL, self.robot_model.get_q_current().tolist()))
        fields.update(unroll_list(RobotArmStateKeys.QD_ACTUAL, self.robot_model.get_qd_current().tolist()))
        fields.update(unroll_list(RobotArmStateKeys.Q_TARGET, self.robot_model.get_q_end().tolist()))
        fields.update(unroll_list(RobotArmStateKeys.TCP_POSE, se3_to_pos_rpy(self.robot_model.get_tcp_pose_current()).tolist()))

        rdata = {
            "measurement": "simulation_state",
            "time": timestamp,
            "tags": {
                "source": "simulator_service"
            },
            "fields": fields,
        }

        return rdata

    def create_state_msg(self):
        timestamp = datetime.fromtimestamp(self.time, timezone.utc).isoformat()
        mdata = {
            RobotArmStateKeys.ROBOT_MODE: self.robot_model.state,
            RobotArmStateKeys.Q_ACTUAL: self.robot_model.get_q_current().tolist(),
            RobotArmStateKeys.QD_ACTUAL: self.robot_model.get_qd_current().tolist(),
            RobotArmStateKeys.Q_TARGET: self.robot_model.q_end.tolist(),
            RobotArmStateKeys.TIMESTAMP: timestamp,
            RobotArmStateKeys.JOINT_MAX_SPEED: self.robot_model.max_velocity,
            RobotArmStateKeys.JOINT_MAX_ACCELERATION: self.robot_model.max_acceleration,
            RobotArmStateKeys.TCP_POSE: se3_to_pos_rpy(self.robot_model.get_tcp_pose_current()).tolist()
        }

        return mdata