import logging
import numpy as np
import time
from datetime import datetime, timezone
import math
import threading
from utils.calculation_functions import se3_to_pos_rpy
from models.robot_model import RobotModel
from communication.rabbitmq import Rabbitmq
from communication.factory import RabbitMQFactory, ROUTING_KEY_PARTICLE, ROUTING_KEY_MODEL_STATE, ROUTING_KEY_STATE, ParticleFilterMsgKeys, RobotArmStateKeys, CtrlMsgFields, CtrlMsgKeys, ROUTING_KEY_CALIBRATION
from communication.protocol import unroll_list
from startup.utils.logging_config import create_service_logger

class ParticleFilterService:
    def __init__(self, publish_period: float = 0.05, start_time: float = time.time()):
        self.publish_period = publish_period
        self.consumer: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.publisher: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.time = start_time
        
        self._l = create_service_logger("particle_filter_service")
    
    def cleanup(self):
        self.consumer.close()
        self.publisher.close()

    def upload_state(self):
        self._l.debug("Uploading state to RabbitMQ.")

        rdata = self.create_recorder_state_msg()
        mdata = self.create_state_msg()

        self.publisher.send_message("robotarm.recorder.particle_filter", rdata)
        self.publisher.send_message(ROUTING_KEY_PARTICLE, mdata)
        
        
    def load_program(self, q_end: np.ndarray, max_velocity: float, acceleration: float) -> None:
        # Set the values in the robot model
        self.robot_model.load_program(q_end, max_velocity, acceleration)


    def read_calibration_message(self, ch, method, properties, message: dict):
        self._l.info(f"Received calibration message, updating model's DH parameters: {message}")
        self.robot_model.update_dh_parameters(d = message['d'], a =message['a'], alpha = message['alpha'])

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
        if self.use_local_mockup_time:
            msg_time = self.start_time + data[RobotArmStateKeys.TIMESTAMP] + self.dead_mockup_time
        else:
            msg_time = time.time()
        timestamp = datetime.fromtimestamp(msg_time, timezone.utc).isoformat()
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

    def read_mockup_state(self, ch, method, properties, message: dict):
        self._l.debug(f"Received mockup state message: {message}")

        data = self.validate_state_message(message)
        if not data:
            return
        
        #if self.is_first_message:
        #    self.set_start_time(message)
        #    self.is_first_message = False

        #if self.use_local_mockup_time:
        #    self.check_for_dead_mockup(message)

        msg = self.format_recorder_state_message(data)

    def step_simulation(self):
        self.time += self.step_size
        self.robot_model.step()
    
    def setup(self, initial_q = [0.0,0.0,0.0,0.0,0.0,0.0], max_velocity = 0, max_acceleration = 0):
        self._l.info("Setting up simulation service.")
        self.robot_model.setup_initial_state(np.array(initial_q), max_velocity, max_acceleration)
        self.publisher.connect_to_server()
        self.consumer.connect_to_server()
        self.consumer.subscribe(routing_key=ROUTING_KEY_MODEL_STATE,
                                on_message_callback=self.read_simulation_state)
        self.consumer.subscribe(routing_key=ROUTING_KEY_STATE,
                                on_message_callback=self.read_mockup_state)
    
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
            ParticleFilterMsgKeys.ROBOT_MODE: self.robot_model.state,
            ParticleFilterMsgKeys.JOINT_MAX_SPEED: math.degrees(self.robot_model.max_velocity),
            ParticleFilterMsgKeys.JOINT_MAX_ACCELERATION: math.degrees(self.robot_model.max_acceleration),
        }

        fields.update(unroll_list(ParticleFilterMsgKeys.Q_ACTUAL, self.robot_model.get_q_current().tolist()))
        fields.update(unroll_list(ParticleFilterMsgKeys.QD_ACTUAL, self.robot_model.get_qd_current().tolist()))
        fields.update(unroll_list(ParticleFilterMsgKeys.Q_TARGET, self.robot_model.get_q_end().tolist()))

        rdata = {
            "measurement": "particle_filter",
            "time": timestamp,
            "tags": {
                "source": "particle_filter_service"
            },
            "fields": fields,
        }

        return rdata

    def create_state_msg(self):
        timestamp = datetime.fromtimestamp(self.time, timezone.utc).isoformat()
        mdata = {
            ParticleFilterMsgKeys.Q_ACTUAL: self.robot_model.get_q_current().tolist(),
            ParticleFilterMsgKeys.QD_ACTUAL: self.robot_model.get_qd_current().tolist(),
            ParticleFilterMsgKeys.Q_TARGET: self.robot_model.q_end.tolist(),
            ParticleFilterMsgKeys.TIMESTAMP: timestamp,
        }

        return mdata