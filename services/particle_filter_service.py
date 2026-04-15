import logging
import numpy as np
import time
from datetime import datetime, timezone
import math
import threading
from utils.calculation_functions import se3_to_pos_rpy
from models.robot_model import RobotModel
from communication.rabbitmq import Rabbitmq
from communication.factory import RabbitMQFactory, ROUTING_KEY_PARTICLE, ROUTING_KEY_MODEL_STATE, ROUTING_KEY_STATE, ROUTING_KEY_CTRL, ParticleFilterMsgKeys, RobotArmStateKeys, CtrlMsgFields, CtrlMsgKeys, ROUTING_KEY_CALIBRATION
from communication.protocol import unroll_list
from startup.utils.logging_config import create_service_logger
from nn_folder.classes.robot_prediction_nn import RobotPredictionNN


class ParticleFilterService:
    def __init__(self, publish_period: float = 0.05, start_time: float = time.time()):
        self.publish_period = publish_period
        self.consumer: Rabbitmq = RabbitMQFactory.create_rabbitmq() # TODO: Check if this runs with multiple threads internally
        self.publisher: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.nn_model: RobotPredictionNN = RobotPredictionNN() # Initialized in the setup function
        self.time = start_time
        self.particle_filter_count = 0
        
        # Could have used Python built in queues but they don't have a pretty reset function
        self.mockup_msg_queue = []
        self.sim_msg_queue = []
        
        self.use_local_mockup_time = False
        self.start_time = 0.0
        self.dead_mockup_time = 0.0
        self.mutex = threading.Lock()
        #self.event = threading.Event()
        self.first_mockup_item_popped = False

    def setup(self, particle_filter_config):
        self._l.info("ParticleFilterService service setup with config ", particle_filter_config)
        self._l = create_service_logger("particle_filter_service")

        # Create the nn
        model_path = particle_filter_config["nn_model_path"]
        self.nn_model.setup(model_path=model_path)
        
        # Connect rabbitmqs
        self.publisher.connect_to_server()
        self.consumer.connect_to_server()

        # Subscribe to the simulation service
        self.consumer.subscribe(routing_key=ROUTING_KEY_MODEL_STATE,
                                on_message_callback=self.read_simulation_state)
        
        # Subscribe to the mockup
        self.consumer.subscribe(routing_key=ROUTING_KEY_STATE,
                                on_message_callback=self.read_mockup_state)
        
        # Subscribe to the ctrl messages. Will be used to keep track of the message counting logic by
        # resetting the message counts when a new control message arrives
        self.consumer.subscribe(routing_key=ROUTING_KEY_CTRL,
                                on_message_callback=self.reset_msg_counter)
    
    def cleanup(self):
        # Close rabbitmqs
        self.consumer.close()
        self.publisher.close()

    def upload_state(self):
        self._l.debug("Uploading state to RabbitMQ.")

        rdata = self.create_recorder_state_msg()
        mdata = self.create_state_msg()

        self.publisher.send_message("robotarm.recorder.particle_filter", rdata)
        self.publisher.send_message(ROUTING_KEY_PARTICLE, mdata)

    # Check that the state message received from the mockup is a valid one
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
        # These might be useful later
        #fields[RobotArmStateKeys.JOINT_MAX_SPEED] = data[RobotArmStateKeys.JOINT_MAX_SPEED]
        #fields[RobotArmStateKeys.JOINT_MAX_ACCELERATION] = data[RobotArmStateKeys.JOINT_MAX_ACCELERATION]

        fields.update(unroll_list(RobotArmStateKeys.Q_ACTUAL, data[RobotArmStateKeys.Q_ACTUAL]))
        fields.update(unroll_list(RobotArmStateKeys.QD_ACTUAL, data[RobotArmStateKeys.QD_ACTUAL]))
        fields.update(unroll_list(RobotArmStateKeys.Q_TARGET, data[RobotArmStateKeys.Q_TARGET]))

        rdata = {
            "measurement": "mockup_state",
            "time": timestamp,
            "tags": {
                "source": "mockup_state_publisher"
            },
            "fields": fields,
        }

        return rdata
    
    def calculate_next_values(self, sim_measurement):
        nn_result = self.nn_model.predict(sim_measurement)
        return nn_result

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

        # current pos, current vel are necessary for the partcile filter
        # data also contains 'joint_max_speed' and 'joint_max_acceleration' in case they become necesarry
        q_current = data[RobotArmStateKeys.Q_ACTUAL]
        qd_current = data[RobotArmStateKeys.QD_ACTUAL]

        # Append the values to a sinlge list, ready to be given to the nn.predict()
        mockup_val = []
        [mockup_val.append(pos) for pos in q_current]
        [mockup_val.append(vel) for vel in qd_current]

        # Acquire the mutex and update shared mockup msg count and queue
        self.mutex.acquire()
        self.mockup_msg_queue.append(mockup_val)
        self.mutex.release()
        
        # Notify the particle_filter_thread
        #self.event.set()
    
    def read_simulation_state(self, ch, method, properties, message: dict):
        self._l.debug(f"Received mockup state message: {message}")
        self.sim_msg_count += 1

        data = self.validate_state_message(message)
        if not data:
            return
        
        #if self.is_first_message:
        #    self.set_start_time(message)
        #    self.is_first_message = False

        #if self.use_local_mockup_time:
        #    self.check_for_dead_mockup(message)

        # current pos, current vel and target pos are necessary for the nn
        # data also contains 'joint_max_speed' and 'joint_max_acceleration' in case they become necesarry
        q_current = data[RobotArmStateKeys.Q_ACTUAL]
        qd_current = data[RobotArmStateKeys.QD_ACTUAL]
        q_target = data[RobotArmStateKeys.Q_TARGET]

        # Append the values to a sinlge list, ready to be given to the nn.predict()
        sim_val = []
        [sim_val.append(pos) for pos in q_current]
        [sim_val.append(vel) for vel in qd_current]
        [sim_val.append(target) for target in q_target]


        # Acquire the mutex and update shared simulation msg count and queue
        self.mutex.acquire()
        self.sim_msg_queue.append(sim_val)
        self.mutex.release()

        # Notify the particle_filter_thread
        #self.event.set()
    
    def particle_filter_calculation(self, nn_prediction, mockup_value):
        pass

      
    # 1. Calculate next values
    # 2. Use the mockup value to calculate with the particle filter
    # 3. Publish the result
    def particle_filter_thread(self):
        # Wait for the event to be notified
        #self.event.wait()

        # Clear the event
        #self.event.clear()

        sim_val = None
        mockup_val = None

        # Lock the shared resources and check values
        self.mutex.acquire()

        # Adjust the mockup queue by removing the first element that is received
        if self.first_mockup_item_popped  == False:
            self.mockup_msg_queue.pop(0)
            self.first_mockup_item_popped = True
        
        # Get the first item from each of the queues if they are not empty
        if len(self.mockup_msg_queue) != 0 and len(self.sim_msg_queue) != 0:
            sim_val = self.sim_msg_queue.pop(0)
            mockup_val = self.mockup_msg_queue.pop(0)
        
        # Release the mutex before doing any further computation
        self.mutex.release()

        # Get the relevant data from the sim_val


        # Use the nn to calculate the next position and velocity values
        if sim_val != None and mockup_val != None:
            self.nn_model.predict(sim_val)
            prediction = self.nn_model.get_prediction()
            particle_result = self.particle_filter_calculation(prediction, mockup_val)

        # Calculate the next 
        # TODO: Possibly use 2 threads, one for each subscriber. Then use threads that block when the mockup value has not yet arrived
        
        pass


    # Reset the message counter, ready for new state transitions
    def reset_msg_counter(self):
        self.mockup_msg_count = 0
        self.sim_msg_count = 0
        self.first_mockup_item_popped = False
    
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