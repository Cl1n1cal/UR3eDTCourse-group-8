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
        self.close_particle_filter_thread = False

        # Create the thread for the particle filtering function. Started in the start_serving method. Joined in the cleanup function
        self.particle_filter_thread = threading.Thread(target=self.particle_filter_thread_function, daemon=True)
        
        # Could have used Python built in queues but they don't have a pretty reset function
        self.mockup_msg_queue = []
        self.sim_msg_queue = []
        
        self.use_local_mockup_time = False
        self.start_time = 0.0
        self.dead_mockup_time = 0.0
        self.mutex = threading.Lock()
        self.first_mockup_item_popped = False

        # Particle filter parameters
        self.mockup_noise = 0.00003  # Standard deviation (m) taken from UR3e datasheet: https://www.universal-robots.com/media/1807464/ur3e_e-series_datasheets_web.pdf
        self.num_particles = 5000
        self.particles = []
        self.weights = []

        # Initialize the particle and weight lists (values are overwritten)
        for i in range(12):
            self.particles.append(np.random.normal(0, 2, self.num_particles))  # Initialize particles around 0
            self.weights.append(np.ones(self.num_particles) / self.num_particles)  # Uniform weights

        # Lists to store results
        self.sim_positions = []
        self.mockup_measurements = []
        self.pf_estimates = []  # Particle filter estimated positions

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
        self.particle_filter_thread.join()

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
    
    
    def read_simulation_state(self, ch, method, properties, message: dict):
        self._l.debug(f"Received mockup state message: {message}")
        self.sim_msg_count += 1

        data = self.validate_state_message(message)
        if not data:
            return

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
    
    def particle_filter_calculation(self, nn_prediction_list, mockup_vals):
        # For every joint and velocity
        for j in range(12):
            # --- Mockup Measurement (with Noise) ---
            mockup_joint = mockup_vals[j]

            # nn prediction
            nn_prediction = nn_prediction_list[j]

            # --- Particle Filter Update ---
            # 1. Motion Update: Particles drift slightly from simulation prediction
            self.particles[j] = nn_prediction + np.random.normal(0, 3, self.num_particles)  # Initialize particles around 0

            # 2. Measurement Update: Weight particles based on mockup (noisy measurement)
            self.weights[j] = np.exp(-0.5 * ((self.particles[j] - mockup_joint) / self.mockup_noise) ** 2)
            self.weights[j] += 1e-300  # Avoid zeros
            self.weights[j] /= np.sum(self.weights[j])  # Normalize weights

            # 3. Resampling: Draw new particles based on weights
            indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights[j])
            self.particles[j] = self.particles[j][indices]

            # Store estimated position as the mean of particles
            self.pf_estimates[j] = np.mean(self.particles[j])

      
    # 1. Calculate next values
    # 2. Use the mockup value to calculate with the particle filter
    # 3. Publish the result
    def particle_filter_thread_function(self):
        while True:

            # Check if the thread should be closed
            if self.close_particle_filter_thread:
                break

            sim_vals = None
            mockup_vals = None

            # Lock the shared resources and check values
            self.mutex.acquire()

            # Adjust the mockup queue by removing the first element that is received
            if self.first_mockup_item_popped  == False:
                self.mockup_msg_queue.pop(0)
                self.first_mockup_item_popped = True
            
            # Get the first item from each of the queues if they are not empty
            if len(self.mockup_msg_queue) != 0 and len(self.sim_msg_queue) != 0:
                sim_vals = self.sim_msg_queue.pop(0)
                mockup_vals = self.mockup_msg_queue.pop(0)
            
            # Release the mutex before doing any further computation
            self.mutex.release()

            # Get the relevant data from the sim_val


            # Use the nn to calculate the next position and velocity values
            if sim_vals != None and mockup_vals != None:
                self.nn_model.predict(sim_vals)
                prediction_list = self.nn_model.get_prediction().tolist()
                self.particle_filter_calculation(prediction_list, mockup_vals)
                self.upload_state()


    # Reset the message counter, ready for new state transitions
    def reset_msg_counter(self):
        self.mutex.acquire()
        self.sim_msg_queue = []
        self.mockup_msg_queue = []
        self.first_mockup_item_popped = False
        self.mutex.release()
    
    def create_recorder_state_msg(self):
        timestamp = datetime.fromtimestamp(self.time, timezone.utc).isoformat()
        pf_results = self.pf_estimates
        q_current = []
        qd_current = []

        fields = {}

        fields.update(unroll_list(ParticleFilterMsgKeys.Q_ACTUAL, q_current))
        fields.update(unroll_list(ParticleFilterMsgKeys.QD_ACTUAL, qd_current))

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
        pf_results = self.pf_estimates
        q_current = []
        qd_current = []

        # Get the position values
        for i in range(0, 6):
            q_current.append(pf_results[i])
        
        # Get the speed values
        for i in range(6, 12):
            qd_current.append(pf_results[i])

        # Insert values into state message
        mdata = {
            ParticleFilterMsgKeys.Q_ACTUAL: q_current,
            ParticleFilterMsgKeys.QD_ACTUAL: qd_current,
            ParticleFilterMsgKeys.TIMESTAMP: timestamp,
        }

        return mdata
    
    def start_serving(self):
        # Start the particle filter thread
        self.particle_filter_thread.start()

        try:
            self.consumer.start_consuming()
        except KeyboardInterrupt:
            self._l.info("Particle filter stopped by user.")
        finally:
            self.cleanup()