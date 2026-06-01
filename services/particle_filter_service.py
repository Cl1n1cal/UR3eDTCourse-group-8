import logging
import numpy as np
import time
from datetime import datetime, timezone
import threading
from communication.rabbitmq import Rabbitmq
from communication.factory import RabbitMQFactory, ROUTING_KEY_PARTICLE, ROUTING_KEY_STATE, ParticleFilterMsgKeys, RobotArmStateKeys
from communication.protocol import unroll_list, ROUTING_KEY_RECORDER
from startup.utils.logging_config import create_service_logger
from nn_folder.classes.robot_prediction_nn import RobotPredictionNN

UPDATE_COUNT = 10

class ParticleFilterService:
    def __init__(self, publish_period: float = 0.05, start_time: float = time.time()):
        self.publish_period = publish_period
        self.consumer: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.publisher: Rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.nn_model: RobotPredictionNN = RobotPredictionNN() # Initialized in the setup function
        self.step_size = 0.1 # Can be overwritten by the setup function
        self.time_stamp = start_time
        self.q_target = []
        self.max_vel = 0.0
        self.max_acc = 0.0
        self.close_particle_filter_thread = False
        self._l = create_service_logger("particle_filter_service")
        self.event = threading.Event() # Used to signal the particle_filter_thread_function when the first mockup value arrives
        self.update_time_counter = UPDATE_COUNT # UPDATE_COUNT makes the service update the timer in the first iteration. See update_time method.

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

        # Particle filter elements
        self.mockup_noise = 0.03
        self.N = 100000 # Number of particles
        self.particles = []
        self.weights = []
        self.pf_estimates = []

    def setup(self, particle_filter_config):
        self._l.info("ParticleFilterService service setup with config ", particle_filter_config)

        # Create the nn
        model_path = particle_filter_config["nn_model_path"]
        self.nn_model.setup(model_path=model_path)

        # Set step size
        self.step_size = particle_filter_config["time_step"]
        
        # Connect rabbitmqs
        self.publisher.connect_to_server()
        self.consumer.connect_to_server()
        
        # Subscribe to the mockup
        self.consumer.subscribe(routing_key=ROUTING_KEY_STATE,
                                on_message_callback=self.read_mockup_state)
        
        self.consumer.subscribe(routing_key=ROUTING_KEY_RECORDER,
                                on_message_callback=self.update_time)
    
    # Close rabbitmqs amd join the particle filter thread
    def cleanup(self):
        self.consumer.close()
        self.publisher.close()
        self.particle_filter_thread.join()

    def upload_state(self, time_stamp):
        self._l.debug("Uploading state to RabbitMQ.")

        rdata = self.create_recorder_state_msg(time_stamp)
        mdata = self.create_state_msg(time_stamp)

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
            print("DATA DID NOT EXIST")
            return
        
        # current pos, current vel are necessary for the partcile filter
        # data also contains 'joint_max_speed' and 'joint_max_acceleration' in case they become necesarry
        q_current = data[RobotArmStateKeys.Q_ACTUAL]
        qd_current = data[RobotArmStateKeys.QD_ACTUAL]
        q_target = data[RobotArmStateKeys.Q_TARGET]
        max_vel = np.deg2rad(data[RobotArmStateKeys.JOINT_MAX_SPEED]) # convert to rads for the nn
        max_acc = np.deg2rad(data[RobotArmStateKeys.JOINT_MAX_ACCELERATION]) # convert to rads for the nn

        # Append the values to a sinlge list, ready to be given to the nn.predict()
        mockup_val = []
        [mockup_val.append(pos) for pos in q_current]
        [mockup_val.append(vel) for vel in qd_current]

        # Acquire the mutex and update shared mockup msg count and queue
        self.mutex.acquire()
        self.mockup_msg_queue.append(mockup_val)
        self.q_target = q_target
        self.max_vel = max_vel
        self.max_acc = max_acc
        self.mutex.release()
        self.event.set()
    


    def particle_filter_thread_function(self):
        self.event.wait()
        mean = np.array(self.mockup_msg_queue.pop())
        std = np.array([0.03]*6 + [0.03]*6)

        self.particles = mean + std * np.random.randn(self.N, 12)

        while True:
            # Check if the thread should be closed
            if self.close_particle_filter_thread:
                break

            # drain stale messages, keep only most recent
            self.mutex.acquire()
            while len(self.mockup_msg_queue) > 1:
                self.mockup_msg_queue.pop()
            if len(self.mockup_msg_queue) == 0:
                mockup_step = None
            else:
                mockup_step = self.mockup_msg_queue.pop()
                q_target = self.q_target.copy()
                max_vel = self.max_vel
                max_acc = self.max_acc
                self.time_stamp = self.time_stamp + self.step_size
                time_stamp = self.time_stamp
 
            self.mutex.release()

            if mockup_step != None:
                # Turn max vel, max acc and step size into a list
                constraints = [max_vel, max_acc, self.step_size]

                # Vectorize q_target and constraints list
                q_target_batch = np.tile(q_target, (self.N, 1))
                contraints_batch = np.tile(constraints, (self.N, 1))

                # --------------------------
                # 1. VECTORIZE STATE SPLIT
                # --------------------------
                joints = self.particles[:, 0:6]
                velocities = self.particles[:, 6:12]

                # --------------------------
                # 2. BUILD NN INPUT (BATCH)
                # --------------------------
                nn_input = np.concatenate([joints, velocities, q_target_batch, contraints_batch], axis=1)
                nn_input = nn_input.astype(np.float32)

                # --------------------------
                # 3. NN PREDICTION (BATCH)
                # --------------------------
                self.nn_model.predict(nn_input)
                pred = np.array(self.nn_model.get_prediction())

                # --------------------------
                # 4. ADD NOISE
                # --------------------------
                # 0.22
                noise = np.random.normal(0, 0.03, (self.N, 12))
                self.particles = pred + noise

                # --------------------------
                # 6. MEASUREMENT UPDATE
                # --------------------------
                measurement = np.array(mockup_step)

                error = self.particles - measurement

                weights = np.exp(-0.5 * np.sum((error / self.mockup_noise)**2, axis=1))
                weights += 1e-300
                weights /= np.sum(weights)

                # --------------------------
                # 7. ESTIMATE
                # --------------------------
                self.pf_estimates = np.sum(weights[:, np.newaxis] * self.particles, axis=0)

                # --------------------------
                # 8. RESAMPLING
                # --------------------------
                indices = np.random.choice(self.N, self.N, p=weights)
                self.particles = self.particles[indices]
                time_stamp = datetime.fromtimestamp(time_stamp, timezone.utc).isoformat()
                self.upload_state(time_stamp=time_stamp)

    
    def create_recorder_state_msg(self, time_stamp):
        pf_results = self.pf_estimates
        q_current = []
        qd_current = []

        # Get the position values
        for i in range(0, 6):
            q_current.append(pf_results[i])
        
        # Get the speed values
        for i in range(6, 12):
            qd_current.append(pf_results[i])

        fields = {}

        fields.update(unroll_list(ParticleFilterMsgKeys.Q_ACTUAL, q_current))
        fields.update(unroll_list(ParticleFilterMsgKeys.QD_ACTUAL, qd_current))

        rdata = {
            "measurement": "particle_filter",
            "time": time_stamp,
            "tags": {
                "source": "particle_filter_service"
            },
            "fields": fields,
        }

        return rdata

    def create_state_msg(self, time_stamp):
        #timestamp = datetime.fromtimestamp(self.time, timezone.utc).isoformat()
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
            ParticleFilterMsgKeys.TIMESTAMP: time_stamp,
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

    def update_time(self, ch, method, properties, message: dict):
        if self.update_time_counter == UPDATE_COUNT:
            self.update_time_counter = 0

            time_stamp = message["time"]
            time_stamp = datetime.fromisoformat(time_stamp).timestamp()
            measurement = message["measurement"]
            
            if time_stamp is None or measurement is None:
                return

            if measurement == "mockup_state":
                self.mutex.acquire()
                self.time_stamp = time_stamp
                self.mutex.release()
        else:
            self.update_time_counter += 1