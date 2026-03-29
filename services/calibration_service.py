from influxdb_client.client.influxdb_client import InfluxDBClient
from startup.utils.logging_config import create_service_logger
from communication.factory import RabbitMQFactory, ROUTING_KEY_CALIBRATION
import logging
import time
import numpy as np
from models.robot_model import create_robot
from scipy.optimize import least_squares
from utils.calculation_functions import se3_to_pos_rpy

class CalibrationService:
    def __init__(self):
        self.influx_db_org = None
        self.influxdb_bucket = None
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self.time_interval = 0
        self.robot = None
        self._l = create_service_logger("calibration_service", level=logging.DEBUG)

    def setup(self, calibration_config):
        self._l.info("Calibration service setup with config ", calibration_config)
        self.client = InfluxDBClient(**calibration_config)
        self.rabbitmq.connect_to_server()
        self.query_api = self.client.query_api()
        self.bucket = calibration_config["bucket"]
        self.org = calibration_config["org"]
        self.time_interval = calibration_config["time_interval"]
        self.dh_guess = np.array(calibration_config["initial_guess"]["d"] + calibration_config["initial_guess"]["a"] + calibration_config["initial_guess"]["alpha"])

    def get_mockup_values(self):
        start = f"-{self.time_interval}s"
        query = f'''
        from(bucket: "{self.bucket}")
        |> range(start: {start})
        |> filter(fn: (r) => r._measurement == "mockup_state")
        |> filter(fn: (r) => contains(value: r._field, set: [
            "q_actual_0",
            "q_actual_1",
            "q_actual_2",
            "q_actual_3",
            "q_actual_4",
            "q_actual_5",
            "tcp_pose_0",
            "tcp_pose_1",
            "tcp_pose_2",
            "tcp_pose_3",
            "tcp_pose_4",
            "tcp_pose_5",
        ]))
        |> pivot(rowKey: ["_time"], columnKey: ["_field"], valueColumn: "_value")
        |> sort(columns: ["_time"])
        '''
        
        tables = self.query_api.query(query, org=self.org)

        data = []
        fields = [f"q_actual_{i}" for i in range(6)]
        fields1 = [f"tcp_pose_{i}" for i in range(6)]

        for table in tables:
            for record in table.records:
                entry = {
                    "time_stamp": record.get_time().isoformat(),
                    **{field: record.values.get(field) for field in fields},
                    **{field: record.values.get(field) for field in fields1}
                }
                data.append(entry)

        return data

    def start_serving(self):
        while True:
            time.sleep(self.time_interval)
            self._l.info(f"Woke up! Time to calibrate DH Parameters. Initial guess: {self.dh_guess}")
            try:
                self.dh_guess = self.estimate_dh_parameters()
            except Exception as e:
                self._l.warning(f"Failed to estimate DH paramers: {e}")
                continue
            self._l.info(f"Estimation done! Sending new DH Parameters: {self.dh_guess}")

            self.rabbitmq.send_message(ROUTING_KEY_CALIBRATION, self.create_calibration_message())

    def create_calibration_message(self):

        msg = {'d':self.dh_guess[0:6].tolist(), 
               'a':self.dh_guess[6:12].tolist(), 
               'alpha':self.dh_guess[12:18].tolist()}
        
        return msg
    
    def estimate_dh_parameters(self):
        #get historical position and tcp pose values from db
        data = self.get_mockup_values()

        def cost(dh_guess):
            #create robot from parameters
            d1, d2, d3, d4, d5, d6, a1, a2, a3, a4, a5, a6, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = dh_guess
            d = [d1, d2, d3, d4, d5, d6]
            a = [a1, a2, a3, a4, a5, a6]
            alpha = [alpha1, alpha2, alpha3, alpha4, alpha5, alpha6]
            try:
                robot = create_robot(d, a, alpha)
            except:
                return

            cost = 0.0

            for sample in data:
                #compute tcp pose given input positions
                q_actual = np.array([sample['q_actual_0'], sample['q_actual_1'], sample['q_actual_2'], sample['q_actual_3'], sample['q_actual_4'], sample['q_actual_5']])
                model_tcp = se3_to_pos_rpy(robot.fkine(q_actual))
                #get cost by summing square of errors
                sample_tcp = np.array([sample['tcp_pose_0'], sample['tcp_pose_1'], sample['tcp_pose_2'], sample['tcp_pose_3'], sample['tcp_pose_4'], sample['tcp_pose_5']])
                #todo: change this to evaluate diference in a nomalized way, so positions over max dislocation and rotations over max rotations
                residuals = sample_tcp - model_tcp
                cost += residuals**2

            return cost
        
        try:
            res = least_squares(cost, self.dh_guess)
        except Exception as e:
            raise e

        return res.x

