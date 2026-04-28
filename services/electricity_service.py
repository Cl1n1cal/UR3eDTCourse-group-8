import numpy as np
import math
import time
from datetime import datetime, timezone
import threading
from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.factory import RabbitMQFactory
from communication.protocol import RobotArmStateKeys, RobotMode, ROUTING_KEY_STATE, ROUTING_KEY_ELECTRICITY
from startup.utils.logging_config import create_service_logger

UR3E_IDLE_POWER_W = 50.0 # W
UR3E_DYNAMIC_POWER_W = 250.0 # W
UR3E_MAX_JOINT_VELOCITY_RAD_S = [
    math.pi,
    math.pi,
    math.pi,
    2*math.pi,
    2*math.pi,
    2*math.pi,
]

class ElectricityService:
    def __init__(self):
        self.influx_db_org = None
        self.influx_db_bucket = None
        self.write_api = None
        self.rabbitmq_publisher = RabbitMQFactory.create_rabbitmq()
        self.rabbitmq_consumer = RabbitMQFactory.create_rabbitmq()
        
        self.price_per_kwh_dkk = 2.50
        self.eur_per_dkk = 0.134
        self.publish_interval = 5.0
        
        self._energy_wh = 0.0
        self._session_start = None
        self._last_tick_time = None
        self._current_power_w = UR3E_IDLE_POWER_W
        self._lock = threading.Lock()
        
        self._l = create_service_logger("electricity_service")
        
        
    def setup(self, electricity_config):
        self._l.info("Electricity service setup with config ", electricity_config)
        self.client = InfluxDBClient(**electricity_config)
        self.rabbitmq_publisher.connect_to_server()
        self.rabbitmq_consumer.connect_to_server()
        self.rabbitmq_consumer.subscribe(routing_key=ROUTING_KEY_STATE, on_message_callback=self._on_state_message)
        
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        self.influx_db_org = electricity_config["org"]
        self.influx_db_bucket = electricity_config["bucket"]
        
        self.price_per_kwh_dkk = electricity_config.get("price_per_kwh_dkk", 2.50)
        self.eur_per_dkk = electricity_config.get("eur_per_dkk", 0.134)
        self.publish_interval = electricity_config.get("publish_interval", 5.0)
        
        
    def _on_state_message(self, ch, method, properties, message):
        now = time.time()
        
        qd_list = message.get(RobotArmStateKeys.QD_ACTUAL)
        mode = message.get(RobotArmStateKeys.ROBOT_MODE)
        
        if qd_list is None:
            return
        
        power_w = self._estimate_power(qd_list, mode)
        
        with self._lock:
            if self._session_start is None:
                self._session_start = datetime.now(timezone.utc)
            if self._last_tick_time is not None:
                dt_s = now - self._last_tick_time
                dt_s = min(dt_s, 1.0)
                self._energy_wh += power_w * dt_s / 3600.0
                
            self._current_power_w = power_w
            self._last_tick_time = now
    
    
    def _estimate_power(self, qd_list, mode):
        if mode == RobotMode.ROBOT_MODE_IDLE or not qd_list:
            return UR3E_IDLE_POWER_W
        
        velocity_load = sum((abs(qd)/UR3E_MAX_JOINT_VELOCITY_RAD_S[i])**2 for i, qd in enumerate(qd_list))
        velocity_load /= max(len(qd_list), 1)
        velocity_load = min(velocity_load, 1.0)

        return UR3E_IDLE_POWER_W + UR3E_DYNAMIC_POWER_W * velocity_load
    
    
    def _publish(self):
        while True:
            time.sleep(self.publish_interval)
            try:
                with self._lock:
                    power_w = self._current_power_w
                    energy_wh = self._energy_wh
                    session_start = self._session_start
                
                if session_start is None:
                    continue
                
                energy_kwh = energy_wh / 1000.0
                cost_dkk = energy_kwh * self.price_per_kwh_dkk
                cost_eur = cost_dkk * self.eur_per_dkk
                session_s = (datetime.now(timezone.utc) - session_start).total_seconds()
                
                self._l.info(f"Power: {power_w} | Energy: {energy_kwh} | Cost: {cost_dkk} DKK ({cost_eur} EUR) | Session: {session_s} s")

                ts = datetime.now(timezone.utc).isoformat()
                
                self.rabbitmq_publisher.send_message(
                    ROUTING_KEY_ELECTRICITY,
                    {
                        "timestamp": ts,
                        "power_w": round(power_w, 2),
                        "energy_kwh": round(energy_kwh, 6),
                        "cost_dkk": round(cost_dkk, 4),
                        "cost_eur": round(cost_eur, 4),
                        "session_seconds": round(session_s, 1),
                        "price_per_kwh_dkk": self.price_per_kwh_dkk,
                    },
                )
                
                self.write_api.write(
                    self.influx_db_bucket,
                    self.influx_db_org,
                    {
                        "measurement": "electricity",
                        "tags": {"source": "electricity_service"},
                        "time": ts,
                        "fields": {
                            "power_w": float(power_w),
                            "energy_kwh": float(energy_kwh),
                            "cost_dkk": float(cost_dkk),
                            "cost_eur": float(cost_eur),
                            "session_seconds": float(session_s),
                            "price_per_kwh_dkk": float(self.price_per_kwh_dkk),
                        },
                    },
                )
            
            except Exception as e:
                self._l.warning(f"Publish failed: {e}", exc_info=e)


    def start_serving(self):
        threading.Thread(target=self._publish, daemon=True).start()
        self._l.info(f"ElectricityService started (price={self.price_per_kwh_dkk} DKK/kWh, publish every {self.publish_interval}s).")
        try:
            self.rabbitmq_consumer.start_consuming()
        except KeyboardInterrupt:
            self.cleanup()
    
    
    def cleanup(self):
        self.rabbitmq_consumer.close()
        self.rabbitmq_publisher.close()
