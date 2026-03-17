
from influxdb_client.client.influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from communication.rabbitmq import Rabbitmq
from communication.protocol import ROUTING_KEY_RECORDER
from communication.factory import RabbitMQFactory

class DBRecorderService:
    def __init__(self, rabbitmq_config, influxdb_config):

        client = InfluxDBClient(**influxdb_config)
        write_api = client.write_api(write_options=SYNCHRONOUS)
        self.write_api = write_api
        self.influx_db_org = influxdb_config["org"]
        self.influxdb_bucket = influxdb_config["bucket"]

        self.rabbitmq: Rabbitmq = RabbitMQFactory.create_rabbitmq()

    def read_record_request(self, ch, method, properties, body_json):
        self.write_api.write(self.influxdb_bucket, self.influx_db_org, body_json)

    def setup(self):
        self.rabbitmq.connect_to_server()

        self.rabbitmq.subscribe(routing_key=ROUTING_KEY_RECORDER,
                           on_message_callback=self.read_record_request)

    def start_recording(self):
        try:
            self.rabbitmq.start_consuming()
        except KeyboardInterrupt:
            self.rabbitmq.close()