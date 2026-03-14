from services.simulation_service import SimulationService
import numpy as np
from startup.start_docker_rabbitmq import start_rabbitmq
from communication.factory import RabbitMQFactory
from communication.protocol import ROUTING_KEY_STATE
import threading

# Description: Tests that we can get state messages from the controller and that they can be sent and received over rabbitmq.

pi = np.pi

# define a callback function to handle received messages
def on_message_received(ch, method, properties, body):
    try:
        print("✓ State:")
        print(body)
    except Exception as e:
        print(f"✗ Failed to decode the message: {e}")


def run_subscriber():
    print("Subscriber connect to server")
    subscriber.connect_to_server()
    print("subscriber subscribe")
    subscriber.subscribe(ROUTING_KEY_STATE, on_message_callback=on_message_received)
    print("Subscriber start consuming")
    subscriber.start_consuming()

# Maybe use later
# ur3e_service.set_fault(15, "stuck_joint") # Example fault injection after 15 steps

if __name__ == "__main__":
    start_rabbitmq() # Start the rabbitmq server (remember to turn on docker first)

    #crerates a rabbitmq instance for the simulation service to publish messages to
    simulation_service = SimulationService()

    print("Creating sub")
    subscriber = RabbitMQFactory.create_rabbitmq()

    threading.Thread(
        target=run_subscriber,
        daemon=True
    ).start()

    q_end = np.array([0.0, -pi/2, pi/2, -pi/2, -pi, 0.0]) # From exercise class
    max_velocity = 60 # deg/s
    acceleration = 80 # deg/s²

    simulation_service.load_program(q_end, max_velocity, acceleration)
    simulation_service.play()
 