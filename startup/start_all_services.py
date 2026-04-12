import signal
import sys
from startup.utils.start_as_daemon import start_as_daemon
from startup.start_docker_rabbitmq import start_docker_rabbitmq
from startup.start_sim_service import start_sim_service
from startup.start_ur3e_mockup import start_robot_arm_mockup
from startup.start_db_recorder_service import start_db_recorder_service
from startup.start_mockup_state_publisher import start_mockup_state_publisher
from startup.start_docker_influxdb import start_docker_influxdb
from startup.start_calibration_service import start_calibration_service
from startup.utils.logging_config import setup_root_logging
from startup.start_visualization_service import start_visualization_service

def signal_handler(sig, frame):
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)  # Handle ^C
    setup_root_logging("all_service_logs")
    start_docker_rabbitmq()
    start_docker_influxdb()
    start_as_daemon(start_db_recorder_service)
    start_as_daemon(start_mockup_state_publisher)
    start_as_daemon(start_robot_arm_mockup)
    start_as_daemon(start_sim_service)
    start_as_daemon(start_calibration_service)
    start_as_daemon(start_visualization_service)
    
    # Keep the main process alive to handle signals
    try:
        while True:
            signal.pause()  # Wait for signals
    except KeyboardInterrupt:
        signal_handler(None, None)  # Fallback if signal.pause fails
