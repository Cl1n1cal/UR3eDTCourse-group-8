import signal
import sys
import time
from startup.start_monitoring_service import start_monitoring_service
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
from startup.start_command_sender import start_command_sender
from startup.start_alarm_manager_service import start_alarm_manager_service
from startup.start_electricity_service import start_electricity_service
from startup.start_joint_rotation_counter_service import start_joint_rotation_counter_service

_processes = []

def _shutdown(sig=None, frame=None):
    print("\nShutting down all services…")
    for p in _processes:
        try:
            p.terminate()
        except Exception:
            pass
    # Give them a moment, then force-kill
    deadline = time.time() + 3.0
    for p in _processes:
        remaining = max(0.0, deadline - time.time())
        p.join(timeout=remaining)
        if p.is_alive():
            p.kill()
    print("All services stopped.")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    setup_root_logging("all_service_logs")
    start_docker_rabbitmq()
    start_docker_influxdb()

    _processes.append(start_as_daemon(start_db_recorder_service))
    _processes.append(start_as_daemon(start_mockup_state_publisher))
    _processes.append(start_as_daemon(start_robot_arm_mockup))
    _processes.append(start_as_daemon(start_sim_service))
    _processes.append(start_as_daemon(start_calibration_service))
    _processes.append(start_as_daemon(start_monitoring_service))
    _processes.append(start_as_daemon(start_alarm_manager_service))
    _processes.append(start_as_daemon(start_electricity_service))
    _processes.append(start_as_daemon(start_joint_rotation_counter_service))
    _processes.append(start_as_daemon(start_command_sender))
    _processes.append(start_as_daemon(start_visualization_service))

    # Keep main thread alive; works on Windows and Unix
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        _shutdown()
