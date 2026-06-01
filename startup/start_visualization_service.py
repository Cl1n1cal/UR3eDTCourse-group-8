import subprocess
import os
import platform

from startup.utils.logging_config import create_service_logger

logger = create_service_logger("start_visualization_service")

def _get_executable_path(system, machine):
    """
    Detects the OS type and returns the path to the appropriate ur3e_mockup executable.

    Returns:
        str: Path to the platform-specific executable

    Raises:
        OSError: If the OS is not supported or executable not found
    """
    if machine.upper() in ["AMD64", "X86_64"]:
        machine = "x86_64"
    
    if machine != "x86_64":
        raise OSError(f"Unsupported machine architecture: {machine}. Supported architectures: x86_64")
    
    current_dir = os.path.dirname(os.path.abspath(__file__))
    vis_dir = os.path.join(current_dir, "..", "ur3e_dt_visualization")
    if system == "Windows":
        executable_name = "UR3e.exe"
        vis_dir = os.path.join(vis_dir, "exports", "windows")
        executable_path = os.path.join(vis_dir, executable_name)
    elif system == "Linux":
        executable_name = "UR3e.x86_64"
        vis_dir = os.path.join(vis_dir, "exports", "linux")
        executable_path = os.path.join(vis_dir, executable_name)
    else:
        raise OSError(
            f"Unsupported operating system: {system}. "
            f"Supported systems: Windows, Linux"
        )
    executable_path = os.path.abspath(os.path.normpath(executable_path))

    if not os.path.exists(executable_path):
        raise FileNotFoundError(f"Executable not found for {system}: {executable_path}")

    return executable_path

def start_visualization_service(ok_queue=None):
    """
    Starts the ur3e_visualization executable and keeps it running.
    Handles graceful shutdown via Ctrl+C (SIGINT).
    """
    # Get the platform-specific executable path
    system = platform.system()
    machine = platform.machine()
    logger.info("Detected OS: %s, Machine: %s", system, machine)
    executable_path = _get_executable_path(system, machine)
    logger.info("Starting executable: %s", executable_path)

    # Start the subprocess
    process = subprocess.Popen([executable_path])

    if ok_queue:
        ok_queue.put("started")

    try:
        # Keep the process running until interrupted
        process.wait()
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        logger.info("Shutting down visualization service...")
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
    finally:
        logger.info("Robot arm visualization service stopped.")