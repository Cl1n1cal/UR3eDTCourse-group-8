from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
from communication import protocol
from models.robot_model import create_robot
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath import SE3
from spatialmath.base import transl, rpy2tr

class InverseKinematicsService:
    def __init__(self, inverse_kinematics_config):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("inverse_kinematics")
        self.robot = create_robot(inverse_kinematics_config['dh_parameters']['d'], inverse_kinematics_config['dh_parameters']['a'], inverse_kinematics_config['dh_parameters']['alpha'])

    def setup(self):
        self._l.info("Inverse kinematics service setup")
        self.rabbitmq.connect_to_server()
        self.rabbitmq.subscribe(routing_key=protocol.ROUTING_KEY_CALIBRATION,
                                on_message_callback=self.read_calibration_message)
        self.rabbitmq.subscribe(routing_key=protocol.ROUTING_KEY_IK,
                                on_message_callback=self.read_control_message)

    def compute_inverse_kinematics(self, target_pose: list[float]):
        #convert target pose to SE3 object if it's not already
        # If you already have tcp = [x, y, z, roll, pitch, yaw]
        xyz = target_pose[:3]
        rpy = target_pose[3:]

        # Rebuild SE3 pose from translation + rotation transform matrices
        T = transl(xyz[0], xyz[1], xyz[2]) @ rpy2tr(rpy[0], rpy[1], rpy[2], order='xyz')
        matrix_pose = SE3(T)
        # Solve IK
        sol = self.robot.ikine_LM(matrix_pose)

        self._l.info(f"Computing inverse kinematics for target pose:\n{target_pose}")
        if sol.success:
            self._l.info(f"Found Joint Solution: {sol.q}")
            q_check = self.robot.fkine(sol.q)
            self._l.debug(f"Check Pose:\n{q_check.t}")
        else:
            self._l.warning("Failed to find joint solution.")

        return sol.q

    def read_calibration_message(self, ch, method, properties, message: dict):
        self._l.debug(f"Received calibration message, updating model's DH parameters: {message}")
        if message.get(protocol.CtrlMsgKeys.TYPE) == protocol.CtrlMsgFields.CALIBRATE_DH_PARAMETERS:
            d = message.get(protocol.CtrlMsgKeys.D)
            a = message.get(protocol.CtrlMsgKeys.A)
            alpha = message.get(protocol.CtrlMsgKeys.ALPHA)
        #check if all parameters are present
            if d is None or a is None or alpha is None:
                self._l.error("Calibration message missing required DH parameters.")
                return
            self.update_dh_parameters(d = d, a = a, alpha = alpha)

    def update_dh_parameters(self, d: list, a: list, alpha: list):
        self._l.debug(f"Updating params to d: {d}, a: {a}, alpha: {alpha}")
        for i in range(6):
            self.robot.links[i] = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i])

    def send_load_program_command(self, position, vel, acc):
        # Construct control message for loading a program
        msg = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.LOAD_PROGRAM,
            protocol.CtrlMsgKeys.JOINT_POSITIONS: [[float(x) for x in position]],
            protocol.CtrlMsgKeys.MAX_VELOCITY: vel,
            protocol.CtrlMsgKeys.ACCELERATION: acc,
        }
        self.send_control_message(msg)

    def send_control_message(self, msg):
        """Send a control message to the UR3e Mockup via RabbitMQ."""
        try:
            self.rabbitmq.send_message(
                    routing_key=protocol.ROUTING_KEY_CTRL,
                    message=msg
            )
            self._l.info(f"Control message: {msg} sent successfully")
        except Exception as e:
            self._l.error(f"Failed to send control message: {e}")
    
    def read_control_message(self, ch, method, properties, message: dict):
        self._l.info(f"Received control message: {message}")
        if message.get(protocol.CtrlMsgKeys.TYPE) == protocol.CtrlMsgFields.LOAD_IK_PROGRAM:
            target_pose = message.get(protocol.CtrlMsgKeys.TARGET_POSE)
            vel = message.get(protocol.CtrlMsgKeys.MAX_VELOCITY, 80)  # Default velocity if not provided
            acc = message.get(protocol.CtrlMsgKeys.ACCELERATION, 60)  # Default acceleration if not provided
            if target_pose is None:
                self._l.error("Control message missing required target pose.")
                return
            ik_solution = self.compute_inverse_kinematics(target_pose)
            self.send_load_program_command(ik_solution, vel=vel, acc=acc)
    
    def start_serving(self):
        self._l.info("Starting inverse kinematics service.")
        try:
            self.rabbitmq.start_consuming()
        except KeyboardInterrupt:
            self.rabbitmq.close()