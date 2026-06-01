from communication import protocol
from communication.factory import RabbitMQFactory
from communication.protocol import ROUTING_KEY_CTRL
from startup.utils.logging_config import create_service_logger

class CommandSender:
    def __init__(self):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("command_sender")

    def setup(self):
        self._l.info("Command sender setup")
        self.rabbitmq.connect_to_server()

    def send_load_program_command(self, position, vel, acc):
        self._l.info(f"Sending load program command with position: {position}, velocity: {vel}, acceleration: {acc}")
        self._send({
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.LOAD_PROGRAM,
            protocol.CtrlMsgKeys.JOINT_POSITIONS: [[float(x) for x in position]],
            protocol.CtrlMsgKeys.MAX_VELOCITY: vel,
            protocol.CtrlMsgKeys.ACCELERATION: acc,
        })

    def send_play_command(self):
        self._l.info("Sending play command")
        self._send({protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PLAY})

    def send_pause_command(self):
        self._l.info("Sending pause command")
        self._send({protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PAUSE})

    def send_stop_command(self):
        self._l.info("Sending stop command")
        self._send({protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.STOP})

    def send_stuck_joint_command(self, joint_indexs):
        self._l.info(f"Sending stuck joint command for joints: {joint_indexs}")
        self._send({
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.INJECT_FAULT,
            protocol.CtrlMsgKeys.FAULT_TYPE: protocol.FaultTypes.STUCK_JOINT,
            protocol.CtrlMsgKeys.JOINTS: joint_indexs,
        })

    def send_wear_command(self, joint_indexs, wear_level, duration):
        self._l.info(f"Sending wear command for joints: {joint_indexs} with wear level: {wear_level} and duration: {duration}")
        self._send({
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.INJECT_FAULT,
            protocol.CtrlMsgKeys.FAULT_TYPE: protocol.FaultTypes.WEAR,
            protocol.CtrlMsgKeys.JOINTS: joint_indexs,
            protocol.CtrlMsgKeys.FAULT_VALUE: wear_level,
            protocol.CtrlMsgKeys.DURATION: duration,
        })

    def _send(self, msg):
        """Send a control message to the UR3e Mockup via RabbitMQ."""
        try:
            self._rmq.send_message(routing_key=ROUTING_KEY_CTRL, message=msg)
            print(f"✓ {msg.get(protocol.CtrlMsgKeys.TYPE)}")
            return True
        except Exception as e:
            self._l.warning(f"Send failed ({e}), reconnecting…")
            try:
                self._rmq = RabbitMQFactory.create_rabbitmq()
                self._rmq.connect_to_server()
                self._rmq.send_message(routing_key=ROUTING_KEY_CTRL, message=msg)
                print(f"✓ {msg.get(protocol.CtrlMsgKeys.TYPE)} (after reconnect)")
                return True
            except Exception as e2:
                print(f"✗ Reconnect failed: {e2}")
                return False

