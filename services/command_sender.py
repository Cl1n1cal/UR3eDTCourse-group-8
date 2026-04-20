from tkinter import messagebox
import math
import customtkinter as ctk
from communication.factory import RabbitMQFactory
from startup.utils.logging_config import create_service_logger
from communication import protocol

class CommandSender:
    def __init__(self):
        self.rabbitmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("command_sender")

    def setup(self):
        self._l.info("Command sender setup")
        self.rabbitmq.connect_to_server()

    def send_load_program_command(self, position, vel, acc):
        # Construct control message for loading a program
        msg = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.LOAD_PROGRAM,
            protocol.CtrlMsgKeys.JOINT_POSITIONS: [[float(x) for x in position]],
            protocol.CtrlMsgKeys.MAX_VELOCITY: vel,
            protocol.CtrlMsgKeys.ACCELERATION: acc,
        }
        self.send_control_message(msg)
    
    def send_play_command(self):
        # send control message for starting program
        msg_start = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PLAY,
        }
        self.send_control_message(msg_start)

    def send_pause_command(self):
        # send control message for pausing program
        msg_pause = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PAUSE,
        }
        self.send_control_message(msg_pause)

    def send_stop_command(self):
        # send control message for stopping program
        msg_stop = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.STOP,
        }
        self.send_control_message(msg_stop)
    
    def send_stuck_joint_command(self, joint_indexs):
        # send control message for simulating a stuck joint
        msg_stuck = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.INJECT_FAULT,
            protocol.CtrlMsgKeys.FAULT_TYPE: protocol.FaultTypes.STUCK_JOINT,
            protocol.CtrlMsgKeys.JOINTS: joint_indexs,
        }
        self.send_control_message(msg_stuck)

    def send_wear_command(self, joint_indexs, wear_level, duration):
        # send control message for simulating wear
        msg_wear = {
            protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.INJECT_FAULT,
            protocol.CtrlMsgKeys.FAULT_TYPE: protocol.FaultTypes.WEAR,
            protocol.CtrlMsgKeys.JOINTS: joint_indexs,
            protocol.CtrlMsgKeys.FAULT_VALUE: wear_level,
            protocol.CtrlMsgKeys.DURATION: duration,
        }
        self.send_control_message(msg_wear)

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


class CommandSenderUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.sender = CommandSender()

        self.title("UR3e Command Sender")
        self.geometry("980x800")
        self.minsize(920, 800)

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.grid_columnconfigure(0, weight=1)
        self.joint_position_sliders = []
        self.joint_position_value_labels = []

        container = ctk.CTkFrame(self)
        container.grid(row=0, column=0, padx=16, pady=16, sticky="nsew")
        container.grid_columnconfigure(0, weight=0)
        container.grid_columnconfigure(1, weight=1)
        container.grid_columnconfigure(2, weight=1)
        container.grid_columnconfigure(3, weight=1)

        title = ctk.CTkLabel(
            container,
            text="UR3e Control Commands",
            font=ctk.CTkFont(size=22, weight="bold"),
        )
        title.grid(row=0, column=0, columnspan=4, pady=(12, 8), padx=12, sticky="w")

        self.status_var = ctk.StringVar(value="Status: not connected")
        status_label = ctk.CTkLabel(container, textvariable=self.status_var)
        status_label.grid(row=1, column=0, columnspan=3, padx=12, pady=(0, 12), sticky="w")

        connect_btn = ctk.CTkButton(container, text="Connect", command=self._connect)
        connect_btn.grid(row=1, column=3, padx=12, pady=(0, 12), sticky="e")

        # LOAD PROGRAM
        load_btn = ctk.CTkButton(
            container,
            text="Load Program",
            command=self._on_load_program,
            width=160,
        )
        load_btn.grid(row=2, column=0, padx=12, pady=(8, 6), sticky="nw")

        positions_frame = ctk.CTkFrame(container)
        positions_frame.grid(row=2, column=1, columnspan=3, padx=(6, 12), pady=(8, 6), sticky="ew")
        positions_frame.grid_columnconfigure(1, weight=1)

        range_label = ctk.CTkLabel(positions_frame, text="Joint positions (-2*pi to 2*pi)")
        range_label.grid(row=0, column=0, columnspan=3, padx=10, pady=(8, 4), sticky="w")

        for idx in range(6):
            joint_label = ctk.CTkLabel(positions_frame, text=f"J{idx}:")
            joint_label.grid(row=idx + 1, column=0, padx=(10, 6), pady=4, sticky="w")

            slider = ctk.CTkSlider(
                positions_frame,
                from_=-2 * math.pi,  # type: ignore[arg-type]
                to=2 * math.pi,  # type: ignore[arg-type]
                number_of_steps=400,
                command=lambda val, i=idx: self._update_position_label(i, val),
            )
            slider.set(0.0)
            slider.grid(row=idx + 1, column=1, padx=6, pady=4, sticky="ew")

            value_label = ctk.CTkLabel(positions_frame, text="0.000")
            value_label.grid(row=idx + 1, column=2, padx=(6, 10), pady=4, sticky="e")

            self.joint_position_sliders.append(slider)
            self.joint_position_value_labels.append(value_label)

        vel_frame = ctk.CTkFrame(container)
        vel_frame.grid(row=3, column=1, padx=6, pady=8, sticky="ew")
        vel_frame.grid_columnconfigure(0, weight=1)

        vel_label = ctk.CTkLabel(vel_frame, text="Velocity")
        vel_label.grid(row=0, column=0, padx=8, pady=(6, 2), sticky="w")

        self.load_vel_entry = ctk.CTkEntry(vel_frame)
        self.load_vel_entry.insert(0, "80")
        self.load_vel_entry.grid(row=1, column=0, padx=8, pady=(0, 6), sticky="ew")

        acc_frame = ctk.CTkFrame(container)
        acc_frame.grid(row=3, column=2, padx=6, pady=8, sticky="ew")
        acc_frame.grid_columnconfigure(0, weight=1)

        acc_label = ctk.CTkLabel(acc_frame, text="Acceleration")
        acc_label.grid(row=0, column=0, padx=8, pady=(6, 2), sticky="w")

        self.load_acc_entry = ctk.CTkEntry(acc_frame)
        self.load_acc_entry.insert(0, "60")
        self.load_acc_entry.grid(row=1, column=0, padx=8, pady=(0, 6), sticky="ew")

        # PLAY / PAUSE / STOP
        play_btn = ctk.CTkButton(container, text="Play", command=self._on_play, width=160)
        play_btn.grid(row=4, column=0, padx=12, pady=8, sticky="w")

        pause_btn = ctk.CTkButton(container, text="Pause", command=self._on_pause, width=160)
        pause_btn.grid(row=5, column=0, padx=12, pady=8, sticky="w")

        stop_btn = ctk.CTkButton(container, text="Stop", command=self._on_stop, width=160)
        stop_btn.grid(row=6, column=0, padx=12, pady=8, sticky="w")

        # STUCK JOINT
        stuck_btn = ctk.CTkButton(
            container,
            text="Inject Stuck Joint",
            command=self._on_stuck_joint,
            width=160,
        )
        stuck_btn.grid(row=7, column=0, padx=12, pady=8, sticky="w")

        stuck_joints_frame = ctk.CTkFrame(container)
        stuck_joints_frame.grid(row=7, column=1, padx=6, pady=8, sticky="w")

        stuck_label = ctk.CTkLabel(stuck_joints_frame, text="Joints")
        stuck_label.grid(row=0, column=0, columnspan=6, padx=3, pady=(4, 0), sticky="w")

        self.stuck_joint_vars = []
        for idx in range(6):
            joint_var = ctk.BooleanVar(value=False)
            joint_checkbox = ctk.CTkCheckBox(
                stuck_joints_frame,
                text=str(idx),
                variable=joint_var,
                onvalue=True,
                offvalue=False,
                width=40,
            )
            joint_checkbox.grid(row=1, column=idx, padx=3, pady=(0, 4), sticky="w")
            self.stuck_joint_vars.append(joint_var)

        # WEAR
        wear_btn = ctk.CTkButton(
            container,
            text="Inject Wear",
            command=self._on_wear,
            width=160,
        )
        wear_btn.grid(row=8, column=0, padx=12, pady=(8, 12), sticky="w")

        wear_joints_frame = ctk.CTkFrame(container)
        wear_joints_frame.grid(row=8, column=1, padx=6, pady=(8, 12), sticky="w")

        wear_joints_label = ctk.CTkLabel(wear_joints_frame, text="Joints")
        wear_joints_label.grid(row=0, column=0, columnspan=6, padx=3, pady=(4, 0), sticky="w")

        self.wear_joint_vars = []
        for idx in range(6):
            joint_var = ctk.BooleanVar(value=False)
            joint_checkbox = ctk.CTkCheckBox(
                wear_joints_frame,
                text=str(idx),
                variable=joint_var,
                onvalue=True,
                offvalue=False,
                width=40,
            )
            joint_checkbox.grid(row=1, column=idx, padx=3, pady=(0, 4), sticky="w")
            self.wear_joint_vars.append(joint_var)

        wear_slider_frame = ctk.CTkFrame(container)
        wear_slider_frame.grid(row=8, column=2, padx=6, pady=(8, 12), sticky="ew")
        wear_slider_frame.grid_columnconfigure(0, weight=1)

        wear_level_label = ctk.CTkLabel(wear_slider_frame, text="Wear Level")
        wear_level_label.grid(row=0, column=0, columnspan=2, padx=8, pady=(6, 2), sticky="w")

        self.wear_level_slider = ctk.CTkSlider(
            wear_slider_frame,
            from_=0.0,  # type: ignore[arg-type]
            to=1.0,  # type: ignore[arg-type]
            number_of_steps=100,
            command=self._update_wear_label,
        )
        self.wear_level_slider.set(0.3)
        self.wear_level_slider.grid(row=1, column=0, padx=8, pady=(0, 8), sticky="ew")

        self.wear_level_value = ctk.CTkLabel(wear_slider_frame, text="0.30")
        self.wear_level_value.grid(row=1, column=1, padx=(0, 8), pady=(0, 8))

        duration_frame = ctk.CTkFrame(container)
        duration_frame.grid(row=8, column=3, padx=12, pady=(8, 12), sticky="ew")
        duration_frame.grid_columnconfigure(0, weight=1)

        duration_label = ctk.CTkLabel(duration_frame, text="Duration (s)")
        duration_label.grid(row=0, column=0, padx=8, pady=(6, 2), sticky="w")

        self.wear_duration_entry = ctk.CTkEntry(duration_frame, placeholder_text="duration (s): 10")
        self.wear_duration_entry.insert(0, "10")
        self.wear_duration_entry.grid(row=1, column=0, padx=8, pady=(0, 6), sticky="ew")

        self._connect()

    def _connect(self):
        try:
            self.sender.setup()
            self.status_var.set("Status: connected")
        except Exception as e:
            self.status_var.set("Status: connection failed")
            messagebox.showerror("Connection Error", f"Failed to connect to RabbitMQ:\n{e}")

    def _update_position_label(self, idx, value):
        self.joint_position_value_labels[idx].configure(text=f"{value:.3f}")

    def _update_wear_label(self, value):
        self.wear_level_value.configure(text=f"{value:.2f}")

    @staticmethod
    def _get_selected_joint_indices(joint_vars):
        return [idx for idx, joint_var in enumerate(joint_vars) if joint_var.get()]

    def _on_load_program(self):
        try:
            position = [slider.get() for slider in self.joint_position_sliders]
            vel = float(self.load_vel_entry.get())
            acc = float(self.load_acc_entry.get())
            self.sender.send_load_program_command(position, vel, acc)
        except Exception as e:
            messagebox.showerror("Input Error", str(e))

    def _on_play(self):
        self.sender.send_play_command()

    def _on_pause(self):
        self.sender.send_pause_command()

    def _on_stop(self):
        self.sender.send_stop_command()

    def _on_stuck_joint(self):
        try:
            joint_indexs = self._get_selected_joint_indices(self.stuck_joint_vars)
            if not joint_indexs:
                raise ValueError("Select at least one joint for stuck joint fault.")
            self.sender.send_stuck_joint_command(joint_indexs)
        except Exception as e:
            messagebox.showerror("Input Error", str(e))

    def _on_wear(self):
        try:
            joint_indexs = self._get_selected_joint_indices(self.wear_joint_vars)
            if not joint_indexs:
                raise ValueError("Select at least one joint for wear fault.")
            wear_level = float(self.wear_level_slider.get())
            duration = float(self.wear_duration_entry.get())
            self.sender.send_wear_command(joint_indexs, wear_level, duration)
        except Exception as e:
            messagebox.showerror("Input Error", str(e))


def run_command_sender_ui():
    app = CommandSenderUI()
    app.mainloop()


if __name__ == "__main__":
    run_command_sender_ui()