import json
import os
import math
import threading
from datetime import datetime
from tkinter import messagebox
import customtkinter as ctk

from services.command_sender import CommandSender
from communication.factory import RabbitMQFactory
from communication.protocol import ROUTING_KEY_ELECTRICITY, ROUTING_KEY_JOINT_ROTATIONS, ROUTING_KEY_MODEL_STATE, ROUTING_KEY_MONITORING, ROUTING_KEY_STATE, MonitoringMsgKeys
from startup.utils.logging_config import create_service_logger

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

_SETTINGS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'settings.json')

C_OK = "#2ecc71"
C_WARN = "#f39c12"
C_HIGH = "#e74c3c"
C_MUTED = "#95a5a6"
C_BLUE = "#5dade2"

class TelemetryHub:
    """Single consumer for all read-only topics."""
    def __init__(self):
        self._rmq = RabbitMQFactory.create_rabbitmq()
        self._l = create_service_logger("telemetry_hub")
        self.on_state: callable = None
        self.on_model_state: callable = None
        self.on_electricity: callable = None
        self.on_rotations: callable = None
        self.on_monitoring: callable = None

    def start(self):
        try:
            self._rmq.connect_to_server()
            self._rmq.subscribe(ROUTING_KEY_STATE, self._h_state)
            self._rmq.subscribe(ROUTING_KEY_MODEL_STATE, self._h_model)
            self._rmq.subscribe(ROUTING_KEY_ELECTRICITY, self._h_elec)
            self._rmq.subscribe(ROUTING_KEY_JOINT_ROTATIONS, self._h_rot)
            self._rmq.subscribe(ROUTING_KEY_MONITORING, self._h_mon)
            threading.Thread(target=self._rmq.start_consuming, daemon=True).start()
        except Exception as e:
            self._l.warning(f"TelemetryHub failed: {e}")

    def _h_state(self, ch, m, p, msg):
        if self.on_state: self.on_state(msg)

    def _h_model(self, ch, m, p, msg):
        if self.on_model_state: self.on_model_state(msg)

    def _h_elec(self, ch, m, p, msg):
        if self.on_electricity: self.on_electricity(msg)

    def _h_rot(self, ch, m, p, msg):
        if self.on_rotations: self.on_rotations(msg)

    def _h_mon(self, ch, m, p, msg):
        if self.on_monitoring: self.on_monitoring(msg)



class DashboardUI(ctk.CTk):
    _state_pt: dict = {}
    _state_sim: dict = {}
    _elec: dict = {}
    _rot: dict = {}
    _monitoring: list = []

    def __init__(self):
        super().__init__()

        self.sender = CommandSender()

        self.title("UR3e Digital Twin — Dashboard")
        self.geometry("1500x980")
        self.minsize(1400, 900)

        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        self._prev_alarm_state: dict[str, bool] = {}

        self._threshold_warned: dict[int, bool] = {}

        self._rot_threshold = 10000.0
        self._price_dkk = 2.47
        self._co2_intensity = 100.0
        self._annual_hours = 2000.0
        self._load_settings()

        header = ctk.CTkFrame(self, height=80)
        header.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 5))
        header.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(header, text="UR3e Digital Twin — Dashboard", font=ctk.CTkFont(size=26, weight="bold")).grid(row=0, column=0, padx=20, pady=20, sticky="w")
        self._status_var = ctk.StringVar(value="Disconnected")
        ctk.CTkLabel(header, textvariable=self._status_var, font=ctk.CTkFont(size=14, weight="bold"), text_color=C_WARN).grid(row=0, column=1, padx=20, pady=20, sticky="e")

        self.tabs = ctk.CTkTabview(self)
        self.tabs.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        self.tabs.add("Control Center & Fault Injection")
        self.tabs.add("Live Robot State")
        self.tabs.add("Sustainability & Lifecycle")
        self.tabs.add("Alarm Monitoring")
        self.tabs.add("Settings")

        self._build_tab_control()
        self._build_tab_live()
        self._build_tab_sustainability()
        self._build_tab_diagnostics()
        self._build_tab_settings()

        # ── telemetry ──
        self._hub = TelemetryHub()
        self._hub.on_state = lambda m: setattr(DashboardUI, "_state_pt", dict(m))
        self._hub.on_model_state = lambda m: setattr(DashboardUI, "_state_sim", dict(m))
        self._hub.on_electricity = lambda m: setattr(DashboardUI, "_elec", dict(m))
        self._hub.on_rotations = lambda m: setattr(DashboardUI, "_rot", dict(m))
        self._hub.on_monitoring = self._on_monitoring_msg
        self._hub.start()

        self._connect()
        self._schedule_refresh()


    def _build_tab_control(self):
        tab = self.tabs.tab("Control Center & Fault Injection")
        tab.grid_columnconfigure(0, weight=3)
        tab.grid_columnconfigure(1, weight=2)
        tab.grid_rowconfigure(0, weight=1)

        left = ctk.CTkScrollableFrame(tab)
        left.grid(row=0, column=0, sticky="nsew", padx=15, pady=15)

        ctk.CTkLabel(left, text="Motion Control", font=ctk.CTkFont(size=22, weight="bold")).pack(anchor="w", padx=10, pady=(10, 20))

        self.joint_position_sliders = []
        self.joint_position_value_labels = []
        sf = ctk.CTkFrame(left)
        sf.pack(fill="x", padx=10, pady=10)
        for idx in range(6):
            row = ctk.CTkFrame(sf)
            row.pack(fill="x", padx=10, pady=8)
            ctk.CTkLabel(row, text=f"Joint {idx}", width=100, font=ctk.CTkFont(weight="bold")).pack(side="left", padx=5)
            sl = ctk.CTkSlider(row, from_=-2*math.pi, to=2*math.pi, number_of_steps=400, command=lambda v, i=idx: self._update_position_label(i, v))
            sl.set(0); sl.pack(side="left", fill="x", expand=True, padx=10)
            vl = ctk.CTkLabel(row, text="0.000", width=70)
            vl.pack(side="right", padx=10)
            self.joint_position_sliders.append(sl)
            self.joint_position_value_labels.append(vl)

        pf = ctk.CTkFrame(left); pf.pack(fill="x", padx=10, pady=10)
        pf.grid_columnconfigure((0, 1), weight=1)
        
        for col, (label, default) in enumerate([("Velocity (°/s)", "80"), ("Acceleration (°/s²)", "60")]):
            f = ctk.CTkFrame(pf); f.grid(row=0, column=col, sticky="ew", padx=10, pady=10)
            ctk.CTkLabel(f, text=label, font=ctk.CTkFont(weight="bold")).pack(anchor="w", padx=10, pady=(10, 5))
            e = ctk.CTkEntry(f); e.insert(0, default); e.pack(fill="x", padx=10, pady=(0, 10))
            
            if col == 0:
                self.load_vel_entry = e
            else:
                self.load_acc_entry = e

        bf = ctk.CTkFrame(left); bf.pack(fill="x", padx=10, pady=20)
        bf.grid_columnconfigure((0, 1, 2, 3), weight=1)
        
        for col, (txt, fg, hov, cmd) in enumerate([
                ("Load Trajectory", None, None, self._on_load_program),
                ("Play", "#27ae60", "#229954", self._on_play),
                ("Pause", "#f39c12", "#d68910", self._on_pause),
                ("Stop", "#c0392b", "#922b21", self._on_stop),
        ]):
            kw = dict(text=txt, height=45, command=cmd)
            
            if fg:
                kw.update(fg_color=fg, hover_color=hov)
            
            ctk.CTkButton(bf, **kw).grid(row=0, column=col, padx=10, pady=15, sticky="ew")

        right = ctk.CTkScrollableFrame(tab)
        right.grid(row=0, column=1, sticky="nsew", padx=15, pady=15)
        ctk.CTkLabel(right, text="Fault Injection", font=ctk.CTkFont(size=22, weight="bold")).pack(anchor="w", padx=10, pady=(10, 20))

        self.stuck_joint_vars = []
        sf2 = ctk.CTkFrame(right); sf2.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(sf2, text="Inject Stuck Joint Fault", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=15, pady=(15, 10))
        cr = ctk.CTkFrame(sf2); cr.pack(fill="x", padx=15, pady=10)
        
        for idx in range(6):
            v = ctk.BooleanVar(value=False)
            ctk.CTkCheckBox(cr, text=f"Joint {idx}", variable=v).pack(side="left", padx=10)
            self.stuck_joint_vars.append(v)
            
        ctk.CTkButton(sf2, text="Inject Stuck Joint", fg_color="#c0392b", command=self._on_stuck_joint).pack(fill="x", padx=15, pady=(10, 15))

        self.wear_joint_vars = []
        wf = ctk.CTkFrame(right); wf.pack(fill="x", padx=10, pady=20)
        ctk.CTkLabel(wf, text="Wear Fault Injection", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=15, pady=(15, 10))
        wr = ctk.CTkFrame(wf); wr.pack(fill="x", padx=15, pady=10)
        
        for idx in range(6):
            v = ctk.BooleanVar(value=False)
            ctk.CTkCheckBox(wr, text=f"Joint {idx}", variable=v).pack(side="left", padx=10)
            self.wear_joint_vars.append(v)
            
        ctk.CTkLabel(wf, text="Wear Level", font=ctk.CTkFont(weight="bold")).pack(anchor="w", padx=15)
        self.wear_level_slider = ctk.CTkSlider(wf, from_=0.0, to=1.0, number_of_steps=100, command=self._update_wear_label)
        self.wear_level_slider.set(0.3); self.wear_level_slider.pack(fill="x", padx=15, pady=10)
        self.wear_level_value = ctk.CTkLabel(wf, text="0.30")
        self.wear_level_value.pack(anchor="e", padx=15)
        
        ctk.CTkLabel(wf, text="Duration (seconds)", font=ctk.CTkFont(weight="bold")).pack(anchor="w", padx=15, pady=(15, 0))
        self.wear_duration_entry = ctk.CTkEntry(wf)
        self.wear_duration_entry.insert(0, "10")
        self.wear_duration_entry.pack(fill="x", padx=15, pady=10)
        
        ctk.CTkButton(wf, text="Inject Wear Fault", fg_color="#8e44ad", command=self._on_wear).pack(fill="x", padx=15, pady=(10, 15))


    def _build_tab_live(self):
        tab = self.tabs.tab("Live Robot State")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(0, weight=1)

        scr = ctk.CTkScrollableFrame(tab)
        scr.grid(row=0, column=0, sticky="nsew", padx=15, pady=15)

        ctk.CTkLabel(scr, text="Live Robot State", font=ctk.CTkFont(size=22, weight="bold")).pack(anchor="w", padx=10, pady=(10, 5))
        self._robot_mode_lbl = ctk.CTkLabel(scr, text="Mode: —", font=ctk.CTkFont(size=15))
        self._robot_mode_lbl.pack(anchor="w", padx=10, pady=(0, 15))

        jf = ctk.CTkFrame(scr); jf.pack(fill="x", padx=10, pady=(0, 20))
        ctk.CTkLabel(jf, text="Joint Positions & Velocities", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=15, pady=(15, 8))

        header = ctk.CTkFrame(jf); header.pack(fill="x", padx=15, pady=(0, 4))
        header.grid_columnconfigure((0,1,2,3,4,5), weight=1)
        for col, h in enumerate(["Joint", "PT pos (rad)", "Sim pos (rad)", "Δq (rad)", "PT vel (rad/s)", "Sim vel (rad/s)"]):
            ctk.CTkLabel(header, text=h, font=ctk.CTkFont(weight="bold")).grid(row=0, column=col, padx=6, pady=6)

        self._live_vars: list[tuple] = []
        for i in range(6):
            row = ctk.CTkFrame(jf); row.pack(fill="x", padx=15, pady=2)
            row.grid_columnconfigure((0,1,2,3,4,5), weight=1)
            ctk.CTkLabel(row, text=f"J{i}", font=ctk.CTkFont(weight="bold")).grid(row=0, column=0, padx=6, pady=6)
            vars_row = []
            
            for col in range(1, 6):
                v = ctk.StringVar(value="—")
                ctk.CTkLabel(row, textvariable=v, font=("Courier", 12)).grid(row=0, column=col, padx=6, pady=6)
                vars_row.append(v)
            
            self._live_vars.append(tuple(vars_row))

        tf = ctk.CTkFrame(scr); tf.pack(fill="x", padx=10, pady=(0, 20))
        ctk.CTkLabel(tf, text="TCP Pose", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=15, pady=(15, 8))

        th = ctk.CTkFrame(tf); th.pack(fill="x", padx=15, pady=(0, 4))
        th.grid_columnconfigure((0,1,2,3,4,5,6), weight=1)
        
        for col, h in enumerate(["Source", "X (m)", "Y (m)", "Z (m)", "Rx (rad)", "Ry (rad)", "Rz (rad)"]):
            ctk.CTkLabel(th, text=h, font=ctk.CTkFont(weight="bold")).grid(row=0, column=col, padx=6, pady=6)

        self._tcp_vars: list[list[ctk.StringVar]] = []
        for row_i, src in enumerate(["PT", "Sim"]):
            row = ctk.CTkFrame(tf); row.pack(fill="x", padx=15, pady=2)
            row.grid_columnconfigure((0,1,2,3,4,5,6), weight=1)
            ctk.CTkLabel(row, text=src, font=ctk.CTkFont(weight="bold")).grid(row=0, column=0, padx=6, pady=6)
            rv = []
            for col in range(1, 7):
                v = ctk.StringVar(value="—")
                ctk.CTkLabel(row, textvariable=v, font=("Courier", 12)).grid(row=0, column=col, padx=6, pady=6)
                rv.append(v)
            self._tcp_vars.append(rv)


    def _build_tab_sustainability(self):
        tab = self.tabs.tab("Sustainability & Lifecycle")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_columnconfigure(1, weight=1)
        tab.grid_rowconfigure(0, weight=1)

        ef = ctk.CTkScrollableFrame(tab)
        ef.grid(row=0, column=0, sticky="nsew", padx=(15, 7), pady=15)
        ctk.CTkLabel(ef, text="⚡  Electricity", font=ctk.CTkFont(size=22, weight="bold")).pack(anchor="w", padx=20, pady=(20, 15))

        self._elec_vars = {}
        elec_rows = [
            ("Power", "Current power draw"),
            ("Energy", "Cumulative energy this session"),
            ("Session", "Session duration"),
            ("Cost (DKK)", "Cumulative cost in DKK"),
            ("Cost (EUR)", "Cumulative cost in EUR"),
            ("Price (DKK/kWh)", "Electricity price used"),
            ("CO₂ Emitted", "Estimated carbon footprint"),
            ("Projected Annual", "At current power, {} h/yr"),
        ]
        for key, tooltip in elec_rows:
            r = ctk.CTkFrame(ef); r.pack(fill="x", padx=20, pady=8)
            ctk.CTkLabel(r, text=key, width=180, anchor="w", font=ctk.CTkFont(weight="bold")).pack(side="left", padx=10, pady=10)
            v = ctk.StringVar(value="—")
            ctk.CTkLabel(r, textvariable=v, font=("Courier", 14)).pack(side="left", padx=10)
            self._elec_vars[key] = v

        self._elec_updated_var = ctk.StringVar(value="No telemetry yet")
        ctk.CTkLabel(ef, textvariable=self._elec_updated_var, text_color=C_MUTED).pack(anchor="w", padx=20, pady=(10, 20))

        rf = ctk.CTkScrollableFrame(tab)
        rf.grid(row=0, column=1, sticky="nsew", padx=(7, 15), pady=15)
        ctk.CTkLabel(rf, text="🔄  Joint Lifecycle / Odometer", font=ctk.CTkFont(size=22, weight="bold")).pack(anchor="w", padx=20, pady=(20, 15))

        self.rotation_progressbars = []
        self.rotation_value_labels = []
        self.rotation_status_labels = []
        self._rot_pct_labels = []
        self._rot_remaining_labels = []

        for i in range(6):
            card = ctk.CTkFrame(rf); card.pack(fill="x", padx=20, pady=10)

            top = ctk.CTkFrame(card); top.pack(fill="x", padx=10, pady=(10, 4))
            ctk.CTkLabel(top, text=f"Joint {i}", width=80, font=ctk.CTkFont(weight="bold")).pack(side="left", padx=5)
            val = ctk.CTkLabel(top, text="0.00 rev", width=100)
            val.pack(side="left", padx=5)
            pct = ctk.CTkLabel(top, text="0.0 %", width=70)
            pct.pack(side="left", padx=5)
            rem = ctk.CTkLabel(top, text="— rem", width=100, text_color=C_MUTED)
            rem.pack(side="left", padx=5)
            status = ctk.CTkLabel(top, text="✓ Healthy", text_color=C_OK)
            status.pack(side="right", padx=10)

            bar = ctk.CTkProgressBar(card, height=14)
            bar.set(0); bar.pack(fill="x", padx=15, pady=(0, 10))

            self.rotation_progressbars.append(bar)
            self.rotation_value_labels.append(val)
            self.rotation_status_labels.append(status)
            self._rot_pct_labels.append(pct)
            self._rot_remaining_labels.append(rem)

        tf2 = ctk.CTkFrame(rf); tf2.pack(fill="x", padx=20, pady=(10, 5))
        ctk.CTkLabel(tf2, text="Total", width=80, font=ctk.CTkFont(weight="bold")).pack(side="left", padx=15, pady=8)
        self._total_rot_var = ctk.StringVar(value="0.00 rev")
        ctk.CTkLabel(tf2, textvariable=self._total_rot_var, font=("Courier", 14)).pack(side="left", padx=5)

        self._rot_updated_var = ctk.StringVar(value="No telemetry yet")
        ctk.CTkLabel(rf, textvariable=self._rot_updated_var, text_color=C_MUTED).pack(anchor="w", padx=20, pady=(0, 20))


    def _build_tab_diagnostics(self):
        tab = self.tabs.tab("Alarm Monitoring")
        tab.grid_columnconfigure(0, weight=1)
        tab.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(tab, text="Alarm Monitoring", font=ctk.CTkFont(size=24, weight="bold")).grid(row=0, column=0, padx=20, pady=(20, 10), sticky="w")

        af = ctk.CTkFrame(tab)
        af.grid(row=1, column=0, padx=20, pady=(0, 20), sticky="nsew")
        tab.grid_rowconfigure(2, weight=1)
        ctk.CTkLabel(af, text="Event Log", font=ctk.CTkFont(size=18, weight="bold")).pack(anchor="w", padx=15, pady=(15, 10))

        btn_row = ctk.CTkFrame(af); btn_row.pack(fill="x", padx=15, pady=(0, 8))
        ctk.CTkButton(btn_row, text="Clear Log", width=100,command=self._clear_alarm_log).pack(side="right")

        self.alarm_textbox = ctk.CTkTextbox(af, height=220, font=("Courier", 12))
        self.alarm_textbox.pack(fill="both", expand=True, padx=15, pady=(0, 15))
        self._alarm_log("[INFO] Diagnostics initialized")


    def _build_tab_settings(self):
        tab = self.tabs.tab("Settings")

        ctk.CTkLabel(tab, text="Runtime Configuration", font=ctk.CTkFont(size=24, weight="bold")).pack(anchor="w", padx=20, pady=(20, 10))

        def section(parent, text):
            ctk.CTkLabel(parent, text=text, font=ctk.CTkFont(size=16, weight="bold"), text_color=C_BLUE).pack(anchor="w", padx=20, pady=(16, 4))

        def field(parent, label, default, attr):
            f = ctk.CTkFrame(parent); f.pack(fill="x", padx=20, pady=6)
            ctk.CTkLabel(f, text=label, width=280, anchor="w", font=ctk.CTkFont(weight="bold")).pack(side="left", padx=10, pady=10)
            e = ctk.CTkEntry(f, width=160); e.insert(0, str(default))
            e.pack(side="left", padx=10)
            setattr(self, attr, e)

        section(tab, "Electricity")
        field(tab, "Price per kWh (DKK)", self._price_dkk, "_s_price")
        field(tab, "CO₂ intensity (g CO₂/kWh)", self._co2_intensity, "_s_co2")
        ctk.CTkLabel(tab, text="  Danish grid ~100 g/kWh (2025/26 — heavy offshore wind)", text_color=C_MUTED, font=ctk.CTkFont(size=11)).pack(anchor="w", padx=30)

        section(tab, "Joint Lifecycle")
        field(tab, "Maintenance threshold (rev)", self._rot_threshold, "_s_threshold")
        ctk.CTkLabel(tab, text="  Joints exceeding this threshold show ⚠ Maintenance Required", text_color=C_MUTED, font=ctk.CTkFont(size=11)).pack(anchor="w", padx=30)

        section(tab, "Projections")
        field(tab, "Annual operating hours", self._annual_hours, "_s_annual_h")
        ctk.CTkLabel(tab, text="  Used for projected annual cost (default: 8 h/day × 250 days)", text_color=C_MUTED, font=ctk.CTkFont(size=11)).pack(anchor="w", padx=30)

        self._settings_feedback = ctk.StringVar(value="")
        ctk.CTkLabel(tab, textvariable=self._settings_feedback, text_color=C_OK).pack(anchor="w", padx=20, pady=(12, 0))
        ctk.CTkButton(tab, text="Apply Settings", height=42, command=self._apply_settings).pack(anchor="e", padx=20, pady=20)

        section(tab, "About")
        ctk.CTkLabel(tab, text="UR3e Digital Twin — Group 8", text_color=C_MUTED, font=ctk.CTkFont(size=11)).pack(anchor="w", padx=24, pady=1)
        ctk.CTkLabel(tab, text="Oliver Thomas  ·  Sebastian From  ·  João Vera  ·  Mikkel Riis", text_color=C_MUTED, font=ctk.CTkFont(size=11)).pack(anchor="w", padx=24, pady=1)
        ctk.CTkLabel(tab, text="Engineering Digital Twins, Aarhus University, 2026", text_color=C_MUTED, font=ctk.CTkFont(size=11)).pack(anchor="w", padx=24, pady=1)
    

    def _on_monitoring_msg(self, msg):
        if not isinstance(msg, list):
            return
        
        DashboardUI._monitoring = list(msg)
        for entry in msg:
            mtype = entry.get(MonitoringMsgKeys.TYPE, "")
            rob = entry.get(MonitoringMsgKeys.ROBUSTNESS_VALUE)
            if rob is None:
                continue
            
            was_ok   = self._prev_alarm_state.get(mtype, True)
            is_alarm = rob < 0
            if is_alarm and was_ok:
                self.after(0, lambda t=mtype, r=rob: self._alarm_log(f"[ALARM] {t}: robustness={r:.4f}"))
                
            self._prev_alarm_state[mtype] = not is_alarm


    def _schedule_refresh(self):
        self.after(250, self._refresh)


    def _refresh(self):
        self._refresh_live()
        self._refresh_electricity()
        self._refresh_rotations()
        self._schedule_refresh()


    def _refresh_live(self):
        pt = DashboardUI._state_pt
        sim = DashboardUI._state_sim
        mode = pt.get("robot_mode", "—")
        self._robot_mode_lbl.configure(text=f"Mode: {mode}")

        pt_q_list = pt.get("q_actual") or []
        sim_q_list = sim.get("q_actual") or []
        pt_qd_list = pt.get("qd_actual") or []
        sim_qd_list = sim.get("qd_actual") or []

        for i, (pt_q, sim_q, dq, pt_qd, sim_qd) in enumerate(self._live_vars):
            pq = pt_q_list[i] if i < len(pt_q_list) else None
            sq = sim_q_list[i] if i < len(sim_q_list) else None
            pqd = pt_qd_list[i] if i < len(pt_qd_list) else None
            sqd = sim_qd_list[i] if i < len(sim_qd_list) else None
            pt_q.set(f"{pq:+.4f}" if pq is not None else "—")
            sim_q.set(f"{sq:+.4f}" if sq is not None else "—")
            if pq is not None and sq is not None:
                dq.set(f"{pq-sq:+.4f}")
            else:
                dq.set("—")
            pt_qd.set(f"{pqd:+.4f}" if pqd is not None else "—")
            sim_qd.set(f"{sqd:+.4f}" if sqd is not None else "—")

        pt_tcp = pt.get("tcp_pose") or []
        sim_tcp = sim.get("tcp_pose") or []
        for col_i in range(6):
            v = pt_tcp[col_i] if col_i < len(pt_tcp) else None
            self._tcp_vars[0][col_i].set(f"{v:.4f}" if v is not None else "—")
            v = sim_tcp[col_i] if col_i < len(sim_tcp) else None
            self._tcp_vars[1][col_i].set(f"{v:.4f}" if v is not None else "—")

    def _refresh_electricity(self):
        d = DashboardUI._elec
        if not d: return

        power_w = d.get("power_w", 0)
        energy = d.get("energy_kwh", 0)
        sess_s = d.get("session_seconds", 0)
        cost_dkk = energy * self._price_dkk
        cost_eur = cost_dkk * 0.1338
        h, r = divmod(int(sess_s), 3600)
        m, s = divmod(r, 60)

        co2_g = energy * self._co2_intensity
        proj_dkk = (power_w / 1000.0) * self._annual_hours * self._price_dkk

        self._elec_vars["Power"].set(f"{power_w:.1f} W")
        self._elec_vars["Energy"].set(f"{energy:.4f} kWh")
        self._elec_vars["Session"].set(f"{h:02d}:{m:02d}:{s:02d}")
        self._elec_vars["Cost (DKK)"].set(f"{cost_dkk:.4f} DKK")
        self._elec_vars["Cost (EUR)"].set(f"{cost_eur:.4f} EUR")
        self._elec_vars["Price (DKK/kWh)"].set(f"{self._price_dkk:.2f}")
        self._elec_vars["CO₂ Emitted"].set(f"{co2_g:.1f} g  ({co2_g/1000:.4f} kg)")
        self._elec_vars["Projected Annual"].set(f"{proj_dkk:.2f} DKK  ({proj_dkk*0.1338:.2f} EUR)")

        ts = d.get("timestamp", "")
        self._elec_updated_var.set(f"Last update: {ts[:19].replace('T',' ')}")

    def _refresh_rotations(self):
        d = DashboardUI._rot
        if not d:
            return

        rotations = d.get("joint_rotations", [])
        threshold = self._rot_threshold

        for i in range(6):
            if i >= len(rotations):
                continue
            
            val = rotations[i]
            ratio = min(val / threshold, 1.0) if threshold > 0 else 0.0
            pct = ratio * 100.0
            remaining = max(threshold - val, 0.0)

            self.rotation_progressbars[i].set(ratio)
            self.rotation_value_labels[i].configure(text=f"{val:.2f} rev")
            self._rot_pct_labels[i].configure(text=f"{pct:.1f} %")
            self._rot_remaining_labels[i].configure(text=f"{remaining:.0f} rem")

            exceeded = threshold > 0 and val >= threshold
            was_ok = not self._threshold_warned.get(i, False)

            if exceeded:
                self.rotation_status_labels[i].configure(text="⚠ Maintenance Required", text_color=C_HIGH)
                self.rotation_progressbars[i].configure(progress_color=C_HIGH)
                if was_ok:
                    self._alarm_log(f"[ALARM] Joint {i} exceeded maintenance threshold — {val:.1f} ≥ {threshold:.0f} rev. Inspection recommended.")
                self._threshold_warned[i] = True
            elif pct > 80:
                self.rotation_status_labels[i].configure(text="⚠ Approaching limit", text_color=C_WARN)
                self.rotation_progressbars[i].configure(progress_color=C_WARN)
                self._threshold_warned[i] = False
            else:
                self.rotation_status_labels[i].configure(text="✓ Healthy", text_color=C_OK)
                self.rotation_progressbars[i].configure(progress_color=C_OK)
                self._threshold_warned[i] = False

        self._total_rot_var.set(f"{d.get('total_rotations', 0):.2f} rev")
        ts = d.get("timestamp", "")
        self._rot_updated_var.set(f"Last update: {ts[:19].replace('T',' ')}")


    def _alarm_log(self, text: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.alarm_textbox.insert("end", f"[{ts}] {text}\n")
        self.alarm_textbox.see("end")

    def _clear_alarm_log(self):
        self.alarm_textbox.delete("1.0", "end")


    def _load_settings(self):
        try:
            with open(_SETTINGS_FILE) as f:
                d = json.load(f)
            self._price_dkk = float(d.get('price_dkk', self._price_dkk))
            self._co2_intensity = float(d.get('co2_intensity', self._co2_intensity))
            self._rot_threshold = float(d.get('rot_threshold', self._rot_threshold))
            self._annual_hours = float(d.get('annual_hours', self._annual_hours))
        except (FileNotFoundError, json.JSONDecodeError, KeyError, ValueError):
            pass

    def _save_settings(self):
        try:
            with open(_SETTINGS_FILE, 'w') as f:
                json.dump({
                    'price_dkk': self._price_dkk,
                    'co2_intensity': self._co2_intensity,
                    'rot_threshold': self._rot_threshold,
                    'annual_hours': self._annual_hours,
                }, f, indent=2)
        except Exception as e:
            self._settings_feedback.set(f"⚠ Saved in memory but could not write file: {e}")

    def _apply_settings(self):
        try:
            self._price_dkk = float(self._s_price.get())
            self._co2_intensity = float(self._s_co2.get())
            self._rot_threshold = float(self._s_threshold.get())
            self._annual_hours = float(self._s_annual_h.get())
            self._threshold_warned.clear()
            self._save_settings()
            self._settings_feedback.set(
                f"✓ Saved — threshold={self._rot_threshold:.0f} rev, "
                f"price={self._price_dkk:.2f} DKK/kWh, "
                f"CO₂={self._co2_intensity:.0f} g/kWh")
        except ValueError as e:
            self._settings_feedback.set(f"✗ Invalid value: {e}")


    def _connect(self):
        try:
            self.sender.setup()
            self._status_var.set("RabbitMQ Connected")
        except Exception as e:
            self._status_var.set("Connection Failed")
            messagebox.showerror("Connection Error", f"Failed to connect:\n{e}")

    def _update_position_label(self, idx, value):
        self.joint_position_value_labels[idx].configure(text=f"{value:.3f}")

    def _update_wear_label(self, value):
        self.wear_level_value.configure(text=f"{value:.2f}")

    @staticmethod
    def _selected(vars_list):
        return [i for i, v in enumerate(vars_list) if v.get()]

    def _on_load_program(self):
        try:
            pos = [s.get() for s in self.joint_position_sliders]
            vel = float(self.load_vel_entry.get())
            acc = float(self.load_acc_entry.get())
            self.sender.send_load_program_command(pos, vel, acc)
            self._alarm_log("[COMMAND] Load trajectory sent")
        except Exception as e:
            messagebox.showerror("Input Error", str(e))

    def _on_play(self):
        self.sender.send_play_command()
        self._alarm_log("[COMMAND] PLAY")

    def _on_pause(self):
        self.sender.send_pause_command()
        self._alarm_log("[COMMAND] PAUSE")

    def _on_stop(self):
        self.sender.send_stop_command()
        self._alarm_log("[COMMAND] STOP")

    def _on_stuck_joint(self):
        try:
            j = self._selected(self.stuck_joint_vars)
            if not j:
                raise ValueError("Select at least one joint.")
            
            self.sender.send_stuck_joint_command(j)
            self._alarm_log(f"[FAULT] Stuck joint injected on joints {j}")
        except Exception as e:
            messagebox.showerror("Input Error", str(e))

    def _on_wear(self):
        try:
            j = self._selected(self.wear_joint_vars)
            if not j:
                raise ValueError("Select at least one joint.")
            
            lvl = float(self.wear_level_slider.get())
            dur = float(self.wear_duration_entry.get())
            self.sender.send_wear_command(j, lvl, dur)
            self._alarm_log(f"[FAULT] Wear injected  joints={j}  level={lvl:.2f}  dur={dur}s")
        except Exception as e:
            messagebox.showerror("Input Error", str(e))


def run_dashboard_ui():
    app = DashboardUI()
    app.mainloop()

if __name__ == "__main__":
    run_dashboard_ui()
