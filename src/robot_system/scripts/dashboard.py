#!/usr/bin/env python3

import tkinter as tk
import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_msgs.msg import String
from std_srvs.srv import Trigger

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# =================================================================
# THEME
# =================================================================

BG_DARK    = "#0f1117"
BG_CARD    = "#161b27"
BG_CARD2   = "#1c2333"

TEXT_PRI   = "#e8e8e8"
TEXT_SEC   = "#6b7280"
TEXT_MUT   = "#4b5563"

C_TEAL     = "#22d3a0"
C_BLUE     = "#3b82f6"
C_PURPLE   = "#a78bfa"
C_AMBER    = "#f59e0b"
C_RED      = "#ef4444"
C_ORANGE   = "#f97316"

FONT_TITLE = ("Inter", 11, "bold")
FONT_VALUE = ("Inter", 22, "bold")
FONT_LABEL = ("Inter", 9)
FONT_SMALL = ("Inter", 8)
FONT_BTN   = ("Inter", 10, "bold")


# =================================================================
# HELPERS
# =================================================================

def battery_color(pct):
    if pct < 20:
        return C_RED
    if pct < 40:
        return C_AMBER
    return C_TEAL


def lidar_status(dist):
    if dist <= 0.5:
        return "EMERGENCY",     C_RED
    if dist <= 1.0:
        return "OBSTACLE STOP", C_ORANGE
    if dist <= 2.0:
        return "SLOWING",       C_AMBER
    return "SAFE",              C_TEAL


def robot_state_color(state):
    s = state.upper()
    if "EMERGENCY" in s:
        return C_RED
    if "OBSTACLE" in s or "STOP" in s:
        return C_ORANGE
    if "SLOW" in s:
        return C_AMBER
    if "NAVIG" in s or "MOVING" in s:
        return C_TEAL
    return TEXT_SEC


def draw_arc_gauge(canvas, cx, cy, r, ratio,
                   color, track_color="#2a2f3e",
                   start_deg=210, span_deg=120,
                   width=12):
    """
    Draw a half-arc gauge on a tk.Canvas.
    ratio : 0.0 – 1.0
    Angles follow tkinter convention (0 = 3 o'clock, CCW positive).
    """
    # Track arc
    canvas.create_arc(
        cx - r, cy - r, cx + r, cy + r,
        start=start_deg, extent=span_deg,
        style=tk.ARC,
        outline=track_color,
        width=width
    )

    # Value arc
    filled_extent = ratio * span_deg
    if filled_extent > 0.5:
        canvas.create_arc(
            cx - r, cy - r, cx + r, cy + r,
            start=start_deg, extent=filled_extent,
            style=tk.ARC,
            outline=color,
            width=width
        )


# =================================================================
# CARD FRAME FACTORY
# =================================================================

def make_card(parent, **pack_kw):
    f = tk.Frame(parent, bg=BG_CARD, bd=0, highlightthickness=1,
                 highlightbackground="#252d3d")
    f.pack(**pack_kw)
    return f


# =================================================================
# DASHBOARD NODE
# =================================================================

class RobotDashboard(Node):

    def __init__(self, root):
        super().__init__('robot_dashboard')

        self.root = root

        # =========================================================
        # ROBOT DATA
        # =========================================================

        self.lidar_distance  = 0.0
        self.battery_level   = 0.0
        self.velocity        = 0.0
        self.robot_state     = "UNKNOWN"
        self.mission_target  = 10.0
        self.mission_travel  = 0.0

        # =========================================================
        # GRAPH DATA
        # =========================================================

        self.start_time = time.time()

        self.time_history    = []
        self.velocity_history = []
        self.lidar_history   = []
        self.battery_history = []
        self.mission_history = []

        self.max_samples = 100

        # =========================================================
        # ROS 2 SERVICE CLIENT
        # =========================================================

        self.reset_client = self.create_client(Trigger, '/emergency_reset')

        # =========================================================
        # ROS 2 SUBSCRIPTIONS
        # =========================================================

        self.create_subscription(Float64, '/lidar_distance',  self.lidar_callback,    10)
        self.create_subscription(Float64, '/battery',         self.battery_callback,  10)
        self.create_subscription(Float64, '/robot_velocity',  self.velocity_callback, 10)
        self.create_subscription(String,  '/robot_state',     self.state_callback,    10)

        # =========================================================
        # WINDOW
        # =========================================================

        self.root.title("Autonomous Mobile Robot — Dashboard")
        self.root.geometry("1200x860")
        self.root.configure(bg=BG_DARK)
        self.root.resizable(True, True)

        self._build_ui()

        self.update_gui()
        self.update_graphs()

    # =============================================================
    # UI BUILD
    # =============================================================

    def _build_ui(self):

        # ---------------------------------------------------------
        # Header
        # ---------------------------------------------------------

        hdr = tk.Frame(self.root, bg=BG_DARK)
        hdr.pack(fill=tk.X, padx=24, pady=(16, 0))

        dot = tk.Canvas(hdr, width=10, height=10, bg=BG_DARK,
                        bd=0, highlightthickness=0)
        dot.pack(side=tk.LEFT, padx=(0, 10))
        dot.create_oval(1, 1, 9, 9, fill=C_TEAL, outline="")
        self._dot_canvas = dot
        self._dot_phase  = 0

        tk.Label(
            hdr,
            text="AUTONOMOUS MOBILE ROBOT",
            font=("Inter", 14, "bold"),
            bg=BG_DARK, fg=TEXT_PRI
        ).pack(side=tk.LEFT)

        tk.Label(
            hdr,
            text="ROS 2 · Navigation & Safety Monitor",
            font=FONT_LABEL,
            bg=BG_DARK, fg=TEXT_SEC
        ).pack(side=tk.LEFT, padx=(12, 0), pady=(3, 0))

        self.clock_label = tk.Label(
            hdr,
            text="",
            font=FONT_LABEL,
            bg=BG_DARK, fg=TEXT_MUT
        )
        self.clock_label.pack(side=tk.RIGHT)

        tk.Frame(self.root, bg="#252d3d", height=1).pack(
            fill=tk.X, padx=24, pady=(10, 14)
        )

        # ---------------------------------------------------------
        # Metric tiles row
        # ---------------------------------------------------------

        tile_row = tk.Frame(self.root, bg=BG_DARK)
        tile_row.pack(fill=tk.X, padx=24, pady=(0, 12))

        self.tile_lidar   = self._make_tile(tile_row, "LiDAR",    C_TEAL)
        self.tile_battery = self._make_tile(tile_row, "Battery",  C_BLUE)
        self.tile_velocity= self._make_tile(tile_row, "Velocity", C_PURPLE)
        self.tile_mission = self._make_tile(tile_row, "Mission",  C_AMBER)

        # ---------------------------------------------------------
        # Middle row  (gauges + state panel)
        # ---------------------------------------------------------

        mid = tk.Frame(self.root, bg=BG_DARK)
        mid.pack(fill=tk.X, padx=24, pady=(0, 12))

        mid.columnconfigure(0, weight=1)
        mid.columnconfigure(1, weight=1)
        mid.columnconfigure(2, weight=1)

        self._build_speed_gauge(mid)
        self._build_battery_gauge(mid)
        self._build_state_panel(mid)

        # ---------------------------------------------------------
        # Charts
        # ---------------------------------------------------------

        self._build_charts()

    # ----- Metric tile -------------------------------------------

    def _make_tile(self, parent, label, color):
        card = tk.Frame(parent, bg=BG_CARD, bd=0,
                        highlightthickness=1,
                        highlightbackground="#252d3d")
        card.pack(side=tk.LEFT, expand=True, fill=tk.X,
                  padx=(0, 10), ipady=12, ipadx=14)

        tk.Label(card, text=label.upper(), font=FONT_SMALL,
                 bg=BG_CARD, fg=TEXT_SEC).pack(anchor="w", padx=14, pady=(10, 0))

        val_lbl = tk.Label(card, text="—", font=("Inter", 24, "bold"),
                           bg=BG_CARD, fg=color)
        val_lbl.pack(anchor="w", padx=14)

        sub_lbl = tk.Label(card, text="", font=FONT_SMALL,
                           bg=BG_CARD, fg=TEXT_MUT)
        sub_lbl.pack(anchor="w", padx=14, pady=(0, 8))

        return val_lbl, sub_lbl

    # ----- Arc speed gauge ---------------------------------------

    def _build_speed_gauge(self, parent):
        card = tk.Frame(parent, bg=BG_CARD, bd=0,
                        highlightthickness=1,
                        highlightbackground="#252d3d")
        card.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=0)

        tk.Label(card, text="Speed", font=FONT_LABEL,
                 bg=BG_CARD, fg=TEXT_SEC).pack(pady=(12, 0))

        self.speed_canvas = tk.Canvas(
            card, width=180, height=120,
            bg=BG_CARD, bd=0, highlightthickness=0
        )
        self.speed_canvas.pack()

        self.speed_val_lbl = tk.Label(
            card, text="0.00 m/s",
            font=("Inter", 18, "bold"),
            bg=BG_CARD, fg=C_PURPLE
        )
        self.speed_val_lbl.pack(pady=(0, 12))

    # ----- Arc battery gauge -------------------------------------

    def _build_battery_gauge(self, parent):
        card = tk.Frame(parent, bg=BG_CARD, bd=0,
                        highlightthickness=1,
                        highlightbackground="#252d3d")
        card.grid(row=0, column=1, sticky="nsew", padx=(0, 8), pady=0)

        tk.Label(card, text="Battery", font=FONT_LABEL,
                 bg=BG_CARD, fg=TEXT_SEC).pack(pady=(12, 0))

        self.battery_canvas = tk.Canvas(
            card, width=180, height=120,
            bg=BG_CARD, bd=0, highlightthickness=0
        )
        self.battery_canvas.pack()

        # Flat bar below arc
        bar_frame = tk.Frame(card, bg=BG_CARD)
        bar_frame.pack(fill=tk.X, padx=20, pady=(0, 4))

        self.bat_track = tk.Canvas(
            bar_frame, height=8, bg="#2a2f3e",
            bd=0, highlightthickness=0
        )
        self.bat_track.pack(fill=tk.X)
        self.bat_fill_id = self.bat_track.create_rectangle(
            0, 0, 0, 8, fill=C_TEAL, outline=""
        )

        self.battery_val_lbl = tk.Label(
            card, text="0 %",
            font=("Inter", 18, "bold"),
            bg=BG_CARD, fg=C_TEAL
        )
        self.battery_val_lbl.pack(pady=(0, 12))

    # ----- State + mission + reset panel -------------------------

    def _build_state_panel(self, parent):
        card = tk.Frame(parent, bg=BG_CARD, bd=0,
                        highlightthickness=1,
                        highlightbackground="#252d3d")
        card.grid(row=0, column=2, sticky="nsew", pady=0)

        # Safety badge
        tk.Label(card, text="Safety status", font=FONT_LABEL,
                 bg=BG_CARD, fg=TEXT_SEC).pack(anchor="w", padx=16, pady=(12, 4))

        badge_frame = tk.Frame(card, bg="#1e2535", bd=0)
        badge_frame.pack(fill=tk.X, padx=16, pady=(0, 8))

        self.safety_dot = tk.Canvas(
            badge_frame, width=10, height=10,
            bg="#1e2535", bd=0, highlightthickness=0
        )
        self.safety_dot.pack(side=tk.LEFT, padx=(10, 6), pady=8)
        self._sdot = self.safety_dot.create_oval(1, 1, 9, 9,
                                                  fill=C_TEAL, outline="")

        self.safety_lbl = tk.Label(
            badge_frame, text="SAFE",
            font=("Inter", 11, "bold"),
            bg="#1e2535", fg=C_TEAL
        )
        self.safety_lbl.pack(side=tk.LEFT, pady=8)

        # Robot state
        tk.Label(card, text="Robot state", font=FONT_LABEL,
                 bg=BG_CARD, fg=TEXT_SEC).pack(anchor="w", padx=16, pady=(4, 0))

        self.state_lbl = tk.Label(
            card, text="UNKNOWN",
            font=("Inter", 12, "bold"),
            bg=BG_CARD, fg=TEXT_SEC
        )
        self.state_lbl.pack(anchor="w", padx=16, pady=(2, 8))

        # Mission progress
        tk.Label(card, text="Mission progress", font=FONT_LABEL,
                 bg=BG_CARD, fg=TEXT_SEC).pack(anchor="w", padx=16)

        self.mission_track = tk.Canvas(
            card, height=6, bg="#2a2f3e",
            bd=0, highlightthickness=0
        )
        self.mission_track.pack(fill=tk.X, padx=16, pady=(4, 2))
        self.mission_fill_id = self.mission_track.create_rectangle(
            0, 0, 0, 6, fill=C_AMBER, outline=""
        )

        miss_meta = tk.Frame(card, bg=BG_CARD)
        miss_meta.pack(fill=tk.X, padx=16, pady=(0, 8))

        self.mis_done_lbl = tk.Label(
            miss_meta, text="Travelled: 0 m",
            font=FONT_SMALL, bg=BG_CARD, fg=TEXT_MUT
        )
        self.mis_done_lbl.pack(side=tk.LEFT)

        self.mis_pct_lbl = tk.Label(
            miss_meta, text="0 %",
            font=("Inter", 8, "bold"), bg=BG_CARD, fg=C_AMBER
        )
        self.mis_pct_lbl.pack(side=tk.RIGHT)

        # Reset button
        self.reset_button = tk.Button(
            card,
            text="Reset emergency",
            font=FONT_BTN,
            bg="#1e2535", fg=C_RED,
            activebackground="#2a1a1a", activeforeground=C_RED,
            relief=tk.FLAT, bd=0,
            highlightthickness=1,
            highlightbackground="#3d1f1f",
            cursor="hand2",
            command=self.reset_emergency,
            width=20
        )
        self.reset_button.pack(pady=(0, 6), padx=16, fill=tk.X)

        self.reset_status_label = tk.Label(
            card, text="",
            font=FONT_SMALL,
            bg=BG_CARD, fg=TEXT_MUT
        )
        self.reset_status_label.pack(pady=(0, 10))

    # ----- Charts panel ------------------------------------------

    def _build_charts(self):
        charts_card = tk.Frame(
            self.root, bg=BG_CARD, bd=0,
            highlightthickness=1, highlightbackground="#252d3d"
        )
        charts_card.pack(fill=tk.BOTH, expand=True, padx=24, pady=(0, 16))

        hdr = tk.Frame(charts_card, bg=BG_CARD)
        hdr.pack(fill=tk.X, padx=16, pady=(12, 4))

        tk.Label(hdr, text="Live sensor data",
                 font=("Inter", 11, "bold"),
                 bg=BG_CARD, fg=TEXT_PRI).pack(side=tk.LEFT)

        legend_data = [
            ("Velocity", C_PURPLE),
            ("LiDAR",    C_TEAL),
            ("Battery",  C_BLUE),
            ("Mission",  C_AMBER),
        ]
        for name, color in legend_data:
            f = tk.Frame(hdr, bg=BG_CARD)
            f.pack(side=tk.RIGHT, padx=8)
            tk.Canvas(f, width=10, height=10, bg=BG_CARD,
                      bd=0, highlightthickness=0).pack(side=tk.LEFT)
            c = tk.Canvas(f, width=10, height=10, bg=BG_CARD,
                          bd=0, highlightthickness=0)
            c.pack(side=tk.LEFT)
            c.create_rectangle(0, 0, 10, 10, fill=color, outline="")
            tk.Label(f, text=name, font=FONT_SMALL,
                     bg=BG_CARD, fg=TEXT_SEC).pack(side=tk.LEFT, padx=(4, 0))

        plt.style.use("dark_background")

        self.figure = plt.Figure(figsize=(12, 3.8), facecolor=BG_CARD)
        gs = gridspec.GridSpec(1, 4, figure=self.figure,
                               wspace=0.35, left=0.05, right=0.97,
                               top=0.85, bottom=0.22)

        chart_cfg = [
            (gs[0], "Velocity (m/s)",  C_PURPLE),
            (gs[1], "LiDAR dist (m)",  C_TEAL),
            (gs[2], "Battery (%)",     C_BLUE),
            (gs[3], "Mission (%)",     C_AMBER),
        ]

        self.axes = []
        for spec, ylabel, color in chart_cfg:
            ax = self.figure.add_subplot(spec)
            ax.set_facecolor(BG_CARD2)
            ax.set_ylabel(ylabel, color=TEXT_SEC, fontsize=8)
            ax.tick_params(colors=TEXT_MUT, labelsize=7)
            ax.spines[:].set_color("#252d3d")
            ax.grid(True, color="#252d3d", linewidth=0.5)
            self.axes.append((ax, color))

        self.graph_canvas = FigureCanvasTkAgg(self.figure, master=charts_card)
        self.graph_canvas.draw()
        self.graph_canvas.get_tk_widget().pack(
            fill=tk.BOTH, expand=True, padx=12, pady=(0, 12)
        )

    # =============================================================
    # ROS 2 CALLBACKS
    # =============================================================

    def lidar_callback(self, msg):
        self.lidar_distance = msg.data

    def battery_callback(self, msg):
        self.battery_level = msg.data

    def velocity_callback(self, msg):
        self.velocity = msg.data

    def state_callback(self, msg):
        self.robot_state = msg.data

    # =============================================================
    # EMERGENCY RESET
    # =============================================================

    def reset_emergency(self):
        if not self.reset_client.service_is_ready():
            self.reset_status_label.config(
                text="Service not available.", fg=C_RED
            )
            self.get_logger().warn("Emergency reset service not available.")
            return

        request = Trigger.Request()
        self.reset_status_label.config(
            text="Sending reset request…", fg=TEXT_SEC
        )
        self.reset_button.config(state=tk.DISABLED)
        future = self.reset_client.call_async(request)
        future.add_done_callback(self.reset_response)

    def reset_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.reset_status_label.config(
                    text="Reset successful.", fg=C_TEAL
                )
                self.get_logger().info("Emergency reset successful.")
            else:
                self.reset_status_label.config(
                    text=response.message, fg=C_AMBER
                )
                self.get_logger().info(
                    "Reset rejected: " + response.message
                )
        except Exception as e:
            self.reset_status_label.config(
                text="Reset failed.", fg=C_RED
            )
            self.get_logger().error(f"Reset failed: {e}")
        finally:
            self.reset_button.config(state=tk.NORMAL)

    # =============================================================
    # GUI UPDATE  (100 ms)
    # =============================================================

    def update_gui(self):

        # ----- Clock ---------------------------------------------
        self.clock_label.config(
            text=time.strftime("%H:%M:%S")
        )

        # ----- Pulse dot -----------------------------------------
        self._dot_phase = (self._dot_phase + 1) % 20
        dot_color = C_TEAL if self._dot_phase < 10 else "#0f6b50"
        self._dot_canvas.itemconfig("all", fill=dot_color)

        # ----- Metric tiles --------------------------------------
        lidar_lbl, lidar_sub = self.tile_lidar
        bat_lbl,   bat_sub   = self.tile_battery
        vel_lbl,   vel_sub   = self.tile_velocity
        mis_lbl,   mis_sub   = self.tile_mission

        lidar_lbl.config(text=f"{self.lidar_distance:.2f} m")
        bat_lbl.config(  text=f"{self.battery_level:.0f} %",
                         fg=battery_color(self.battery_level))
        vel_lbl.config(  text=f"{self.velocity:.2f} m/s")

        miss_pct = min(100.0, (self.mission_travel / self.mission_target) * 100)
        mis_lbl.config(text=f"{miss_pct:.1f} %")
        mis_sub.config(text=f"Rem: {self.mission_target - self.mission_travel:.2f} m")

        safety, safety_color = lidar_status(self.lidar_distance)
        lidar_sub.config(text=safety, fg=safety_color)
        bat_sub.config(
            text=("⚠ Low" if self.battery_level < 20
                  else "Charge soon" if self.battery_level < 40
                  else "Nominal")
        )

        # ----- Speed arc gauge -----------------------------------
        self.speed_canvas.delete("all")
        draw_arc_gauge(
            self.speed_canvas, cx=90, cy=90, r=65,
            ratio=min(self.velocity / 1.0, 1.0),
            color=C_PURPLE,
            start_deg=210, span_deg=120
        )
        self.speed_val_lbl.config(
            text=f"{self.velocity:.2f} m/s", fg=C_PURPLE
        )
        # tick labels
        self.speed_canvas.create_text(
            22, 100, text="0",    fill=TEXT_MUT, font=FONT_SMALL
        )
        self.speed_canvas.create_text(
            90, 28,  text="0.5",  fill=TEXT_MUT, font=FONT_SMALL
        )
        self.speed_canvas.create_text(
            158, 100, text="1.0", fill=TEXT_MUT, font=FONT_SMALL
        )

        # ----- Battery arc + bar gauge ---------------------------
        bat_ratio = self.battery_level / 100.0
        bat_col   = battery_color(self.battery_level)

        self.battery_canvas.delete("all")
        draw_arc_gauge(
            self.battery_canvas, cx=90, cy=90, r=65,
            ratio=bat_ratio,
            color=bat_col,
            start_deg=210, span_deg=120
        )
        self.battery_canvas.create_text(
            22,  100, text="0",    fill=TEXT_MUT, font=FONT_SMALL
        )
        self.battery_canvas.create_text(
            90,   28, text="50",   fill=TEXT_MUT, font=FONT_SMALL
        )
        self.battery_canvas.create_text(
            158, 100, text="100",  fill=TEXT_MUT, font=FONT_SMALL
        )
        self.battery_val_lbl.config(
            text=f"{self.battery_level:.0f} %", fg=bat_col
        )

        # flat bar
        self.bat_track.update_idletasks()
        w = self.bat_track.winfo_width()
        self.bat_track.coords(
            self.bat_fill_id,
            0, 0, max(0, w * bat_ratio), 8
        )
        self.bat_track.itemconfig(self.bat_fill_id, fill=bat_col)

        # ----- Safety badge --------------------------------------
        self.safety_lbl.config(text=safety, fg=safety_color)
        self.safety_dot.itemconfig(self._sdot, fill=safety_color)
        badge_bg = {
            C_RED:    "#2a1212",
            C_ORANGE: "#2a1a0a",
            C_AMBER:  "#2a2010",
            C_TEAL:   "#0a2a20",
        }.get(safety_color, "#1e2535")
        self.safety_lbl.master.config(bg=badge_bg)
        self.safety_lbl.config(bg=badge_bg)
        self.safety_dot.config(bg=badge_bg)

        # robot state
        sc = robot_state_color(self.robot_state)
        self.state_lbl.config(text=self.robot_state, fg=sc)

        # mission bar
        self.mission_track.update_idletasks()
        mw = self.mission_track.winfo_width()
        self.mission_track.coords(
            self.mission_fill_id,
            0, 0, max(0, mw * (miss_pct / 100.0)), 6
        )
        self.mis_pct_lbl.config(text=f"{miss_pct:.1f} %")
        self.mis_done_lbl.config(
            text=f"Travelled: {self.mission_travel:.2f} m"
        )

        self.root.after(100, self.update_gui)

    # =============================================================
    # GRAPH UPDATE  (500 ms)
    # =============================================================

    def update_graphs(self):
        elapsed = time.time() - self.start_time

        self.time_history.append(elapsed)
        self.velocity_history.append(self.velocity)
        self.lidar_history.append(self.lidar_distance)
        self.battery_history.append(self.battery_level)
        self.mission_history.append(
            min(100.0, (self.mission_travel / self.mission_target) * 100)
        )

        if len(self.time_history) > self.max_samples:
            self.time_history.pop(0)
            self.velocity_history.pop(0)
            self.lidar_history.pop(0)
            self.battery_history.pop(0)
            self.mission_history.pop(0)

        data_sets = [
            self.velocity_history,
            self.lidar_history,
            self.battery_history,
            self.mission_history,
        ]

        for (ax, color), data in zip(self.axes, data_sets):
            ax.clear()
            ax.set_facecolor(BG_CARD2)
            ax.plot(
                self.time_history, data,
                color=color, linewidth=1.5
            )
            ax.fill_between(
                self.time_history, data,
                color=color, alpha=0.08
            )
            ax.tick_params(colors=TEXT_MUT, labelsize=7)
            ax.spines[:].set_color("#252d3d")
            ax.grid(True, color="#252d3d", linewidth=0.5)
            ax.set_xlabel("Time (s)", color=TEXT_MUT, fontsize=7)

        self.figure.tight_layout(pad=0.8)
        self.graph_canvas.draw_idle()

        self.root.after(500, self.update_graphs)


# =================================================================
# MAIN
# =================================================================

def main():
    rclpy.init()

    root = tk.Tk()
    dashboard = RobotDashboard(root)

    def ros_spin():
        if rclpy.ok():
            rclpy.spin_once(dashboard, timeout_sec=0.01)
            root.after(10, ros_spin)

    root.after(10, ros_spin)
    root.mainloop()

    dashboard.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
