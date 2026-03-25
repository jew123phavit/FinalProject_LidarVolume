import sys
import json
import csv
import numpy as np
import paho.mqtt.client as mqtt

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFileDialog, QDoubleSpinBox, QGroupBox,
    QCheckBox, QGridLayout, QFrame
)
from PyQt6.QtCore import pyqtSignal, QObject, Qt, QTimer
import pyqtgraph.opengl as gl

# ── MQTT ──────────────────────────────────────────────────────────────────────
MQTT_BROKER      = '100.117.126.91'
MQTT_PORT        = 1883
MQTT_TOPIC_DATA  = 'mapping/points_3d'
MQTT_TOPIC_CMD   = 'control/command'
MQTT_TOPIC_STATE = '/scanner/state'

MAX_POINTS      = 120000
PARKING_MAX_PTS = 2000

STATES_SCAN_ACTIVE = {'SCAN_CW', 'SCAN_CCW', 'PAUSE_CW', 'PAUSE_CCW'}

# ── ค่าเริ่มต้น Silo ──────────────────────────────────────────────────────────
DEFAULT_DIAMETER = 44.0   # cm
DEFAULT_HEIGHT   = 45.5   # cm


# ════════════════════════════════════════════════════════════════════════════
# MQTT Worker
# ════════════════════════════════════════════════════════════════════════════
class MqttWorker(QObject):
    data_received  = pyqtSignal(object)
    state_received = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(MQTT_TOPIC_DATA)
            client.subscribe(MQTT_TOPIC_STATE)
            print("MQTT Connected")

    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic == MQTT_TOPIC_STATE:
                self.state_received.emit(msg.payload.decode().strip().upper())
            else:
                payload = json.loads(msg.payload.decode())
                pts = payload.get('points_m', [])
                if pts:
                    self.data_received.emit(pts)
        except Exception as e:
            print(f"MQTT msg error: {e}")

    def start(self):
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"MQTT Connection Failed: {e}")

    def publish(self, target, action):
        self.client.publish(
            MQTT_TOPIC_CMD,
            json.dumps({"target": target, "action": action}),
            qos=1
        )


# ════════════════════════════════════════════════════════════════════════════
# Main GUI
# ════════════════════════════════════════════════════════════════════════════
class SiloGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Silo 3D — Volume Measurement System")
        self.resize(1350, 900)
        self.setStyleSheet("background-color:#121212; color:#e0e0e0;")

        # ── Silo dimensions ───────────────────────────────────────────────────
        self.silo_radius  = DEFAULT_DIAMETER / 2.0   # cm
        self.silo_height  = DEFAULT_HEIGHT            # cm
        self._update_capacity()

        # ── FLOOR_Z ───────────────────────────────────────────────────────────
        self.floor_z        = -self.silo_height
        self.floor_z_locked = False

        # ── State ─────────────────────────────────────────────────────────────
        self.motor_state  = 'IDLE'
        self.is_scan_mode = False
        self.scan_done    = False
        # baseline_volume: volume ที่คำนวณได้จาก scan ถังเปล่า
        # ใช้ลบออกจากผลคำนวณทุกครั้ง → ถังเปล่าจะได้ 0 พอดี
        self.baseline_volume = 0.0

        # ── Buffers ───────────────────────────────────────────────────────────
        self.scan_pts    = np.zeros((0, 3), dtype=np.float32)
        self.parking_pts = np.zeros((0, 3), dtype=np.float32)

        # ── View control ──────────────────────────────────────────────────────
        self.hide_wall    = False   # ซ่อนจุดผนัง (ไม่ลบ)
        self._wall_mask   = None    # boolean mask จุดผนัง ใน scan_pts
        self._debug_done  = False

        # ── Build UI ──────────────────────────────────────────────────────────
        self._build_ui()

        # ── Render timer (main thread, 10 Hz) ────────────────────────────────
        self._needs_render = False
        self._render_timer = QTimer()
        self._render_timer.setInterval(100)
        self._render_timer.timeout.connect(self._render)
        self._render_timer.start()

        # ── MQTT ──────────────────────────────────────────────────────────────
        self.mqtt = MqttWorker()
        self.mqtt.data_received.connect(
            self._on_data, Qt.ConnectionType.QueuedConnection)
        self.mqtt.state_received.connect(
            self._on_state, Qt.ConnectionType.QueuedConnection)
        self.mqtt.start()

    # ════════════════════════════════════════════════════════════════════════
    # UI
    # ════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        side = QVBoxLayout()
        side.setSpacing(6)

        def grp(title):
            g = QGroupBox(title)
            g.setStyleSheet(
                "QGroupBox{color:#5dade2;font-weight:bold;border:1px solid #2e4f6e;"
                "border-radius:5px;margin-top:8px;padding-top:4px;}"
                "QGroupBox::title{subcontrol-origin:margin;left:8px;}"
            )
            return g

        def btn(text, color=None):
            b = QPushButton(text)
            s = "QPushButton{color:#e0e0e0;border:1px solid #2e4f6e;border-radius:5px;padding:7px;}"
            if color:
                s += f"QPushButton{{background-color:{color};}}"
            b.setStyleSheet(s)
            return b

        def spinbox(val, mn, mx, step=0.5, dec=1):
            s = QDoubleSpinBox()
            s.setRange(mn, mx)
            s.setSingleStep(step)
            s.setDecimals(dec)
            s.setValue(val)
            s.setStyleSheet("color:#e0e0e0; background:#1a1a2e; border:1px solid #2e4f6e; border-radius:3px; padding:3px;")
            return s

        # ── Status ────────────────────────────────────────────────────────────
        g = grp("System Status")
        v = QVBoxLayout(g)
        self.lbl_status = QLabel("STATUS: READY  |  MODE: PARKING")
        self.lbl_status.setStyleSheet("font-weight:bold; color:#00bcd4;")
        self.lbl_pts    = QLabel("Points: 0")
        self.lbl_floor  = QLabel(f"FLOOR_Z: {self.floor_z:.1f} cm")
        self.lbl_floor.setStyleSheet("color:#f39c12; font-size:11px;")
        v.addWidget(self.lbl_status)
        v.addWidget(self.lbl_pts)
        v.addWidget(self.lbl_floor)
        side.addWidget(g)

        # ── Silo Dimensions ───────────────────────────────────────────────────
        g2 = grp("Silo Dimensions")
        grd = QGridLayout(g2)
        grd.addWidget(QLabel("Diameter (cm):"), 0, 0)
        self.spin_diam = spinbox(DEFAULT_DIAMETER, 1, 500, 1.0)
        self.spin_diam.valueChanged.connect(self._on_dim_changed)
        grd.addWidget(self.spin_diam, 0, 1)

        grd.addWidget(QLabel("Height (cm):"), 1, 0)
        self.spin_h = spinbox(DEFAULT_HEIGHT, 1, 1000, 1.0)
        self.spin_h.valueChanged.connect(self._on_dim_changed)
        grd.addWidget(self.spin_h, 1, 1)

        b_autosize = btn("Auto-Detect Size from Scan", "#0f3460")
        b_autosize.clicked.connect(self._auto_detect_size)
        grd.addWidget(b_autosize, 2, 0, 1, 2)

        self.lbl_capacity = QLabel(f"Capacity: {self.total_capacity:,.0f} cm³")
        self.lbl_capacity.setStyleSheet("color:#aaa; font-size:11px;")
        grd.addWidget(self.lbl_capacity, 3, 0, 1, 2)
        side.addWidget(g2)

        # ── Volume Result ─────────────────────────────────────────────────────
        g3 = grp("Volume Result")
        v3 = QVBoxLayout(g3)
        self.lbl_vol = QLabel("EMPTY SPACE: ---\nMATERIAL VOL: ---\nFILL LEVEL: ---")
        self.lbl_vol.setStyleSheet("font-size:14px; font-weight:bold; color:#2ecc71; margin:4px 0;")
        v3.addWidget(self.lbl_vol)
        b_recalc = btn("↺  Recalculate", "#145a32")
        b_recalc.clicked.connect(self.calculate_volume)
        v3.addWidget(b_recalc)
        side.addWidget(g3)

        # ── Scan Control ──────────────────────────────────────────────────────
        g4 = grp("Scan Control")
        v4 = QVBoxLayout(g4)
        b_start    = btn("▶  Start New Scan",        "#145a32")
        b_stop     = btn("■  Stop Scan",               "#7b241c")
        b_calibrate= btn("⊙  Calibrate Empty Tank",   "#0f4c75")
        b_reset    = btn("✕  RESET & HOME",            "#4a235a")
        b_start.clicked.connect(self.start_scan)
        b_stop.clicked.connect(self.stop_scan)
        b_calibrate.clicked.connect(self.calibrate_empty_tank)
        b_reset.clicked.connect(self.reset_all)
        v4.addWidget(b_start)
        v4.addWidget(b_stop)
        v4.addWidget(b_calibrate)
        v4.addWidget(b_reset)
        self.lbl_baseline = QLabel("Baseline: 0.0 cm³ (not set)")
        self.lbl_baseline.setStyleSheet("color:#5dade2; font-size:11px;")
        v4.addWidget(self.lbl_baseline)
        side.addWidget(g4)

        # ── View Options ──────────────────────────────────────────────────────
        g5 = grp("View Options")
        v5 = QVBoxLayout(g5)

        self.chk_hide_wall = QCheckBox("Hide Wall Points (r > 85% radius)")
        self.chk_hide_wall.setStyleSheet("color:#e0e0e0;")
        self.chk_hide_wall.toggled.connect(self._on_hide_wall_toggled)
        v5.addWidget(self.chk_hide_wall)

        # FLOOR_Z calibration
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color:#2e4f6e;")
        v5.addWidget(sep)

        v5.addWidget(QLabel("Manual FLOOR_Z (cm):"))
        self.spin_floor = spinbox(-DEFAULT_HEIGHT, -500, 0, 0.5)
        self.spin_floor.valueChanged.connect(self._floor_manual)
        v5.addWidget(self.spin_floor)

        b_autofloor = btn("Auto-Detect FLOOR_Z", "#0f3460")
        b_autofloor.clicked.connect(lambda: self._auto_detect_floor(silent=False))
        v5.addWidget(b_autofloor)
        side.addWidget(g5)

        # ── File ──────────────────────────────────────────────────────────────
        g6 = grp("File")
        v6 = QVBoxLayout(g6)
        b_save = btn("💾  Save CSV")
        b_open = btn("📂  Open CSV")
        b_save.clicked.connect(self.save_csv)
        b_open.clicked.connect(self.open_csv)
        v6.addWidget(b_save)
        v6.addWidget(b_open)
        side.addWidget(g6)

        side.addStretch()
        root.addLayout(side, 1)

        # ── 3D View ───────────────────────────────────────────────────────────
        self.view3d = gl.GLViewWidget()
        self.view3d.setBackgroundColor('#0a0a14')
        self.view3d.setCameraPosition(distance=150, elevation=30, azimuth=45)

        self.grid3d = gl.GLGridItem()
        self.grid3d.setSize(100, 100, 1)
        self.grid3d.translate(0, 0, self.floor_z)
        self.view3d.addItem(self.grid3d)

        self.scatter3d = gl.GLScatterPlotItem()
        self.scatter3d.setGLOptions('translucent')
        self.view3d.addItem(self.scatter3d)

        root.addWidget(self.view3d, 4)

    # ════════════════════════════════════════════════════════════════════════
    # DIMENSION HELPERS
    # ════════════════════════════════════════════════════════════════════════
    def _update_capacity(self):
        self.total_capacity = np.pi * (self.silo_radius ** 2) * self.silo_height

    def _on_dim_changed(self):
        self.silo_radius = self.spin_diam.value() / 2.0
        self.silo_height = self.spin_h.value()
        self._update_capacity()
        self.lbl_capacity.setText(f"Capacity: {self.total_capacity:,.0f} cm³")
        # อัปเดต floor_z ถ้าไม่ได้ lock
        if not self.floor_z_locked:
            self.floor_z = -self.silo_height
            self.spin_floor.blockSignals(True)
            self.spin_floor.setValue(self.floor_z)
            self.spin_floor.blockSignals(False)
            self._update_floor_ui()
        self._needs_render = True

    def _auto_detect_size(self):
        """ตรวจหาขนาด silo จาก scan ของถังเปล่า"""
        pts = self.scan_pts if self.is_scan_mode else self.parking_pts
        if len(pts) < 200:
            self._set_status("ต้องการข้อมูล ≥ 200 pts สำหรับ auto-detect")
            return

        # หา radius จาก percentile 95 ของ r
        r = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        detected_r = float(np.percentile(r, 92))

        # หา height จาก Z range
        z = pts[:, 2]
        z_top    = float(np.percentile(z, 97))
        z_bottom = float(np.percentile(z, 3))
        detected_h = abs(z_top - z_bottom)

        self.spin_diam.setValue(round(detected_r * 2, 1))
        self.spin_h.setValue(round(detected_h, 1))
        self._set_status(
            f"Auto-Detect: D={detected_r*2:.1f} cm, H={detected_h:.1f} cm"
        )

    # ════════════════════════════════════════════════════════════════════════
    # MQTT CALLBACKS
    # ════════════════════════════════════════════════════════════════════════
    def _on_state(self, state: str):
        if state == self.motor_state:
            return
        prev = self.motor_state
        self.motor_state = state
        print(f"[State] {prev} → {state}")

        if state == 'HOMING':
            self.is_scan_mode = True
            self.scan_done    = False
            self.scan_pts     = np.zeros((0, 3), dtype=np.float32)
            self._wall_mask   = None
            self._debug_done  = False
            self._set_status("HOMING...", scan_mode=True)
            self._needs_render = True

        elif state == 'TO_START':
            self.is_scan_mode = True
            self._set_status("MOVING TO START  (LiDAR blocked)", scan_mode=True)

        elif state in STATES_SCAN_ACTIVE:
            self.is_scan_mode = True
            self._set_status(f"SCANNING ({state})", scan_mode=True)

        elif state in ('PARKING', 'IDLE'):
            self.is_scan_mode  = False
            self.parking_pts   = np.zeros((0, 3), dtype=np.float32)
            label = "SCAN COMPLETE — PARKING VIEW" if prev in STATES_SCAN_ACTIVE or prev == 'PARKING' else state
            self._set_status(label, scan_mode=False)
            self._needs_render = True

    def _on_data(self, pts_list):
        if not pts_list:
            return
        try:
            new_pts = np.array(pts_list, dtype=np.float32) * 100.0  # m → cm
        except Exception as e:
            print(f"[_on_data] convert error: {e}")
            return

        if not self._debug_done and len(new_pts) > 0:
            self._debug_done = True
            print(f"[DATA] {len(new_pts)} pts | X:{new_pts[:,0].min():.1f}~{new_pts[:,0].max():.1f}"
                  f" Y:{new_pts[:,1].min():.1f}~{new_pts[:,1].max():.1f}"
                  f" Z:{new_pts[:,2].min():.1f}~{new_pts[:,2].max():.1f}")

        if self.is_scan_mode:
            # ── SCAN MODE ──────────────────────────────────────────────────────
            if self.scan_done:
                return
            rem = MAX_POINTS - len(self.scan_pts)
            if rem <= 0:
                return
            chunk = new_pts[:rem]
            self.scan_pts = chunk if len(self.scan_pts) == 0 else np.vstack((self.scan_pts, chunk))

            if not self.floor_z_locked and len(self.scan_pts) >= 500:
                self._auto_detect_floor(silent=True)

            n = len(self.scan_pts)
            self.lbl_pts.setText(f"Points: {n:,} / {MAX_POINTS:,}")
            self._needs_render = True

            if n >= MAX_POINTS:
                self.scan_done = True
                self._set_status(f"SCAN COMPLETE — คำนวณปริมาตร...", scan_mode=True)
                # ── Auto-calculate volume ──────────────────────────────────────
                QTimer.singleShot(300, self.calculate_volume)

        else:
            # ── PARKING FIFO MODE ──────────────────────────────────────────────
            self.parking_pts = (
                new_pts if len(self.parking_pts) == 0
                else np.vstack((self.parking_pts, new_pts))
            )
            if len(self.parking_pts) > PARKING_MAX_PTS:
                self.parking_pts = self.parking_pts[-PARKING_MAX_PTS:]
            self.lbl_pts.setText(f"Points: {len(self.parking_pts):,}  (parking preview)")
            self._needs_render = True

    # ════════════════════════════════════════════════════════════════════════
    # RENDER
    # ════════════════════════════════════════════════════════════════════════
    def _on_hide_wall_toggled(self, checked):
        self.hide_wall     = checked
        self._needs_render = True

    def _render(self):
        if not self._needs_render:
            return
        self._needs_render = False

        pts = self.scan_pts if self.is_scan_mode else self.parking_pts

        if len(pts) == 0:
            self.scatter3d.setData(
                pos=np.zeros((1, 3), dtype=np.float32),
                color=np.zeros((1, 4), dtype=np.float32), size=1
            )
            return

        if self.is_scan_mode:
            # คำนวณ wall mask (r > 85% radius)
            r = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
            self._wall_mask = r > (self.silo_radius * 0.86)

            if self.hide_wall:
                # แสดงเฉพาะจุดไม่ใช่ผนัง
                inner = pts[~self._wall_mask]
                if len(inner) == 0:
                    return
                display_pts = inner
            else:
                display_pts = pts

            # heat-map ตาม Z
            z = display_pts[:, 2]
            z_min, z_max = self.floor_z, self.floor_z + self.silo_height
            norm = np.clip(
                (z - z_min) / max(z_max - z_min, 1e-6), 0.0, 1.0
            ).astype(np.float32)
            colors = np.ones((len(display_pts), 4), dtype=np.float32)
            colors[:, 0] = norm
            colors[:, 1] = 1.0 - norm
            colors[:, 2] = 0.0
            colors[:, 3] = 0.86

            # จุดผนังสีเทาอ่อน (ถ้าไม่ซ่อน)
            if not self.hide_wall:
                wall_idx = np.where(self._wall_mask)[0]
                colors[wall_idx, 0] = 0.55
                colors[wall_idx, 1] = 0.55
                colors[wall_idx, 2] = 0.55
                colors[wall_idx, 3] = 0.4

        else:
            # Parking: cyan
            display_pts = pts
            colors = np.ones((len(pts), 4), dtype=np.float32)
            colors[:, 0] = 0.0
            colors[:, 1] = 0.8
            colors[:, 2] = 1.0
            colors[:, 3] = 0.7

        self.scatter3d.setData(pos=display_pts, color=colors, size=3)

    # ════════════════════════════════════════════════════════════════════════
    # FLOOR_Z
    # ════════════════════════════════════════════════════════════════════════
    def _auto_detect_floor(self, silent=False):
        pts = self.scan_pts if len(self.scan_pts) > 0 else self.parking_pts
        if len(pts) < 200:
            if not silent:
                self._set_status("ต้องการข้อมูล ≥ 200 pts")
            return
        r_sq = pts[:, 0]**2 + pts[:, 1]**2
        mask = r_sq < (self.silo_radius * 0.7) ** 2
        z_sample = pts[mask, 2] if mask.sum() >= 50 else pts[:, 2]
        self.floor_z = float(np.percentile(z_sample, 3))

        self.spin_floor.blockSignals(True)
        self.spin_floor.setValue(round(self.floor_z, 1))
        self.spin_floor.blockSignals(False)
        self._update_floor_ui()
        if not silent:
            self._set_status(f"Auto FLOOR_Z = {self.floor_z:.1f} cm")

    def _floor_manual(self, value):
        self.floor_z        = value
        self.floor_z_locked = True
        self._update_floor_ui()
        self._needs_render  = True

    def _update_floor_ui(self):
        mode = "manual" if self.floor_z_locked else "auto"
        self.lbl_floor.setText(f"FLOOR_Z: {self.floor_z:.1f} cm  ({mode})")
        self.grid3d.resetTransform()
        self.grid3d.translate(0, 0, self.floor_z)

    # ════════════════════════════════════════════════════════════════════════
    # VOLUME CALCULATION
    # ════════════════════════════════════════════════════════════════════════
    def calibrate_empty_tank(self):
        """
        Scan ถังเปล่าแล้วกด Calibrate
        โปรแกรมจะคำนวณ baseline_volume แล้วเก็บไว้
        ทุกการคำนวณต่อไปจะลบ baseline นี้ออกอัตโนมัติ
        → ถังเปล่าจะได้ 0 cm³ พอดี
        """
        if len(self.scan_pts) < 50:
            self._set_status("scan ถังเปล่าก่อน แล้วค่อยกด Calibrate")
            return

        # คำนวณ volume จาก scan ปัจจุบัน (ควรเป็นถังเปล่า)
        raw_vol = self._compute_raw_volume()
        if raw_vol is None:
            return

        self.baseline_volume = raw_vol
        self.lbl_baseline.setText(
            f"Baseline: {self.baseline_volume:,.1f} cm³  (calibrated)"
        )
        self.lbl_baseline.setStyleSheet("color:#2ecc71; font-size:11px;")
        self._set_status(
            f"Calibrated! Baseline = {self.baseline_volume:,.1f} cm³  (ถังเปล่า)"
        )
        print(f"[Calibrate] baseline_volume = {self.baseline_volume:.1f} cm³")

    def _compute_raw_volume(self):
        """คำนวณ raw volume โดยไม่ลบ baseline (ใช้ใน calibrate และ calculate)"""
        pts = self.scan_pts
        if len(pts) < 50:
            return None
        try:
            from scipy.interpolate import griddata
            r     = np.sqrt(pts[:,0]**2 + pts[:,1]**2)
            inner = pts[r <= self.silo_radius * 0.86]
            if len(inner) < 50:
                return 0.0
            x,y,z = inner[:,0],inner[:,1],inner[:,2]
            step  = max(1, len(x)//20000)
            xs,ys,zs = x[::step],y[::step],z[::step]
            xi = np.linspace(-self.silo_radius, self.silo_radius, 80)
            yi = np.linspace(-self.silo_radius, self.silo_radius, 80)
            Xg,Yg = np.meshgrid(xi,yi)
            circle = (Xg**2+Yg**2) <= self.silo_radius**2
            Zg = griddata((xs,ys),zs,(Xg,Yg),method='linear')
            if np.isnan(Zg).any():
                Zg[np.isnan(Zg)] = griddata((xs,ys),zs,(Xg,Yg),method='nearest')[np.isnan(Zg)]
            Zg[~circle] = self.floor_z
            Zg = np.maximum(Zg, self.floor_z)
            h  = Zg - self.floor_z
            h[~circle] = 0.0
            vol = float(np.trapezoid(np.trapezoid(h,yi,axis=0),xi))
            return max(0.0, min(vol, self.total_capacity))
        except Exception as e:
            print(f"[_compute_raw_volume] {e}")
            return None

    def calculate_volume(self):
        """
        คำนวณปริมาตรวัสดุด้วย 2D Trapezoidal Rule
        - ใช้เฉพาะจุดภายใน (r < 85% radius) ไม่รวมจุดผนัง
        - ลบ baseline_volume (ถังเปล่า) ออกเสมอ → ถังเปล่า = 0
        """
        pts = self.scan_pts
        if len(pts) < 50:
            self.lbl_vol.setText(
                f"EMPTY SPACE: {self.total_capacity:,.1f} cm³\n"
                f"MATERIAL VOL: 0.0 cm³\nFILL LEVEL: 0.0%"
            )
            return

        try:
            # คำนวณ raw volume ด้วย _compute_raw_volume
            raw_vol = self._compute_raw_volume()
            if raw_vol is None:
                return

            # ── ลบ baseline (volume ของถังเปล่า) ออก ─────────────────────────
            # เพื่อให้ถังเปล่า = 0 cm³ พอดี
            # และลด systematic error จาก cone shape + FLOOR_Z offset
            material_vol = max(0.0, raw_vol - self.baseline_volume)
            material_vol = min(material_vol, self.total_capacity)
            empty_space  = self.total_capacity - material_vol

            fill = material_vol / max(self.total_capacity, 1e-6) * 100.0

            baseline_str = f"  baseline={self.baseline_volume:.1f}" if self.baseline_volume > 0 else "  (no baseline)"
            self.lbl_vol.setText(
                f"EMPTY SPACE: {empty_space:,.1f} cm³\n"
                f"MATERIAL VOL: {material_vol:,.1f} cm³\n"
                f"FILL LEVEL: {fill:.1f}%"
            )
            self._set_status(
                f"Volume OK  raw={raw_vol:.1f}{baseline_str}"
            )
            print(f"[VOL] raw={raw_vol:.1f} baseline={self.baseline_volume:.1f} mat={material_vol:.1f} fill={fill:.1f}%")

        except Exception as e:
            print(f"[VOL ERROR] {e}")
            self._set_status(f"Calc Error: {e}")

    def start_scan(self):
        self.is_scan_mode   = True
        self.scan_done      = False
        self.scan_pts       = np.zeros((0, 3), dtype=np.float32)
        self.parking_pts    = np.zeros((0, 3), dtype=np.float32)
        self._wall_mask     = None
        self._debug_done    = False
        self.floor_z_locked = False
        self.lbl_vol.setText("EMPTY SPACE: ---\nMATERIAL VOL: ---\nFILL LEVEL: ---")
        self._needs_render  = True
        self._set_status("STARTING...", scan_mode=True)
        self.mqtt.publish("motor", "start")

    def stop_scan(self):
        self._set_status("STOPPED")
        self.mqtt.publish("motor", "stop")

    def reset_all(self):
        self.is_scan_mode   = False
        self.scan_done      = False
        self.scan_pts       = np.zeros((0, 3), dtype=np.float32)
        self.parking_pts    = np.zeros((0, 3), dtype=np.float32)
        self._wall_mask     = None
        self._debug_done    = False
        self.motor_state    = 'IDLE'
        self.floor_z        = -self.silo_height
        self.floor_z_locked = False
        self.spin_floor.blockSignals(True)
        self.spin_floor.setValue(self.floor_z)
        self.spin_floor.blockSignals(False)
        self._update_floor_ui()
        self.lbl_vol.setText("EMPTY SPACE: ---\nMATERIAL VOL: ---\nFILL LEVEL: ---")
        self._needs_render  = True
        self._set_status("RESET & READY  |  MODE: PARKING", scan_mode=False)
        self.mqtt.publish("motor", "home")
        self.mqtt.publish("system", "reset")

    # ════════════════════════════════════════════════════════════════════════
    # HELPERS
    # ════════════════════════════════════════════════════════════════════════
    def _set_status(self, text, scan_mode=None):
        if scan_mode is True:
            mode, color = "SCAN", "#2ecc71"
        elif scan_mode is False:
            mode, color = "PARKING", "#00bcd4"
        else:
            mode  = "SCAN" if self.is_scan_mode else "PARKING"
            color = "#2ecc71" if self.is_scan_mode else "#00bcd4"
        self.lbl_status.setText(f"STATUS: {text}  |  MODE: {mode}")
        self.lbl_status.setStyleSheet(f"font-weight:bold; color:{color};")

    # ════════════════════════════════════════════════════════════════════════
    # FILE I/O
    # ════════════════════════════════════════════════════════════════════════
    def save_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Data", "", "CSV Files (*.csv)")
        if path:
            with open(path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x_cm', 'y_cm', 'z_cm', 'floor_z_cm',
                             'silo_diameter_cm', 'silo_height_cm'])
                for row in self.scan_pts:
                    w.writerow([*row, self.floor_z,
                                self.silo_radius * 2, self.silo_height])
            self._set_status(f"Saved {len(self.scan_pts):,} pts")

    def open_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Data", "", "CSV Files (*.csv)")
        if not path:
            return
        data, fz, diam, h = [], None, None, None
        with open(path, 'r') as f:
            r = csv.reader(f)
            hdr = next(r)
            for row in r:
                data.append([float(row[0]), float(row[1]), float(row[2])])
                if fz is None:
                    try:
                        if 'floor_z_cm' in hdr:
                            fz = float(row[hdr.index('floor_z_cm')])
                        if 'silo_diameter_cm' in hdr:
                            diam = float(row[hdr.index('silo_diameter_cm')])
                        if 'silo_height_cm' in hdr:
                            h = float(row[hdr.index('silo_height_cm')])
                    except Exception:
                        pass

        self.scan_pts     = np.array(data, dtype=np.float32)
        self.is_scan_mode = True
        self.scan_done    = True

        if diam:
            self.spin_diam.setValue(diam)
        if h:
            self.spin_h.setValue(h)
        if fz is not None:
            self.floor_z        = fz
            self.floor_z_locked = True
            self.spin_floor.blockSignals(True)
            self.spin_floor.setValue(fz)
            self.spin_floor.blockSignals(False)
            self._update_floor_ui()

        self.lbl_pts.setText(f"Points: {len(self.scan_pts):,}")
        self._needs_render = True
        self._set_status(f"Loaded {len(self.scan_pts):,} pts", scan_mode=True)
        QTimer.singleShot(300, self.calculate_volume)


# ════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = SiloGUI()
    window.show()
    sys.exit(app.exec())