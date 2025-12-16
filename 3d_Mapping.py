import sys
import json
import numpy as np
import paho.mqtt.client as mqtt
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame)
from PyQt6.QtCore import QTimer, pyqtSignal, QObject
import pyqtgraph.opengl as gl
import pyqtgraph as pg

# --- MQTT Config ---
MQTT_BROKER = '100.117.126.91' # IP ของ RPi
MQTT_PORT = 1883
MQTT_TOPIC_DATA = 'mapping/points_3d'
MQTT_TOPIC_CMD = 'control/command'

class MqttWorker(QObject):
    data_received = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self):
        try:
            self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.client.loop_start()
        except Exception as e:
            print(f"MQTT Connection Error: {e}")

    def on_connect(self, client, userdata, flags, rc):
        print("MQTT Connected!")
        client.subscribe(MQTT_TOPIC_DATA)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            if 'points_m' in payload:
                self.data_received.emit(payload['points_m'])
        except Exception as e:
            print(f"Data Error: {e}")

    def send_command(self, target, action):
        payload = json.dumps({"target": target, "action": action})
        self.client.publish(MQTT_TOPIC_CMD, payload, qos=1)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR 3D Volume Scanner (PyQtGraph)")
        self.resize(1000, 700)

        # Data Storage
        self.all_points = np.zeros((0, 3)) # เก็บจุดทั้งหมดตรงนี้
        
        # Setup UI
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)

        # 1. 3D Viewport (Left Side)
        self.view_widget = gl.GLViewWidget()
        self.view_widget.setCameraPosition(distance=3, elevation=30, azimuth=45)
        
        # Add Grid
        g = gl.GLGridItem()
        g.setSize(x=2, y=2, z=2)
        g.setSpacing(x=0.1, y=0.1, z=0.1)
        self.view_widget.addItem(g)
        
        # Add Scatter Plot Item (Points)
        self.scatter = gl.GLScatterPlotItem(pos=np.zeros((1,3)), size=3, color=(0,1,0,0.5), pxMode=True)
        self.view_widget.addItem(self.scatter)
        
        self.layout.addWidget(self.view_widget, stretch=4)

        # 2. Control Panel (Right Side)
        self.ctrl_panel = QFrame()
        self.ctrl_panel.setFixedWidth(200)
        self.ctrl_layout = QVBoxLayout(self.ctrl_panel)
        
        # Motor Controls
        self.ctrl_layout.addWidget(QLabel("<b>Motor Control</b>"))
        self.btn_motor_start = QPushButton("Start Motor")
        self.btn_motor_stop = QPushButton("Stop Motor")
        self.btn_motor_start.clicked.connect(lambda: self.send_cmd('motor', 'start'))
        self.btn_motor_stop.clicked.connect(lambda: self.send_cmd('motor', 'stop'))
        self.ctrl_layout.addWidget(self.btn_motor_start)
        self.ctrl_layout.addWidget(self.btn_motor_stop)
        
        self.ctrl_layout.addSpacing(20)

        # LiDAR Controls
        self.ctrl_layout.addWidget(QLabel("<b>LiDAR Data</b>"))
        self.btn_lidar_resume = QPushButton("Resume Data")
        self.btn_lidar_pause = QPushButton("Pause Data")
        self.btn_lidar_resume.clicked.connect(lambda: self.send_cmd('lidar', 'start'))
        self.btn_lidar_pause.clicked.connect(lambda: self.send_cmd('lidar', 'stop'))
        self.ctrl_layout.addWidget(self.btn_lidar_resume)
        self.ctrl_layout.addWidget(self.btn_lidar_pause)

        self.ctrl_layout.addSpacing(20)

        # System Controls
        self.btn_clear = QPushButton("Clear / Reset")
        self.btn_clear.setStyleSheet("background-color: #ffcccc;")
        self.btn_clear.clicked.connect(self.clear_data)
        self.ctrl_layout.addWidget(self.btn_clear)
        
        self.lbl_points = QLabel("Points: 0")
        self.ctrl_layout.addWidget(self.lbl_points)

        self.ctrl_layout.addStretch()
        self.layout.addWidget(self.ctrl_panel, stretch=1)

        # Setup MQTT
        self.mqtt = MqttWorker()
        self.mqtt.data_received.connect(self.update_points)
        self.mqtt.start()

    def send_cmd(self, target, action):
        print(f"Sending: {target} -> {action}")
        self.mqtt.send_command(target, action)

    def clear_data(self):
        self.all_points = np.zeros((0, 3))
        self.scatter.setData(pos=self.all_points)
        self.lbl_points.setText(f"Points: 0")
        self.send_cmd('system', 'reset')

    def update_points(self, new_points_list):
        # Convert to numpy
        new_pts = np.array(new_points_list)
        
        # Accumulate (สะสมจุด)
        if len(self.all_points) == 0:
            self.all_points = new_pts
        else:
            self.all_points = np.vstack((self.all_points, new_pts))
        
        # Update Visualization
        # สีขึ้นอยู่กับความสูง (Z)
        color = np.ones((len(self.all_points), 4))
        color[:, 0] = 0 # R
        color[:, 1] = 1 # G
        color[:, 2] = 0 # B
        color[:, 3] = 0.6 # Alpha
        
        # ไล่สีตามแกน Z (สมมติสูง 0-1 เมตร)
        z_vals = self.all_points[:, 2]
        color[:, 0] = np.clip(z_vals, 0, 1) 
        color[:, 1] = 1 - np.clip(z_vals, 0, 1)

        self.scatter.setData(pos=self.all_points, color=color)
        self.lbl_points.setText(f"Points: {len(self.all_points)}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())