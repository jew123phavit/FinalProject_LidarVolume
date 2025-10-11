import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, Slider, RadioButtons 
import sys

# --- MQTT Settings ---
MQTT_BROKER = '100.117.126.91' 
MQTT_PORT = 1883
MQTT_TOPIC_MAP = 'mapping/grid'
MQTT_TOPIC_RESET = 'mapping/reset' 

# --- Global variable for data and settings ---
latest_map_data = {}
map_resolution = 0.05 # 1 pixel = 0.05 เมตร
MAP_SIZE_OPTIONS = [1.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0] 
current_map_size = max(MAP_SIZE_OPTIONS) # เริ่มต้นที่ 8.0 เมตร (ค่าสูงสุด)
# *** ตัวแปรสำหรับเก็บ Grid Map ปัจจุบัน (เพื่อให้ reset_map เข้าถึงได้) ***
current_grid = np.zeros((100, 100)) # ตั้งขนาดเริ่มต้นเผื่อไว้

# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(8, 8)) 
map_plot = ax.imshow(current_grid, cmap='binary', origin='lower', vmin=-1, vmax=100)
center_marker = ax.plot(0, 0, marker='+', color='cyan', markersize=10, markeredgewidth=2, label='LiDAR Position')[0]

plt.subplots_adjust(bottom=0.25) 

# --- MQTT Client Instance ---
client = mqtt.Client()

# --- MQTT Callbacks (ไม่มีการเปลี่ยนแปลง) ---
def on_connect(client, userdata, flags, rc):
    """Callback เมื่อเชื่อมต่อ MQTT สำเร็จ/ล้มเหลว"""
    if rc == 0:
        print(f"Connected to MQTT Broker: {MQTT_BROKER}")
        client.subscribe(MQTT_TOPIC_MAP)
        print(f"Subscribed to '{MQTT_TOPIC_MAP}'")
    else:
        print(f"Connection failed with code {rc}. Check network/broker status.")
        
def on_disconnect(client, userdata, rc):
    """Callback เมื่อตัดการเชื่อมต่อ"""
    print("Disconnected from MQTT Broker.")

def on_message(client, userdata, msg):
    """Callback เพื่ออัปเดตข้อมูลแผนที่"""
    global latest_map_data, map_resolution
    try:
        if msg.topic == MQTT_TOPIC_MAP:
            data = json.loads(msg.payload.decode())
            
            if 'resolution' in data and data['resolution'] > 0:
                map_resolution = data['resolution']
            
            latest_map_data = data
            
    except Exception as e:
        print(f"Error processing message: {e}")

# --- Plotting Functions ---

def set_plot_limits(size_m):
    """ตั้งค่าขอบเขตแกนเป็นหน่วยเมตร"""
    global current_map_size
    current_map_size = size_m
    ax.set_xlim([-size_m, size_m])
    ax.set_ylim([-size_m, size_m])
    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f"Accumulated 2D Lidar Map (Range: {size_m:.2f} m)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    
    # วาด Scale/Grid ใหม่
    if size_m <= 1.0:
        step = 0.10 
    elif size_m <= 4.0:
        step = 0.50 
    else:
        step = 1.0 

    major_ticks = np.arange(-size_m, size_m + 0.001, step)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    ax.grid(which='major', color='gray', linestyle='--', alpha=0.5)

def update_plot(frame):
    """อัปเดตกราฟด้วยข้อมูลแผนที่ใหม่"""
    global map_plot, latest_map_data, map_resolution, current_map_size, current_grid
    
    if 'data' in latest_map_data:
        width = latest_map_data['width']
        height = latest_map_data['height']
        resolution = map_resolution
        
        max_extent = (width * resolution) / 2
        
        # *** อัปเดต current_grid ด้วยข้อมูลใหม่ ***
        grid = np.array(latest_map_data['data']).reshape((height, width))
        current_grid = np.clip(grid, -1, 100) # อัปเดตตัวแปร global
        
        extent = [-max_extent, max_extent, -max_extent, max_extent]
        
        map_plot.set_data(current_grid)
        map_plot.set_extent(extent)
        
        set_plot_limits(current_map_size)
        
        latest_map_data.clear()
    
    # ถ้าไม่มีข้อมูลใหม่เข้า ก็ยังคงแสดง current_grid ล่าสุด
    return map_plot, center_marker

# --- Widget Callbacks ---

def reset_map(event):
    """ส่งคำสั่ง Reset ไปยัง RPi และเคลียร์ภาพหน้าจอทันที"""
    global current_grid, map_plot, fig
    
    if client.is_connected():
        print("Sending RESET command via MQTT (QoS 1)...")
        # *** ส่งข้อความด้วย QoS=1 ***
        client.publish(MQTT_TOPIC_RESET, payload="reset", qos=1)
        
        # *** เคลียร์ภาพหน้าจอทันที (UI Feedback) ***
        # สร้าง Grid ว่างเปล่าขนาดเท่าเดิม โดยกำหนดค่าเป็น -1 (Unknown)
        if current_grid.size > 0:
             cleared_grid = np.full(current_grid.shape, -1, dtype=current_grid.dtype)
        else:
             # หากยังไม่มีข้อมูล (ขนาด 100x100 ตามค่าเริ่มต้น)
             cleared_grid = np.full((100, 100), -1) 
             
        current_grid = cleared_grid # อัปเดตตัวแปร global
        map_plot.set_data(current_grid)
        
        # บังคับให้ Matplotlib วาดใหม่ทันที
        fig.canvas.draw_idle()
        print("Map screen cleared locally.")
    else:
        print("Cannot send RESET command: MQTT client is disconnected.")

def select_range(label):
    """Callback สำหรับ Radio Buttons (ตัวเลือกขนาดแผนที่)"""
    size_m = float(label.split()[0]) # ดึงค่าตัวเลขจาก Label
    set_plot_limits(size_m)
    print(f"Display Range set to {size_m:.2f} m.")

# --- Main Application Logic (ไม่มีการเปลี่ยนแปลง) ---
def main():
    global client, current_map_size
    
    # 1. ตั้งค่า MQTT
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start()
    except Exception as e:
        print(f"Could not connect to MQTT Broker: {e}")
        sys.exit()

    # ตั้งค่าขอบเขตเริ่มต้น
    set_plot_limits(current_map_size)
    
    # 2. สร้าง Widgets ควบคุม

    # A. ปุ่ม Reset
    ax_reset = plt.axes([0.10, 0.05, 0.1, 0.05])
    btn_reset = Button(ax_reset, 'RESET')
    btn_reset.on_clicked(reset_map)

    # B. Radio Buttons สำหรับเลือก Range/Zoom (1.0m - 8.0m)
    ax_range = plt.axes([0.70, 0.05, 0.20, 0.15])
    range_labels = [f"{s:.2f} m" for s in MAP_SIZE_OPTIONS]
    radio_range = RadioButtons(ax_range, range_labels)
    radio_range.on_clicked(select_range)
    
    initial_index = MAP_SIZE_OPTIONS.index(current_map_size)
    radio_range.set_active(initial_index)

    # 3. รัน Animation และแสดงผล
    ani = animation.FuncAnimation(fig, update_plot, blit=False, interval=500)

    plt.show()

    # 4. ปิดการทำงานเมื่อหน้าต่างปิด
    client.loop_stop()
    client.disconnect()
    print("Application closed.")

if __name__ == '__main__':
    main()