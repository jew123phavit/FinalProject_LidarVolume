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
# *** ปรับปรุง: ตัวเลือกขนาดแผนที่ใหม่ ***
MAP_SIZE_OPTIONS = [1.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0] 
current_map_size = max(MAP_SIZE_OPTIONS) # เริ่มต้นที่ 8.0 เมตร (ค่าสูงสุด)

# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(8, 8)) 
# ใช้ 'binary' หรือ 'gray_r' เพื่อแสดงผลแผนที่ (0=ว่าง, 100=สิ่งกีดขวาง)
map_plot = ax.imshow(np.zeros((50, 50)), cmap='binary', origin='lower', vmin=-1, vmax=100)
center_marker = ax.plot(0, 0, marker='+', color='cyan', markersize=10, markeredgewidth=2, label='LiDAR Position')[0]

plt.subplots_adjust(bottom=0.25) # เว้นที่ว่างด้านล่างสำหรับ Widgets

# --- MQTT Client Instance ---
client = mqtt.Client()

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
            
            # ดึงค่า resolution มาจาก payload (ถ้ามี)
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
    # สร้าง Ticks ตามขอบเขตที่เลือก
    if size_m <= 1.0:
        step = 0.10 # สำหรับ 1m, ใช้ 0.1m ต่อช่อง
    elif size_m <= 4.0:
        step = 0.50 # สำหรับ 3m, 4m, ใช้ 0.5m ต่อช่อง
    else: # สำหรับขนาดใหญ่ (5m ขึ้นไป)
        step = 1.0 # ใช้ 1.0m ต่อช่อง เพื่อไม่ให้ตารางถี่เกินไป

    major_ticks = np.arange(-size_m, size_m + 0.001, step)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    ax.grid(which='major', color='gray', linestyle='--', alpha=0.5)

def update_plot(frame):
    """อัปเดตกราฟด้วยข้อมูลแผนที่ใหม่"""
    global map_plot, latest_map_data, map_resolution, current_map_size
    
    if 'data' in latest_map_data:
        width = latest_map_data['width']
        height = latest_map_data['height']
        resolution = map_resolution
        
        # คำนวณขอบเขตของแผนที่ในหน่วยเมตร
        max_extent = (width * resolution) / 2
        
        # แปลง list กลับเป็น numpy array 2D
        # np.clip ใช้เพื่อจำกัดค่าให้แผนที่ดูชัดเจนยิ่งขึ้น (0=ว่าง, 100=สิ่งกีดขวาง)
        grid = np.array(latest_map_data['data']).reshape((height, width))
        grid = np.clip(grid, -1, 100)
        
        # Matplotlib extent: [x_min, x_max, y_min, y_max]
        extent = [-max_extent, max_extent, -max_extent, max_extent]
        
        # อัปเดตข้อมูลภาพใน plot
        map_plot.set_data(grid)
        map_plot.set_extent(extent)
        
        # ตั้งค่าแกน X/Y ให้เป็นหน่วยเมตรตามขนาดที่ผู้ใช้เลือก
        set_plot_limits(current_map_size)
        
        # เคลียร์ข้อมูลเก่าเพื่อรอรับอัปเดตใหม่
        latest_map_data.clear()
        
    # center_marker เป็นตัวบอกตำแหน่งเริ่มต้น (0,0) ของ LiDAR/หุ่นยนต์
    return map_plot, center_marker

# --- Widget Callbacks ---

def reset_map(event):
    """Callback สำหรับปุ่ม Reset: ส่งคำสั่ง MQTT ไปยัง RPi"""
    if client.is_connected():
        print("Sending RESET command via MQTT...")
        # ส่งข้อความควบคุมไปที่ RPi เพื่อสั่งให้รีเซ็ต Grid Map
        client.publish(MQTT_TOPIC_RESET, payload="reset", qos=0)
    else:
        print("Cannot send RESET command: MQTT client is disconnected.")

def select_range(label):
    """Callback สำหรับ Radio Buttons (ตัวเลือกขนาดแผนที่)"""
    size_m = float(label.split()[0]) # ดึงค่าตัวเลขจาก Label
    set_plot_limits(size_m)
    print(f"Display Range set to {size_m:.2f} m.")

# --- Main Application Logic ---
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
    # สร้าง Label จากตัวเลข (เช่น "1.00 m")
    range_labels = [f"{s:.2f} m" for s in MAP_SIZE_OPTIONS]
    radio_range = RadioButtons(ax_range, range_labels)
    radio_range.on_clicked(select_range)
    
    # เลือกค่าเริ่มต้นที่ 8.0 m (ค่าสูงสุด)
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