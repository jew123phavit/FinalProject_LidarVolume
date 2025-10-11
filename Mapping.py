import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, RadioButtons
import sys

# --- MQTT Settings (ไม่มีการเปลี่ยนแปลง) ---
MQTT_BROKER = '100.117.126.91' 
MQTT_PORT = 1883
MQTT_TOPIC_MAP = 'mapping/grid'
MQTT_TOPIC_RESET = 'mapping/reset' 

# --- Global variable for data and settings ---
latest_map_data = {}
map_resolution = 0.05 # สมมติฐานเริ่มต้น: 1 pixel = 0.05 เมตร
MAP_SIZE_OPTIONS = [0.10, 0.20, 0.40, 0.60, 0.80, 2.50] 
current_map_size = max(MAP_SIZE_OPTIONS) 

# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(8, 8)) 

# *** การแก้ไขหลัก: เปลี่ยน Colormap เป็น 'jet' (น้ำเงิน -> แดง) ***
# vmin/vmax คือช่วงค่าของ Grid Map (ปกติ -1 ถึง 100)
map_plot = ax.imshow(np.zeros((50, 50)), cmap='jet', origin='lower', vmin=0, vmax=100)
center_marker = ax.plot(0, 0, marker='+', color='cyan', markersize=10, markeredgewidth=2, label='LiDAR Position')[0]

plt.subplots_adjust(bottom=0.25) 

# --- MQTT Client Instance ---
client = mqtt.Client()

# --- MQTT Callbacks (ไม่มีการเปลี่ยนแปลง) ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT Broker: {MQTT_BROKER}")
        client.subscribe(MQTT_TOPIC_MAP)
        print(f"Subscribed to '{MQTT_TOPIC_MAP}'")
    else:
        print(f"Connection failed with code {rc}. Check network/broker status.")
        
def on_disconnect(client, userdata, rc):
    print("Disconnected from MQTT Broker.")

def on_message(client, userdata, msg):
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
    """ตั้งค่าขอบเขตแกนเป็นหน่วยเมตรและปรับ Grid Lines"""
    global current_map_size
    current_map_size = size_m
    ax.set_xlim([-size_m, size_m])
    ax.set_ylim([-size_m, size_m])
    ax.set_aspect('equal', adjustable='box')
    ax.set_title(f"Accumulated 2D Lidar Map (Range: {size_m:.2f} m)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    
    # ปรับช่วง Grid Lines
    major_ticks = np.arange(-size_m, size_m + 0.001, 0.5 if size_m >= 1.0 else 0.10) 
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    ax.grid(which='major', color='gray', linestyle='--', alpha=0.5)

def update_plot(frame):
    """อัปเดตกราฟด้วยข้อมูลแผนที่ใหม่ พร้อมการจัดการสีตามเงื่อนไข"""
    global map_plot, latest_map_data, map_resolution, current_map_size
    
    if 'data' in latest_map_data:
        width = latest_map_data['width']
        height = latest_map_data['height']
        resolution = map_resolution
        
        max_extent = (width * resolution) / 2
        
        grid_flat = np.array(latest_map_data['data'])
        grid = grid_flat.reshape((height, width))
        
        # 1. *** การปรับปรุงสีตามเงื่อนไข (ตามความต้องการของคุณ) ***
        # Grid Map ที่ได้รับมามักมีค่า:
        # -1 = Unknown (ยังไม่สแกน)
        # 0-50 = Free Space (พื้นที่ว่างเปล่า)
        # 51-100 = Occupied (วัตถุ)
        
        # สร้าง array ที่จะใช้สำหรับความโปร่งใส (Alpha)
        alpha_map = np.ones_like(grid, dtype=float)
        
        # - ถ้าเป็น Unknown (-1) หรือ Free Space (0-50) ให้ตั้งค่า Alpha เป็น 0 (โปร่งใส/ไม่มีสี)
        #   (นี่คือส่วนที่ทำให้หลังวัตถุหรือพื้นที่ว่างเปล่าไม่แสดงสี)
        alpha_map[grid < 50] = 0.0 
        alpha_map[grid == -1] = 0.0
        
        # - ส่วนที่เป็น Occupied (>50) ให้มีสี (Alpha = 1)
        
        # 2. ผนวก Alpha Channel เข้ากับ Colormap
        # ดึง Colormap ที่ใช้ ('jet')
        cmap = map_plot.get_cmap()
        
        # แปลงค่า Grid ให้เป็นสี (RGBA) ตาม Colormap
        colored_map = cmap((grid - map_plot.get_clim()[0]) / (map_plot.get_clim()[1] - map_plot.get_clim()[0]))
        
        # ใส่ค่าความโปร่งใส (Alpha) ที่คำนวณไว้
        colored_map[..., 3] = alpha_map 

        # 3. อัปเดตข้อมูลภาพใน plot
        extent = [-max_extent, max_extent, -max_extent, max_extent]
        map_plot.set_data(colored_map) # ส่งข้อมูลสีที่ปรับ Alpha แล้ว
        map_plot.set_extent(extent)
        
        set_plot_limits(current_map_size)
        
        latest_map_data.clear()
        
    return map_plot, center_marker

# --- Widget Callbacks (ไม่มีการเปลี่ยนแปลง) ---
def reset_map(event):
    if client.is_connected():
        print("Sending RESET command via MQTT...")
        client.publish(MQTT_TOPIC_RESET, payload="reset", qos=0)
    else:
        print("Cannot send RESET command: MQTT client is disconnected.")

def select_range(label):
    size_m = float(label.split()[0]) 
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

    set_plot_limits(current_map_size)
    
    # 2. สร้าง Widgets ควบคุม
    ax_reset = plt.axes([0.10, 0.05, 0.1, 0.05])
    btn_reset = Button(ax_reset, 'RESET')
    btn_reset.on_clicked(reset_map)

    ax_range = plt.axes([0.70, 0.05, 0.20, 0.15])
    range_labels = [f"{s:.2f} m" for s in MAP_SIZE_OPTIONS]
    radio_range = RadioButtons(ax_range, range_labels)
    radio_range.on_clicked(select_range)
    
    initial_index = MAP_SIZE_OPTIONS.index(current_map_size)
    radio_range.set_active(initial_index)

    # 3. รัน Animation และแสดงผล
    ani = animation.FuncAnimation(fig, update_plot, blit=False, interval=500)
    
    # *** เพิ่ม Colorbar เพื่อแสดงความหมายของเฉดสี ***
    # สร้าง Dummy Image สำหรับ Colorbar (สำคัญมาก)
    dummy_im = ax.imshow(np.array([[0, 100]]), cmap='jet', vmin=0, vmax=100)
    cbar = fig.colorbar(dummy_im, ax=ax, orientation='vertical', fraction=0.046, pad=0.04)
    cbar.ax.set_ylabel('Occupancy Probability (0=Free, 100=Occupied)', rotation=-90, va="bottom")


    plt.show()

    # 4. ปิดการทำงานเมื่อหน้าต่างปิด
    client.loop_stop()
    client.disconnect()
    print("Application closed.")

if __name__ == '__main__':
    main()