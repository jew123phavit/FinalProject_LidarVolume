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
map_resolution = 0.05 
MAP_SIZE_OPTIONS = [0.10, 0.20, 0.40, 0.60, 0.80, 2.50] 
current_map_size = max(MAP_SIZE_OPTIONS) 

# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(8, 8)) 

# *** การแก้ไขหลัก: Colormap เป็น 'binary_r' ***
# binary_r: 0 (ค่าต่ำ) -> ขาว, 100 (ค่าสูง) -> ดำ
# vmin=0: ทำให้ -1 (Unknown) ถูกปัดไปที่ค่าต่ำสุด คือ "ขาว" 
map_plot = ax.imshow(np.zeros((50, 50)), cmap='binary_r', origin='lower', vmin=0, vmax=100) 
# Marker ตำแหน่ง LiDAR เป็นวงกลมสีแดง
center_marker = ax.plot(0, 0, marker='o', color='red', markersize=8, markeredgewidth=2, label='LiDAR Position')[0]

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
    ax.set_title(f"2D Point Accumulation Map (Range: {size_m:.2f} m)", fontsize=14)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    
    major_ticks = np.arange(-size_m, size_m + 0.001, 0.5 if size_m >= 1.0 else 0.10) 
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
        
        max_extent = (width * resolution) / 2
        
        grid_flat = np.array(latest_map_data['data'])
        grid = grid_flat.reshape((height, width))
        
        # *** การแก้ไข: แปลง -1 (Unknown) ให้เป็น 0 (Free) เพื่อให้เป็นสีขาว ***
        # โค้ดนี้จะช่วยให้พื้นหลังเป็นสีขาวสะอาดตาเหมือนในภาพ Point Cloud
        grid[grid == -1] = 0 
        grid = np.clip(grid, 0, 100) # จำกัดค่าให้อยู่ในช่วง 0-100
        
        extent = [-max_extent, max_extent, -max_extent, max_extent]
        
        map_plot.set_data(grid)
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
    
    # Colorbar
    dummy_im = ax.imshow(np.array([[0, 100]]), cmap='binary_r', vmin=0, vmax=100)
    cbar = fig.colorbar(dummy_im, ax=ax, orientation='vertical', fraction=0.046, pad=0.04)
    cbar.ax.set_ylabel('Occupancy Probability (0=Free, 100=Occupied)', rotation=-90, va="bottom")


    plt.show()

    # 4. ปิดการทำงานเมื่อหน้าต่างปิด
    client.loop_stop()
    client.disconnect()
    print("Application closed.")

if __name__ == '__main__':
    main()