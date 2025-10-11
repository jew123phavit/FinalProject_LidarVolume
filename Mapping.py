import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

# --- MQTT Settings ---
MQTT_BROKER = '100.117.126.91' # <-- แก้เป็น IP ของ RPi
MQTT_PORT = 1883
MQTT_TOPIC_MAP = 'mapping/grid'

# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(2, 2))
# สร้าง plot แบบภาพ (imshow) รอไว้
map_plot = ax.imshow(np.zeros((2, 2)), cmap='gray_r', origin='lower', vmin=-1, vmax=100)

# --- Global variable for data ---
latest_map_data = {}

def on_message(client, userdata, msg):
    """Callback to update map data."""
    global latest_map_data
    try:
        if msg.topic == MQTT_TOPIC_MAP:
            print("--> Received map data via MQTT!") # <-- เพิ่มบรรทัดนี้
            latest_map_data = json.loads(msg.payload.decode())
    except Exception as e:
        print(f"Error processing message: {e}")

def update_plot(frame):
    """Updates the plot with the new map data."""
    if 'data' in latest_map_data:
        width = latest_map_data['width']
        height = latest_map_data['height']
        
        # แปลง list กลับเป็น numpy array 2D
        grid = np.array(latest_map_data['data']).reshape((height, width))
        
        # อัปเดตข้อมูลภาพใน plot
        map_plot.set_data(grid)
        
        # เคลียร์ข้อมูลเก่าเพื่อรอรับอัปเดตใหม่
        latest_map_data.clear()
        
    return map_plot,

def init_plot():
    ax.set_title("Accumulated 2D Lidar Map")
    ax.set_xlabel("Pixels")
    ax.set_ylabel("Pixels")
    return map_plot,

# --- Main Application Logic ---
def main():
    client = mqtt.Client()
    client.on_message = on_message
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.subscribe(MQTT_TOPIC_MAP)
        client.loop_start()
        print(f"Connected and subscribed to '{MQTT_TOPIC_MAP}'")
    except Exception as e:
        print(f"Could not connect to MQTT Broker: {e}")
        sys.exit()

    ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, blit=False, interval=500)

    plt.show()

    client.loop_stop()
    print("Application closed.")

if __name__ == '__main__':
<<<<<<< HEAD
    main()

    
=======
    main()
>>>>>>> 223f290bbd772c4c9b73e16cd2ced31418585ad3
