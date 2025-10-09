import paho.mqtt.client as mqtt
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys

# --- MQTT Settings ---
MQTT_BROKER = '100.117.126.91' # <--- !! แก้เป็น IP Address ของ Raspberry Pi !!
MQTT_PORT = 1883

# --- Global variables to store latest data ---
latest_map_data = {}
latest_scan_points = []
latest_pose = {'x': 0, 'y': 0, 'theta': 0}
map_needs_update = True

def on_message(client, userdata, msg):
    """Callback function to process incoming MQTT messages."""
    global latest_map_data, latest_scan_points, latest_pose, map_needs_update
    
    try:
        topic = msg.topic
        payload = json.loads(msg.payload.decode())
        
        if topic == 'slam/map':
            latest_map_data = payload
            map_needs_update = True # Set flag to redraw the map
            print("Map data updated.")
        elif topic == 'slam/scan':
            # Convert ranges to numpy array for easier processing
            num_points = len(payload['ranges'])
            angles = np.linspace(payload['angle_min'], payload['angle_min'] + payload['angle_increment'] * (num_points - 1), num_points)
            ranges = np.array(payload['ranges'])
            
            # Convert polar to cartesian coordinates (relative to lidar)
            valid_indices = ranges > 0
            x = ranges[valid_indices] * np.cos(angles[valid_indices])
            y = ranges[valid_indices] * np.sin(angles[valid_indices])
            latest_scan_points = np.c_[x, y]
        elif topic == 'slam/pose':
            latest_pose = payload
            
    except Exception as e:
        print(f"Error processing message on topic {msg.topic}: {e}")


# --- Matplotlib Setup ---
fig, ax = plt.subplots(figsize=(10, 10))
map_plot = None # Placeholder for the map image plot
scan_plot, = ax.plot([], [], 'o', color='red', markersize=1) # Note the comma for unpacking
pose_plot, = ax.plot([], [], 'o', color='lime', markersize=8) # Robot center
pose_dir_plot, = ax.plot([], [], '-', color='lime', linewidth=2) # Robot direction line

def init_plot():
    """Initializes the plot elements."""
    ax.set_aspect('equal', 'box')
    ax.set_title("Real-time SLAM Visualization")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.grid(True)
    return map_plot, scan_plot, pose_plot, pose_dir_plot

def update_plot(frame):
    """Function called by FuncAnimation to update the plot."""
    global map_needs_update
    
    # 1. Draw the map (only if it has been updated)
    if map_needs_update and 'data' in latest_map_data:
        info = latest_map_data
        width, height = info['width'], info['height']
        resolution = info['resolution']
        origin_x, origin_y = info['origin_x'], info['origin_y']
        
        # Reshape data and prepare image
        map_array = np.array(info['data']).reshape((height, width))
        # Map values: 100=occupied(black), 0=free(white), -1=unknown(grey)
        img = np.zeros((height, width), dtype=np.uint8)
        img[map_array == 0] = 255 # Free space
        img[map_array == -1] = 128 # Unknown space
        
        # Plot the map as an image
        ax.imshow(img, cmap='gray', origin='lower', 
                  extent=[origin_x, origin_x + width * resolution, 
                          origin_y, origin_y + height * resolution])
        print("Map redrawn on plot.")
        map_needs_update = False

    # 2. Draw the robot's pose
    pose_x = latest_pose['x']
    pose_y = latest_pose['y']
    theta = latest_pose['theta']
    pose_plot.set_data(pose_x, pose_y)
    
    # Draw a line for robot's direction
    dir_len = 0.2 # 20cm line
    dir_x = pose_x + dir_len * np.cos(theta)
    dir_y = pose_y + dir_len * np.sin(theta)
    pose_dir_plot.set_data([pose_x, dir_x], [pose_y, dir_y])

    # 3. Transform and draw the laser scan points
    if len(latest_scan_points) > 0:
        # Create 2D rotation matrix
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)], 
                                    [np.sin(theta), np.cos(theta)]])
        # Rotate and then translate the points
        transformed_points = latest_scan_points @ rotation_matrix.T
        transformed_points[:, 0] += pose_x
        transformed_points[:, 1] += pose_y
        
        scan_plot.set_data(transformed_points[:, 0], transformed_points[:, 1])

    return scan_plot, pose_plot, pose_dir_plot

# --- Main Application Logic ---
def main():
    # Setup MQTT Client
    client = mqtt.Client()
    client.on_message = on_message
    try:
        print(f"Connecting to MQTT Broker at {MQTT_BROKER}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.subscribe([('slam/map', 0), ('slam/scan', 0), ('slam/pose', 0)])
        client.loop_start()
        print("MQTT Connected and listening.")
    except Exception as e:
        print(f"Could not connect to MQTT Broker: {e}")
        sys.exit()

    # Create and run the animation
    ani = animation.FuncAnimation(fig, update_plot, init_func=init_plot, 
                                  blit=False, interval=100, save_count=50)
    plt.show()

    # Cleanup
    client.loop_stop()
    print("Application closed.")

if __name__ == '__main__':
    main()