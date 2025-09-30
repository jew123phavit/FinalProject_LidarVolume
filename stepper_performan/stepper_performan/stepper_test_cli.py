# โปรแกรมนี้ถูกปรับโครงสร้างใหม่ให้สามารถรันผ่าน `ros2 run` ได้อย่างถูกต้อง

import subprocess
import time
import sys

# --- ค่าคงที่และ Topic (เหมือนเดิม) ---
TOPIC_ANGLE = "/target_angle"
TOPIC_DIR = "/target_dir"
TOPIC_SPEED = "/target_speed"
MSG_TYPE_FLOAT = "std_msgs/msg/Float32"
MSG_TYPE_STRING = "std_msgs/msg/String"

# --- ฟังก์ชันหลัก (เหมือนเดิม) ---
def run_ros_command(command, wait_after_command=0.1):
    print(f"Executing: {command}")
    try:
        subprocess.run(command, shell=True, check=True, capture_output=True, text=True)
        time.sleep(wait_after_command)
    except subprocess.CalledProcessError as e:
        print(f"!!! ERROR executing command: {command}")
        print(f"--- stderr ---\n{e.stderr}")
        print("Please make sure the ROS2 nodes are running correctly.")
        sys.exit(1)

def set_speed(speed_dps):
    cmd = f"ros2 topic pub --once {TOPIC_SPEED} {MSG_TYPE_FLOAT} \"{{data: {speed_dps}}}\""
    run_ros_command(cmd)

def set_direction(direction):
    cmd = f"ros2 topic pub --once {TOPIC_DIR} {MSG_TYPE_STRING} \"{{data: '{direction}'}}\""
    run_ros_command(cmd)

def move_to_angle(angle_deg, wait_time):
    cmd = f"ros2 topic pub --once {TOPIC_ANGLE} {MSG_TYPE_FLOAT} \"{{data: {angle_deg}}}\""
    run_ros_command(cmd)
    print(f"Moving to {angle_deg}°, waiting for {wait_time} seconds...")
    time.sleep(wait_time)
    print("...Move complete.")

# --- โหมดการทดสอบ (เหมือนเดิม) ---
def mode_1_sweep_360_steps():
    print("\n--- Starting Mode 1: 360° Sweep (90° steps) ---")
    speed = 100.0; wait_per_step = (90 / speed) + 1.0; set_speed(speed)
    print("\n>>> Forward Trip (CW)"); set_direction('CW')
    move_to_angle(90, wait_per_step); move_to_angle(180, wait_per_step)
    move_to_angle(270, wait_per_step); move_to_angle(0, wait_per_step)
    print("\n>>> Return Trip (CCW)"); set_direction('CCW')
    move_to_angle(270, wait_per_step); move_to_angle(180, wait_per_step)
    move_to_angle(90, wait_per_step); move_to_angle(0, wait_per_step)
    print("--- Mode 1 Finished ---")

def mode_2_sweep_180():
    print("\n--- Starting Mode 2: 0-180-0° Sweep ---")
    speed = 120.0; wait_time = (180 / speed) + 1.5; set_speed(speed)
    print("\n>>> Forward Trip 0 -> 180 (CW)"); set_direction('CW'); move_to_angle(180, wait_time)
    print("\n>>> Return Trip 180 -> 0 (CCW)"); set_direction('CCW'); move_to_angle(0, wait_time)
    print("--- Mode 2 Finished ---")

def mode_3_sweep_360_full():
    print("\n--- Starting Mode 3: 360° Full Sweep (Simulated) ---")
    speed = 120.0; wait_time = (360 / speed) + 4.5; set_speed(speed)
    print("\n>>> Forward Trip 0 -> 360 (CW)"); set_direction('CW'); move_to_angle(359, wait_time)
    print("\n>>> Return Trip 360 -> 0 (CCW)"); set_direction('CCW'); move_to_angle(0, wait_time)
    print("--- Mode 2 Finished ---")

def auto_mode():
    print("\n--- Starting Auto Mode (Alternating Mode 2 and 1) ---"); print("Press Ctrl+C to stop.")
    try:
        while True:
            mode_2_sweep_180(); print("\nPausing for 3 seconds..."); time.sleep(3)
            mode_1_sweep_360_steps(); print("\nPausing for 3 seconds..."); time.sleep(3)
    except KeyboardInterrupt:
        print("\n--- Auto Mode Stopped by user ---")

# --- (สำคัญ) แก้ไขส่วนนี้ใหม่ทั้งหมด ---
def main_menu():
    # ส่วนของเมนูเหมือนเดิม
    while True:
        print("\n===== Stepper Motor Test Menu =====")
        print("1. Mode 1: 360° Sweep (90° steps)")
        print("2. Mode 2: 0-180-0° Sweep")
        print("3. Mode 3: 360° Full Sweep")
        print("4. Auto Mode (Alternate Mode 2 and 1)")
        print("5. Exit")
        choice = input("Please select a mode (1-5): ")

        if choice == '1': mode_1_sweep_360_steps()
        elif choice == '2': mode_2_sweep_180()
        elif choice == '3': mode_3_sweep_360_full()
        elif choice == '4': auto_mode()
        elif choice == '5': print("Exiting program."); break
        else: print("Invalid choice, please try again.")

def main():
    """
    นี่คือ "หน้าที่หลัก" ที่ ROS2 จะมาเรียก
    """
    # เราแค่เรียกฟังก์ชันเมนูของเราจากตรงนี้
    main_menu()

if __name__ == "__main__":
    # บรรทัดนี้เพื่อให้เรารันด้วย `python3` ได้เหมือนเดิมด้วย
    main()