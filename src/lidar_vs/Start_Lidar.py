import paramiko
import time
import sys
import signal

def start_car_control():
    host = "192.168.0.100"
    user = "mxck"
    password = "mxck" 
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"--- Connecting to {host} ---")
        client.connect(hostname=host, username=user, password=password, timeout=10)
        print("✅ SSH Connection Successful!")

        shell = client.invoke_shell(term='xterm', width=80, height=24)
        time.sleep(1)

        print("--- Starting Docker ---")
        shell.send("cd ~/mxck2_ws\n")
        shell.send("./run_ros_docker.sh\n")
        time.sleep(1)
        shell.send(password + "\n")
        time.sleep(3) 

        print("🚀 Launching System (Lidar, Camera, Foxglove)...")
        cmd = "ros2 launch mxck_run mxck_run_launch.py run_camera:=true run_lidar:=true run_foxglove:=true broadcast_tf:=true\n"
        shell.send(cmd)
        
        print("\n--- SYSTEM IS RUNNING ---")
        print("Press Ctrl+C in this terminal to STOP the car and exit.")

        try:
            while True:
                if shell.recv_ready():
                    output = shell.recv(1024).decode('utf-8', errors='ignore')
                    print(output, end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\n\n🛑 Stopping the car... Sending Control+C to remote system.")
            shell.send(chr(3)) 
            time.sleep(3) 
            
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if 'shell' in locals():
            shell.close()
        client.close()
        print("\nConnection closed safely.")

if __name__ == "__main__":
    start_car_control()
    