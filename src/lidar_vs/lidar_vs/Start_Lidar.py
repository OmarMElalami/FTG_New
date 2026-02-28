import subprocess
import signal
import shutil


def start_car_control():
    host = "192.168.0.100"
    user = "mxck"
    password = "mxck"

    remote_cmd = (
        "cd ~/mxck2_ws && "
        "echo mxck | sudo -S ./run_ros_docker.sh && "
        "sleep 3 && "
        "ros2 launch mxck_run mxck_run_launch.py "
        "run_camera:=true run_lidar:=true run_foxglove:=true broadcast_tf:=true"
    )

    if shutil.which("sshpass"):
        cmd = [
            "sshpass", "-p", password,
            "ssh", "-o", "StrictHostKeyChecking=no",
            f"{user}@{host}",
            remote_cmd,
        ]
    else:
        cmd = [
            "ssh", "-o", "StrictHostKeyChecking=no",
            f"{user}@{host}",
            remote_cmd,
        ]

    print(f"--- Connecting to {host} ---")
    proc = None
    try:
        proc = subprocess.Popen(cmd)
        print("\n--- SYSTEM IS RUNNING ---")
        print("Press Ctrl+C in this terminal to STOP the car and exit.")
        proc.wait()
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping the car...")
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
        print("\nConnection closed safely.")


def main():
    start_car_control()

if __name__ == "__main__":
    main()
