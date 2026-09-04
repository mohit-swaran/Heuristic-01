import socket
import time
import pygame

# ESP-Link Network Configuration
ESP_IP = "192.168.1.21"  # Change to your esp-link module's IP address
ESP_PORT = 23  # Default transparent serial port for esp-link

# Initialize Pygame and Joystick
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
  print("No joystick detected. Please connect your controller.")
  exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to: {joystick.get_name()}")

# Connect to esp-link via TCP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
  sock.connect((ESP_IP, ESP_PORT))
  print(f"Connected to esp-link at {ESP_IP}:{ESP_PORT}")
except Exception as e:
  print(f"Socket connection failed: {e}")
  exit()

try:
  while True:
    pygame.event.pump()

    # Read Left Joystick X-axis for turning (usually axis 0)
    turn = joystick.get_axis(0)

    # Read Triggers (Axis indices can vary by controller/OS, typically 2 for LT, 5 for RT)
    # Triggers usually range from -1.0 (released) to 1.0 (fully pressed).
    # Map each from 0.0 (released) to 1.0 (fully pressed):
    raw_lt = joystick.get_axis(2)  # Left Trigger (Backward)
    raw_rt = joystick.get_axis(5)  # Right Trigger (Forward)

    speed_back = (raw_lt + 1.0) / 2.0
    speed_fwd = (raw_rt + 1.0) / 2.0

    # Net speed: Forward minus Backward (results in -1.0 to 1.0 range)
    # Scale it by your max desired speed in m/s (e.g., 0.5 m/s max)
    max_linear_speed = 1.2
    speed = (speed_fwd - speed_back) * max_linear_speed

    # Format command string (e.g., "S:-0.30,T:0.20\n")
    command = f"S:{speed:.2f},T:{turn:.2f}\n"
    sock.sendall(command.encode("utf-8"))

    time.sleep(0.05)  # 20Hz update rate

except KeyboardInterrupt:
  print("\nStopping joystick transmission...")
  sock.close()
  pygame.quit()
