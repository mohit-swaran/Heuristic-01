# Heuristic-01 - Autonomous Mazesolving Robot
<img src="https://github.com/user-attachments/assets/18340c36-3ac3-4ed2-8062-c1b23e79f355" width="500"/>

## Hardware Architecture
* **Microcontroller:** STM32 (configured via STM32CubeIDE)
* **Real-Time Operating System:** FreeRTOS
* **Sensors:** 
  * Time-of-Flight (ToF) sensors (VL53L0X)
  * Inertial Measurement Unit (IMU) ISM330DHCX and MMC5983MA
  * **ESP-01 Wi-Fi Module:** Running `esp-link` for UART-to-Wi-Fi serial bridging, wireless debugging, and real-time telemetry streaming.
* **Actuators:** DC motors with encoders driven by custom PWM and PID control loops.

## Control & Telemetry Pipeline
* **Wireless Joystick Control:** Joystick inputs are captured on a host laptop and transmitted over the internet/Wi-Fi to the ESP-01.
* **Serial Bridging:** `esp-link` receives the network packets and forwards the control commands directly to the STM32's UART interface.
* **Real-Time Feedback:** Simultaneously streams telemetry data back through the same bridge for performance monitoring and debugging.

## MuJoCo Implementation
<img width="1920" height="1052" alt="Screenshot from 2026-09-05 21-38-37" src="https://github.com/user-attachments/assets/4d52ec35-9f9b-489a-8cc8-1d3a42af40ba" />

### Launching the Simulation
To launch the interactive MuJoCo viewer, run the launcher script from the `RL_workspace` directory so relative asset paths resolve correctly:

```
cd RL_workspace

# If the virtual environment does not exist,
# Create a new virtual environment named 'venv'
python3 -m venv venv

# if virtual environment exists
# Activate the new environment
source venv/bin/activate

# Launch simulation viewer
python3 launcher/simple_mujoco_launcher.py
```
