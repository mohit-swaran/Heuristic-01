import os
from pathlib import Path
import time

# Import third-party libraries
from IPython.display import clear_output
import mujoco
import mujoco.viewer

# Dynamically resolve root project directory
ROOT = Path(__file__).resolve().parents[2]

MJCF_PATH = ROOT / "RL_workspace" / "model" / "heuristic01_simple.xml"
MOTOR_SPEED_STEP = 0.1     # Speed step increment per keypress
MOTOR_SPEED_LIMIT = 1.0    # Speed limit range [-1.0, 1.0]
PRINT_EVERY = 50           # Sim loop iterations before printing sensor readings

# Corrected Actuator names (matching heuristic01_simple.xml)
LEFT_MOTOR = "left_wheel_motor"
RIGHT_MOTOR = "right_wheel_motor"

# Sensor names (matching MJCF file)
IMU_ACCEL = "imu_accel"
IMU_GYRO = "imu_gyro"
IMU_MAG = "imu_mag"
IMU_ORIENTATION = "imu_orientation"
LEFT_WHEEL_POS = "left_wheel_pos"
RIGHT_WHEEL_POS = "right_wheel_pos"
LEFT_WHEEL_VEL = "left_wheel_vel"
RIGHT_WHEEL_VEL = "right_wheel_vel"
TOF_FRONT = "tof_front"
TOF_LEFT = "tof_left"
TOF_RIGHT = "tof_right"

# GLFW Keycodes
KEY_BACKSPACE = 259
KEY_UP = 265
KEY_DOWN = 264
KEY_LEFT = 263
KEY_RIGHT = 262

# Global speed tracker
ctrl = {"left": 0.0, "right": 0.0}

def clamp(x):
    """Limit motor speed and direction to minimum and maximum range."""
    return max(-MOTOR_SPEED_LIMIT, min(MOTOR_SPEED_LIMIT, x))

def key_callback(keycode):
    """Handle keyboard input in passive viewer."""
    if keycode == KEY_BACKSPACE:
        ctrl["left"] = ctrl["right"] = 0.0
        return

    if keycode == KEY_UP:
        ctrl["left"]  = clamp(ctrl["left"]  + MOTOR_SPEED_STEP)
        ctrl["right"] = clamp(ctrl["right"] + MOTOR_SPEED_STEP)
    elif keycode == KEY_DOWN:
        ctrl["left"]  = clamp(ctrl["left"]  - MOTOR_SPEED_STEP)
        ctrl["right"] = clamp(ctrl["right"] - MOTOR_SPEED_STEP)
    elif keycode == KEY_LEFT:
        ctrl["left"]  = clamp(ctrl["left"]  - MOTOR_SPEED_STEP)
        ctrl["right"] = clamp(ctrl["right"] + MOTOR_SPEED_STEP)
    elif keycode == KEY_RIGHT:
        ctrl["left"]  = clamp(ctrl["left"]  + MOTOR_SPEED_STEP)
        ctrl["right"] = clamp(ctrl["right"] - MOTOR_SPEED_STEP)

# Load model and data structures
model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
data = mujoco.MjData(model)

# Resolve Actuator IDs from MJCF
left_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, LEFT_MOTOR)
right_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, RIGHT_MOTOR)

# Safety check for missing IDs
if left_motor_id == -1 or right_motor_id == -1:
    raise ValueError(
        f"Actuator ID lookup failed (left_id={left_motor_id}, right_id={right_motor_id}). "
        f"Ensure '{LEFT_MOTOR}' and '{RIGHT_MOTOR}' exist under <actuator> in {MJCF_PATH}"
    )

print(f"Left motor ID: {left_motor_id}")
print(f"Right motor ID: {right_motor_id}")

# Reset simulation data to defaults
mujoco.mj_resetData(model, data)

# Launch interactive passive viewer
steps = 0
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    # Set default viewpoint camera
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [0, 0, 0.05]
    viewer.cam.distance = 0.8 
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -25

    while viewer.is_running():
        step_start = time.time()

        # Update control signals
        data.ctrl[left_motor_id] = ctrl["left"]
        data.ctrl[right_motor_id] = ctrl["right"]

        # Advance physics step
        mujoco.mj_step(model, data)

        # Sync GUI state
        viewer.sync()

        # Print sensor readings at specified interval
        steps += 1
        if steps % PRINT_EVERY == 0:
            accel = data.sensor(IMU_ACCEL).data
            gyro = data.sensor(IMU_GYRO).data
            mag = data.sensor(IMU_MAG).data
            orientation = data.sensor(IMU_ORIENTATION).data
            left_pos = data.sensor(LEFT_WHEEL_POS).data
            right_pos = data.sensor(RIGHT_WHEEL_POS).data
            left_vel = data.sensor(LEFT_WHEEL_VEL).data
            right_vel = data.sensor(RIGHT_WHEEL_VEL).data
            tof_right = data.sensor(TOF_RIGHT).data
            tof_left = data.sensor(TOF_LEFT).data
            tof_front = data.sensor(TOF_FRONT).data

            clear_output(wait=True)
            print(f"Accel:       {accel}")
            print(f"Gyro:        {gyro}")
            print(f"Mag:         {mag}")
            print(f"Orientation: {orientation}")
            print(f"Left pos:    {left_pos}")
            print(f"Right pos:   {right_pos}")
            print(f"Left vel:    {left_vel}")
            print(f"Right vel:   {right_vel}")
            print(f"TOF right:   {tof_right}")
            print(f"TOF left:    {tof_left}")
            print(f"TOF front:   {tof_front}")

        # Sync loop iteration speed with physics timestep
        slack = model.opt.timestep - (time.time() - step_start)
        if slack > 0:
            time.sleep(slack)