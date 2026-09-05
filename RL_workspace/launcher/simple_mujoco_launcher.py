import os
from pathlib import Path
import time

# Import third-party libraries
from IPython.display import clear_output
import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[2]

MJCF_PATH = ROOT / "RL_workspace" / "model" / "heuristic01_simple.xml"
MOTOR_SPEED_STEP = 0.1     # How much to increase/decrease motor speed by
MOTOR_SPEED_LIMIT = 1.0    # Max motor speed in each direction
PRINT_EVERY = 50           # Number of sim loop iterations before printing sensor readings

# Actuator names (from MJCF file)
LEFT_MOTOR = "left_motor"
RIGHT_MOTOR = "right_motor"

# Sensor names (from MJCF file)
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

# Keys
KEY_BACKSPACE = 259
KEY_UP = 265
KEY_DOWN = 264

# Global: speed of each motor
ctrl = {"left": 0.0, "right": 0.0}

def clamp(x):
    """Limit motor speed and direction to a minimum and maximum"""
    return max(-MOTOR_SPEED_LIMIT, min(MOTOR_SPEED_LIMIT, x))

def key_callback(keycode):
    """Handle a key press"""
    # MuJoCo resets the sim with backspace, but we also need to stop the motors
    if keycode == KEY_BACKSPACE:
        ctrl["left"] = ctrl["right"] = 0.0
        return

    # For each up or down arrow press, increase or decrease both motor speeds
    if keycode == KEY_UP:
        ctrl["left"]  = clamp(ctrl["left"]  + MOTOR_SPEED_STEP)
        ctrl["right"] = clamp(ctrl["right"] + MOTOR_SPEED_STEP)
    elif keycode == KEY_DOWN:
        ctrl["left"]  = clamp(ctrl["left"]  - MOTOR_SPEED_STEP)
        ctrl["right"] = clamp(ctrl["right"] - MOTOR_SPEED_STEP)
    else:
        return

# Load model into MuJoCo
model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

# Use model to get the simulation state
data  = mujoco.MjData(model)

# Get ID of actuators from MJCF names
left_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, LEFT_MOTOR)
right_motor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, RIGHT_MOTOR)

# Print IDs
print(f"Left motor ID: {left_motor_id}")
print(f"Right motor ID: {right_motor_id}")

# Resets simulation data to defaults
mujoco.mj_resetData(model, data)

# Launch MuJoCo simulator and GUI
steps = 0
with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    # Define free-look camera (control with mouse), looking at robot's back-right
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.lookat[:] = [0, 0, 0.05]
    viewer.cam.distance  = 0.8 
    viewer.cam.azimuth   = 45
    viewer.cam.elevation = -25

    # Simulation loop
    while viewer.is_running():
        step_start = time.time()

        # Write the latest ctrl values into the sim.
        data.ctrl[left_motor_id] = ctrl["left"]
        data.ctrl[right_motor_id] = ctrl["right"]

        # Advance simulation by one step (determined by timestep attribute in MJCF file)
        mujoco.mj_step(model, data)

        # Render the current simulation state
        viewer.sync()

        # Print the sensor readings every PRINT_EVERY steps
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

            # Clear output before printing so we don't flood it
            clear_output(wait=True)
            print(f"Accel:      {accel}")
            print(f"Gyro:       {gyro}")
            print(f"Mag:        {mag}")
            print(f"Orentation: {orientation}")
            print(f"Left pos:   {left_pos}")
            print(f"Right pos:  {right_pos}")
            print(f"Left vel:   {left_vel}")
            print(f"Right vel:  {right_vel}")
            print(f"TOF right:  {tof_right}")
            print(f"TOF left:   {tof_left}")
            print(f"TOF front:  {tof_front}")

        # Make sure the loop iteration time matches the MJCF timestep time (2 ms)
        # i.e. slow the simulation down so we can see the robot behave in real time
        slack = model.opt.timestep - (time.time() - step_start)
        if slack > 0:
            time.sleep(slack)