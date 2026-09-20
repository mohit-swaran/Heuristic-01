"""
Gymnasium wrapper for MuJoCo simulator that loads and runs the environment for 
Heuristic-01 (ChallengerX).

Focus (Phase 1): Free-Space Waypoint Tracking.
The robot learns to navigate to a randomized local (x, y) coordinate using differential steering.
"""

import math
import csv
from datetime import datetime
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import mujoco.viewer

class Heuristic01Env(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 144}

    def __init__(
        self,
        mjcf_path="/home/mohit/STM32CubeIDE/workspace_2.2.0/Heuristic-01/RL_workspace/model/heuristic01_phase1.xml",
        max_steps=5000,
        render_mode=None,
        alpha_yaw=0.98,
        sensor_tof_left="tof_left",
        sensor_tof_front="tof_front",
        sensor_tof_right="tof_right",
        sensor_imu_gyro="imu_gyro",
        sensor_imu_mag="imu_mag", 
        actuator_left_motor="left_motor",
        actuator_right_motor="right_motor",
        crash_dist=0.02,          
        forward_reward_coef=100.0,
        action_penalty_coef=0.01,
        action_delta_penalty_coef=0.05,
        spin_penalty_coef=0.1,
        heading_penalty_coef=3.0,
        log_csv_path="step_log.csv"  
    ):
        self.model = mujoco.MjModel.from_xml_path(str(mjcf_path))
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode

        self.log_csv_path = log_csv_path
        self._csv_file = open(self.log_csv_path, mode="w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        
        # Added target_local_x and target_local_y to CSV logging
        self._csv_writer.writerow([
            "timestamp", "step", "gt_x", "gt_y", "target_local_x", "target_local_y", 
            "dist_to_goal", "tof_l", "tof_f", "tof_r", "yaw", "yaw_rate", "reward"
        ])

        self.sensor_tof_left = sensor_tof_left
        self.sensor_tof_front = sensor_tof_front
        self.sensor_tof_right = sensor_tof_right
        self.sensor_imu_gyro = sensor_imu_gyro
        self.sensor_imu_mag = sensor_imu_mag

        self.left_motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_left_motor)
        self.right_motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_right_motor)

        self.target_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "target_waypoint")

        # PHASE 1 FIX: OBSERVATION SPACE EXPANSION
        # Layout: [tof_l, tof_f, tof_r, target_local_x, target_local_y, sin_yaw, cos_yaw, yaw_rate, prev_act_l, prev_act_r]
        obs_low = np.array([0.0, 0.0, 0.0, -2.0, -2.0, -1.0, -1.0, -10.0, -1.0, -1.0], dtype=np.float32)
        obs_high = np.array([1.0, 1.0, 1.0,  2.0,  2.0,  1.0,  1.0,  10.0,  1.0,  1.0], dtype=np.float32)
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float32)

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self.alpha_yaw = alpha_yaw
        self.crash_dist = crash_dist
        self.max_steps = max_steps
        self.forward_reward_coef = forward_reward_coef
        self.action_penalty_coef = action_penalty_coef
        self.spin_penalty_coef = spin_penalty_coef
        self.action_delta_penalty_coef = action_delta_penalty_coef
        self.heading_penalty_coef = heading_penalty_coef
        
        self._prev_action = np.zeros(2, dtype=np.float32)
        self._yaw = 0.0
        self._viewer = None
        self._step_count = 0
        self._prev_dist_to_goal = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

    def _get_obs(self):
        tof_l = self.data.sensor(self.sensor_tof_left).data[0]
        tof_f = self.data.sensor(self.sensor_tof_front).data[0]
        tof_r = self.data.sensor(self.sensor_tof_right).data[0]

        tofs = np.array([tof_l, tof_f, tof_r], dtype=np.float32)
        tofs[tofs < 0] = 1.0 
        tofs = np.clip(tofs, 0.0, 1.0)

        yaw_rate = float(self.data.sensor(self.sensor_imu_gyro).data[2])

        # --- FIX: Use Ground Truth Yaw from Quaternions ---
        w, x, y, z = self.data.qpos[3:7]
        self._yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        # --------------------------------------------------

        sin_yaw = math.sin(self._yaw)
        cos_yaw = math.cos(self._yaw)

        gt_x = float(self.data.qpos[0])
        gt_y = float(self.data.qpos[1])
        dx = self.goal_x - gt_x
        dy = self.goal_y - gt_y
        
        target_local_x = dx * math.cos(-self._yaw) - dy * math.sin(-self._yaw)
        target_local_y = dx * math.sin(-self._yaw) + dy * math.cos(-self._yaw)

        return np.array([
            tofs[0], tofs[1], tofs[2], 
            target_local_x, target_local_y,
            sin_yaw, cos_yaw, yaw_rate,
            self._prev_action[0], self._prev_action[1]
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)

        initial_yaw_offset = self.np_random.uniform(-0.2, 0.2)
        self.data.qpos[3:7] = self._euler_to_quaternion(0, 0, initial_yaw_offset)
        mujoco.mj_forward(self.model, self.data)

        # PHASE 1 FIX: Randomize Goal Coordinate (0.5m to 1.5m away)
        goal_angle = self.np_random.uniform(-math.pi, math.pi)
        goal_radius = self.np_random.uniform(0.5, 1.5)
        self.goal_x = goal_radius * math.cos(goal_angle)
        self.goal_y = goal_radius * math.sin(goal_angle)

        print(f"Reset: Goal set to (x={self.goal_x:.3f}, y={self.goal_y:.3f})")
        if self.target_site_id != -1:
            # Set Z to 0.05m so it hovers slightly above the floor
            self.model.site_pos[self.target_site_id] = [self.goal_x, self.goal_y, 0.05]

        # Call forward to update the kinematic tree with the new site position
        mujoco.mj_forward(self.model, self.data)

        gt_x = self.data.qpos[0]
        gt_y = self.data.qpos[1]

        self._prev_action = np.zeros(2, dtype=np.float32)
        self._prev_dist_to_goal = math.hypot(self.goal_x - gt_x, self.goal_y - gt_y)

        self._step_count = 0
        mag_x = self.data.sensor(self.sensor_imu_mag).data[0]
        mag_y = self.data.sensor(self.sensor_imu_mag).data[1]
        self._yaw = -math.atan2(mag_y, mag_x)

        return self._get_obs(), {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        self.data.ctrl[self.left_motor_id] = action[0]
        self.data.ctrl[self.right_motor_id] = action[1]

        action_delta = action - self._prev_action
        self._prev_action = action.copy()

        mujoco.mj_step(self.model, self.data)
        self._step_count += 1

        obs = self._get_obs()
        # Unpack the new 10-feature observation space
        tof_l, tof_f, tof_r, tgt_loc_x, tgt_loc_y, sin_yaw, cos_yaw, yaw_rate, prev_act_l, prev_act_r = obs

        gt_x = float(self.data.qpos[0])
        gt_y = float(self.data.qpos[1])

        current_dist_to_goal = math.hypot(self.goal_x - gt_x, self.goal_y - gt_y)
        progress_reward = (self._prev_dist_to_goal - current_dist_to_goal)
        self._prev_dist_to_goal = current_dist_to_goal

        target_yaw = math.atan2(self.goal_y - gt_y, self.goal_x - gt_x)
        heading_error = target_yaw - self._yaw
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        alignment = math.cos(heading_error)

        crashed = (tof_f < self.crash_dist) or (tof_r < self.crash_dist) or (tof_l < self.crash_dist)
        reached_goal = current_dist_to_goal < 0.10  # 10cm tolerance for reaching target

        if crashed:
            reward = -1000.0
            terminated = True
        elif reached_goal:
            reward = 1000.0
            terminated = True
        else:
            terminated = False

            # 1. Goal Progress Reward

            reward = self.forward_reward_coef * progress_reward
            
            # Grant a small continuous bonus purely for looking at the red sphere
            reward += 0.5 * alignment


            # 2. Reversing Penalty (Encourage forward driving instead of backing into the target)
            if action[0] < -0.1 and action[1] < -0.1:
                reward -= 0.5 

            # 3. Stability Penalties
            reward -= self.spin_penalty_coef * abs(yaw_rate)
            reward -= self.action_penalty_coef * np.sum(action**2)
            reward -= self.action_delta_penalty_coef * np.sum(action_delta**2)
            
            reward -= 0.05  # Time penalty

        truncated = self._step_count >= self.max_steps

        # Log to CSV
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        self._csv_writer.writerow([
            timestamp, self._step_count, f"{gt_x:+.4f}", f"{gt_y:+.4f}",
            f"{tgt_loc_x:+.4f}", f"{tgt_loc_y:+.4f}", f"{current_dist_to_goal:+.4f}",
            f"{tof_l:.3f}", f"{tof_f:.3f}", f"{tof_r:.3f}",
            f"{self._yaw:+.3f}", f"{yaw_rate:+.3f}", f"{reward:+.4f}"
        ])
        self._csv_file.flush()

        return obs, reward, terminated, truncated, {}

    def render(self):
        if self.render_mode != "human":
            return
        if self._viewer is None:
            self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self._viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self._viewer.cam.trackbodyid = 1
            self._viewer.cam.distance = 1.5
            self._viewer.cam.elevation = -60

        self._viewer.sync()

    def close(self):
        if hasattr(self, "_csv_file") and self._csv_file is not None:
            self._csv_file.close()
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def _euler_to_quaternion(self, roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]