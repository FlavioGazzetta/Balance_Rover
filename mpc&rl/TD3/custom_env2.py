import gym
from gym import spaces
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion as PyQuaternion
import torch
import torch.nn as nn
import torch.optim as optim
import time

class CorrectionNetwork(nn.Module):
    def __init__(self, input_dim, output_dim=2):
        super(CorrectionNetwork, self).__init__()
        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, output_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class CustomEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()
        if not rospy.core.is_initialized():
            rospy.init_node('gym_environment', anonymous=True)

        # ROS communication setup
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.teb_cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.teb_cmd_vel_callback)

        # Define action and observation space
        scan_size = 360  # Assuming laser scan data size is 360
        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(4 + scan_size,), dtype=np.float32)  # 2 for current velocity, 2 for TEB velocity, and 360 for LaserScan data

        # Initialize state variables
        self.current_scan = np.zeros(scan_size, dtype=np.float32)
        self.current_velocity = np.zeros(2, dtype=np.float32)
        self.current_position = np.zeros(3, dtype=np.float32)  # x, y, theta
        self.teb_velocity = np.zeros(2, dtype=np.float32)  # Linear and angular velocity from TEB
        self.goal_position = np.array([-2.0, 12.0], dtype=np.float32)
        self.robot_name = 'jackal'
        self.collision_threshold = 0.2  # Distance threshold for collision detection

        # Set random seed
        self.seed()
        self.last_time = rospy.Time.now().to_sec()

        # Initialize correction network and optimizer
        input_dim = 4 + scan_size  # 2 for current velocity + 2 for TEB velocity + scan data
        self.correction_network = CorrectionNetwork(input_dim)
        self.optimizer = optim.Adam(self.correction_network.parameters(), lr=0.001)
        self.loss_fn = nn.MSELoss()

        # Record the last reset time
        self.last_reset_time = rospy.Time.now().to_sec()

    def scan_callback(self, data):
        # Process laser scan data
        scan_data = np.array(data.ranges, dtype=np.float32)
        scan_data = np.where(np.isinf(scan_data), 5, scan_data)  # Replace infinite values with 5
        scan_data = np.where(np.isnan(scan_data), 0, scan_data)  # Replace NaN values with 0
        self.current_scan = scan_data[:360]  # Assuming we're only using the first 360 degrees

    def teb_cmd_vel_callback(self, msg):
        # Ensure valid TEB velocity data is received
        if msg.linear.x != 0 or msg.angular.z != 0:
            self.teb_velocity[0] = msg.linear.x
            self.teb_velocity[1] = msg.angular.z
        else:
            print("Received zero TEB velocities")

    def model_states_callback(self, data):
        try:
            index = data.name.index(self.robot_name)
            pos = data.pose[index].position
            ori = data.pose[index].orientation
            quaternion = PyQuaternion(ori.w, ori.x, ori.y, ori.z)
            _, _, yaw = quaternion.yaw_pitch_roll
            self.current_position = np.array([pos.x, pos.y, yaw], dtype=np.float32)
            self.current_velocity[0] = data.twist[index].linear.x
            self.current_velocity[1] = data.twist[index].angular.z
        except ValueError:
            pass  # Robot not in the model list

    def get_state(self):
        """Get the current state, including current velocity, TEB velocity, and laser scan data."""
        state = np.concatenate((self.current_velocity, self.teb_velocity, self.current_scan), axis=0)
        return state

    def step(self, action):
        try:
            # 1. Use TEB planner velocity as the base action
            base_action = self.teb_velocity.copy()  # Get the current TEB planned velocity

            # 2. Use correction network to generate correction
            state = self.get_state()  # Get the current state (velocity, TEB velocity, laser scan data)
            state_tensor = torch.FloatTensor(state).unsqueeze(0)

            # Get correction
            correction = self.correction_network(state_tensor).detach().numpy().flatten()

            # Combine the base action with the correction
            corrected_action = base_action + correction

            # Clip the corrected action to the action space limits
            corrected_action = np.clip(corrected_action, self.action_space.low, self.action_space.high)

            # 3. Publish the corrected action as a Twist message
            twist = Twist()
            twist.linear.x = corrected_action[0]
            twist.angular.z = corrected_action[1]
            self.cmd_vel_pub.publish(twist)

            # 4. Wait 0.1 seconds
            rospy.sleep(0.1)

            # Ensure data is being received
            while np.all(self.current_scan == 0) or np.all(self.current_velocity == 0):
                print("No valid data received. Pausing...")
                rospy.sleep(0.1)

            # 5. Compute the reward
            reward = self.compute_reward(base_action, correction)

            # 6. Train the correction network
            self.train_network(state, correction)

            # Generate new observation (including new actual velocity and laser scan data)
            observation = self.get_state()

            # Determine if the episode is terminated (e.g., collision, goal reached)
            terminated = False  # Replace with the actual condition for termination
            truncated = False  # Replace with the condition for truncation if needed
            info = {}

            # Handle ROS time moved backwards as a termination event
            return observation, reward, terminated, truncated, info

        except rospy.exceptions.ROSTimeMovedBackwardsException as e:
            print("ROS time moved backwards, handling the exception and continuing...")
            rospy.sleep(1.0)
            # In case of ROS time moving backwards, treat this as a termination condition
            return self.get_state(), 0.0, True, False, {}

    def reset(self, seed=None, options=None):
        current_time = rospy.Time.now().to_sec()
        time_since_last_reset = current_time - self.last_reset_time

        if time_since_last_reset < 10.0:
            print(f"Reset skipped: only {time_since_last_reset} seconds since last reset.")
            return self.get_state(), {}

        self.last_reset_time = current_time

        super().reset(seed=seed)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        try:
            rospy.sleep(1)
        except rospy.ROSInterruptException:
            pass

        self.current_scan = np.zeros(360, dtype=np.float32)
        self.current_velocity = np.zeros(2, dtype=np.float32)
        self.current_position = np.zeros(3, dtype=np.float32)
        self.teb_velocity = np.zeros(2, dtype=np.float32)
        self.last_time = rospy.Time.now().to_sec()
        observation = self.get_state()
        print("Environment reset: Observation shape:", observation.shape)
        return observation, {}

    def compute_reward(self, action, correction):
        corrected_action = action + correction
        action_error = np.abs(self.current_velocity - action)  # Compare actual velocity with original TEB velocity
        total_action_error = np.sum(action_error)

        # Base reward: penalize for deviation from desired action
        reward = -np.exp(total_action_error)  # The larger the error, the lower the reward
        reward = 0.03 * reward

        # Encourage small action errors
        if np.all(action_error < 0.15):
            reward += 0.5
        elif np.all(action_error < 0.05):
            reward += 1.5

        # Calculate the direction towards the goal
        goal_direction = self.goal_position - self.current_position[:2]  # Goal direction vector (x, y)
        goal_distance = np.linalg.norm(goal_direction)  # Distance to the goal
        goal_direction /= goal_distance  # Normalize to get a unit vector

        # Calculate the robot's movement direction based on its yaw angle (orientation)
        theta = self.current_position[2]  # Robot's yaw angle (theta)
        movement_direction = np.array([np.cos(theta), np.sin(theta)])  # Direction vector based on orientation

        # Calculate alignment with goal direction
        alignment = np.dot(movement_direction, goal_direction)  # Dot product for alignment
        alignment_reward = 2.0 * alignment  # Scale alignment reward (adjust scaling factor as needed)
        reward += alignment_reward



        # Additional reward for being close and heading to the goal
        if goal_distance < 0.5:
            if alignment > 0.9:  # Almost perfectly aligned with the goal direction
                reward += 3.0  # Strong reward for being close and well-aligned
            elif alignment > 0.5:  # Moderately aligned with the goal direction
                reward += 1.5  # Moderate reward for being close and somewhat aligned
        elif goal_distance < 0.1:
            if alignment > 0.9:  # Almost perfectly aligned with the goal direction
                reward += 5.0  # Very strong reward for being very close and well-aligned
            elif alignment > 0.5:  # Moderately aligned with the goal direction
                reward += 3.0  # Strong reward for being very close and somewhat aligned

        # Penalty if far from the goal
        if goal_distance > 5.0:
            # Penalty for moving opposite to the goal direction
            if alignment < 0:
                opposite_penalty = -2.0 * (1 - alignment)  # Scale penalty (adjust scaling factor as needed)
                reward += opposite_penalty

        # Print debug information (optional)
        # print(f"Action: {action}, Current Velocity: {self.current_velocity}, Reward: {reward}")

        return reward

    def is_done(self):
        # Task completion is handled by an external navigation system, so always return False
        return False

    def train_network(self, state, correction):
        start_time = time.time()

        self.correction_network.train()
        self.optimizer.zero_grad()
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        target_correction = torch.FloatTensor(correction).unsqueeze(0)
        predicted_correction = self.correction_network(state_tensor)
        loss = self.loss_fn(predicted_correction, target_correction)
        loss.backward()
        self.optimizer.step()

        training_time = time.time() - start_time
        # print(f"Training time: {training_time:.4f} seconds")

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

gym.envs.registration.register(
    id='CustomEnv2-v0',
    entry_point='custom_env2:CustomEnv',
    max_episode_steps=1000,  # 设置每个 episode 的最大步数
)
