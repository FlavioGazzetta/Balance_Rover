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

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.teb_cmd_vel_sub = rospy.Subscriber('/move_base/cmd_vel', Twist, self.teb_cmd_vel_callback)

        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(364,), dtype=np.float32)

        self.current_scan = np.zeros(360, dtype=np.float32)
        self.current_velocity = np.zeros(2, dtype=np.float32)
        self.current_position = np.zeros(3, dtype=np.float32)  # x, y, theta
        self.teb_velocity = np.zeros(2, dtype=np.float32)  # Linear and angular velocity from TEB
        self.goal_position = np.array([5.0, 5.0], dtype=np.float32)
        self.robot_name = 'jackal'
        self.collision_threshold = 0.2  # Distance threshold for collision detection
        self.seed()
        self.last_time = rospy.Time.now().to_sec()
        self.stop_threshold = 0.01  # Velocity threshold to consider stopped
        self.stop_duration = 10.0  # Duration in seconds
        self.stop_start_time = None
        self.stopped = False

        # Initialize the correction network and optimizer
        input_dim = 364  # 2 for current velocity + 360 for laser scan + 2 for TEB velocity
        self.correction_network = CorrectionNetwork(input_dim)
        self.optimizer = optim.Adam(self.correction_network.parameters(), lr=0.001)
        self.loss_fn = nn.MSELoss()

        # Variables to hold the TEB command and actual velocities for correction
        self.last_teb_command = np.zeros(2, dtype=np.float32)
        self.last_actual_velocity = np.zeros(2, dtype=np.float32)

        # Time of the last reset
        self.last_reset_time = rospy.Time.now().to_sec()
        self.collision_detected = False

    def scan_callback(self, data):
        # Process the laser scan data
        scan_data = np.array(data.ranges, dtype=np.float32)
        scan_data = np.where(np.isinf(scan_data), 5, scan_data)  # Replace infinite values with 5
        scan_data = np.where(np.isnan(scan_data), 0, scan_data)  # Replace NaN values with 0
        scan_data = scan_data[:360]
        self.current_scan = scan_data

    def teb_cmd_vel_callback(self, msg):
        self.teb_velocity[0] = msg.linear.x
        self.teb_velocity[1] = msg.angular.z

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
            pass  # The robot is not in the list of models

    def step(self, action):
        # Publish the TEB command as is
        twist = Twist()
        twist.linear.x = self.teb_velocity[0]
        twist.angular.z = self.teb_velocity[1]
        self.cmd_vel_pub.publish(twist)

        # Wait for the robot to execute the command
        rospy.sleep(0.1)

        # Collect the actual velocity after executing the TEB command
        actual_velocity = self.current_velocity.copy()

        # Store the last TEB command and actual velocity for correction in the next step
        self.last_teb_command = self.teb_velocity.copy()
        self.last_actual_velocity = actual_velocity

        # Calculate correction for the next TEB command
        state = np.concatenate((self.last_actual_velocity, self.current_scan, self.last_teb_command))
        state_tensor = torch.FloatTensor(state).unsqueeze(0)

        # Get correction from the neural network
        correction = self.correction_network(state_tensor).detach().numpy().flatten()

        # Apply the correction to the next TEB command
        corrected_action = self.teb_velocity + correction


        # Create and publish the Twist message with the corrected action
        twist.linear.x = corrected_action[0]
        twist.angular.z = corrected_action[1]
        self.cmd_vel_pub.publish(twist)

        observation = np.concatenate((self.current_velocity, self.current_scan, self.teb_velocity))
        reward = self.compute_reward(action, correction)
        done = self.is_done()
        info = {}

        # Logging for debugging
        print(
            f"Action: {action}, Correction: {correction}, Corrected Action: {corrected_action}, Reward: {reward}, Done: {done}")

        # Train the correction network
        self.train_network(state, correction)

        return observation, reward, done, info

    def reset(self, seed=None, options=None):
        current_time = rospy.Time.now().to_sec()
        time_since_last_reset = current_time - self.last_reset_time

        if not self.collision_detected and time_since_last_reset < 10.0:
            print(f"Reset skipped: only {time_since_last_reset} seconds since last reset.")
            return np.concatenate((self.current_velocity, self.current_scan, self.teb_velocity)), {}

        self.last_reset_time = current_time
        self.collision_detected = False

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
        self.last_teb_command = np.zeros(2, dtype=np.float32)
        self.last_actual_velocity = np.zeros(2, dtype=np.float32)
        self.last_time = rospy.Time.now().to_sec()
        observation = np.concatenate((self.current_velocity, self.current_scan, self.teb_velocity))
        print("Environment reset: Observation shape:", observation.shape)
        return observation, {}

    def compute_reward(self, action, correction):
        corrected_action = action + correction
        action_error = np.abs(corrected_action - self.current_velocity)
        total_action_error = np.sum(action_error)
        reward = -np.exp(total_action_error)
        reward = 0.1 * reward

        if np.all(action_error < 0.25):
            reward += 0.5
        elif np.all(action_error < 0.05):
            reward += 1.5

        print(f"Computed reward: {reward} for expected action: {action}, actual action: {self.current_velocity}")
        return reward

    def is_done(self):
        collision = self.check_collision_condition()
        goal_distance = np.linalg.norm(self.goal_position - self.current_position[:2])

        # Check if a collision is detected
        if collision:
            self.collision_detected = True
            print("Done: Collision detected")
            return True

        # Check if the goal is reached
        if goal_distance < 0.5:
            print("Done: Goal reached")
            return True

        # Check if the robot has stopped moving
        if np.abs(self.current_velocity[0]) < self.stop_threshold and np.abs(
                self.current_velocity[1]) < self.stop_threshold:
            if not self.stopped:
                # Start the timer when the robot first stops
                self.stopped = True
                self.stop_start_time = rospy.Time.now().to_sec()
            elif rospy.Time.now().to_sec() - self.stop_start_time > self.stop_duration:
                print("Done: Stopped moving for more than 10 seconds")
                return True
        else:
            # Reset the timer if the robot moves again
            self.stopped = False
            self.stop_start_time = None

        print("Not done")
        return False

    def check_collision_condition(self):
        if self.current_scan.size > 0:
            min_distance = np.min(self.current_scan)
            collision = min_distance < self.collision_threshold
            print(
                f"Collision check: {collision}, Min scan distance: {min_distance}, Threshold: {self.collision_threshold}")
            return collision
        else:
            print("Collision check: No scan data available")
            return False

    def train_network(self, state, correction):
        self.correction_network.train()
        self.optimizer.zero_grad()
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        target_correction = torch.FloatTensor(correction).unsqueeze(0)
        predicted_correction = self.correction_network(state_tensor)
        loss = self.loss_fn(predicted_correction, target_correction)
        loss.backward()
        self.optimizer.step()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]


gym.envs.registration.register(
    id='CustomEnv1-v0',
    entry_point='custom_env1:CustomEnv',
)
