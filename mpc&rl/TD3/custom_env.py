import gym
from gym import spaces
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from pyquaternion import Quaternion as PyQuaternion

class CustomEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(CustomEnv, self).__init__()
        if not rospy.core.is_initialized():
            rospy.init_node('gym_environment', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)

        self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(362,), dtype=np.float32)

        self.current_scan = np.zeros(360, dtype=np.float32)
        self.current_velocity = np.zeros(2, dtype=np.float32)
        self.current_position = np.zeros(3, dtype=np.float32)  # x, y, theta
        self.goal_position = np.array([5.0, 5.0], dtype=np.float32)
        self.robot_name = 'jackal'
        self.collision_threshold = 0.2  # Distance threshold for collision detection
        self.seed()
        self.last_time = rospy.Time.now().to_sec()
        self.stop_threshold = 0.01  # Velocity threshold to consider stopped
        self.stop_duration = 5.0  # Duration in seconds
        self.stop_start_time = None
        self.stopped = False

    def scan_callback(self, data):
        # Process the laser scan data
        scan_data = np.array(data.ranges, dtype=np.float32)
        scan_data = np.where(np.isinf(scan_data), 5, scan_data)  # Replace infinite values with 5
        scan_data = np.where(np.isnan(scan_data), 0, scan_data)  # Replace NaN values with 0

        # Keep the entire scan or ensure to always have a fixed size subset
        # Here assuming to use only 360 points for simplicity
        scan_data = scan_data[:360]
        self.current_scan = scan_data

    # Update the CustomEnv class with the above scan_callback method

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
        twist = Twist()
        twist.linear.x = action[0]
        twist.angular.z = action[1]
        self.cmd_vel_pub.publish(twist)

        current_time = rospy.Time.now().to_sec()
        time_elapsed = current_time - self.last_time
        self.last_time = current_time

        try:
            rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            pass

        observation = np.concatenate((self.current_velocity, self.current_scan))
        reward = self.compute_reward(action)
        done = self.is_done()
        info = {}
        print(f"Step completed: Action: {action}, Reward: {reward}, Done: {done}, Observation shape: {observation.shape}")
        return observation, reward, done, info

    def reset(self, seed=None, options=None):
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
        self.last_time = rospy.Time.now().to_sec()
        observation = np.concatenate((self.current_velocity, self.current_scan))
        print("Environment reset: Observation shape:", observation.shape)
        return observation, {}

    def compute_reward(self, action):
        action_error = np.abs(action - self.current_velocity)
        total_action_error = np.sum(action_error)
        reward = -np.exp(total_action_error)

        if np.all(action_error < 0.05):
            reward += 1.0


        print(f"Computed reward: {reward} for expected action: {action}, actual action: {self.current_velocity}, goal distance: {goal_distance}")
        return reward

    def is_done(self):
        collision = self.check_collision_condition()
        goal_distance = np.linalg.norm(self.goal_position - self.current_position[:2])

        # Check if a collision is detected
        if collision:
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
                print("Done: Stopped moving for more than 5 seconds")
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
            print(f"Collision check: {collision}, Min scan distance: {min_distance}, Threshold: {self.collision_threshold}")
            return collision
        else:
            print("Collision check: No scan data available")
            return False

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

gym.envs.registration.register(
    id='CustomEnv-v0',
    entry_point='custom_env:CustomEnv',
)
