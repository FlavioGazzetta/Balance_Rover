import numpy as np
import torch
import torch.nn as nn
import gym
import argparse
import os
import subprocess
import time
from utils import ReplayBuffer
from TD3 import TD3
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Quaternion
from pyquaternion import Quaternion as PyQuaternion
import rospy
from threading import Thread, Lock
import rospkg
from os.path import join

# Ensure the custom environment is registered
import custom_env1

INIT_POSITION = [-2.25, 3, 1.57]
GOAL_POSITION = [-1.5, 8]
goal_lock = Lock()

class AdaptiveController(nn.Module):
    def __init__(self, input_dim, output_dim=2):  # Ensure output_dim is 2
        super(AdaptiveController, self).__init__()
        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, output_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)  # Output should have shape (batch_size, 2)
        return x

def reset_robot_initial_position(robot_name):
    initial_pose = ModelState()
    initial_pose.model_name = robot_name
    initial_pose.pose = Pose()
    initial_pose.pose.position.x = INIT_POSITION[0]
    initial_pose.pose.position.y = INIT_POSITION[1]
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation = Quaternion(*PyQuaternion(axis=[0, 0, 1], angle=INIT_POSITION[2]).elements)

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        response = set_model_state(initial_pose)
        if not response.success:
            print(f"Failed to reset robot position: {response.status_message}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5





import actionlib
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction



def launch_gazebo_and_teb(base_path, goal_position, front_laser=True, gui=True):
    """
    Launch Gazebo with a specific world and front_laser config, and then launch the TEB planner.

    Args:
        base_path (str): The base path where the launch files are located.
        goal_position (list): The goal position [x, y, theta].
        front_laser (bool): Whether to enable the front laser (True) or not (False).
        gui (bool): Whether to launch Gazebo with the GUI.

    Returns:
        tuple: Process handles for Gazebo and the TEB planner.
    """
    config = 'front_laser' if front_laser else 'base'
    launch_file_gazebo = join(base_path, 'launch', 'gazebo_launch_laser.launch')
    world_path = join(base_path, "worlds/BARN/world_1.world")

    gazebo_args = [
        'roslaunch', launch_file_gazebo,
        f'world_name:={world_path}',
        f'config:={config}',
        f'gui:={str(gui).lower()}'
    ]
    gazebo_process = subprocess.Popen(gazebo_args)
    time.sleep(5)  # Ensure Gazebo is fully launched


    launch_file_teb = join(base_path, 'launch/move_base_TEB.launch')
    teb_process = subprocess.Popen(['roslaunch', launch_file_teb])

    # Send navigation goal
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    nav_as.wait_for_server()

    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom'
    mb_goal.target_pose.pose.position.x = goal_position[0]
    mb_goal.target_pose.pose.position.y = goal_position[1]
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    nav_as.send_goal(mb_goal)
    return gazebo_process, teb_process



def navigation_loop(policy, env, replay_buffer, max_action, expl_noise):
    global goal_lock
    while not rospy.is_shutdown():
        state, _ = env.reset()
        state = np.array(state, dtype=np.float32)
        episode_reward = 0
        done = False
        start_time = rospy.get_time()

        while not done:
            with goal_lock:
                state = np.array(state, dtype=np.float32)
                action = policy.select_action(state)

                if np.isnan(action).any() or np.isinf(action).any() or not np.isfinite(max_action) or max_action <= 0:
                    print("Invalid values in action or max_action. Using random action.")
                    max_action_safe = min(1.0, max_action) if np.isfinite(max_action) and max_action > 0 else 1.0
                    action = np.random.uniform(low=-max_action_safe, high=max_action_safe, size=action.shape)
                else:
                    noise = np.random.normal(0, max_action * expl_noise, size=action.shape)
                    action = (action + noise).clip(-max_action, max_action)

                action = np.array(action[:2], dtype=np.float32)

                next_state, reward, done, _ = env.step(action)
                next_state = np.array(next_state, dtype=np.float32)
                done_bool = float(done)

                print(f"State shape: {state.shape}, Next state shape: {next_state.shape}")
                replay_buffer.add(state, action, next_state, reward, done_bool)
                state = next_state
                episode_reward += reward

                print(f"Action: {action}, Reward: {reward}, Done: {done}, State: {state.shape}")

                if compute_distance(env.current_position[:2], GOAL_POSITION) < 0.5:
                    print("Goal reached!")
                    break

                # Check for other termination conditions like timeout or collisions
                curr_time = rospy.get_time()
                if curr_time - start_time > 100:  # Example timeout
                    print("Timeout reached!")
                    break

        print(f"Episode Reward: {episode_reward}")
        reset_robot_initial_position('jackal')
        time.sleep(1)  # Wait before the next episode


def training_loop(policy, replay_buffer, batch_size):
    while not rospy.is_shutdown():
        # Ensure enough samples are in the replay buffer before training
        if replay_buffer.size < batch_size:
            print(f"Waiting for replay buffer to fill... ({replay_buffer.size}/{batch_size})")
            time.sleep(1)
            continue
        policy.train(replay_buffer, batch_size)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", default="TD3")
    parser.add_argument("--env", default="CustomEnv1-v0")
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--start_timesteps", default=25e3, type=int)
    parser.add_argument("--eval_freq", default=5e3, type=int)
    parser.add_argument("--max_timesteps", default=1e6, type=int)
    parser.add_argument("--expl_noise", default=0.1, type=float)
    parser.add_argument("--batch_size", default=256, type=int)
    parser.add_argument("--discount", default=0.99, type=float)
    parser.add_argument("--tau", default=0.005, type=float)
    parser.add_argument("--policy_noise", default=0.2)
    parser.add_argument("--noise_clip", default=0.5)
    parser.add_argument("--policy_freq", default=2, type=int)
    parser.add_argument("--save_model", action="store_true")
    parser.add_argument("--load_model", default="")
    args = parser.parse_args()

    file_name = f"{args.policy}_{args.env}_{args.seed}"
    print("---------------------------------------")
    print(f"Policy: {args.policy}, Env: {args.env}, Seed: {args.seed}")
    print("---------------------------------------")

    if not os.path.exists("./results"):
        os.makedirs("./results")

    if args.save_model and not os.path.exists("./models"):
        os.makedirs("./models")

    # Initialize ROS node
    rospy.init_node('td3_training', anonymous=True)

    # Launch Gazebo and TEB planner
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')
    gazebo_process, teb_process = launch_gazebo_and_teb(base_path, GOAL_POSITION)

    env = gym.make(args.env)
    env.seed(args.seed)
    env.action_space.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    state_dim = env.observation_space.shape[0]
    action_dim = 2  # Only 2-dimensional actions are required: linear and angular velocity
    max_action = float(env.action_space.high[0])

    kwargs = {
        "state_dim": state_dim,
        "action_dim": action_dim,
        "max_action": max_action,
        "discount": args.discount,
        "tau": args.tau,
    }

    if args.policy == "TD3":
        kwargs["policy_noise"] = args.policy_noise * max_action
        kwargs["noise_clip"] = args.noise_clip * max_action
        kwargs["policy_freq"] = args.policy_freq
        policy = TD3(**kwargs)
    elif args.policy == "DDPG":
        policy = DDPG(**kwargs)

    if args.load_model != "":
        policy_file = file_name if args.load_model == "default" else args.load_model
        policy.load(f"./models/{policy_file}")

    replay_buffer = ReplayBuffer(state_dim, action_dim)

    # Start navigation in a separate thread
    nav_thread = Thread(target=navigation_loop, args=(policy, env, replay_buffer, max_action, args.expl_noise))
    nav_thread.start()

    # Start training in the main thread
    training_loop(policy, replay_buffer, args.batch_size)

    nav_thread.join()

    # Cleanup
    gazebo_process.terminate()
    teb_process.terminate()

