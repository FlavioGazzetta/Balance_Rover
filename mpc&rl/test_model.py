import time
import argparse
import subprocess
import os
from os.path import join
import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from gazebo_simulation import GazeboSimulation
import torch
import torch.nn as nn

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 400)  # Adjust state_dim to 724 if needed
        self.layer_2 = nn.Linear(400, 300)
        self.layer_3 = nn.Linear(300, action_dim)
        self.max_action = max_action

    def forward(self, x):
        x = torch.relu(self.layer_1(x))
        x = torch.relu(self.layer_2(x))
        x = self.max_action * torch.tanh(self.layer_3(x))
        return x

# Update the state_dim to the correct size
state_dim = 724
action_dim = 2   #Two actions (linear and angular velocities)
max_action = 1.0 

actor_model = Actor(state_dim, action_dim, max_action)
# Constants
INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 9]  # relative to the initial position

# Global variables
teb_velocities = []
teb_last_timestep = None
last_scan_data = None

# Load the trained actor model(use your file path)
actor_model_path = '/home/z/jackal_ws1/src/the-barn-challenge/TD3/models/TD3_CustomEnv2-v0_0_actor'

actor_model.eval()

# Utility functions
def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5
    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift
    return (gazebo_x, gazebo_y)


def flatten_state(scan_data, teb_velocities, current_velocities):
    # Ensure you have the correct sizes here
    assert len(scan_data) == 360, "Expected 360 laser scan data points"
    assert len(teb_velocities) == 2, "Expected 2 TEB velocity values"
    assert len(current_velocities) == 2, "Expected 2 current velocity values"

    # Combine the scan data and velocities into a single flat array
    return np.concatenate([current_velocities, teb_velocities, scan_data])





def cmd_vel_callback(msg):
    global teb_velocities, teb_last_timestep
    teb_time = rospy.get_time()
    teb_velocities.append((teb_time, msg.linear.x, msg.linear.y, msg.angular.z))
    teb_last_timestep = (teb_time, msg.linear.x, msg.linear.y, msg.angular.z)

def scan_callback(msg):
    global last_scan_data
    last_scan_data = np.array(msg.ranges, dtype=np.float32)
    last_scan_data = np.clip(last_scan_data, 0, 10)  # Clip the values for safety

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Test BARN navigation challenge with TD3 model')
    parser.add_argument('--gui', action="store_true", help='Launch Gazebo with GUI')
    parser.add_argument('--laser', action="store_true", help='Enable the front laser sensor')
    parser.add_argument('--out', type=str, default="out.txt", help='Output file for navigation results')
    args = parser.parse_args()

    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')

    if args.laser:
        os.environ["JACKAL_LASER"] = "1"
        os.environ["JACKAL_LASER_MODEL"] = "ust10"
        os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
    else:
        os.environ["JACKAL_LASER"] = "0"

    times = []  # Array to store navigation times

    for world_idx in range(1, 201):  # Loop through world indexes from 1 to 200
        for repeat in range(5):  # Repeat each world 5 times
            print(f"Starting navigation for world {world_idx}, repetition {repeat + 1}/5")

            if world_idx < 300:  # static environment from 0-299
                world_name = f"BARN/world_{world_idx}.world"
                INIT_POSITION = [-2.25, 3, 1.57]
                GOAL_POSITION = [0, 10]
            elif world_idx < 360:  # Dynamic environment from 300-359
                world_name = f"DynaBARN/world_{world_idx - 300}.world"
                INIT_POSITION = [11, 0, 3.14]
                GOAL_POSITION = [-20, 0]
            else:
                raise ValueError(f"World index {world_idx} does not exist")

            print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" % world_name)
            os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")

            launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
            world_name = join(base_path, "worlds", world_name)

            gazebo_process = subprocess.Popen([
                'roslaunch',
                launch_file,
                'world_name:=' + world_name,
                'config:=' + ('front_laser' if args.laser else 'base'),
                'gui:=' + ("true" if args.gui else "false")
            ])

            time.sleep(5)  # wait for Gazebo to be created

            rospy.init_node('gym', anonymous=True)
            rospy.set_param('/use_sim_time', True)

            # GazeboSimulation provides useful interface to communicate with gazebo
            gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)

            init_coor = (INIT_POSITION[0], INIT_POSITION[1])
            goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])

            rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
            rospy.Subscriber('/front/scan', LaserScan, scan_callback)

            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = True

            while compute_distance(init_coor, curr_coor) > 0.1 or collided:
                gazebo_sim.reset()  # Reset to the initial position
                pos = gazebo_sim.get_model_state().pose.position
                curr_coor = (pos.x, pos.y)
                collided = gazebo_sim.get_hard_collision()
                time.sleep(1)

            curr_time = rospy.get_time()
            start_time = curr_time
            collided = False
            last_pos = gazebo_sim.get_model_state().pose.position

            while compute_distance(goal_coor, curr_coor) > 1 and not collided and curr_time - start_time < 100:
                # Flatten the state for the model
                velocity_data = [gazebo_sim.get_model_state().twist.linear.x, gazebo_sim.get_model_state().twist.angular.z]
                teb_velocity = [teb_last_timestep[1], teb_last_timestep[3]] if teb_last_timestep else [0, 0]
                state = flatten_state(last_scan_data, teb_velocity, velocity_data)
                state_tensor = torch.FloatTensor(state).unsqueeze(0)

                # Get action from actor model
                action = actor_model(state_tensor).detach().numpy().flatten()

                # Send velocity command
                twist = Twist()
                twist.linear.x = action[0]
                twist.angular.z = action[1]
                rospy.Publisher('/cmd_vel', Twist, queue_size=10).publish(twist)

                curr_time = rospy.get_time()
                pos = gazebo_sim.get_model_state().pose.position
                curr_coor = (pos.x, pos.y)
                print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" % (curr_time - start_time, *curr_coor), end="\r")
                collided = gazebo_sim.get_hard_collision()

            print("\n>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
            success = False
            nav_time = 0
            if collided:
                status = "collided"
                nav_time = 200.0
            elif curr_time - start_time >= 100:
                status = "timeout"
                nav_time = 200.0
            else:
                status = "succeeded"
                success = True
                nav_time = curr_time - start_time

            print(f"Navigation {status} with time {nav_time:.4f} (s)")
            times.append(nav_time)

            with open(args.out, "a") as f:
                f.write(f"{world_idx} {int(success)} {int(collided)} {int(curr_time - start_time >= 100)} "
                        f"{curr_time - start_time:.4f}\n")

            gazebo_process.terminate()
            gazebo_process.wait()
            time.sleep(5)

    # Print all times at the end
    print("All navigation times:", times)
