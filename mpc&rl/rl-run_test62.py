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

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 9]  # relative to the initial position

teb_velocities = []
teb_last_timestep = None
last_scan_data = None

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
    RADIUS = 0.075
    r_shift = -RADIUS - (30 * RADIUS * 2)
    c_shift = RADIUS + 5

    gazebo_x = x * (RADIUS * 2) + r_shift
    gazebo_y = y * (RADIUS * 2) + c_shift

    return (gazebo_x, gazebo_y)

def cmd_vel_callback(msg):
    global teb_velocities, teb_last_timestep
    teb_time = rospy.get_time()
    teb_velocities.append((teb_time, msg.linear.x, msg.linear.y, msg.angular.z))
    teb_last_timestep = (teb_time, msg.linear.x, msg.linear.y, msg.angular.z)

def scan_callback(msg):
    global last_scan_data
    last_scan_data = msg

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='test BARN navigation challenge')
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

    # Array to store the time needed for each navigation
    times = []

    for world_idx in range(62, 63):  # Loop through world indexes from 1 to 200
        for repeat in range(1000):  # Repeat each world 5 times
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

            # Pass the laser config as an argument to the launch file
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

            # Reset robot position
            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)
            collided = True

            while compute_distance(init_coor, curr_coor) > 0.1 or collided:
                gazebo_sim.reset()  # Reset to the initial position
                pos = gazebo_sim.get_model_state().pose.position
                curr_coor = (pos.x, pos.y)
                collided = gazebo_sim.get_hard_collision()
                time.sleep(1)

            # Launch the navigation stack inside the loop
            nav_stack_process = subprocess.Popen([
                'roslaunch',
                join(base_path, '..', 'jackal_helper/launch/move_base_DWA.launch'),
            ])

            # Send goal to move_base
            import actionlib
            from geometry_msgs.msg import Quaternion
            from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

            nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.frame_id = 'odom'
            mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
            mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
            mb_goal.target_pose.pose.position.z = 0
            mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

            nav_as.wait_for_server()
            nav_as.send_goal(mb_goal)

            # Start navigation and monitor progress
            curr_time = rospy.get_time()
            pos = gazebo_sim.get_model_state().pose.position
            curr_coor = (pos.x, pos.y)

            while compute_distance(init_coor, curr_coor) < 0.1:
                curr_time = rospy.get_time()
                pos = gazebo_sim.get_model_state().pose.position
                curr_coor = (pos.x, pos.y)
                time.sleep(0.01)

            start_time = curr_time
            collided = False

            last_time = rospy.get_time()
            last_pos = gazebo_sim.get_model_state().pose.position

            while compute_distance(goal_coor, curr_coor) > 1 and not collided and curr_time - start_time < 100:
                curr_time = rospy.get_time()
                pos = gazebo_sim.get_model_state().pose.position
                curr_coor = (pos.x, pos.y)
                print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" % (curr_time - start_time, *curr_coor), end="\r")
                collided = gazebo_sim.get_hard_collision()

                # actual_linear_velocity = compute_distance((last_pos.x, last_pos.y), (pos.x, pos.y)) / (
                            # curr_time - last_time)
                # actual_angular_velocity = abs(gazebo_sim.get_model_state().pose.orientation.z - last_pos.z) / (
                        # curr_time - last_time)

                last_time = curr_time
                last_pos = pos

                while rospy.get_time() - curr_time < 0.1:
                    time.sleep(0.01)

            print("\n>>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
            success = False
            nav_time = 0
            if collided:
                status = "collided"
                nav_time = 200.0  # Set 200 seconds for collisions
            elif curr_time - start_time >= 100:
                status = "timeout"
                nav_time = 200.0  # Set 200 seconds for timeouts
            else:
                status = "succeeded"
                success = True
                nav_time = curr_time - start_time  # Record the actual navigation time

            print(f"Navigation {status} with time {nav_time:.4f} (s)")
            times.append(nav_time)  # Append the time to the array

            if world_idx >= 300:  # DynaBARN environment which does not have a planned path
                path_length = GOAL_POSITION[0] - INIT_POSITION[0]
            else:
                path_file_name = join(base_path, "worlds/BARN/path_files", f"path_{world_idx}.npy")
                path_array = np.load(path_file_name)
                path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
                path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
                path_array = np.insert(path_array, len(path_array),
                                       (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]),
                                       axis=0)
                path_length = 0
                for p1, p2 in zip(path_array[:-1], path_array[1:]):
                    path_length += compute_distance(p1, p2)

            optimal_time = path_length / 2
            actual_time = curr_time - start_time
            nav_metric = int(success) * optimal_time / np.clip(actual_time, 2 * optimal_time, 8 * optimal_time)
            print(f"Navigation metric: {nav_metric:.4f}")

            with open(args.out, "a") as f:
                f.write(f"{world_idx} {int(success)} {int(collided)} {int(curr_time - start_time >= 100)} "
                        f"{curr_time - start_time:.4f} {nav_metric:.4f}\n")

            # Terminate the navigation stack process after each iteration
            nav_stack_process.terminate()
            nav_stack_process.wait()
            time.sleep(5)  # wait

            # Terminate the Gazebo simulation process after each iteration
            gazebo_process.terminate()
            gazebo_process.wait()
            time.sleep(5)  # wait

            print("All navigation  until current, times:", times)


    # After all loops, output the times array
    # print("All navigation  until current, times:", times)
