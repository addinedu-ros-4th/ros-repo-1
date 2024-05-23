#! /usr/bin/env python3
 
import time 
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np 

from src.include.astar_planning import *

def make_target_pose_msg(navigator, target):
    try:
        quat_target = quaternion_from_euler(0, 0, target[2])
    except:
        quat_target = (0.0,0.0,0.0,1.0)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = target[0]
    goal_pose.pose.position.y = target[1]
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = quat_target[0]
    goal_pose.pose.orientation.y = quat_target[1]
    goal_pose.pose.orientation.z = quat_target[2]
    goal_pose.pose.orientation.w = quat_target[3]

    return goal_pose

def angle_pose_to_pose(pose):
    x, X, y, Y = pose
    angle = np.arctan2(Y-y, X-x)
    return angle

def main():
    rclpy.init()

    sx, sy = 0.0, 0.0
    gx, gy = 5.67, -3.28

    goal_poses = []
    goal_poses.append([sx,sy])
    goal_poses.append([gx,gy])

    for pose, next_pose in zip(goal_poses[:-1], goal_poses[1:]):
        pose_to_pose = [pose[0], next_pose[0], pose[1], next_pose[1]]
        quat = angle_pose_to_pose(pose_to_pose)
        pose.append(quat)

    goal_msg_list = []
    navigator = BasicNavigator()
    for target in goal_poses:
        goal_msg_list.append(make_target_pose_msg(navigator, target))
        
    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_msg_list)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=50.0):
            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=10000.0):
                navigator.cancelTask()


    result = navigator.getResult()

    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()