#! /usr/bin/env python
from __future__ import print_function

import csv
import glob
import os
import signal
import sys

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class joint_trajectory_player:
    def __init__(self, namespace=""):
        rospy.init_node('joint_trajectory_player')
        namespace = namespace.split(":=")[-1]
        self.action_server_ns = rospy.get_param('player/action_server')
        self.folder = rospy.get_param(namespace + "/folder")
        self.trajectory_file = rospy.get_param(namespace + "/trajectory_file")
        self.roslaunch = rospy.get_param(namespace + "/roslaunch", False)

        self.client = actionlib.SimpleActionClient(
            self.action_server_ns, FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def handle_abort(self, signum, frame):
        rospy.loginfo("Aborted")
        self.client.cancel_goal()
        exit(0)

    def play(self):
        filename = self.select_trajectory()
        rospy.loginfo("Loading from %s.", filename)
        traj_points = list()
        try:
            with open(filename, mode='r') as csv_file:
                csv_reader = csv.reader(csv_file)
                read_column_names = False
                for row in csv_reader:
                    if not read_column_names:
                        read_column_names = True
                        # Last column is the timestamp
                        joints = row[:-1]
                        continue
                    traj_points.append(list(map(float, row)))
        except IOError:
            rospy.logerr("Could not open file. Aborting.")
            exit(1)
        rospy.loginfo("Loaded trajectory with %i joints and %i data points.",
                      len(joints), len(traj_points))

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joints
        for p in traj_points:
            point = JointTrajectoryPoint()
            point.positions = p[:-1]
            point.time_from_start = rospy.Duration(p[-1])
            goal.trajectory.points.append(point)
        rospy.loginfo("Sending to server. Trajectory length is %f seconds.", traj_points[-1][-1])
        self.client.send_goal_and_wait(goal)
        rospy.loginfo("Finished. Got result:\n%s", self.client.get_result())
        return

    def select_trajectory(self):
        if self.trajectory_file:
            return self.trajectory_file
        if not self.folder:
            rospy.logfatal("Either 'trajectory_file' or 'folder' needs to be specified.")
        files = list()
        os.chdir(self.folder)
        for file in glob.glob("*.csv"):
            files.append(file)
        rospy.loginfo("The folder contains %i files:", len(files))
        for it, file in enumerate(files):
            if self.roslaunch:
                rospy.loginfo("- %s/%s", self.folder, file)
            else:
                rospy.loginfo("%i: %s", it, file)
        if not self.roslaunch:
            response = input("Choose a file: ")
            if not response:
                return files[-1]
            else:
                return files[int(response)]
        else:
            rospy.loginfo(
                "Using a launch file does not allow interaction. Copy a filename and set 'trajectory_file'.")
            exit(0)


if __name__ == '__main__':
    player = joint_trajectory_player(sys.argv[1])
    signal.signal(signal.SIGINT, player.handle_abort)
    player.play()
