#! /usr/bin/env python

from __future__ import print_function
import copy
import csv
import signal
from datetime import datetime

import rospy
from sensor_msgs.msg import JointState


class joint_trajectory_recorder:
    def __init__(self, suffix=""):
        rospy.init_node('joint_state_recorder')
        self.joint_states_topic = rospy.get_param('recorder/joint_states_topic')
        self.joints = rospy.get_param('recorder/joints')
        self.record_rate = rospy.get_param('recorder/record_rate')

        self.joint_states = list()
        self.joint_state = [None] * len(self.joints)
        self.start_ros_time = 0
        self.start_wall_time = 0
        self.suffix = suffix
        self.recording = False

        # Subscribe to joint_states
        rospy.Subscriber(self.joint_states_topic, JointState, self.joint_states_callback)
        # rospy returns 0 sometimes. For reference:
        # https://answers.ros.org/question/300956/rospytimenow-returns-0/
        while rospy.Time.now().to_sec() == 0.0:
            rospy.sleep(0.01)

    def joint_states_callback(self, js):
        for joint in self.joints:
            try:
                index = js.name.index(joint)
            except ValueError:
                continue
            self.joint_state[self.joints.index(joint)] = js.position[index]

    def handle_abort(self, signum, frame):
        rospy.loginfo("Aborted")
        self.recording = False

        fields = copy.deepcopy(self.joints)
        fields.append("time")

        traj_name = "/tmp/" + self.start_wall_time.isoformat() + "_trajectory"
        if len(self.suffix):
            traj_name += suffix
        traj_name += ".csv"

        with open(traj_name, "w") as f:
            write = csv.writer(f)
            write.writerow(fields)
            write.writerows(self.joint_states)
        exit(0)

    def record(self):
        self.recording = True
        self.start_ros_time = rospy.get_rostime()
        self.start_wall_time = datetime.now()
        self.joint_states = list()
        rate = rospy.Rate(self.record_rate)
        rospy.loginfo("Started recording at time %s at rate %f Hz. End with ctrl+c.",
                      self.start_wall_time.isoformat(), self.record_rate)

        while self.recording:
            traj_duration = rospy.get_rostime() - self.start_ros_time
            if len(self.joint_state) == len(self.joints):
                state_with_time = copy.deepcopy(self.joint_state)
                state_with_time.append(traj_duration.to_sec())
                self.joint_states.append(state_with_time)
            rospy.loginfo_throttle(1, "%f seconds passed. Recorded %i data points.",
                                   traj_duration.to_sec(), len(self.joint_states))
            rate.sleep()


if __name__ == '__main__':
    recorder = joint_trajectory_recorder()
    signal.signal(signal.SIGINT, recorder.handle_abort)
    recorder.record()
