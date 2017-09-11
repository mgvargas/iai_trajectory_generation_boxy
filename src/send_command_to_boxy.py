#!/usr/bin/env python
# Copyright (c) 2017 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Minerva Gabriela Vargas Gleason <minervavargasg@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
from iai_trajectory_generation_boxy.msg import ProjectedGraspingActionResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import reset_naive_sim
import thread
import copy


class VelCommands:
    def __init__(self):
        self.received = False
        self.integrator_max = 5

        self.pub = rospy.Publisher('whole_body_controller/velocity_cmd', JointState, queue_size=10)
        self.pub_error_p = rospy.Publisher('command/error_p', Float32MultiArray, queue_size=10)
        self.pub_error_v = rospy.Publisher('command/error_v', Float32MultiArray, queue_size=10)
        self.pub_int_p = rospy.Publisher('command/int_p', Float32MultiArray, queue_size=10)
        self.pub_int_v = rospy.Publisher('command/int_v', Float32MultiArray, queue_size=10)
        self.pub_velocity = rospy.Publisher('/simulator/commands', JointState, queue_size=3)

        rospy.Subscriber("projected_grasping_server/result", ProjectedGraspingActionResult, self.traj_callback)
        self.joint_list = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

    def joint_callback(self, joints):
        self.joint_names = joints.name
        joint_values = joints.position
        joint_vel = joints.velocity
        self.joint_pos = {}
        self.joint_vel = {}
        self.v_lock = thread.allocate_lock()
        with self.v_lock:
            for i, joint in enumerate(self.joint_names):
                if "neck" not in joint and "gripper" not in joint:
                    self.joint_pos[joint] = joint_values[i]
                    self.joint_vel[joint] = joint_vel[i]

    def traj_callback(self, msg):
        self.trajectory = msg.result.selected_trajectory.trajectory
        if len(self.trajectory) > 0:
            self.received = True
        print 'got it'

    def calculate(self):
        if self.received is False:
            rospy.sleep(0.2)
        else:
            reset_naive_sim.reset_simulator()
            print "Trajectory received"
            self.received = False
            desired_pos = {}
            desired_vel = {}
            num_joints = len(self.trajectory[0].name)
            self.P_vel = [1]*num_joints
            self.P_pos = [1]*num_joints
            self.I_vel = [0.0]*num_joints
            self.I_pos = [0.0]*num_joints
            self.integrator_max = 100
            traj_init_time = self.trajectory[0].header.stamp
            start_time = rospy.Time.now()

            for step in self.trajectory:
                boxy_command = JointState()
                integrator_pos = [0.0] * num_joints
                integrator_vel = [0.0] * num_joints
                traj_stamp = step.header.stamp
                joint_names = []

                for i, joint in enumerate(step.name):
                    if "neck" not in joint and "gripper" not in joint:
                        desired_pos[joint] = step.position[i]
                        desired_vel[joint] = step.velocity[i]
                        joint_names.append(joint)

                now = rospy.Time.now()
                time_elapsed = now - start_time

                while time_elapsed <= (traj_stamp - traj_init_time + rospy.Duration.from_sec(0.1)):
                    error_p = Float32MultiArray()
                    # error_v = Float32MultiArray()
                    # int_p = Float32MultiArray()
                    # int_v = Float32MultiArray()
                    error_p.layout.dim.append(MultiArrayDimension())
                    error_p.layout.dim.append(MultiArrayDimension())
                    error_p.layout.dim[0].label = 'error_vel'
                    error_p.layout.dim[0].size = len(joint_names)
                    error_p.layout.dim[0].stride = 0
                    error_v = copy.deepcopy(error_p)
                    int_p = copy.deepcopy(error_p)
                    int_v = copy.deepcopy(error_p)

                    for n, joint in enumerate(joint_names):
                        try:
                            with self.v_lock:
                                error_pos = desired_pos[joint] - self.joint_pos[joint]
                                error_vel = desired_vel[joint] - self.joint_vel[joint]
                        except KeyError:
                            print 'Skipping joint', joint
                        else:
                            integrator_pos[n] += error_pos
                            if integrator_pos[n] >= self.integrator_max:
                                integrator_pos[n] = self.integrator_max
                            elif integrator_pos[n] < -self.integrator_max:
                                integrator_pos[n] = -self.integrator_max

                            integrator_vel[n] += error_vel
                            if integrator_vel[n] >= self.integrator_max:
                                integrator_vel[n] = self.integrator_max
                            elif integrator_vel[n] < -self.integrator_max:
                                integrator_vel[n] = -self.integrator_max

                            control_pos = error_pos*self.P_pos[n] + integrator_pos[n]*self.I_pos[n]
                            control_vel = error_vel*self.P_vel[n] + integrator_vel[n]*self.I_vel[n]

                            boxy_command.name.append(joint)
                            boxy_command.velocity.append(control_pos + control_vel)

                            error_p.data.append(round(error_pos,4))
                            error_v.data.append(round(error_vel,4))
                            int_p.data.append(round(integrator_pos[n],4))
                            int_v.data.append(round(integrator_vel[n],4))

                    boxy_command.header.stamp = now
                    # self.pub.publish(boxy_command)
                    self.pub_error_p.publish(error_p)
                    self.pub_error_v.publish(error_v)
                    self.pub_int_p.publish(int_p)
                    self.pub_int_v.publish(int_v)
                    self.pub_velocity.publish(boxy_command)
                    time_elapsed = rospy.Time.now() - start_time


def main():
    rospy.init_node('send_traj_to_boxy')
    rospy.Rate(100)
    command = VelCommands()
    while not rospy.is_shutdown():
        command.calculate()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass