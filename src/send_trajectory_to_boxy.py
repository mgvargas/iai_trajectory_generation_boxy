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
from iai_trajectory_generation_boxy.msg import PIControllerError
from sensor_msgs.msg import JointState
import reset_naive_sim
import threading
import rospkg
import yaml
import lift_object
import test_plotter
from random import randrange
from iai_trajectory_generation_boxy.srv import TrajectoryEvaluation


class VelCommands:
    def __init__(self):
        self.r = rospy.Rate(200)
        self.joint_pos = {}
        self.joint_vel = {}
        self.received = False

        self.v_lock = threading.Lock()
        self.pub_commands = rospy.Publisher('whole_body_controller/velocity_cmd', JointState, queue_size=10)
        self.pub_error = rospy.Publisher('controller_values', PIControllerError, queue_size=10)
        self.pub_simulator = rospy.Publisher('/simulator/commands', JointState, queue_size=3)

        rospy.Subscriber("projected_grasping_server/result", ProjectedGraspingActionResult, self.traj_callback)
        # self.joint_list = rospy.Subscriber('/simulate/joint_states', JointState, self.joint_callback)
        self.joint_list = rospy.Subscriber('/joint_states', JointState, self.joint_callback)

        rospy.loginfo('PI controller: Waiting for trajectory')

    def joint_callback(self, joints):
        self.joint_names = joints.name
        joint_values = joints.position
        joint_vel = joints.velocity

        with self.v_lock:
            for i, joint in enumerate(self.joint_names):
                if "neck" not in joint and "gripper" not in joint:
                    self.joint_pos[joint] = joint_values[i]
                    self.joint_vel[joint] = joint_vel[i]

    def traj_callback(self, msg):
        self.trajectory = msg.result.selected_trajectory.trajectory
        if len(self.trajectory) > 0:
            self.received = True

    def calculate(self):
        if self.received is False:
            rospy.sleep(0.2)
        else:
            reset_naive_sim.reset_simulator()

            rospy.loginfo("PI controller: Trajectory received")
            self.received = False
            desired_pos = {}
            desired_vel = {}
            num_joints = len(self.trajectory[0].name)

            # Controller gains
            self.P_vel = [0]*num_joints
            self.P_pos = [4.8]*num_joints
            self.P_pos[0] = self.P_pos[1] = 2.0
            # self.P_vel[0] = self.P_vel[1] = 0.4
            self.I_vel = [0.0]*num_joints
            self.I_pos = [0.1]*num_joints
            self.integrator_max = 10
            traj_init_time = self.trajectory[0].header.stamp
            start_time = rospy.Time.now()
            x, y = 0, 0

            # test_plotter.main(randrange(0, 100) / 100.0, randrange(0, 10) / 10.0, randrange(0, 100) / 100.0)

            for step in self.trajectory:
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
                cont_values = PIControllerError()
                y += 1
                while time_elapsed <= (traj_stamp - traj_init_time):
                    x += 1
                    boxy_command = JointState()
                    cont_values.des_p = [0.0] * len(joint_names)
                    cont_values.real_p = [0.0] * len(joint_names)
                    cont_values.error_p = [0.0] * len(joint_names)
                    cont_values.error_vel = [0.0] * len(joint_names)
                    cont_values.integral_p = [0.0] * len(joint_names)
                    cont_values.integral_vel = [0.0] * len(joint_names)
                    for n, joint in enumerate(joint_names):
                        try:
                            with self.v_lock:
                                error_pos = desired_pos[joint] - self.joint_pos[joint]
                                error_vel = desired_vel[joint] - self.joint_vel[joint]
                        except KeyError:
                            print '------ Skipping calc', joint
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

                            try:
                                with self.v_lock:
                                    cont_values.real_p[n] = self.joint_pos[joint]
                            except KeyError:
                                print '------ Skipping publish', joint
                            cont_values.des_p[n] = desired_pos[joint]
                            cont_values.error_p[n] = error_pos
                            cont_values.error_vel[n] = self.joint_vel[joint]
                            cont_values.integral_p[n] = integrator_pos[n]
                            cont_values.integral_vel[n] = integrator_vel[n]

                    boxy_command.header.stamp = now

                    self.pub_commands.publish(boxy_command)
                    self.pub_error.publish(cont_values)
                    # self.pub_velocity.publish(boxy_command)
                    time_elapsed = rospy.Time.now() - start_time
                    self.r.sleep()

            print 'time: ', rospy.Time.now().secs - start_time.secs
            self.lift_object()
            trajectory_evaluation(x, y, error_pos, rospy.Time.now().secs - start_time.secs)
            # rospy.sleep(4)
            # reset_naive_sim.reset_simulator()

    def lift_object(self):
        act = rospy.get_param('should_lift', True)
        if act:
            try:
                # Open YAML configuration file
                pack = rospkg.RosPack()
                dir = pack.get_path('iai_trajectory_generation_boxy') + '/config/controller_param.yaml'
                stream = open(dir, 'r')
                data = yaml.load(stream)
                data['start_config'] = self.joint_pos

                # Write file
                with open(dir, 'w') as outfile:
                    yaml.dump(data, outfile, default_flow_style=False)
            except yaml.YAMLError, KeyError:
                rospy.logerr("Unexpected error while writing controller configuration YAML file:"), sys.exc_info()[0]
                return -1
            rospy.sleep(0.2)
            reset_naive_sim.reset_simulator()
            lift_object.lift()


def trajectory_evaluation(iter, length, error, time):
    g_pose = rospy.get_param('grasped_pose')
    pack = rospkg.RosPack()
    dir = pack.get_path('iai_trajectory_generation_boxy') + '/test_scores/robot_test.txt'
    with open(dir, 'a') as f:
        f.write('grasped pose:'+ g_pose + ' \n'  + 'length: ' + str(length))
        f.write(', iter: ' + str(iter) +', error: ' + str(error*100) +', duration: ' + str(time) + '\n')


def main():
    rospy.init_node('send_traj_to_boxy')
    rospy.Rate(200)
    command = VelCommands()
    while not rospy.is_shutdown():
        command.calculate()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
