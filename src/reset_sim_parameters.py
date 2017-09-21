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
import yaml
import rospkg
import sys
from sensor_msgs.msg import JointState
import reset_naive_sim

class Resetting:
    def __init__(self):
        self.joint_list = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.joint_pos = {}
        self.got_it = False

    def joint_callback(self, joints):
        self.joint_names = joints.name
        joint_values = joints.position
        self.got_it = True

        for i, joint in enumerate(self.joint_names):
            if "tag" not in joint and "gripper" not in joint and "neck_tool0_controller" not in joint:
                self.joint_pos[joint] = joint_values[i]


    def init_joint_state(self):
        if not self.got_it:
            rospy.sleep(0.2)
        else:
        # Write a YAML file with the parameters for the simulated controller
            try:
                # Open YAML configuration file
                pack = rospkg.RosPack()
                dir = pack.get_path('iai_trajectory_generation_boxy') + '/config/controller_param.yaml'
                stream = open(dir, 'r')
                data = yaml.load(stream)

            except yaml.YAMLError:
                rospy.logerr("Unexpected error while reading initial configuration YAML file:"), exc_info()[0]
                return -1
            else:
                print "Reseting simulator"
                data['start_config'] = self.joint_pos

                # Write file
                with open(dir, 'w') as outfile:
                    yaml.dump(data, outfile, default_flow_style=False)

                # Rewrite ROS parameters
                for joint in self.joint_pos:
                    rospy.set_param('/simulator/start_config/'+joint, self.joint_pos[joint])
                rospy.sleep(0.2)
                reset_naive_sim.reset_simulator()
                print 'done'
                sys.exit()

        return 0


def reset_simulator():
    r = Resetting()
    while not rospy.is_shutdown():
        r.init_joint_state()


if __name__ == '__main__':
    try:
        rospy.init_node('reset')
        rospy.Rate(1)
        reset_simulator()
    except rospy.ROSInterruptException:
        pass
