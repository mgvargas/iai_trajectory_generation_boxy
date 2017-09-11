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
import rospkg
import sys
import yaml


def yaml_writer(joint_w_values):
    # Write a YAML file with the parameters for the simulated controller
    try:
        # Open YAML configuration file
        pack = rospkg.RosPack()
        dir = pack.get_path('iai_trajectory_generation_boxy') + '/config/controller_param.yaml'
        stream = open(dir, 'r')
        data = yaml.load(stream)
        data['start_config'] = joint_w_values

        # Write file
        with open(dir, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)
    except yaml.YAMLError:
        rospy.logerr("Unexpected error while writing controller configuration YAML file:"), sys.exc_info()[0]
        return -1


def zero():
    joints_w_values = {
        'left_arm_0_joint': -1.0424206770318125,
        'left_arm_1_joint': 0.7856426143571664,
        'left_arm_2_joint': -0.6005778452870986,
        'left_arm_3_joint': -1.664941382894658,
        'left_arm_4_joint': -1.9399642647890991,
        'left_arm_5_joint': -1.6535906309055022,
        'left_arm_6_joint': -2.572054542448777,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506,
        'odom_x_joint': -1.8,
        'odom_y_joint': -0.2,
        'odom_z_joint': -0.07,
        'right_arm_0_joint': 0.9318561286348706,
        'right_arm_1_joint': -0.617115289630847,
        'right_arm_2_joint': -2.5853490151650087,
        'right_arm_3_joint': -1.5708929937611393,
        'right_arm_4_joint': -1.3310840639433288,
        'right_arm_5_joint': 1.7323502336223708,
        'right_arm_6_joint': -0.9896413787705906,
        'triangle_base_joint': -0.31}
    return joints_w_values


def one():
    joints_w_values = {
        'odom_x_joint': -1.8,
        'odom_y_joint': 0.3,
        'odom_z_joint': -0.07,
        'left_arm_0_joint': -0.10456647767360518,
        'left_arm_1_joint': 1.029796254224278,
        'left_arm_2_joint': -1.288067942098535,
        'left_arm_3_joint': -0.6949976889629007,
        'left_arm_4_joint': -2.5945720631578344,
        'left_arm_5_joint': -1.9141940305249716,
        'left_arm_6_joint': -1.3956831415804725,
        'right_arm_0_joint': 0.3980990661937716,
        'right_arm_1_joint': -0.22682036102428657,
        'right_arm_2_joint': -2.519665972562763,
        'right_arm_3_joint': -1.0591165004990204,
        'right_arm_4_joint': -1.07072089632891,
        'right_arm_5_joint': 1.661148155502822,
        'right_arm_6_joint': -2.00116550815594,
        'triangle_base_joint': -0.19461386714773432,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506
    }
    return joints_w_values


def two():
    joints_w_values = {
        'left_arm_0_joint': -0.9615,
        'left_arm_1_joint': 0.8983,
        'left_arm_2_joint': -0.6097,
        'left_arm_3_joint': -1.5235,
        'left_arm_4_joint': -1.837,
        'left_arm_5_joint': -1.6221,
        'left_arm_6_joint': -2.48,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506,
        'odom_x_joint': -2.5,
        'odom_y_joint': 0.5,
        'odom_z_joint': -0.07,
        'right_arm_0_joint': 0.955,
        'right_arm_1_joint': -0.591,
        'right_arm_2_joint': -2.509,
        'right_arm_3_joint': -1.457,
        'right_arm_4_joint': -1.234,
        'right_arm_5_joint': 1.804,
        'right_arm_6_joint': -1.068,
        'triangle_base_joint': -0.5}
    return joints_w_values


def three():
    joints_w_values = {
        'odom_x_joint': -2.2,
        'odom_y_joint': 0.5,
        'odom_z_joint': -0.07,
        'left_arm_0_joint': -1.557,
        'left_arm_1_joint': 0.8408,
        'left_arm_2_joint': 0.0997,
        'left_arm_3_joint': -1.4161,
        'left_arm_4_joint': -2.251,
        'left_arm_5_joint': -1.482,
        'left_arm_6_joint': -2.324,
        'right_arm_0_joint': 1.03542,
        'right_arm_1_joint': -0.621414,
        'right_arm_2_joint': -2.560179,
        'right_arm_3_joint': -1.467654,
        'right_arm_4_joint': -1.296731,
        'right_arm_5_joint': 1.8575937,
        'right_arm_6_joint': -1.0270237,
        'triangle_base_joint': -0.41,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506}
    return joints_w_values


def four():
    joints_w_values = {
        'odom_x_joint': -2.5,
        'odom_y_joint': 1.06,
        'odom_z_joint': -0.07,
        'left_arm_0_joint': -1.7129450795042869,
        'left_arm_1_joint':  0.5999335018191572,
        'left_arm_2_joint': 0.04660023567009371,
        'left_arm_3_joint': -1.8599999999999897,
        'left_arm_4_joint': -2.3279241623992997,
        'left_arm_5_joint': -1.4812329512323177,
        'left_arm_6_joint': -2.579523558981309,
        'right_arm_0_joint': 0.9031112794890239,
        'right_arm_1_joint': -0.4039121646177973,
        'right_arm_2_joint': -2.5999999999999783,
        'right_arm_3_joint': -1.6117120285581779,
        'right_arm_4_joint': -0.8580671212943127,
        'right_arm_5_joint': 1.9899999999999893,
        'right_arm_6_joint': -1.0752377540035896,
        'triangle_base_joint': -0.25941441542417976,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506
    }
    return joints_w_values


def five():
    joints_w_values = {
        'odom_x_joint':-1.5,
        'odom_y_joint': 0.6,
        'odom_z_joint': -0.07,
        'left_arm_0_joint': -0.2602758811845149,
        'left_arm_1_joint': 0.5699576299562188,
        'left_arm_2_joint': -0.8360587280567272,
        'left_arm_3_joint': -1.590070040825777,
        'left_arm_4_joint': -2.537669649858849,
        'left_arm_5_joint': -1.178846334671371,
        'left_arm_6_joint': -1.7240697677555108,
        'right_arm_0_joint': 0.3548792764770302,
        'right_arm_1_joint': -0.7342053410025529,
        'right_arm_2_joint': -2.500505290058151,
        'right_arm_3_joint': -1.0565600096681835,
        'right_arm_4_joint': -0.6428302806193216,
        'right_arm_5_joint': 1.789959638331654,
        'right_arm_6_joint': -1.537211638335914,
        'triangle_base_joint': -0.1495481641584547,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506
    }
    return joints_w_values


def six():
    joints_w_values = {
        'odom_x_joint': -1.5,
        'odom_y_joint': 0.0,
        'odom_z_joint': -0.07,
        'left_arm_0_joint': -0.02737896573905153,
        'left_arm_1_joint': 0.6346913283386225,
        'left_arm_2_joint': -1.0208696610005272,
        'left_arm_3_joint': -1.4127033362970067,
        'left_arm_4_joint': -2.461101342274959,
        'left_arm_5_joint': -1.062781974441715,
        'left_arm_6_joint': -1.555507230976369,
        'right_arm_0_joint': -0.5891928530304978,
        'right_arm_1_joint': -0.7776772581633704,
        'right_arm_2_joint': -1.7386497360661208,
        'right_arm_3_joint': -1.7229435066594225,
        'right_arm_4_joint': -1.2801661171529497,
        'right_arm_5_joint': 0.3852927345385064,
        'right_arm_6_joint': -1.2714225400541546,
        'triangle_base_joint': -0.014998105556107942,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506
    }
    return joints_w_values


def seven():
    joints_w_values = {
        'odom_x_joint': -1.2,
        'odom_y_joint': 0.0,
        'odom_z_joint': -0.07,
        'left_arm_0_joint':-1.5939017921498873,
        'left_arm_1_joint': 0.8390743106005174,
        'left_arm_2_joint': 0.0854756106501257,
        'left_arm_3_joint': -1.358306360009408,
        'left_arm_4_joint': -2.271386583316387,
        'left_arm_5_joint': -1.554795183790794,
        'left_arm_6_joint': -2.303716987463937,
        'right_arm_0_joint': 0.822462125108796,
        'right_arm_1_joint': -0.6310336025351757,
        'right_arm_2_joint': -2.599671870840573,
        'right_arm_3_joint': -1.592543876588925,
        'right_arm_4_joint': -1.3963310121304198,
        'right_arm_5_joint': 1.632141298729036,
        'right_arm_6_joint': -0.66445876942373,
        'triangle_base_joint':-0.34682312271737925,
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506
    }
    return joints_w_values

'''def seven():
    joints_w_values = {
        'odom_x_joint':
        'odom_y_joint':
        'odom_z_joint':
        'left_arm_0_joint':
        'left_arm_1_joint':
        'left_arm_2_joint':
        'left_arm_3_joint':
        'left_arm_4_joint':
        'left_arm_5_joint':
        'left_arm_6_joint':
        'right_arm_0_joint':
        'right_arm_1_joint':
        'right_arm_2_joint':
        'right_arm_3_joint':
        'right_arm_4_joint':
        'right_arm_5_joint':
        'right_arm_6_joint':
        'triangle_base_joint':
        'neck_shoulder_lift_joint': -3.0928,
        'neck_shoulder_pan_joint': -1.67144,
        'neck_wrist_1_joint': 1.23036,
        'neck_wrist_2_joint': 1.54493,
        'neck_wrist_3_joint': 0.04506
    }
    return joints_w_values'''


def select_option():
    config_number = sys.argv[1]

    options = {0: zero,
               1: one,
               2: two,
               3: three,
               4: four,
               5: five,
               6: six,
               7: seven,
               }

    joint_w_values = options[int(config_number)]()
    yaml_writer(joint_w_values)


if __name__ == '__main__':
    select_option()