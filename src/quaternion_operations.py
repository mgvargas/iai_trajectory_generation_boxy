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

import math
import numpy as np


def rotation_to_quaternion(rot_matrix):
    w = math.sqrt(1 + rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2]) / 2
    x = (rot_matrix[2, 1] - rot_matrix[1, 2]) / (4 * w)
    y = (rot_matrix[0, 2] - rot_matrix[2, 0]) / (4 * w)
    z = (rot_matrix[1, 0] - rot_matrix[0, 1]) / (4 * w)
    return np.array([x, y, z, w])


def quaternion_to_euler(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), (1 - 2 * (x * x - y * y)))
    t2 = 2.0 * (w * y - z * x)
    t2 = 1 if t2 > 1 else t2
    t2 = -1 if t2 < -1 else t2
    pitch = math.asin(t2)
    t3 = 2 * (w * z + x * y)
    t4 = 1.0 - 2 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return np.array([roll, pitch, yaw])


def q_mult(q1, q2):
    # Quaternion multiplication
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return np.array([x, y, z, w])


def q_norm(q):
    x, y, z, w = q
    norm = math.pow(w, 2) + math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2)
    return norm

def normalize_quat(q):
    q_array = np.array(q)
    norm = q_norm(q)
    return q_array/norm

def q_inv(q1):
    x1, y1, z1, w1 = q1
    d = q_norm(q1)
    w = w1 / d
    x = -x1 / d
    y = -y1 / d
    z = -z1 / d
    return np.array([x, y, z, w])

def quat_to_axisangle(q):
    x, y, z, w = q
    if w > 1:
        x, y, z, w = normalize_quat(q)
    try:
        theta = 2 * math.acos(w)
    except ValueError:
        theta = 2 * math.acos(w + 0.01)
    s = math.sqrt(1 - w * w)
    if s < 0.001:
        ax = x
        ay = y
        az = z
    else:
        ax = x / s
        ay = y / s
        az = z / s
    return np.array([ax, ay, az, theta])


def quaternion_error(q_e, q_d):
    # q_e = current orientation, q_d = desired orientation
    x, y, z, w_d = q_d
    # Split quaternions in vector and w
    r_d = np.array([q_d[0], q_d[1], q_d[2]])
    r_e = np.array([q_e[0], q_e[1], q_e[2]])
    w_e = q_e[3]
    # Skew symmetric matrix
    s = np.array([0, -z, y, z, 0, x, -y, x, 0]).reshape(3,3)

    error = w_e * r_d - w_d * r_e - s.dot(r_d)
    return error


def slerp(q0, q1, t=1.0):
    # Interpolation between 2 quaternions, from 0 <= t <= 1
    dot_threshold = 0.9995
    q0_norm = normalize_quat(q0)
    q1_norm = normalize_quat(q1)
    dot = q0_norm[0] * q1_norm[0] + q0_norm[1] * q1_norm[1] + q0_norm[2] * q1_norm[2] + q0_norm[3] * q1_norm[3]

    # if quaternions are too close, do linear interpolation
    if abs(dot) > dot_threshold:
        q_slerp = q0_norm + t * (q1_norm - q0_norm)
        return q_slerp
    # if slerp wants to take the long path (>180 deg), change it
    if dot < 0.0:
        q1_norm = -q1_norm
        dot = -dot
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    q2 = q1_norm - q0_norm * dot
    q2_norm = self.normalize_quat(q2)

    return q0 * math.cos(theta) + q2_norm * math.sin(theta)


def main():
    print 'Some simple operations with quaternions'


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass