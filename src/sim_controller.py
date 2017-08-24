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

import numpy as np
import sys
from math import radians, pow, sqrt, pi
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PyReturnValue as returnValue
from qpoases import PySQProblem as SQProblem
import quaternion_operations as qo
import PyKDL as kdl
import actionlib
# Plot for debugging
import plotly.offline
import plotly.graph_objs as go
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from iai_naive_kinematics_sim.msg import ProjectionClock
from iai_trajectory_generation_boxy.msg import MoveToGPAction, MoveToGPFeedback, MoveToGPResult
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_slerp, quaternion_from_euler, euler_from_quaternion
from urdf_parser_py.urdf import URDF

from kdl_parser import kdl_tree_from_urdf_model


class MoveToGPServer:

    def __init__(self):
        self._feedback = MoveToGPFeedback()
        self._result = MoveToGPResult()
        self.goal_received = False
        self._action_name = 'move_to_gp'
        self.action_server = actionlib.ActionServer(self._action_name, MoveToGPAction, self.action_callback,
                                                    self.cancel_cb, auto_start=False)
        self.action_server.start()
        self.action_status = GoalStatus()

        # Variable definition
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'
        self.gripper = self.grip_right
        self.frame_base = 'base_link'
        self.nJoints = 8+3
        self.arm = 'left'
        self.joint_values = np.empty([self.nJoints])
        self.joint_values_kdl = kdl.JntArray(self.nJoints)
        self.eef_pose = kdl.Frame()
        # TODO: find appropriate max acceleration
        #self.accel_max = np.array([0.3, 0.3, 0.3, 0.01, 0.64, 0.64, 0.75, 0.75, 0.75, 1.05, 1.05])
        self.accel_max = np.array([0.3, 0.3, 0.3, 1.02, 1.9, 1.9, 1.95, 1.95, 1.95, 2.05, 2.05])

        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.pub_clock = rospy.Publisher('/simulator/projection_clock', ProjectionClock, queue_size=3)
        self.pub_velocity = rospy.Publisher('/simulator/commands', JointState, queue_size=3)
        self.pub_plot = rospy.Publisher('/data_to_plot', PoseArray, queue_size=10)

        print 'Ready'

        # Wait until goal is received
        rospy.Timer(rospy.Duration(0.05), self.generate_trajectory)

    def joint_callback(self, data):
        # Save current joint state
        self.current_state = data
        # Getting current joint values of the arms
        self.all_joint_names = data.name
        self.all_joint_values = data.position
        self.start_arm = [0]*(self.nJoints-1)
        self.start_odom = [0]*3
        a, n = 4, 0#1
        if self.arm == 'right':
            arm = 'right_arm'
            self.gripper = self.grip_right
        else:
            arm = 'left_arm'
            self.gripper = self.grip_left

        for i, x in enumerate(self.all_joint_names):
            if arm in x and (a-3) < (self.nJoints):
                try:
                    self.joint_values[a] = self.all_joint_values[i]
                    self.joint_values_kdl[a] = self.all_joint_values[i]
                    self.start_arm[a-4] = i
                    a += 1
                except IndexError:
                    index_error = True
            elif 'triangle_base' in x:
                try:
                    self.joint_values[3] = self.all_joint_values[i]
                    self.joint_values_kdl[3] = self.all_joint_values[i]
                    self.triang_list_pos = i
                except IndexError:
                    index_error = True
            elif 'odom' in x:
                try:
                    self.joint_values[n] = self.all_joint_values[i]
                    self.joint_values_kdl[n] = self.all_joint_values[i]
                    self.start_odom[n] = i
                    n += 1
                except IndexError:
                    index_error = True

        try:
            self.calc_eef_position(self.joint_values)
        except AttributeError:
            pass

    def action_callback(self, cb):
        # Received goal flags
        self.action_status.status = 1
        self.action_server.publish_status()
        self.goal_received = True
        self.active_goal_flag = True
        self.reach_pregrasp = False

        # Goal definition
        self.goal_id = self.action_status.goal_id = cb.goal.goal_id
        self.goal_pose = cb.goal.goal.grasping_pose
        self.pre_grasp = cb.goal.goal.pre_grasping
        print self.pre_grasp
        self.pose_name = self.goal_pose.child_frame_id
        self.arm = cb.goal.goal.arm
        self._feedback.sim_trajectory = []

        rospy.loginfo('Action %s: Executing, creating trajectory to grasping pose %s.'
                      % (self._action_name, self.pose_name))
        return 0

    def cancel_cb(self, cb):
        self.active_goal_flag = False
        self.goal_received = False
        self.action_server.internal_cancel_callback(goal_id=self.goal_id)
        self.action_status.status = 4
        self.success = False
        rospy.logwarn('Action cancelled by client.')
        self._result.trajectory = self._feedback.sim_trajectory
        self.action_server.publish_result(self.action_status, self._result)
        self.action_server.publish_status()
        return 0

    def get_urdf(self):
        try:
            self.urdf_model = URDF.from_parameter_server()
        except (OSError, LookupError) as error:
            rospy.logerr("Unexpected error while reading URDF:", error), sys.exc_info()[0]

    def generate_trajectory(self, time):
        if self.goal_received:
            self.get_urdf()
            init_error_pos = self.qpoases_config()
            np.set_printoptions(suppress=True, precision=2, linewidth=180)
            self.qpoases_calculation(init_error_pos)

            if self.success:
                self._result.trajectory = self._feedback.sim_trajectory
                rospy.loginfo('Action %s: Succeeded' % self._action_name)
                self.action_status.status = 3
                self.action_server.publish_result(self.action_status, self._result)
            self.active_goal_flag = False
            self.goal_received = False
        return 0

    def calc_posit_error(self, error_posit):
        # Error threshold
        threshold = 0.02 * self.prop_gain

        self.calc_eef_position(self.joint_values)
        eef_posit = np.array([self.eef_pose.p[0], self.eef_pose.p[1], self.eef_pose.p[2]])

        # Select between pre-grasping pose and grasping pose
        if max(error_posit) > threshold and not self.reach_pregrasp:
            error_posit = (self.pregrasp_posit - eef_posit) * self.prop_gain
        else:
            self.reach_pregrasp = True
            error_posit = (self.goal_posit - eef_posit) * self.prop_gain
        limit_p = [abs(x/self.prop_gain) for x in error_posit]
        return error_posit, limit_p

    def calc_orient_error(self, eef_orient, goal_orient, thresh, limit_p):
        # Error threshold
        precision_o = 0.17 # 10 deg tolerance
        z_threshold = 0.4 # 23 deg tolerance
        reached_orientation = False

        eef_inv = qo.q_inv(eef_orient)
        rot_vector = qo.q_mult(eef_inv, goal_orient)
        rot_error = sqrt(pow(rot_vector[0], 2) + pow(rot_vector[1], 2) + pow(rot_vector[2], 2))

        if (thresh - rot_error) >= 0:
            scaling = 1
        else:
            scaling = thresh/rot_error

        slerp = quaternion_slerp(eef_orient, goal_orient, scaling)
        quat_error = qo.q_mult(slerp, eef_inv)
        error_orient = euler_from_quaternion(quat_error)

        # Stopping conditions
        x_y_error = sqrt(pow(rot_vector[0], 2) + pow(rot_vector[1], 2))
        if x_y_error < precision_o and  error_orient[2] < z_threshold:
            reached_orientation = True
        '''print 'eef_ori:   ', euler_from_quaternion(eef_orient)
        print 'slerp:     ', euler_from_quaternion(slerp)
        print 'goal_ori:  ', self.goal_orient_euler
        print 'ori error: ', error_orient
        print 'scale ', scaling'''
        return error_orient, reached_orientation

    def qpoases_config(self):
        # QPOases needs as config parameters:
        # Nr and name of Joints - self.joint_names
        # Initial joint value   - init_joint_val
        # Links name            - links
        # Links length          - self.links_length
        # Joint weights         - self.jweights
        # Target pose           - self.goal_pose
        np.set_printoptions(suppress=True, precision=3, linewidth=120)

        # Variables
        slack_limit = 400
        self.n_slack = 6
        self.prop_gain = 3
        self.prop_gain_orient = 1.5
        self.sweights = np.ones((self.n_slack))*4

        # Joint limits: self.joint_limits_upper, self.joint_limits_lower
        self.kinem_chain(self.gripper)
        # Set initial joint vel to 0
        joint_velocity = [0] * self.nJoints

        # Calculate acceleration limits
        [ac_lim_lower, ac_lim_upper] = self.acceleration_limits(joint_velocity)

        # Error in EEF orientation, if error in Z > 90 deg, rotate goal 180 deg
        self.calc_eef_position(self.joint_values)
        self.goal_quat = np.array([self.goal_pose.transform.rotation.x, self.goal_pose.transform.rotation.y,
                         self.goal_pose.transform.rotation.z, self.goal_pose.transform.rotation.w])
        self.goal_orient_euler= euler_from_quaternion(self.goal_quat)

        eef_orient_quat = qo.rotation_to_quaternion(self.eef_pose.M)

        limit_rot_z = abs(euler_from_quaternion(eef_orient_quat)[2] - self.goal_orient_euler[2])
        if limit_rot_z > radians(90):
            print 'rotate goal in Z'
            rot = quaternion_from_euler(0, 0, radians(180))
            self.goal_quat  = qo.q_mult(self.goal_quat, rot)
            self.goal_orient_euler = euler_from_quaternion(self.goal_quat)

        error_orient = np.array([0.0, 0.0, 0.0])

        # Error in EEF position
        self.pregrasp_posit = np.array([self.pre_grasp.transform.translation.x, self.pre_grasp.transform.translation.y,
                               self.pre_grasp.transform.translation.z])
        self.goal_posit = np.array([self.goal_pose.transform.translation.x, self.goal_pose.transform.translation.y,
                                    self.goal_pose.transform.translation.z])
        error_posit, lim = self.calc_posit_error(np.array([1.0, 1.0, 1.0]))

        # Get jacobian
        self.jacobian = self.get_jacobian()

        # Create matrix A, one slack factor per DOF of the EEF
        A_goal = np.hstack((self.jacobian, np.eye(self.n_slack)))
        A_joint_lim = np.hstack((np.eye(self.nJoints), np.zeros((self.nJoints, self.n_slack))))
        A_accel_lim = np.hstack((np.eye(self.nJoints), np.zeros((self.nJoints, self.n_slack))))
        self.A = np.concatenate((A_goal, A_joint_lim, A_accel_lim), axis=0)
        self.problem_size = np.shape(self.A)
        # print 'A ', np.shape(self.A), '\n',self.A

        # Boundary vectors of A
        self.joint_dist_lower_lim = np.empty([self.nJoints])
        self.joint_dist_upper_lim = np.empty([self.nJoints])
        self.joint_dist_lower_lim = np.subtract(self.joint_limits_lower, self.joint_values)
        self.joint_dist_upper_lim = np.subtract(self.joint_limits_upper, self.joint_values)
        self.lbA = np.hstack((error_posit, error_orient, self.joint_dist_lower_lim, ac_lim_lower))
        self.ubA = np.hstack((error_posit, error_orient, self.joint_dist_upper_lim, ac_lim_upper))

        # Create vector g
        self.g = np.zeros(self.nJoints+self.n_slack)

        # Boundary vectors of the state vector
        slack_vel = np.ones((self.n_slack))*slack_limit
        self.lb = np.hstack((-self.joint_vel_limits, -slack_vel))
        self.ub = np.hstack((self.joint_vel_limits, slack_vel))
        # print 'lb ', np.shape(self.lb), '\n', self.lb
        # print 'ub ', np.shape(self.ub), '\n', self.ub

        # Define matrix H
        self.H = np.diag(np.hstack((np.diag(self.jweights), self.sweights)))
        # print 'self.H ', np.shape(self.H), '\n',self.H
        return error_posit

    def qpoases_calculation(self, error_posit):
        # Variable initialization
        self._feedback.sim_trajectory = [self.current_state]
        error_orient = np.array([0.0, 0.0, 0.0])
        limit_p = [abs(x) for x in error_posit]
        eef_pose_array = PoseArray()
        reached_orientation = False

        # Setting up QProblem object
        vel_calculation = SQProblem(self.problem_size[1], self.problem_size[0])
        options = Options()
        options.setToDefault()
        options.printLevel = PrintLevel.LOW
        vel_calculation.setOptions(options)
        Opt = np.zeros(self.problem_size[1])
        i = 0
        precision = 0.015

        # Config iai_naive_kinematics_sim, send commands
        clock = ProjectionClock()
        velocity_msg = JointState()
        velocity_msg.name = self.all_joint_names
        velocity_msg.header.stamp = rospy.get_rostime()
        velocity_array = [0.0 for x in range(len(self.all_joint_names))]
        effort_array = [0.0 for x in range(len(self.all_joint_names))]
        velocity_msg.velocity = velocity_array
        velocity_msg.effort = effort_array
        velocity_msg.position = effort_array
        clock.now = rospy.get_rostime()
        clock.period.nsecs = 10000000
        self.pub_clock.publish(clock)

        # Plot for debugging
        t = np.array([i])
        base_weight = np.array(self.jweights[0, 0])
        arm_weight = np.array(self.jweights[4, 4])
        triang_weight = np.array(self.jweights[3, 3])
        low, pos, high, vel_p, error = [], [], [], [], []

        for x in range(6):
            error.append(np.array([0]))
        for x in range(8):
            low.append(np.array([self.lbA[9+x]/2]))
            pos.append(np.array(self.joint_values[x+3]))
            vel_p.append(np.array(Opt[x+3]))
            high.append(np.array([self.ubA[9+x]/2]))

        while max(limit_p) > precision or not reached_orientation or not self.reach_pregrasp:
            # tic = rospy.get_rostime()
            i += 1
            # Check if client cancelled goal
            if not self.active_goal_flag:
                self.success = False
                break

            if i > 500:
                self.abort_goal('time_out')
                break

            # Solve QP, redefine limit vectors of A.
            eef_pose_msg = Pose()
            nWSR = np.array([150])
            [ac_lim_lower, ac_lim_upper] = self.acceleration_limits(Opt)
            self.joint_dist_lower_lim = self.joint_limits_lower - self.joint_values
            self.joint_dist_upper_lim = self.joint_limits_upper - self.joint_values
            self.lbA = np.hstack((error_posit, error_orient, self.joint_dist_lower_lim, ac_lim_lower))
            self.ubA = np.hstack((error_posit, error_orient, self.joint_dist_upper_lim, ac_lim_upper))

            # Recalculate H matrix
            self.calculate_weigths(error_posit)

            vel_calculation = SQProblem(self.problem_size[1], self.problem_size[0])
            vel_calculation.setOptions(options)

            return_value = vel_calculation.init(self.H, self.g, self.A, self.lb, self.ub, self.lbA, self.ubA, nWSR)
            vel_calculation.getPrimalSolution(Opt)

            if return_value != returnValue.SUCCESSFUL_RETURN:
                rospy.logerr("QP-Problem returned without success! ")
                self.abort_goal('infeasible')
                break

            for x,vel in enumerate(Opt[4:(self.nJoints)]):
                velocity_msg.velocity[self.start_arm[x]] = vel

            velocity_msg.velocity[self.triang_list_pos] = Opt[3]
            velocity_msg.velocity[0] = Opt[0]
            velocity_msg.velocity[1] = Opt[1]

            # for n,j in enumerate(velocity_msg.name[:-10]):
            #    print '%s: %.3f'%(j, velocity_msg.velocity[n])

            # Recalculate Error in EEF position
            error_posit, limit_p = self.calc_posit_error(error_posit)

            # Recalculate Error in EEF orientation
            eef_orient = qo.rotation_to_quaternion(self.eef_pose.M)
            error_orient, reached_orientation = self.calc_orient_error(eef_orient, self.goal_quat, 0.35, limit_p)

            # Print velocity for iai_naive_kinematics_sim
            velocity_msg.header.stamp = rospy.get_rostime()
            self.pub_velocity.publish(velocity_msg)
            clock.now = rospy.get_rostime()
            self.pub_clock.publish(clock)

            # Action feedback
            self.action_status.status = 1
            self._feedback.sim_status = self.action_status.text = 'Calculating trajectory'
            self._feedback.sim_trajectory.append(self.current_state)
            self.action_server.publish_feedback(self.action_status, self._feedback)
            self.action_server.publish_status()

            # Store EEF pose for plotting
            eef_pose_msg.position.x = self.eef_pose.p[0]
            eef_pose_msg.position.y = self.eef_pose.p[1]
            eef_pose_msg.position.z = self.eef_pose.p[2]
            eef_pose_msg.orientation.x = eef_orient[0]
            eef_pose_msg.orientation.y = eef_orient[1]
            eef_pose_msg.orientation.z = eef_orient[2]
            eef_pose_msg.orientation.w = eef_orient[3]
            eef_pose_array.poses.append(eef_pose_msg)

            # Plot for debuging
            '''for x in range(8):
                low[x] = np.hstack((low[x], self.lbA[9+x]/2))
                pos[x] = np.hstack((pos[x], self.joint_values[x+3]))
                vel_p[x] = np.hstack((vel_p[x], Opt[x+3]))
                high[x] = np.hstack((high[x], self.ubA[9+x]/2))
            for x in range(6):
                if x < 3:
                    error[x] = np.hstack((error[x], self.lbA[x]/self.prop_gain))
                else:
                    # error[x] = np.hstack((error[x], self.lbA[x]/self.prop_gain_orient))
                    # e = self.goal_orient_euler - euler_from_quaternion(eef_orient)
                    e = error_orient
                    error[x] = np.hstack((error[x], e[x-3]))
            base_weight = np.hstack((base_weight, self.jweights[0, 0]))
            arm_weight = np.hstack((arm_weight, self.jweights[4, 4]))
            triang_weight = np.hstack((triang_weight, self.jweights[3, 3]))
            t = np.hstack((t, i))'''

            self.success = True

            if i > 1000:
                self.abort_goal('time_out')
                break

            print '\n iter: ', i
            # toc = rospy.get_rostime()
            # print (toc.nsecs-tic.nsecs)/10e9, 'sec Elapsed'
            print 'joint_vel: ', Opt[:-6]
            print 'slack    : ', Opt[-6:]
            '''print 'eef_ori:   ', euler_from_quaternion(eef_orient)
            print 'ori error: ', error_orient
            print 'goal_ori:  ', self.goal_orient_euler
            print '\neef error:  ', eef_posit
            print 'pos error:  ', error_posit/self.prop_gain
            print 'goal error: ', self.goal_posit
            print 'lbA ', np.shape(self.lbA), '\n', self.lbA[6:]
            print 'ubA ', np.shape(self.ubA), '\n',self.ubA[6:]'''

        eef_pose_array.header.stamp = rospy.get_rostime()
        eef_pose_array.header.frame_id = self.gripper
        self.pub_plot.publish(eef_pose_array)

        # Plot
        '''t_base = go.Scatter(
            y=base_weight, x=t, marker=dict(size=4,),
            mode='lines+markers', name='base_weight')
        t_arm = go.Scatter(
            y=arm_weight, x=t, marker=dict(size=4,),
            mode='lines+markers', name='arm_weight')
        t_triang = go.Scatter(
            y=triang_weight, x=t, marker=dict(size=4, ),
            mode='lines+markers', name='triangle_weight')

        for x in range(8):
            t_low_lim = go.Scatter(
                y=low[x], x=t, marker=dict(size=4, ),
                mode='lines', name='joint_lim_low/2')
            t_p0 = go.Scatter(
                y=pos[x], x=t, marker=dict(size=4, ),
                mode='lines+markers', name='pos')
            t_v = go.Scatter(
                y=vel_p[x], x=t, marker=dict(size=4, ),
                mode='lines+markers', name='vel')
            t_up_lim = go.Scatter(
                y=high[x], x=t, marker=dict(size=4, ),
                mode='lines', name='joint_lim_hi/2')
            data = [t_low_lim, t_p0, t_v, t_up_lim]
            layout = dict(title="Joint "+str(x),
                          xaxis=dict(title='Iterations', autotick=False, dtick=25, gridwidth=3, ),
                          yaxis=dict(title='Position / Velocity'), )
            fig = dict(data=data, layout=layout)
            plotly.offline.plot(fig, filename='html/joint_limits'+str(x)+'.html')
        data = []
        for x in range(6):
            t_err = go.Scatter(
                y=error[x], x=t, marker=dict(size=4, ),
                mode='lines+markers', name='error'+str(x))
            data.append(t_err)
        layout = dict(title="Errors [position] [orientation].",
                      xaxis=dict(title='Iterations', autotick=False, dtick=25, gridwidth=3, ),
                      yaxis=dict(title='Error', gridwidth=3, ), )
        fig = dict(data=data, layout=layout)
        plotly.offline.plot(fig, filename='html/error.html', image='png', image_filename='error')

        data = [t_base, t_arm, t_triang]
        layout = dict(title="Weighs.",
                      xaxis=dict(title='Iterations', autotick=False, dtick=25, gridwidth=3,),
                      yaxis=dict(title='Weights',gridwidth=3,),)
        fig = dict(data=data, layout=layout)
        plotly.offline.plot(fig, filename='html/weights.html', image='png', image_filename='weights')'''

        return 0

    def abort_goal(self, reason):
        self.success = False
        self.action_server.internal_cancel_callback(goal_id=self.goal_id)
        self.action_status.status = 4  # Aborted
        if reason == 'time_out':
            self.action_status.text = 'Trajectory generation took too long.'
            rospy.logerr('Action %s aborted due to time out' % self._action_name)
        else:
            self.action_status.text = 'QP infeasibility problem.'
            rospy.logerr('Action %s aborted due to infeasibility of QP problem' % self._action_name)
        self.action_server.publish_status()
        self._result.trajectory = self._feedback.sim_trajectory
        self.action_server.publish_result(self.action_status, self._result)

        self.active_goal_flag = False
        self.goal_received = False

    def kinem_chain(self, name_frame_end, name_frame_base='odom'):
        # Transform URDF to Chain() for the joints between 'name_frame_end' and 'name_frame_base'
        self.chain = kdl.Chain()
        ik_lambda = 0.35

        try:
            self.joint_names = self.urdf_model.get_chain(name_frame_base, name_frame_end, links=False, fixed=False)
            self.name_frame_in = name_frame_base
            self.name_frame_out = name_frame_end

            # rospy.loginfo("Will control the following joints: %s" %(self.joint_names))

            self.kdl_tree = kdl_tree_from_urdf_model(self.urdf_model)
            self.chain = self.kdl_tree.getChain(name_frame_base, name_frame_end)
            self.kdl_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
            self.kdl_ikv_solver = kdl.ChainIkSolverVel_wdls(self.chain)
            self.kdl_ikv_solver.setLambda(ik_lambda)
            self.nJoints = self.chain.getNrOfJoints()

            # Default Task and Joint weights
            self.tweights = np.identity(6)
            # weight matrix with 1 in diagonal to make use of all the joints.
            self.jweights = np.identity(self.nJoints)

            self.kdl_ikv_solver.setWeightTS(self.tweights.tolist())
            self.kdl_ikv_solver.setWeightJS(self.jweights.tolist())

            # Fill the list with the joint limits
            self.joint_limits_lower = np.empty(self.nJoints)
            self.joint_limits_upper = np.empty(self.nJoints)
            self.joint_vel_limits = np.empty(self.nJoints)

            for n, jnt_name in enumerate(self.joint_names):
                jnt = self.urdf_model.joint_map[jnt_name]
                if jnt.limit is not None:
                    if jnt.limit.lower is None:
                        self.joint_limits_lower[n] = -0.07
                    else:
                        self.joint_limits_lower[n] = jnt.limit.lower
                    if jnt.limit.upper is None:
                        self.joint_limits_upper[n] = -0.07
                    else:
                        self.joint_limits_upper[n] = jnt.limit.upper

                    self.joint_vel_limits[n] = jnt.limit.velocity
        except (RuntimeError, TypeError, NameError):
            rospy.logerr("Unexpected error:", sys.exc_info()[0])
            rospy.logerr('Could not re-init the kinematic chain')
            self.name_frame_out = ''

    def calc_eef_position(self, joint_val):
        # Calculates current EEF pose and stores it in self.eef_pose
        joint_posit = kdl.JntArray(self.nJoints)
        for n, joint in enumerate(joint_posit):
            joint_posit[n] = joint_val[n]
        kinem_status = self.kdl_fk_solver.JntToCart(joint_posit, self.eef_pose)
        if kinem_status>=0:
            pass
        else:
            rospy.logerr('Could not calculate forward kinematics')
        return self.eef_pose

    def acceleration_limits(self,joint_vel):
        ac_lim_lower = np.empty(0)
        ac_lim_upper = np.empty(0)
        for n,a in enumerate(self.accel_max):
            v = self.joint_vel_limits[n]
            ac_lower = ((v - a)/ v) * joint_vel[n] - a
            ac_upper = ((v - a)/ v) * joint_vel[n] + a
            ac_lim_lower = np.hstack((ac_lim_lower, ac_lower))
            ac_lim_upper = np.hstack((ac_lim_upper, ac_upper))
        return ac_lim_lower, ac_lim_upper

    def get_jacobian(self):
        # Obtain jacobian for the selected arm
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        jacobian = kdl.Jacobian(self.nJoints)
        self.jac_solver.JntToJac(self.joint_values_kdl, jacobian)

        jac_array = np.empty(0)
        for row in range(jacobian.rows()):
            for col in range(jacobian.columns()):
                jac_array = np.hstack((jac_array, jacobian[row,col]))
        jac_array = np.reshape(jac_array, (jacobian.rows(), jacobian.columns()))
        return jac_array

    def calculate_weigths(self, error_posit):
        # Weight of an active/inactive joint
        active_joint = 1e-2
        inactive_joint = 10
        # Weights of the slack vector
        self.sweights[:] = 1

        # Distance range of changing weight values
        a, b = 0.3, 0.7
        w_len = len(np.diag(self.jweights))
        jweights = np.ones(w_len)
        size_jac = np.shape(self.jacobian)

        # Find bigger distance to goal (x or y)
        dist = max(abs(error_posit[0]), abs(error_posit[1]))

        # If the robot is too far away, move only the base
        if dist >= b:
            # print 'far'
            jweights = np.ones(w_len)*inactive_joint
            jweights[3]-= 1
            if abs(error_posit[0]) > b:
                jweights[0] = active_joint
                self.sweights[0] = inactive_joint
            if abs(error_posit[1]) > b:
                jweights[1] = active_joint
                self.sweights[1] = inactive_joint
            # Arm not active
            for x in range(size_jac[0]):
                for y in range(size_jac[1]):
                    self.A[x,y] = 0
            # Base active
            self.A[0, 0] = 1
            self.A[1, 1] = 1

        # Weights will go from 0.01 to 10, depending on the distance to the goal
        elif a < dist < b:
            # print 'middle'
            for x in range(size_jac[0]):
                for y in range(size_jac[1]):
                    self.A[x, y] = self.jacobian[x, y]
            self.A[0, 0] = 0.5
            self.A[1, 1] = 0.5
            # Base joints will have lower weights when far from the goal
            for n,w in enumerate(jweights):
                new_w = (inactive_joint - active_joint)*(dist - a)/(b - a) + active_joint
                jweights[n] = new_w
            jweights[0] = ((active_joint -inactive_joint)*(dist - a)/(b - a) + inactive_joint)
            jweights[1] = ((active_joint -inactive_joint)*(dist - a)/(b - a) + inactive_joint)

        # if the robot is close to the goal
        else:
            # print 'close'
            for x in range(size_jac[0]):
                for y in range(size_jac[1]):
                    self.A[x, y] = self.jacobian[x, y]

            jweights = np.ones(w_len)*active_joint
            jweights[0] = inactive_joint
            jweights[1] = inactive_joint
            # Triangle base weight
            jweights[3] = (active_joint -inactive_joint)*(abs(error_posit[2]) - 0.0)/(0.6 - 0.0) + inactive_joint
            if jweights[3] < active_joint:
                jweights[3] = active_joint
            # self.sweights = np.ones(len(self.sweights))*inactive_joint/2
            self.A[0, 0] = 0.1
            self.A[1, 1] = 0.1

        jweights[2] = inactive_joint*10

        # Reset matrix H
        self.jweights = np.diag(jweights)
        self.H = np.diag(np.hstack((np.diag(self.jweights), self.sweights)))
        '''print 'A: ', self.A[:size_jac[0],:size_jac[1]]
        print '-- Base weight: [%.4f, %.4f] \n--  Triangle [%.4f] Arm weight: [%.4f]'\
              %(jweights[0], jweights[1], jweights[3], jweights[4])
        print '-- Slack weight: [%.2f, %.2f]'%(self.sweights[0], self.sweights[3])'''


def main():
    try:
        rospy.init_node('move_to_gp_server')
        rospy.Rate(200)
        a = MoveToGPServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
