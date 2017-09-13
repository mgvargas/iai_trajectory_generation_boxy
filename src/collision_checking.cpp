/*
 Copyright (c) 2017 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)

 Author: Minerva Gabriela Vargas Gleason <minervavargasg@gmail.com>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.*/

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iai_trajectory_generation_boxy/CollisionEvaluation.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

using namespace std;

geometry_msgs::TransformStamped get_tf(string destination_frame, string origin_frame){
    geometry_msgs::TransformStamped g_transform;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try{
       listener.waitForTransform(destination_frame, origin_frame, ros::Time(0), ros::Duration(10.0) );
       listener.lookupTransform( destination_frame, origin_frame, ros::Time(0), transform);
     }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    g_transform.header.frame_id = origin_frame;
    g_transform.child_frame_id = destination_frame;
    g_transform.transform.translation.x = transform.getOrigin().x();
    g_transform.transform.translation.y = transform.getOrigin().y();
    g_transform.transform.translation.z = transform.getOrigin().z();
    g_transform.transform.rotation.x = transform.getRotation().x();
    g_transform.transform.rotation.y = transform.getRotation().y();
    g_transform.transform.rotation.z = transform.getRotation().z();
    g_transform.transform.rotation.w = transform.getRotation().w();

    return g_transform;
}

moveit_msgs::CollisionObject create_collision_object(visualization_msgs::Marker marker){
    std::string mesh_path;
    shapes::Mesh* m;
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    moveit_msgs::CollisionObject collision_object;
    Eigen::Vector3d b(0.001,0.001,0.001);

    mesh_path = marker.mesh_resource;
    if (marker.ns == "kitchen_table" || marker.ns == "cup" ){    // Mesh scale
        Eigen::Vector3d s(1.0,1.0,1.0);
        b = s;}
    m = shapes::createMeshFromResource(mesh_path,b);
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
    collision_object.meshes.push_back(mesh);
    collision_object.id = marker.ns;
    collision_object.header.frame_id = marker.header.frame_id;
    collision_object.header.stamp = ros::Time::now();
    collision_object.mesh_poses.push_back(marker.pose);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}

bool evaluate_collision(iai_trajectory_generation_boxy::CollisionEvaluation::Request  &req,
                       iai_trajectory_generation_boxy::CollisionEvaluation::Response &res)
{
    // Define robot model, planning scene
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

    // Robot state
    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("whole_robot");

    //Get position of /odom wrt /map
    geometry_msgs::TransformStamped map_transform;
    map_transform = get_tf("/odom", "/map");
    planning_scene.getTransformsNonConst().setTransform(map_transform);

    // Set transform from '/camera_optical_frame' to '/map'.
    geometry_msgs::TransformStamped camera_t;
    camera_t = get_tf("/odom", "/camera_optical_frame");

    planning_scene.getTransformsNonConst().setTransform(camera_t);

    // Get current TF state, moving base to it's correct position
    planning_scene.getCurrentState();
    Eigen::Affine3d base_rel_planning_frame;
    current_state.setJointPositions(kinematic_model->getRootJoint(), base_rel_planning_frame);

    // Variables
    int num_of_joints = req.trajectory.trajectory[0].position.size();
    int num_of_objects = req.objects.markers.size();
    std::vector<double> joint_values;
    std::vector<string> joint_names;
    std::vector<std::string> object_ids;

    // Collision Objects
    moveit_msgs::CollisionObject collision_object;
    collision_request.group_name = "whole_robot";

    // Create collision objects from markers
    for (int j=0; j<num_of_objects; j=j+1){
        collision_object = create_collision_object(req.objects.markers[j]);
        object_ids.push_back(collision_object.id);
        planning_scene.processCollisionObjectMsg(collision_object);
    }
    ROS_INFO("Service CollisionChecking: Adding collision objects");

    // Checking for collisions
    bool state_validity;
    bool collision_found;
    double distance;
    float min_collision = 100;
    map<string, double> joint_state;

    // Iterating thought trajectory
    for(int step=0; step<req.trajectory.trajectory.size(); step=step+1){
        joint_values = req.trajectory.trajectory[step].position;
        joint_names = req.trajectory.trajectory[step].name;
        for(int x=0; x<num_of_joints; x=x+1){
               joint_state[joint_names[x]] = joint_values[x];
        }
        current_state.setJointGroupPositions(joint_model_group, joint_values);
        current_state.setVariablePositions(joint_state);
        planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
        collision_found = collision_result.collision;
        if (collision_found == false){
            distance = planning_scene.distanceToCollision(current_state);
            //cout << "distance" << distance << endl;
            if (distance < min_collision){
                min_collision = distance;
           }
        }
        state_validity = planning_scene.isStateValid(current_state, "whole_robot", true);
        if (collision_found == true){
            min_collision = -1;
            ROS_INFO("Service CollisionChecking: Collision found, discarding trajectory.");
            break;
        }
        collision_result.clear();
    }

    // Remove objects from the scene
    moveit_msgs::CollisionObject remove_objects;
    remove_objects.operation =  remove_objects.REMOVE;
    planning_scene.processCollisionObjectMsg(remove_objects);

    // Send back service answer
    res.min_collision_distance = min_collision;
    cout << "Min distance to collision; " << min_collision << endl << endl;

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_evaluation_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("collision_evaluation", evaluate_collision);
    ROS_INFO("Ready to check for collisions.");
    ros::spin();

    return 0;
    }
