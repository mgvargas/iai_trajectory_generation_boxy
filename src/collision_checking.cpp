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

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <iai_trajectory_generation_boxy/CollisionEvaluation.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

using namespace std;

bool evaluate_collision(iai_trajectory_generation_boxy::CollisionEvaluation::Request  &req,
                       iai_trajectory_generation_boxy::CollisionEvaluation::Response &res)
{
    //cout << "request " << req.trajectory.trajectory[0];
    // Define robot model, planning scene and collision detector
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene::PlanningScene planning_scene(kinematic_model);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

    // Variables
    int num_of_joints = req.trajectory.trajectory[0].position.size();
    int num_of_objects = req.objects.markers.size();
    float joint_values[num_of_joints];
    string joint_names[num_of_joints];
    float collision_found;
    std::string mesh_path;
    shapes::Mesh* m;
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;
    float b[] = {0.001, 0.001, 0.001};

    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("whole_robot");

    // Get Rospackage

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject collision_object;

    for (int j=0; j<=num_of_objects; j=j+1){
        cout << req.objects.markers[j] << endl;
        std::string path = req.objects.markers[j].mesh_resource;
        for (int s=0; s<=path.length(); s=s+1){
            char i = path[s];
            mesh_path[s] = i;}
        cout << mesh_path;
        mesh_path = req.objects.markers[j].mesh_resource;
        m = shapes::createMeshFromResource(mesh_path);
        shapes::constructMsgFromShape(m, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
        collision_object.meshes.resize(1);
        collision_object.mesh_poses.resize(1);
        collision_object.meshes[0] = mesh;
        //collision_object.id = req.objects.markers[j].ns;
        collision_object.mesh_poses[0] = req.objects.markers[j].pose;
        //collision_objects.push_back(collision_object);
    }
    planning_scene_interface.addCollisionObjects(collision_objects);

    /*for(int step=0; step<=req.trajectory.size(); step=step+1){
        joint_values = req.trajectory[step].position;
        current_state.setJointGroupPositions(joint_model_group, joint_values);
        planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
        collision_found = collision_result.collision;
        collision_result.clear();
    }

    ROS_INFO_STREAM("Test 1: Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");*/

    //------current_state.setToRandomPositions();

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
