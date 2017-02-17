// This file is part of RPG-YTC- the RPG youBot Torque Controller
//
// RPG-YTC is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-YTC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-YTC.  If not, see <http://www.gnu.org/licenses/>.

#include "ros/ros.h"
#include "trajectory_generator/JStoJS.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Eigen/Dense"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <boost/units/systems/si.hpp>
#include <torque_control/torque_trajectoryAction.h>
#include <torque_control/step.h>
#include <actionlib/client/simple_action_client.h>
#include "rpg_youbot_common/rpg_youbot_common.h"

using namespace std;

int main(int argc, char **argv) {
  // init ros node handle and services
  ros::init(argc, argv, "joint_tester");
  ros::NodeHandle nh;

  ros::Rate lr(500);

  ros::ServiceClient traj_client = nh.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");
  ros::Publisher arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);

  actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("torque_control", true);

  double first_pt[5];
  bool feasible = false;
  brics_actuator::JointPositions start_position, end_position;
  brics_actuator::JointVelocities start_velocity, end_velocity;
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint point;
  trajectory_generator::JStoJS rotation;

  // create JS positions
  brics_actuator::JointValue value;
  value.joint_uri = "arm_joint_1";
  value.unit = "rad";
  value.value = 1.00;
  start_position.positions.push_back(value);

  value.value = 3.00;
  end_position.positions.push_back(value);

  // set all other positions to 0
  for (int i = 2; i <= 5; i++) {
      brics_actuator::JointValue temp;
      std::stringstream joint_name;
      joint_name << "arm_joint_" << i;
      temp.joint_uri = joint_name.str();
      temp.unit = "rad";
      temp.value = 0;
      start_position.positions.push_back(temp);
      end_position.positions.push_back(temp);
  }

  // set all velocities to 0
  for (int i = 1; i <= 5; i++) {
      brics_actuator::JointValue temp;
      std::stringstream joint_name;
      joint_name << "arm_joint_" << i;
      temp.joint_uri = joint_name.str();
      temp.unit = "s^-1 rad";
      temp.value = 0;
      start_velocity.velocities.push_back(temp);
      end_velocity.velocities.push_back(temp);
  }

  rotation.request.start_pos = start_position;
  rotation.request.start_vel = start_velocity;

  rotation.request.end_pos = end_position;
  rotation.request.end_vel = end_velocity;

  //original
  //rotation.request.max_vel = 0.199999; // max
  //rotation.request.max_acc = 0.999999; // max
  rotation.request.max_vel = 0.15; // max
  rotation.request.max_acc = 0.25; // max


  if (traj_client.call(rotation)) {
    if (rotation.response.feasible) {
      feasible = true;
      traj = rotation.response.trajectory;
    } else {
      cout << "JS 2 JS not feasible" << endl;
    }

  } else {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  //torque control

  if (feasible) {

    // move arm to start position
    point = traj.points.back();
    int i = 0;

    // copy all 5 joint values of start position
    while (!point.positions.empty()) {
      first_pt[i] = point.positions.back();
      i++;
      point.positions.pop_back();
    }

    cout << "Publishing arm cmd" << endl;
/*    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    lr.sleep();
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    lr.sleep();
*/    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    lr.sleep();
    sleep(1);

    // execute torque controlled movement
    ROS_INFO("READY FOR TORQUE?");
    // wait for console input
    int x;
    cin >> x;
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");

    torque_control::torque_trajectoryGoal goal;
    goal.trajectory = traj;

    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout) {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    } else {
      ROS_INFO("Action did not finish before the time out.");
    }
  }

  return 0;
}

