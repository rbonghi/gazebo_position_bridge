/**
*
*  \author     Raffaello Bonghi <raffaello.bonghi@officinerobotiche.it>
*  \copyright  Copyright (c) 2014-2015, Officine Robotiche, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Raffaello Bonghi. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to developers@officinerobotiche.it
*
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetModelState.h>



ros::ServiceClient client;
std::string robot_name;

void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    /// Build message service for Gazebo
    gazebo_msgs::SetModelState srv;
    /// Set model name
    srv.request.model_state.model_name = robot_name;
    /// Set new pose from odometry information
    srv.request.model_state.pose = msg.get()->pose.pose;
    /// Set new twist from odometry information
    srv.request.model_state.twist = msg.get()->twist.twist;
    ROS_INFO("Send position");
    if(client.call(srv)) {
        ROS_INFO("Set new position");
    } else {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gazebo_position_bridge");
    ros::NodeHandle nh;

    ROS_INFO("Started");
    /// Load robot name
    nh.param<std::string>("robot_name", robot_name, "robot_model");
    /// Service to set a new position
    client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    /// Subscriber to receive information from position and orientation robot
    ros::Subscriber sub = nh.subscribe("/gazebo/set_model_state", 1000, positionCallback);

    /// Process remainder of ROS callbacks separately, mainly ControlManager related
    ros::spin();

    return 0;
}
