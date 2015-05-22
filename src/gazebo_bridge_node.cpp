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
*     * Neither the name of Officine Robotiche. nor the
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
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>

ros::ServiceClient client;

void get_model_state(const ros::TimerEvent&)
{
    gazebo_msgs::GetModelState srv;
    ROS_INFO("Callback 1 triggered");
    if (client.call(srv)) {
        ROS_INFO("Sum: %ld", (long int)srv.response.pose.position.x);
    } else {
        ROS_ERROR("Failed to call service /gazebo/get_model_state");
    }
}

void set_model_state(const ros::TimerEvent&)
{
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = "robot -- TODO";
    //srv.request.model_state.pose
    //"{model_state: { model_name: rrbot, pose: { position: { x: 1, y: 1 ,z: 10 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }";
    ROS_INFO("Set position");
    if(client.call(srv)) {
        ROS_INFO("Set new position");
    } else {
        ROS_ERROR("Failed to call service /gazebo/set_model_state");
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "gazebo_position_bridge");
    ros::NodeHandle nh, private_nh("~");

    ROS_INFO("Started");

    //Timer
#ifdef GET
    ros::Timer timer = nh.createTimer(ros::Duration(1), get_model_state);
    client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
#endif
    ros::Timer timer = nh.createTimer(ros::Duration(1), set_model_state);
    client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // Process remainder of ROS callbacks separately, mainly ControlManager related
    ros::spin();

    return 0;
}
