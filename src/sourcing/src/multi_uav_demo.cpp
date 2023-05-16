/*************************************************************************
@file           multi_uav_demo.cpp
@date           2023/05/16 16:51
@author         wuminjiang
@email          wuminjiang@sia.cn
@description    a state machine demo for testing that one node controls 3
                UAVs.
*************************************************************************/

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "sourcing/Concentration.h"


/***********************node-based object definition*********************/
//Not define subscribed topic here because we only use them once.

//published topic
ros::Publisher cmd_vel_pub;

// client
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient concentration_client;

//msg data subscibed
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::TwistStamped current_velocity;

//msg data published
geometry_msgs::TwistStamped cmd_vel;

//srv data subscribed
//sourcing::Concentration concentration_srv;

/***********************const variable definition************************/

/***************************variable definition**************************/

/************************callback function definition********************/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_velocity = *msg;
}

/****************************function define*****************************/



/*********************computational function define**********************/
// Get the concentration at pose
float getConcentration(geometry_msgs::PoseStamped pose)
{
    sourcing::Concentration concentration_srv;
    concentration_srv.request.x = pose.pose.position.x + 1000;
    concentration_srv.request.y = pose.pose.position.y;
    concentration_srv.request.z = pose.pose.position.z - 1;
    if(concentration_client.call(concentration_srv))
    {
        ROS_INFO("The concentration is : %f", concentration_srv.response.concentration);
    }
    else
    {
        ROS_ERROR("Failed to call service concentration");
    }
    return (concentration_srv.response.concentration);
}


/****************************main function*******************************/
int main(int argc, char** argv){
    // ROS node init
    ros::init(argc, argv, "sourcing_node");
    ros::NodeHandle nh;

    // Subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, velocity_cb);

    // Publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

    // client
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    concentration_client = nh.serviceClient<sourcing::Concentration>("concentration");


    cmd_vel.header.frame_id = "map";

    // 等待mavros节点连接到飞控
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        ros::Rate(1).sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;ros::Rate rate(20.0);

    
    ros::Time last_request = ros::Time::now();

    


    while(ros::ok())
    {
        // If not, switch to OFFBOARD mode and make vehicle armed
        if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
            {
            ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else 
        {
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
            {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success) 
                {
                ROS_INFO("Vehicle armed");
                }
            last_request = ros::Time::now();
            }
        }
        // 发布控制指令
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel_pub.publish(cmd_vel);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

