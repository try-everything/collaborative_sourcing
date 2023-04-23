/*************************************************************************
@file           sourcing.cpp
@date           2023/04/20 16:06
@author         wuminjiang
@email          wuminjiang@sia.cn
@description    a state machine for mission of collaborate sourcing
*************************************************************************/

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "sourcing/Concentration.h"


/***********************node-based object definition*********************/
//Not define subscribed topic here because we only use they once.

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
sourcing::Concentration concentration_srv;


/***********************const variable definition************************/
// 设置覆盖搜索任务的目标点和速度
static const double search_area_x = 5.0; // 搜索区域的x边长
static const double search_area_y = 5.0; // 搜索区域的y边长
static const double target_height = 0.4; // 目标搜索高度
static const double target_speed = 1.0; // 目标搜索速度
static const double target_pitch = 0.0; // 目标搜索俯仰角


/***************************variable definition**************************/
//float concentrationMap[2000][1000];



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


/****************************function declare****************************/



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
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        ros::Rate(1).sleep();
    }

    //test
    // The initial position
    double init_pos_x = current_pose.pose.position.x;
    double init_pos_y = current_pose.pose.position.y;
    double init_pos_z = current_pose.pose.position.z;
    double curr_pos_x = init_pos_x;
    double curr_pos_y = init_pos_y;
    double curr_pos_z = init_pos_z;
    double curr_target_speed = target_speed;
    // The flag for reaching the search boundary 
    bool reached_end_x = false;
    bool reached_end_y = false;
    // 定位到初始搜索位置
    cmd_vel.twist.linear.x = target_speed;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.linear.z = 0.0;
    cmd_vel.twist.angular.x = target_pitch;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;

<<<<<<< HEAD
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
=======
    //转向标志
    int flagX = 0;
    //int flagY = 0;
>>>>>>> test

    ros::Rate rate(20.0);

    
    ros::Time last_request = ros::Time::now();

<<<<<<< HEAD
    //转向标志
    int flagX = 0;
    int flagY = 0;
=======
    
>>>>>>> test


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

        // Update current position
        curr_pos_x = current_pose.pose.position.x;
        curr_pos_y = current_pose.pose.position.y;
        curr_pos_z = current_pose.pose.position.z;
        // Get the concentration
        concentration_srv.request.x = curr_pos_x + 1000;
        concentration_srv.request.y = curr_pos_y ;
        concentration_srv.request.z = curr_pos_z;
        if(concentration_client.call(concentration_srv))
        {
            ROS_INFO("The concentration is : %f", concentration_srv.response.concentration);
        }
        else
        {
            ROS_ERROR("Failed to call service concentration");
        }


        // 检查是否到达搜索区域边界
        if (abs(curr_pos_x - init_pos_x) >= search_area_x / 2.0){
            reached_end_x = true;
        }
        if (abs(curr_pos_y - init_pos_y) >= search_area_y / 2.0){
            reached_end_y = true;
        }

        // 如果到达搜索区域边界，则转向反方向继续搜索
        if (reached_end_x)
        {
            if(flagX == 0)
            {
                curr_target_speed = -curr_target_speed;
                cmd_vel.twist.linear.x = curr_target_speed;
                //cmd_vel.twist.angular.y = 180.0;
                flagX = 1;
                ROS_INFO("I reach the X end!");
            } 
            else if(abs(curr_pos_x - init_pos_x) < search_area_x / 2.0)
            {
                flagX = 0;
                reached_end_x = false;
                ROS_INFO("I am back!");
            }  
        }
        if (reached_end_y)
        {
            if(flagY == 0)
            {
                cmd_vel.twist.linear.y = -curr_target_speed;
                cmd_vel.twist.angular.y = -90.0;
                flagY = 1;
                ROS_INFO("I reach the Y end!");
            }
            else if(abs(curr_pos_y - init_pos_y) < search_area_x / 2.0)
            {
                flagY = 0;
                reached_end_y = false;
            }  
        }


        // 如果当前高度不等于目标高度，则调整高度
        if (curr_pos_z != target_height){
            cmd_vel.twist.linear.z = (target_height - curr_pos_z) * 0.5;
        }else{
            cmd_vel.twist.linear.z = 0.0;
        }

        // 发布控制指令
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel_pub.publish(cmd_vel);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

