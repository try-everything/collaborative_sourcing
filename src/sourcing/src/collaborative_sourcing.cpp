/*************************************************************************
@file           collaborative_sourcing.cpp
@date           2023/05/18 15:05
                2023/05/29 23:00
@author         wuminjiang
@email          wuminjiang@sia.cn
@description    a state machine for collaborative sourcing by 3 UAVs.
*************************************************************************/

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>
#include <string>



#include "sourcing/Concentration.h"

using namespace std;


/***********************node-based object definition*********************/
//Not define subscribed topic here because we only use them once.

//published topic
ros::Publisher uav0_cmd_vel_pub;
ros::Publisher uav1_cmd_vel_pub;
ros::Publisher uav2_cmd_vel_pub;
ros::Publisher uav0_local_pos_pub;
ros::Publisher uav1_local_pos_pub;
ros::Publisher uav2_local_pos_pub;

// client
ros::ServiceClient uav0_arming_client;
ros::ServiceClient uav1_arming_client;
ros::ServiceClient uav2_arming_client;
ros::ServiceClient uav0_set_mode_client;
ros::ServiceClient uav1_set_mode_client;
ros::ServiceClient uav2_set_mode_client;
ros::ServiceClient uav0_set_mode_client_posctl;
ros::ServiceClient uav1_set_mode_client_posctl;
ros::ServiceClient uav2_set_mode_client_posctl;
ros::ServiceClient concentration_client;

//msg data subscibed
mavros_msgs::State uav0_current_state;
mavros_msgs::State uav1_current_state;
mavros_msgs::State uav2_current_state;
geometry_msgs::PoseStamped uav0_current_pose;
geometry_msgs::PoseStamped uav1_current_pose;
geometry_msgs::PoseStamped uav2_current_pose;
geometry_msgs::TwistStamped uav0_current_velocity;
geometry_msgs::TwistStamped uav1_current_velocity;
geometry_msgs::TwistStamped uav2_current_velocity;

//msg data published
// geometry_msgs::PoseStamped uav0_pose_pub;
// geometry_msgs::PoseStamped uav1_pose_pub;
// geometry_msgs::PoseStamped uav2_pose_pub;
geometry_msgs::PoseStamped position_A;
geometry_msgs::PoseStamped position_B;
geometry_msgs::PoseStamped position_C;
geometry_msgs::TwistStamped uav0_cmd_vel;
geometry_msgs::TwistStamped uav1_cmd_vel;
geometry_msgs::TwistStamped uav2_cmd_vel;



/***********************const variable definition************************/
#define NS0 "uav0"
#define NS1 "uav1"
#define NS2 "uav2"

static const float TAKEOFF_VELOCITY = 1.0;

// states
static const int TakeOff = 1;
static const int UnknowAreaSearch = 2;
static const int BoundarySearch = 3;
static const int SourceSearch = 4;
static const int FineCoverage = 5;
static const int Land = 10;

/***************************variable definition**************************/
int current_mission_state = TakeOff;



/************************callback function definition********************/
void uav0_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav0_current_state = *msg;
}
void uav1_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav1_current_state = *msg;
}
void uav2_state_cb(const mavros_msgs::State::ConstPtr& msg){
    uav2_current_state = *msg;
}

void uav0_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav0_current_pose = *msg;
}
void uav1_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav1_current_pose = *msg;
}
void uav2_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    uav2_current_pose = *msg;
}

void uav0_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    uav0_current_velocity = *msg;
}
void uav1_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    uav1_current_velocity = *msg;
}
void uav2_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    uav2_current_velocity = *msg;
}

/****************************function define*****************************/
void stateMachineFunction()
{
    switch(current_mission_state)
    {
        case TakeOff:
        {
            
        }
        break;
        case UnknowAreaSearch:
        {
            unknowAreaSearchFunction();
        }
        break;
        case BoundarySearch:
        {
            boundarySearchFunction();
        }
        break;
        case SourceSearch:
        {
            sourceSearchFunction();
        }
        break;
        case FineCoverage:
        {
            fineCoverageFunction();
        }
        break;

        case Land:
        break;
    }
}

void unknowAreaSearchFunction()
{

}

void boundarySearchFunction()
{

}
void sourceSearchFunction()
{

}

void fineCoverageFunction()
{

}

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
    ros::init(argc, argv, "collaborative_sourcing");
    ros::NodeHandle nh;

    //position of A
    position_A.pose.position.x = 5;
    position_A.pose.position.y = 0;
    position_A.pose.position.z = 5;
    //position of B
    position_B.pose.position.x = 15;
    position_B.pose.position.y = 0;
    position_B.pose.position.z = 5;
    //position of C
    position_C.pose.position.x = 15;
    position_C.pose.position.y = 5;
    position_C.pose.position.z = 5;


    string NS;
    /*-------uav0-------*/
    NS = string(NS0);
    // Subscriber
    ros::Subscriber uav0_state_sub = nh.subscribe<mavros_msgs::State>(NS+"/mavros/state", 10, uav0_state_cb);
    ros::Subscriber uav0_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(NS+"/mavros/local_position/pose", 10, uav0_pose_cb);
    ros::Subscriber uav0_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(NS+"/mavros/local_position/velocity", 10, uav0_velocity_cb);

    // Publisher
    uav0_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(NS+"/mavros/setpoint_velocity/cmd_vel", 10);
    uav0_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(NS+"/mavros/setpoint_position/local",10);

    // client
    uav0_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(NS+"/mavros/cmd/arming");
    uav0_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(NS+"/mavros/set_mode");
    uav0_set_mode_client_posctl = nh.serviceClient<mavros_msgs::SetMode>(string(NS)+"/mavros/set_mode");
    /*------------------*/

    /*-------uav1-------*/
    NS = string(NS1);
    // Subscriber
    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>(NS+"/mavros/state", 10, uav1_state_cb);
    ros::Subscriber uav1_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(NS+"/mavros/local_position/pose", 10, uav1_pose_cb);
    ros::Subscriber uav1_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(NS+"/mavros/local_position/velocity", 10, uav1_velocity_cb);

    // Publisher
    uav1_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(NS+"/mavros/setpoint_velocity/cmd_vel", 10);
    uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(NS+"/mavros/setpoint_position/local",10);

    // client
    uav1_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(NS+"/mavros/cmd/arming");
    uav1_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(NS+"/mavros/set_mode");
    uav1_set_mode_client_posctl = nh.serviceClient<mavros_msgs::SetMode>(string(NS)+"/mavros/set_mode");
    /*------------------*/

    /*-------uav2-------*/
    NS = string(NS2);
    // Subscriber
    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>(NS+"/mavros/state", 10, uav2_state_cb);
    ros::Subscriber uav2_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(NS+"/mavros/local_position/pose", 10, uav2_pose_cb);
    ros::Subscriber uav2_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(NS+"/mavros/local_position/velocity", 10, uav2_velocity_cb);

    // Publisher
    uav2_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(NS+"/mavros/setpoint_velocity/cmd_vel", 10);
    uav2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(NS+"/mavros/setpoint_position/local",10);

    // client
    uav2_arming_client = nh.serviceClient<mavros_msgs::CommandBool>(NS+"/mavros/cmd/arming");
    uav2_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(NS+"/mavros/set_mode");
    uav2_set_mode_client_posctl = nh.serviceClient<mavros_msgs::SetMode>(string(NS)+"/mavros/set_mode");
    /*------------------*/


    concentration_client = nh.serviceClient<sourcing::Concentration>("concentration");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // 等待mavros节点连接到飞控
    while(ros::ok() && !uav0_current_state.connected &&
    !uav1_current_state.connected && !uav2_current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //service client:switch mode to position control
    
    mavros_msgs::SetMode posc_set_mode;
	posc_set_mode.request.custom_mode = "POSCTL";

    ros::Time last_request = ros::Time::now();

    //firstly switch to position control mode before switch to offboard mode
    while(ros::ok() && 
        (uav0_current_state.mode != "POSCTL" || 
        uav1_current_state.mode != "POSCTL" || 
        uav2_current_state.mode != "POSCTL"))
    {
        if(ros::Time::now() - last_request > ros::Duration(5.0))
        {
            if(uav0_set_mode_client_posctl.call(posc_set_mode) && posc_set_mode.response.mode_sent && 
            uav1_set_mode_client_posctl.call(posc_set_mode) && posc_set_mode.response.mode_sent && 
            uav2_set_mode_client_posctl.call(posc_set_mode) && posc_set_mode.response.mode_sent)
            {
                ROS_INFO("Position control enabled");
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }

    uav0_cmd_vel.twist.linear.z = TAKEOFF_VELOCITY;
    uav1_cmd_vel.twist.linear.z = TAKEOFF_VELOCITY;
    uav2_cmd_vel.twist.linear.z = TAKEOFF_VELOCITY;
    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i)
    {
        uav0_cmd_vel_pub.publish(uav0_cmd_vel);
        uav1_cmd_vel_pub.publish(uav1_cmd_vel);
        uav2_cmd_vel_pub.publish(uav2_cmd_vel);
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode set_mode;
    set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
      
    
    last_request = ros::Time::now();


    while(ros::ok())
    {
        // if offboard, armed, state machine
        if(uav0_current_state.mode == "OFFBOARD" && uav0_current_state.armed &&
        uav1_current_state.mode == "OFFBOARD" && uav1_current_state.armed &&
        uav2_current_state.mode == "OFFBOARD" && uav2_current_state.armed)
        {
            stateMachineFunction();
        }
        else if(current_mission_state == TakeOff)
        {
            // before take off
            
            // uav0_cmd_vel_pub.publish(uav0_cmd_vel);
            // uav1_cmd_vel_pub.publish(uav1_cmd_vel);
            // uav2_cmd_vel_pub.publish(uav2_cmd_vel);
            uav0_local_pos_pub.publish(position_A);
            uav1_local_pos_pub.publish(position_B);
            uav2_local_pos_pub.publish(position_C);
        }



        // If not, switch to OFFBOARD mode and make vehicle armed
        if( current_mission_state == TakeOff &&
        uav0_current_state.mode != "OFFBOARD" &&
        uav1_current_state.mode != "OFFBOARD" &&
        uav2_current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( uav0_set_mode_client.call(set_mode) && set_mode.response.mode_sent && 
            uav1_set_mode_client.call(set_mode) && set_mode.response.mode_sent && 
            uav2_set_mode_client.call(set_mode) && set_mode.response.mode_sent) 
            {
            ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else if( !uav0_current_state.armed && !uav1_current_state.armed && !uav2_current_state.armed && 
        (ros::Time::now() - last_request > ros::Duration(5.0))) 
        {
            if( !uav0_current_state.armed && uav0_arming_client.call(arm_cmd) && arm_cmd.response.success) 
            {
                ROS_INFO("uav0 armed");
            }
            if( !uav1_current_state.armed && uav1_arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("uav1 armed");
            }
            if( !uav2_current_state.armed && uav2_arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("uav2 armed");
            }
            last_request = ros::Time::now();
        }
        

        // switch to land mode
        if((uav0_current_state.armed || uav1_current_state.armed ||
        uav2_current_state.armed) && current_mission_state == Land)
        {
            set_mode.request.custom_mode = "AUTO.LAND";
            if(ros::Time::now() - last_request > ros::Duration(5.0))
            {
                if(uav0_current_state.mode != "MANUAL" && uav0_current_state.mode != "AUTO.LAND")
                {
                    if(uav0_set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                    {
                        ROS_INFO("UAV0 AUTO LANDING"); 
                    }
                }
                if(uav1_current_state.mode != "MANUAL" && uav1_current_state.mode != "AUTO.LAND")
                {
                    if(uav1_set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                    {
                        ROS_INFO("UAV1 AUTO LANDING"); 
                    }
                }
                if(uav2_current_state.mode != "MANUAL" && uav2_current_state.mode != "AUTO.LAND")
                {
                    if(uav2_set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                    {
                        ROS_INFO("UAV2 AUTO LANDING"); 
                    }
                }

                last_request = ros::Time::now();
            } 
        }


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

