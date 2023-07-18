/*************************************************************************
@file           multi_uav_demo.cpp
@date           2023/06/30 11:42
                2023/07/06 15:36
@author         wuminjiang
@email          wuminjiang@sia.cn
@description    a state machine demo for testing that one node controls 3
                UAVs according to specific watpoints.
*************************************************************************/

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

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
geometry_msgs::PoseStamped uav0_pose_pub;
geometry_msgs::PoseStamped uav1_pose_pub;
geometry_msgs::PoseStamped uav2_pose_pub;
geometry_msgs::TwistStamped uav0_cmd_vel;
geometry_msgs::TwistStamped uav1_cmd_vel;
geometry_msgs::TwistStamped uav2_cmd_vel;


/***********************const variable definition************************/
#define NS0 "uav0"
#define NS1 "uav1"
#define NS2 "uav2"

//coordinate alignment
float XDistance10 = 5.0;
float YDistance10 = 0.0;
float ZDistance10 = 0.0;
float XDistance20 = 10.0;
float YDistance20 = 0.0;
float ZDistance20 = 0.0;

//home offset
float XDelta = -1000;
float YDelta = 0;

#define Proportion  0.01

#define TEST 0

//states
static const int Approach = 1;
static const int FollowWaypoint = 2;

static const float UnitTime = 1.0;

#define ApproachNumber 100
#define WaypointNumber0 5133
#define WaypointNumber1 4223
#define WaypointNumber2 4761

/***************************variable definition**************************/

int current_mission_state = Approach;

int ApproachPositionReady = 0;
int WaypointReady = 0;

// geometry_msgs::PoseStamped position_A;
// geometry_msgs::PoseStamped position_B;
// geometry_msgs::PoseStamped position_C;

geometry_msgs::PoseStamped uav0_approach_position[ApproachNumber];
geometry_msgs::PoseStamped uav1_approach_position[ApproachNumber];
geometry_msgs::PoseStamped uav2_approach_position[ApproachNumber];

geometry_msgs::PoseStamped uav0_waypoint[WaypointNumber0];
geometry_msgs::PoseStamped uav1_waypoint[WaypointNumber1];
geometry_msgs::PoseStamped uav2_waypoint[WaypointNumber2];

#if TEST == 1
float WaypointTest0[WaypointNumber0][3] = {{5, 10, 1}, {4, 11, 1}, {6, 13, 1}, {0, 15, 1}};
float WaypointTest1[WaypointNumber1][3] = {{10, 10, 2}, {9, 11, 2}, {11, 13, 2}, {5, 15, 2}};
float WaypointTest2[WaypointNumber2][3] = {{15, 10, 3}, {14, 11, 3}, {16, 13, 3}, {10, 15, 3}};
#else
float WaypointTest0[WaypointNumber0][3] = {0};
float WaypointTest1[WaypointNumber1][3] = {0};
float WaypointTest2[WaypointNumber2][3] = {0};
#endif


ros::Time CurrentTime;
ros::Time StartTime;

//different height to avoid collision
float Uav0Height = 1.0;
float Uav1Height = 2.0;
float Uav2Height = 3.0;

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
void approachFunction();
void followWaypointFunction();

vector<vector<int>> parseCSV(const string& filename)
{
    vector<vector<int>> data;
    ifstream file(filename);
    string line;

    while (getline(file, line)) {
        stringstream ss(line);
        vector<int> row;
        string cell;

        while (getline(ss, cell, ',')) {
            row.push_back(stoi(cell));
        }

        data.push_back(row);
    }

    return data;
}

void getWaypointFunction()
{
    if(TEST == 1)
    {
        for(int i = 0; i < WaypointNumber0; i++)
        {
            uav0_waypoint[i].pose.position.x = WaypointTest0[i][0];
            uav0_waypoint[i].pose.position.y = WaypointTest0[i][1];
            uav0_waypoint[i].pose.position.z = WaypointTest0[i][2];
        }
        for(int i = 0; i < WaypointNumber1; i++)
        {
            uav1_waypoint[i].pose.position.x = WaypointTest1[i][0];
            uav1_waypoint[i].pose.position.y = WaypointTest1[i][1];
            uav1_waypoint[i].pose.position.z = WaypointTest1[i][2];
        }
        for(int i = 0; i < WaypointNumber2; i++)
        {
            uav2_waypoint[i].pose.position.x = WaypointTest2[i][0];
            uav2_waypoint[i].pose.position.y = WaypointTest2[i][1];
            uav2_waypoint[i].pose.position.z = WaypointTest2[i][2];
        }
        WaypointReady = 1;
        ROS_INFO("Waypoint position test is ready!");
    }
    else
    {
        //read file here
        
        // vector<vector<int>> data0 = parseCSV("....../src/sourcing/data/waypoint0.csv");
        // vector<vector<int>> data1 = parseCSV("....../src/sourcing/data/waypoint1.csv");
        // vector<vector<int>> data2 = parseCSV("....../src/sourcing/data/waypoint2.csv");
        vector<vector<int>> data0 = parseCSV("/home/study/collaborative_sourcing/src/sourcing/data/waypoint0.csv");
        vector<vector<int>> data1 = parseCSV("/home/study/collaborative_sourcing/src/sourcing/data/waypoint1.csv");
        vector<vector<int>> data2 = parseCSV("/home/study/collaborative_sourcing/src/sourcing/data/waypoint2.csv");

        for(int i = 0;i < WaypointNumber0; i++)
        {
            uav0_waypoint[i].pose.position.x = Proportion * (data0[i][0] + XDelta);
            uav0_waypoint[i].pose.position.y = Proportion * (data0[i][1] + YDelta);
            uav0_waypoint[i].pose.position.z = Uav0Height;
        }
        for(int i = 0;i < WaypointNumber1; i++)
        {
            uav1_waypoint[i].pose.position.x = Proportion * (data1[i][0] + XDelta);
            uav1_waypoint[i].pose.position.y = Proportion * (data1[i][1] + YDelta);
            uav1_waypoint[i].pose.position.z = Uav1Height;
        }
        for(int i = 0;i < WaypointNumber2; i++)
        {
            uav2_waypoint[i].pose.position.x = Proportion * (data2[i][0] + XDelta);
            uav2_waypoint[i].pose.position.y = Proportion * (data2[i][1] + YDelta);
            uav2_waypoint[i].pose.position.z = Uav2Height;
        }

        WaypointReady = 1;
        ROS_INFO("Waypoint position is got!");
    
    }
}

void stateMachineFunction()
{
    switch(current_mission_state)
    {
        case Approach:
        {
            approachFunction();
        }
        break;
        case FollowWaypoint:
        {
            followWaypointFunction();
        }
        break;
    
        // case Land:
        break;
    }
}

void approachFunction()
{
    if((CurrentTime == StartTime) && (WaypointReady == 1))
    {
        for(int i = 0; i < ApproachNumber; i++)
        {
            uav0_approach_position[i].pose.position.x = uav0_current_pose.pose.position.x + (i+1) * (uav0_waypoint[0].pose.position.x - uav0_current_pose.pose.position.x) / ApproachNumber;
            uav0_approach_position[i].pose.position.y = uav0_current_pose.pose.position.y + (i+1) * (uav0_waypoint[0].pose.position.y - uav0_current_pose.pose.position.y) / ApproachNumber;
            uav0_approach_position[i].pose.position.z = Uav0Height;

            uav1_approach_position[i].pose.position.x = uav1_current_pose.pose.position.x + (i+1) * (uav1_waypoint[0].pose.position.x - XDistance10 - uav1_current_pose.pose.position.x) / ApproachNumber;
            uav1_approach_position[i].pose.position.y = uav1_current_pose.pose.position.y + (i+1) * (uav1_waypoint[0].pose.position.y - YDistance10 - uav1_current_pose.pose.position.y) / ApproachNumber;
            uav1_approach_position[i].pose.position.z = Uav1Height;

            uav2_approach_position[i].pose.position.x = uav2_current_pose.pose.position.x + (i+1) * (uav2_waypoint[0].pose.position.x - XDistance20 - uav2_current_pose.pose.position.x) / ApproachNumber;
            uav2_approach_position[i].pose.position.y = uav2_current_pose.pose.position.y + (i+1) * (uav2_waypoint[0].pose.position.y - YDistance20 - uav2_current_pose.pose.position.y) / ApproachNumber;
            uav2_approach_position[i].pose.position.z = Uav2Height;
        }
        CurrentTime = ros::Time::now();
        ApproachPositionReady = 1;
        ROS_INFO("Approach position is ready!");
    }
    else if((CurrentTime - StartTime) < ros::Duration(ApproachNumber*UnitTime))
    {
        if(ApproachPositionReady == 0)
        {
            ROS_INFO("Error: Approach position is not ready!");
            uav0_local_pos_pub.publish(uav0_current_pose);
            uav1_local_pos_pub.publish(uav1_current_pose);
            uav2_local_pos_pub.publish(uav2_current_pose);
            return;
        }

        CurrentTime = ros::Time::now();
        int i = (int)((CurrentTime.toSec() - StartTime.toSec()) / UnitTime);
        ROS_INFO("The number is %d", i);
        if(i >= 0 && i < ApproachNumber)
        {
            uav0_local_pos_pub.publish(uav0_approach_position[i]);
            uav1_local_pos_pub.publish(uav1_approach_position[i]);
            uav2_local_pos_pub.publish(uav2_approach_position[i]);
        }
        else
        {
            ROS_INFO("Approach mission is complete!");
            current_mission_state = FollowWaypoint;
            StartTime = ros::Time::now();
            CurrentTime = StartTime;
        }
        
    }
}

void followWaypointFunction()
{
     if(WaypointReady == 0)
    {
        ROS_INFO("Error: Waypoint is not ready!");
        uav0_local_pos_pub.publish(uav0_current_pose);
        uav1_local_pos_pub.publish(uav1_current_pose);
        uav2_local_pos_pub.publish(uav2_current_pose);
        return;
    }

    CurrentTime = ros::Time::now();
    int i = (int)((CurrentTime.toSec() - StartTime.toSec()) / UnitTime);

    if(i >= 0 && i < WaypointNumber0)
    {
        uav0_local_pos_pub.publish(uav0_waypoint[i]);
    }
    else
    {
        uav0_local_pos_pub.publish(uav0_waypoint[WaypointNumber0 - 1]);
    }

    if(i >= 0 && i < WaypointNumber1)
    {
        geometry_msgs::PoseStamped TempPosition;
        TempPosition.pose.position.x = uav1_waypoint[i].pose.position.x - XDistance10;
        TempPosition.pose.position.y = uav1_waypoint[i].pose.position.y - YDistance10;
        TempPosition.pose.position.z = uav1_waypoint[i].pose.position.z - ZDistance10;
        uav1_local_pos_pub.publish(TempPosition);
    }
    else
    {
        geometry_msgs::PoseStamped TempPosition;
        TempPosition.pose.position.x = uav1_waypoint[WaypointNumber1 - 1].pose.position.x - XDistance10;
        TempPosition.pose.position.y = uav1_waypoint[WaypointNumber1 - 1].pose.position.y - YDistance10;
        TempPosition.pose.position.z = uav1_waypoint[WaypointNumber1 - 1].pose.position.z - ZDistance10;
        uav1_local_pos_pub.publish(TempPosition);
    }

    if(i >= 0 && i < WaypointNumber2)
    {
        geometry_msgs::PoseStamped TempPosition;
        TempPosition.pose.position.x = uav2_waypoint[i].pose.position.x - XDistance20;
        TempPosition.pose.position.y = uav2_waypoint[i].pose.position.y - YDistance20;
        TempPosition.pose.position.z = uav2_waypoint[i].pose.position.z - ZDistance20;
        uav2_local_pos_pub.publish(TempPosition);
    }
    else
    {
        geometry_msgs::PoseStamped TempPosition;
        TempPosition.pose.position.x = uav2_waypoint[WaypointNumber2 - 1].pose.position.x - XDistance20;
        TempPosition.pose.position.y = uav2_waypoint[WaypointNumber2 - 1].pose.position.y - YDistance20;
        TempPosition.pose.position.z = uav2_waypoint[WaypointNumber2 - 1].pose.position.z - ZDistance20;
        uav2_local_pos_pub.publish(TempPosition);
    }

    if (i>WaypointNumber0 && i>WaypointNumber1 && i>WaypointNumber2)
    {
        ROS_INFO("Mission complete!");
    }
}

/****************************main function*******************************/
int main(int argc, char** argv){
    //ROS node init
    ros::init(argc, argv, "multi_uav_waypoint_node");
    ros::NodeHandle nh;


    string NS;
    /*-------uav0-------*/
    NS = string(NS0);
    //Subscriber
    ros::Subscriber uav0_state_sub = nh.subscribe<mavros_msgs::State>(NS+"/mavros/state", 10, uav0_state_cb);
    ros::Subscriber uav0_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(NS+"/mavros/local_position/pose", 10, uav0_pose_cb);
    ros::Subscriber uav0_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(NS+"/mavros/local_position/velocity", 10, uav0_velocity_cb);

    //Publisher
    uav0_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(NS+"/mavros/setpoint_velocity/cmd_vel", 10);
    uav0_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(NS+"/mavros/setpoint_position/local",10);

    /*------------------*/

    /*-------uav1-------*/
    NS = string(NS1);
    //Subscriber
    ros::Subscriber uav1_state_sub = nh.subscribe<mavros_msgs::State>(NS+"/mavros/state", 10, uav1_state_cb);
    ros::Subscriber uav1_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(NS+"/mavros/local_position/pose", 10, uav1_pose_cb);
    ros::Subscriber uav1_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(NS+"/mavros/local_position/velocity", 10, uav1_velocity_cb);

    //Publisher
    uav1_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(NS+"/mavros/setpoint_velocity/cmd_vel", 10);
    uav1_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(NS+"/mavros/setpoint_position/local",10);

    /*------------------*/

    /*-------uav2-------*/
    NS = string(NS2);
    //Subscriber
    ros::Subscriber uav2_state_sub = nh.subscribe<mavros_msgs::State>(NS+"/mavros/state", 10, uav2_state_cb);
    ros::Subscriber uav2_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(NS+"/mavros/local_position/pose", 10, uav2_pose_cb);
    ros::Subscriber uav2_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>(NS+"/mavros/local_position/velocity", 10, uav2_velocity_cb);

    //Publisher
    uav2_cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>(NS+"/mavros/setpoint_velocity/cmd_vel", 10);
    uav2_local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(NS+"/mavros/setpoint_position/local",10);

    /*------------------*/
    
    //get waypoint data here
    getWaypointFunction();
    // char* path = getcwd(path,50);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    //等待mavros节点连接到飞控
    while(ros::ok() && !uav0_current_state.connected &&
    !uav1_current_state.connected && !uav2_current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        
    }
    ROS_INFO("Connect success!");

    int SendPointFlag = 0;

    while(ros::ok())
    {
        //
        if( uav0_current_state.mode == "OFFBOARD" &&
        uav1_current_state.mode == "OFFBOARD" &&
        uav2_current_state.mode == "OFFBOARD" &&
        SendPointFlag == 0)
        {
            SendPointFlag = 1;
            StartTime = ros::Time::now();
            CurrentTime = StartTime;
        }
        else if(SendPointFlag == 1)
        {
            stateMachineFunction();
        }
        else
        {
            //发布控制指令
            uav0_local_pos_pub.publish(uav0_current_pose);
            uav1_local_pos_pub.publish(uav1_current_pose);
            uav2_local_pos_pub.publish(uav2_current_pose);
        }
        


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

