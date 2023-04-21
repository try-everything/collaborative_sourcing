/*************************************************************************
@file           concentration_server.cpp
@date           2023/04/20 22:12
@author         wuminjiang
@email          wuminjiang@sia.cn
@description    calculate concentration at point(x,y,z)
*************************************************************************/

#include "ros/ros.h"
#include "sourcing/Concentration.h"
#include <cmath>

/***************************constant definition**************************/
#define PI 3.141592
#define eps 1e-32

/***************************variable definition**************************/
// Diffusion coefficient parameter: a,b,c,d
static const double a = 0.2684;
static const double b = 0.9709;
static const double c = 0.0103;
static const double d = 1.4270;
// Source strength
static const float Q = 1000.0;
// Wind speed
static const double u = 1.0;
// Effective height of leakage
static const double H = 100;


// Calculate concentration
bool calConcentration(sourcing::Concentration::Request &req, sourcing::Concentration::Response &res)
{
    // Diffusion coefficient in the y-direction and z-direction
    double sigy = 0.0;
    double sigz = 0.0;
    // Intermediate variable
    double Qpi = 0.0;
    double ex1 = 0.0;
    double ex2 = 0.0;
    float C = 0.0;
    int x = (int)round(req.x);
    int y = (int)round(req.y);
    int z = (int)round(req.z);

    sigy = a * pow(x, b);
    sigz = c * pow(x, d);
    Qpi = 2*Q / (2*PI*u*sigy*sigz + eps);
    ex1 = exp(-0.5 * pow((y / (sigy+eps)), 2));
    ex2 = exp(-0.5 * pow(((z-H) / (sigz+eps)), 2)) + exp(-0.5*pow(((z+H) / (sigz+eps)), 2));
    C = (float)Qpi * ex1 * ex2;

    res.concentration = C;

    ROS_INFO("request: x=%f, y=%f, z=%f", req.x, req.y, req.z);
    ROS_INFO("real: x=%d, y=%d, z=%d", x, y, z);
    ROS_INFO("sending back response: [%f]", res.concentration);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "concentration_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("concentration", calConcentration);
    ROS_INFO("Ready to calculate concentration.");
    ros::spin();

    return 0;
}