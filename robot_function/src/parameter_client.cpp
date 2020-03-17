#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_parameter/PickPlaceArmConfig.h>
#include <boost/function.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include "robot_function/parameter_client.h"

using namespace std;


typedef boost::function<void(const dynamic_parameter::PickPlaceArmConfig &)> CallBack;


ParamClient::ParamClient()
{
    param_arm = new double[ 6 ] { 0, 0, 0, 0, 0, 0 };
    dynamic_reconfigure::Client<dynamic_parameter::PickPlaceArmConfig> client(_server_arm, boost::bind(&ParamClient::armparam_callback, this, _1));

}



// typedef boost::function<void(const dynamic_parameter::PickPlaceArmConfig &)> CallBack;

void ParamClient::armparam_callback(const dynamic_parameter::PickPlaceArmConfig &data)
{
    param_arm[0] = data.T2S1arm;
    // param_arm[1] = data.T2S1arm;
    // param_arm[2] = data.T2S1arm;
    // param_arm[3] = data.T2S1arm;
    // param_arm[4] = data.T2S1arm;
    // param_arm[5] = data.T2S1arm;

    ROS_INFO("double: %f", data.T2S1arm);
}

ParamClient::~ParamClient() 
{
    if(param_arm) // True if tones_freq is not a null pointer
        delete[] param_arm;
}


void ParamClient::get_param_arm(ros::NodeHandle nh)
{

}