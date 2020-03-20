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
    _param_arm = new double[ 6 ] { 0 };
    _param_flag = new bool[4]{false};
    _param_gripper = new std::string[6]{"none"};
    // dynamic_reconfigure::Client<dynamic_parameter::PickPlaceArmConfig> client(_server_arm, boost::bind(&ParamClient::armparam_callback, this, _1));

}



// typedef boost::function<void(const dynamic_parameter::PickPlaceArmConfig &)> CallBack;

void ParamClient::armparam_callback(const dynamic_parameter::PickPlaceArmConfig &data)
{
    _param_arm[0] = data.T2S1arm;
    // param_arm[1] = data.T2S1arm;
    // param_arm[2] = data.T2S1arm;
    // param_arm[3] = data.T2S1arm;
    // param_arm[4] = data.T2S1arm;
    // param_arm[5] = data.T2S1arm;

    ROS_INFO("double: %f", data.T2S1arm);
}

ParamClient::~ParamClient() 
{
    // True if tones_freq is not a null pointer
    if(_param_arm)
    {
        delete[] _param_arm;  
    } 
    
    if(_param_gripper) // True if tones_freq is not a null pointer
    {
        delete[] _param_gripper;
    }    

    if(_param_flag) // True if tones_freq is not a null pointer
    {
        delete[] _param_flag;
    }  
}


bool ParamClient::get_param_arm(ros::NodeHandle nh, double param_temp[], int size)
{
    for (int i = 0; i<size; i++)
    {
        _param_topic_arm = "/" + _server_arm + "/" + _param_name_arm[i];
        if (nh.getParam(_param_topic_arm, _param_arm[i]))
        {
            // ROS_INFO("double: %f",  param_arm[i]);
            param_temp[i] =  _param_arm[i];
        }
        else
        {
            return false;
        }
    }
    return true;
    
}

bool ParamClient::get_param_gripper(ros::NodeHandle nh, std::string param_temp[], int size)
{
    for (int i = 0; i<size; i++)
    {
        _param_topic_gripper = "/" + _server_gripper + "/" + _param_name_gripper[i];
        if (nh.getParam(_param_topic_arm, _param_gripper[i]))
        {
            // ROS_INFO("double: %f",  param_arm[i]);
            param_temp[i] =  _param_gripper[i];
        }
        else
        {
            return false;
        }
    }
    return true;
    
}

bool ParamClient::get_param_flag(ros::NodeHandle nh, bool param_temp[], int size)
{
    for (int i = 0; i<size; i++)
    {
        _param_topic_flag = "/" + _server_flag + "/" + _param_name_flag[i];
        if (nh.getParam(_param_topic_flag, _param_flag[i]))
        {
            // ROS_INFO("double: %f",  param_arm[i]);
            param_temp[i] =  _param_flag[i];
        }
        else
        {
            return false;
        }
    }
    return true;
    
}