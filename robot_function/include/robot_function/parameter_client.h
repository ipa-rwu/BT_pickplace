#ifndef PARAMETER_CLIENT_H_
#define PARAMETER_CLIENT_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/client.h>
#include <dynamic_parameter/PickPlaceArmConfig.h>
#include <dynamic_parameter/PickPlaceGripConfig.h>

#include <boost/function.hpp>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/bind.hpp>

class ParamClient
{
    private:
    /* data */

        const std::string _server_arm = "arm_param_server";
        std::string _param_name_arm[6] = {"T2S1arm", "T2S2arm", "T2S3arm", "T3S1arm", "T3S2arm", "T3S3arm"};
        std::string _param_topic_arm;
        double* _param_arm; 

        const std::string _server_flag = "flag_param_server";
        std::string _param_name_flag[4] = {"FHelp" ,"FFoundObj", "FPicked", "FPlaced" };
        std::string _param_topic_flag;
        bool* _param_flag; 

        const std::string _server_gripper = "gripper_param_server";
        std::string _param_name_gripper[6] = {"T2S1gripper", "T2S2gripper", "T2S3gripper", "T3S1gripper", "T3S2gripper", "T3S3gripper"};
        std::string _param_topic_gripper;
        std::string* _param_gripper; 


    public:
        ParamClient();
        ~ParamClient();

        std::string param_gripper[6];

        double T2S1arm;


        void armparam_callback(const dynamic_parameter::PickPlaceArmConfig &data);
        void gripparam_callback(const dynamic_parameter::PickPlaceGripConfig &data);

        bool get_param_arm(ros::NodeHandle nh, double param_temp[], int size);
        bool get_param_gripper(ros::NodeHandle nh, std::string param_temp[], int size);
        bool get_param_flag(ros::NodeHandle nh, bool param_temp[], int size);
};


#endif /* PARAMETER_CLIENT_H_ */
