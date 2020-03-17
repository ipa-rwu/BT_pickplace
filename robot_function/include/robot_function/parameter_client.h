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

        const std::string _server_arm = "arm_param";
        std::string param_arm_name[6] = {"T2S1arm", "T2S2arm", "T2S3arm", "T3S1arm", "T3S2arm", "T3S3arm"};
        std::string param_topic;
        double* param_arm; 

        double T2S2arm;
        double T2S3arm;
        double T3S1arm;
        double T3S2arm;
        double T3S3arm;


    public:
        ParamClient();
        ~ParamClient();

        std::string param_gripper[6];

        double T2S1arm;


        void armparam_callback(const dynamic_parameter::PickPlaceArmConfig &data);
        void gripparam_callback(const dynamic_parameter::PickPlaceGripConfig &data);

        bool get_param_arm(ros::NodeHandle nh, double param_arm_temp[], int size);
};


#endif /* PARAMETER_CLIENT_H_ */
