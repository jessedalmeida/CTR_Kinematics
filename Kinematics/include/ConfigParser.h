/*=========================================================================
  Program:   EPOS ConfigParser
  Language:  C++
  
  Jason Shrand, Vanderbilt University 2021. All rights reserved.
  DO NOT REDISTRIBUTE WITHOUT PERMISSION
  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE. See the above notices for more information.
=========================================================================*/
#ifndef CONFIG_PARSER_H
#define CONFIG_PARSER_H

#include <fstream>
#include <iostream>
#include <json/json.h>
#include <json/value.h>
#include <string>
#include "medlab_types.h"
#include "Constants.h"


class ConfigParser
{
    public:

    ////////////////////////////////////////////////////////
    //                        Structs                     //
    ////////////////////////////////////////////////////////


    // Configuration struct for running prostate robot
    typedef struct config_t
    {        
        //////////////////////////////
        //    Arm configuration     //
        //////////////////////////////
        unsigned int needle_tube_count; // The number of tubes composing the needle arm (2 or 3)
        unsigned int gripper_tube_count; // The number of tubes composing the gripper arm (2 or 3)

        unsigned int left_instrument; // Which instrument is in the left position (Constants.GRIPPER or Constants.NEEDLE)
        unsigned int right_instrument; // Which instrument is in the right position (Constants.GRIPPER or Constants.NEEDLE)

        //////////////////////////////
        //       Tube geometry      //
        //////////////////////////////

        CTR3RobotParams needle_3tubes; // The 3-tube parameters for the needle arm. Only valid if needle_tube_count is 3
        CTR2RobotParams needle_2tubes; // The 2-tube parameters for the needle arm. Only valid if needle_tube_count is 2

        CTR3RobotParams gripper_3tubes; // The 3-tube parameters for the gripper arm. Only valid if gripper_tube_count is 3
        CTR2RobotParams gripper_2tubes; // The 2-tube parameters for the gripper arm. Only valid if gripper_tube_count is 2

        //////////////////////////////
        //        ROS Topics        //
        //////////////////////////////




    } configuration;
    ////////////////////////////////////////////////////////

    // Constructor
    ConfigParser();

    // Parse the configuration parameters from the provided JSON file into the struct
    bool parse(unsigned int &num_motors, MotorType &motor_type, std::string &position_ros_topic);

    private:

  	const char* m_config_file;
  };
}

#endif // CONFIG_PARSER_H