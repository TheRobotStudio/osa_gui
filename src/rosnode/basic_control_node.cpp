/*
 * Copyright (c) 2017, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file basic_control_node.cpp
 * @author Cyril Jourdan
 * @date Mar 14, 2017
 * @version OSA 0.1.0
 * @brief Implementation file for the ROS class BasicControlNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 14, 2017
 */

/*! Includes */
#include <basic_control_node.h>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "robot_defines.h"

/*! Defines */
#define LOOP_RATE	HEART_BEAT

/*! Namespaces */
using namespace osa_gui;
using namespace rosnode;

BasicControlNode::BasicControlNode() :
	robot_name_(""),
	robot_can_device_(""),
	number_epos_boards_(0),
	dof_name_list_(QList<std::string>()),
	dof_type_list_(QList<std::string>()),
	dof_node_id_list_(QList<int>()),
	dof_controller_list_(QList<std::string>()),
	dof_motor_list_(QList<std::string>()),
	dof_inverted_list_(QList<bool>()),
	dof_mode_list_(QList<std::string>()),
	dof_value_list_(QList<int>()),
	enable_publish_(true)
{

}

BasicControlNode::~BasicControlNode()
{
	//delete motor_data_array_;

	if(ros::isStarted())
	{
	  ros::shutdown(); // this is necessary after using ros::start().
	  ros::waitForShutdown();
	}
	wait();
}

bool BasicControlNode::init()
{
	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_gui_robotBasicControlNode");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh;

	// Grab the parameters //TODO avoid WET code, do DRY code, same as in class CANLayer od osa_communication package
	try
	{
		//load robot parameters
		if(!nh.param("/robot/name", robot_name_, std::string("my_robot")))
		{
			ROS_WARN("No /robot/name found in YAML config file");
		}

		if(!nh.param("/robot/dof", number_epos_boards_, int(2)))
		{
			ROS_WARN("No /robot/dof found in YAML config file");
		}

		if(!nh.param("/robot/can_device", robot_can_device_, std::string("can0")))
		{
			ROS_WARN("No /robot/can_device found in YAML config file");
		}

		ROS_INFO("Robot name=%s, dof=%d, can=%s", robot_name_.c_str(), number_epos_boards_, robot_can_device_.c_str());
/*
		//load mobile_base parameters
		if(nh.searchParam("/mobile_base", mobile_base_str))
		{
			ROS_INFO("/mobile_base found in YAML config file");
		}
		else
		{
			ROS_WARN("No /mobile_base found in YAML config file");
		}
*/
		//load controllers parameters
		//Example:
		//controller1: {node_id: 1, name: 'right wheel', type: 'EPOS4', inverted: true, motor: 'EC90', mode: 'PROFILE_VELOCITY_MODE', value: 0}

		bool dof_exist = true;
		//start with controller 1
		int dof_idx = 1;
		std::string rad_str = "dof"; //common radical name

		while(dof_exist)
		{
			//create the string "controller+index" to search for the controller parameter with that index number
			std::ostringstream dof_idx_path;
			dof_idx_path << rad_str << dof_idx;

			std::string absolute_str = "absolute_str";

			//ROS_INFO("string=%s", dof_idx_path.str().c_str());

			if(nh.searchParam(dof_idx_path.str(), absolute_str))
			{
				//ROS_INFO("%s found in YAML config file", dof_idx_path.str().c_str());
				//ROS_INFO("absolute_str = %s", absolute_str.c_str());

				//create variables to store the controller parameters:
				std:: string name;
				std:: string type;
				int node_id = 0;
				std:: string controller;
				std:: string motor;
				bool inverted;
				std:: string mode;
				int value;

				//grab the parameters of the current controller

				//name
				std::ostringstream name_path;
				name_path << absolute_str << "/name";
				if(!nh.getParam(name_path.str(), name))
				{
					ROS_ERROR("Can't grab param name for %s", dof_idx_path.str().c_str());
					return false;
				}

				//type
				std::ostringstream type_path;
				type_path << absolute_str << "/type";
				if(!nh.getParam(type_path.str(), type))
				{
					ROS_ERROR("Can't grab param type for %s", dof_idx_path.str().c_str());
					return false;
				}

				//node_id
				std::ostringstream node_id_path;
				node_id_path << absolute_str << "/node_id";
				if(!nh.getParam(node_id_path.str(), node_id))
				{
					ROS_ERROR("Can't grab param node_id for %s", dof_idx_path.str().c_str());
					return false;
				}

				//controller
				std::ostringstream controller_path;
				controller_path << absolute_str << "/controller";
				if(!nh.getParam(controller_path.str(), controller))
				{
					ROS_ERROR("Can't grab param controller for %s", dof_idx_path.str().c_str());
					return false;
				}

				//motor
				std::ostringstream motor_path;
				motor_path << absolute_str << "/motor";
				if(!nh.getParam(motor_path.str(), motor))
				{
					ROS_ERROR("Can't grab param motor for %s", dof_idx_path.str().c_str());
					return false;
				}

				//inverted
				std::ostringstream inverted_path;
				inverted_path << absolute_str << "/inverted";
				if(!nh.getParam(inverted_path.str(), inverted))
				{
					ROS_ERROR("Can't grab param inverted for %s", dof_idx_path.str().c_str());
					return false;
				}

				//mode
				std::ostringstream mode_path;
				mode_path << absolute_str << "/mode";
				if(!nh.getParam(mode_path.str(), mode))
				{
					ROS_ERROR("Can't grab param mode for %s", dof_idx_path.str().c_str());
					return false;
				}

				//value
				std::ostringstream value_path;
				value_path << absolute_str << "/value";
				if(!nh.getParam(value_path.str(), value))
				{
					ROS_ERROR("Can't grab param value for %s", dof_idx_path.str().c_str());
					return false;
				}

				//print the dof parameters
				ROS_INFO("%s : name[%s], type[%s], node_id[%d], controller[%s], motor[%s], inverted[%d], mode[%s], value[%d]", dof_idx_path.str().c_str(),
						name.c_str(), type.c_str(), node_id, controller.c_str(), motor.c_str(), inverted, mode.c_str(), value);

				//save the dof data in the attributes
				//number_epos_boards_
				dof_name_list_.push_back(name);
				dof_type_list_.push_back(type);
				dof_node_id_list_.push_back(node_id);
				dof_controller_list_.push_back(controller);
				dof_motor_list_.push_back(motor);
				dof_inverted_list_.push_back(inverted);
				dof_mode_list_.push_back(mode);
				dof_value_list_.push_back(value);

				//increment to search for the next controller
				dof_idx++;
			}
			else
			{
				dof_exist = false;
				//ROS_INFO("No more controllers found in YAML config file");
			}

			//dof_exist = false;
		}

		dof_idx--;
		if(number_epos_boards_ == dof_idx) ROS_INFO("Same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, dof_idx);
		else
		{
			ROS_WARN("Not the same number of DOF(%d) and controllers(%d) defined in the YAML config file!", number_epos_boards_, dof_idx);
			throw 1;
		}

		/*
		nh.param("can_device", can_device_str, std::string("can0"));
		//mot1
		nh.param("controller1_type", controller1_type_str, std::string("EPOS4"));
		nh.param("motor1_type", motor1_type_str, std::string("EC90"));
		nh.param("motor1_inverted", motor1_inverted_bool, bool(true));
		nh.param("mode1", mode1_str, std::string("PROFILE_VELOCITY_MODE"));
		nh.param("value1", value1_int, int(0));
		//mot2
		nh.param("controller2_type", controller2_type_str, std::string("EPOS4"));
		nh.param("motor2_type", motor2_type_str, std::string("EC90"));
		nh.param("motor2_inverted", motor2_inverted_bool, bool(false));
		nh.param("mode2", mode2_str, std::string("PROFILE_VELOCITY_MODE"));
		//nh.param("value2", value2_int, int(0));
		nh.param("value2", value2_int);
		*/

		ROS_INFO("Parameters loaded successfully!\n");
	}
	catch(int exception)
	{
		ROS_ERROR("Parameters didn't load correctly!");
		ROS_ERROR("Please modify your YAML config file and try again.");

		return false;
	}

	//Publishers
	pub_motor_cmd_array_ = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//Subscribers
	sub_motor_data_array_ = nh.subscribe("/motor_data_array", 10, &BasicControlNode::motorDataArrayCallback, this);

	//create the commands multi array
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = number_epos_boards_; //NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = number_epos_boards_; //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "epos";
/*
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].label = "motors";
*/
	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(number_epos_boards_); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

/*
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "motors";
	motor_cmd_array_.layout.data_offset = 0;
	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(NUMBER_MAX_EPOS2_PER_SLAVE);


	for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
	{
		motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array_.motor_cmd[i].value = 0;
	}
*/

	//m_playback = false;
	start();

	return true;
}

void BasicControlNode::run()
{
	ros::Rate r(LOOP_RATE);

	while(ros::ok())
	{
		ros::spinOnce();


		//publish if enabled
		if(enable_publish_) pub_motor_cmd_array_.publish(motor_cmd_array_);

		//TODO check when the Q_SLOT is executed, make sure no command is reset before being sent.
		resetMotorCmdMultiArray();
/*
		//erase the command
		for(int i=0; i<NUMBER_MAX_EPOS2_PER_SLAVE; i++)
		{
			motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
			motor_cmd_array_.motor_cmd[i].value = 0;
		}
*/
		r.sleep();
	}

	//std::cout << "BasicControlNode (ROS node) shutdown, proceeding to close the GUI." << std::endl;
	ROS_INFO("BasicControlNode (ROS node) shutdown, proceeding to close the GUI.");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

/*
osa_msgs::MotorDataMultiArray BasicControlNode::getPMotorData_ma()
{
	return motor_data_array_;
}
*/

//callbacks
void BasicControlNode::motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array_ = *data;

	//osa_msgs::MotorDataMultiArray *m_pData_ma = 0;
	//*m_pData_ma = motor_data_array_;
	//ROS_INFO("Q_EMIT motorDataReceived(data)");

	Q_EMIT motorDataReceived(motor_data_array_);
}

int BasicControlNode::setEnablePublish(bool state)
{
	enable_publish_ = state;

	return 0;
}

void BasicControlNode::resetMotorCmdMultiArray()
{
	for(int i=0; i<number_epos_boards_; i++)
	{
			motor_cmd_array_.motor_cmd[i].node_id = 0;
			motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
			motor_cmd_array_.motor_cmd[i].value = 0;
	}
}

//Q_SLOTS
void BasicControlNode::setMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array)
{
	motor_cmd_array_ = motor_cmd_array;
}

//Q_SLOTS
void BasicControlNode::updateMotorCmdArray(osa_msgs::MotorCmd motorCmd)
{
	//TODO check if this line works
	//motor_cmd_array_.motor_cmd[motorCmd.node_id -1] = motorCmd;
	int index = dof_node_id_list_.indexOf(motorCmd.node_id);

	if(index != -1) //TODO exception
	{
		motor_cmd_array_.motor_cmd[index].node_id = motorCmd.node_id;
		motor_cmd_array_.motor_cmd[index].command = motorCmd.command;
		motor_cmd_array_.motor_cmd[index].value = motorCmd.value;
	}
	else
	{
		ROS_WARN("No node_id %d found in the list!", motorCmd.node_id);
	}
}
