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

BasicControlNode::BasicControlNode() : enable_publish_(true)
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

	//Publishers
	pub_motor_cmd_array_ = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//Subscribers
	sub_motor_data_array_ = nh.subscribe("/motor_data_array", 10, &BasicControlNode::motorDataArrayCallback, this);

	//create the commands multi array
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "slaves";

	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].label = "motors";

	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

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
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			//motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].node_id = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SEND_DUMB_MESSAGE;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0;
		}
	}
}

//Q_SLOTS
void BasicControlNode::setMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array)
{
	motor_cmd_array_ = motor_cmd_array;
}

//Q_SLOTS
void BasicControlNode::updateMotorCmd_ma(osa_msgs::MotorCmd motorCmd)
{
	//TODO check if this line works
	//motor_cmd_array_.motor_cmd[motorCmd.node_id -1] = motorCmd;
	int index = motorCmd.node_id-1; //(motorCmd.slaveBoardID-1)*NUMBER_MAX_EPOS2_PER_SLAVE + motorCmd.node_id-1;

	//update one specific command based on the Slave ID and Node ID.
	//motor_cmd_array_.motor_cmd[index].slaveBoardID = motorCmd.slaveBoardID;
	motor_cmd_array_.motor_cmd[index].node_id = motorCmd.node_id;
	motor_cmd_array_.motor_cmd[index].command = motorCmd.command;
	motor_cmd_array_.motor_cmd[index].value = motorCmd.value;
}
