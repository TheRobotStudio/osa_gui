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
 * @file sequencer_node.cpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for the ROS class SequencerNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 21, 2017
 */

/*! Includes */
#include <ros/ros.h>
#include <ros/network.h>
#include <sequencer_node.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "robot_defines.h"

/*! Defines */
#define LOOP_RATE				HEART_BEAT

/*! Namespaces */
using namespace osa_gui;
using namespace rosnode;

/*! Implementation */
SequencerNode::SequencerNode() :
		enable_publish_(false),
		pause_(0)
{
}

SequencerNode::~SequencerNode()
{
	//delete motor_data_array_;

	if(ros::isStarted())
	{
	  ros::shutdown(); // explicitly needed since we use ros::start();
	  ros::waitForShutdown();
	}
	wait();
}

bool SequencerNode::init()
{
	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc, init_argv, "osa_gui_robotSequencerNode");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh;

	//Publishers
	pub_motor_cmd_array_ = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//Subscribers
	sub_motor_data_array_ = nh.subscribe("/motor_data_array", 10, &SequencerNode::motorDataArrayCallback, this);

	//create the commands multi array
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = 2; //NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = 2; //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "epos";
/*
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].label = "motors";
*/
	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(2); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

	start();

	return true;
}

void SequencerNode::run()
{
	ros::Rate r(LOOP_RATE);

	while(ros::ok())
	{
		//if(pause_ == 0)
		//{
			ros::spinOnce();

			//publish if enabled
			if(enable_publish_) pub_motor_cmd_array_.publish(motor_cmd_array_);

			//ros::Duration(5).sleep();

			resetMotorCmdMultiArray();

			r.sleep();
	/*	}
		else //apply a pause
		{
			ROS_INFO("SequencerNode::run : Apply a %d ms pause.", pause_);
			double sleep = (double)pause_;
			sleep /= 1000;
			ros::Duration(sleep).sleep();

			//reset the pause
			pause_ = 0;
		}*/

		//r.sleep(); //here?
	}

	//std::cout << "SequencerNode (ROS node) shutdown, proceeding to close the GUI." << std::endl;
	ROS_INFO("SequencerNode (ROS node) shutdown, proceeding to close the GUI.");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

int SequencerNode::setPause(int msPause)
{
	pause_ = msPause;

	return 0;
}

void SequencerNode::motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array_ = *data;
	Q_EMIT motorDataReceived(motor_data_array_);
}

int SequencerNode::setMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array)
{
	motor_cmd_array_ = motor_cmd_array;

	return 0;
}

int SequencerNode::setEnablePublish(bool state)
{
	enable_publish_ = state;

	return 0;
}

void SequencerNode::updateMotorCmdArray(osa_msgs::MotorCmdMultiArray motor_cmd_array)
{
	//TODO uncomment to enable the transmission
	//this is comming from the Face tracking node which drives the Head, Slave Board 1

	for(int i=0; i<2; i++)
	{
		motor_cmd_array_.motor_cmd[i].node_id = motor_cmd_array.motor_cmd[i].node_id;
		motor_cmd_array_.motor_cmd[i].command = motor_cmd_array.motor_cmd[i].command;
		motor_cmd_array_.motor_cmd[i].value = motor_cmd_array.motor_cmd[i].value;
	}

}

void SequencerNode::resetMotorCmdMultiArray()
{
	for(int i=0; i<2; i++)
	{
		motor_cmd_array_.motor_cmd[i].node_id = 0;
		motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array_.motor_cmd[i].value = 0;
	}
}
