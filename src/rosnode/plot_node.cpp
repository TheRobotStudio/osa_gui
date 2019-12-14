/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 * @file /src/rosnode/plot_node.cpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version 0.1.0
 * @brief Implementation file for the ROS class PlotNode.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 17, 2017
 */


/*! Includes */
#include <plot_node.h>
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

PlotNode::PlotNode()
{
}

PlotNode::~PlotNode()
{
	if(ros::isStarted())
	{
	  ros::shutdown(); // explicitly needed since we use ros::start();
	  ros::waitForShutdown();
	}
	wait();
}

bool PlotNode::init()
{
	int init_argc = 0;
	char** init_argv = 0;
	ros::init(init_argc,init_argv,"osa_PlotNode");

	if (! ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh;

	//Subscribers
	sub_motor_data_array_ = nh.subscribe("/motor_data_array", 10, &PlotNode::motorDataArrayCallback, this);

	start();
	return true;
}

void PlotNode::motorDataArrayCallback(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	motor_data_array_ = *data;
	Q_EMIT motorDataReceived();
}

void PlotNode::run()
{
	ros::Rate loop_rate(LOOP_RATE);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();

	}

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

osa_msgs::MotorDataMultiArray PlotNode::getMotorDataArray()
{
	return motor_data_array_;
}
