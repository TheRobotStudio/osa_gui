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
 * @file /src/PlotNode.cpp
 *
 * @brief
 *
 * @date 17 March 2017
 **/


/*! Includes */
#include <include/rosnode/PlotNode.hpp>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "robotDefines.h"

/*! Defines */
#define LOOP_RATE				HEART_BEAT

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
	m_sub_motorDataArray = nh.subscribe("/motor_data_array", 10, &PlotNode::motorDataArray_cb, this);

	start();
	return true;
}

void PlotNode::motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	m_motorData_ma = *data;
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

osa_msgs::MotorDataMultiArray PlotNode::getMotorData_ma()
{
	return m_motorData_ma;
}

