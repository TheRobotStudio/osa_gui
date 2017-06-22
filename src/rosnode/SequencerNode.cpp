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
 * @file SequencerNode.hpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for the ROS class SequencerNode.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 21, 2017
 */

/*! Includes */
#include <include/rosnode/SequencerNode.hpp>
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

/*! Implementation */
SequencerNode::SequencerNode() :
		m_enablePublish(false),
		m_pause(0)
{
}

SequencerNode::~SequencerNode()
{
	//delete m_motorData_ma;

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
	m_pub_motorCmdArray = nh.advertise<osa_msgs::MotorCmdMultiArray>("/set_motor_commands", 100, true);

	//Subscribers
	m_sub_motorDataArray = nh.subscribe("/motor_data_array", 10, &SequencerNode::motorDataArray_cb, this);

	//create the commands multi array
	m_motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m_motorCmd_ma.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	m_motorCmd_ma.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorCmd_ma.layout.dim[0].label = "slaves";

	m_motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m_motorCmd_ma.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorCmd_ma.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorCmd_ma.layout.dim[1].label = "motors";

	m_motorCmd_ma.layout.data_offset = 0;

	m_motorCmd_ma.motorCmd.clear();
	m_motorCmd_ma.motorCmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

	start();

	return true;
}

void SequencerNode::run()
{
	ros::Rate r(LOOP_RATE);

	while(ros::ok())
	{
		//if(m_pause == 0)
		//{
			ros::spinOnce();

			//publish if enabled
			if(m_enablePublish) m_pub_motorCmdArray.publish(m_motorCmd_ma);

			//ros::Duration(5).sleep();

			resetMotorCmdMultiArray();

			r.sleep();
	/*	}
		else //apply a pause
		{
			ROS_INFO("SequencerNode::run : Apply a %d ms pause.", m_pause);
			double sleep = (double)m_pause;
			sleep /= 1000;
			ros::Duration(sleep).sleep();

			//reset the pause
			m_pause = 0;
		}*/

		//r.sleep(); //here?
	}

	//std::cout << "SequencerNode (ROS node) shutdown, proceeding to close the GUI." << std::endl;
	ROS_INFO("SequencerNode (ROS node) shutdown, proceeding to close the GUI.");
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

int SequencerNode::setPause(int msPause)
{
	m_pause = msPause;

	return 0;
}



//callbacks
void SequencerNode::motorDataArray_cb(const osa_msgs::MotorDataMultiArrayConstPtr& data)
{
	m_motorData_ma = *data;
	Q_EMIT motorDataReceived(m_motorData_ma);
}

//setters
int SequencerNode::setMotorCmd_ma(osa_msgs::MotorCmdMultiArray motorCmd_ma)
{
	m_motorCmd_ma = motorCmd_ma;

	return 0;
}

int SequencerNode::setEnablePublish(bool state)
{
	m_enablePublish = state;

	return 0;
}

//Q_SLOT
void SequencerNode::updateMotorCmd_ma(osa_msgs::MotorCmdMultiArray motorCmd_ma)
{
	//TODO uncomment to enable the transmission
	//this is comming from the Face tracking node which drives the Head, Slave Board 1

	for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
	{
		//m_motorCmd_ma.motorCmd[j].slaveBoardID = i + 1;
		//m_motorCmd_ma.motorCmd[j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
		m_motorCmd_ma.motorCmd[j].command = motorCmd_ma.motorCmd[j].command;
		m_motorCmd_ma.motorCmd[j].value = motorCmd_ma.motorCmd[j].value;
	}

}

void SequencerNode::resetMotorCmdMultiArray()
{
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SEND_DUMB_MESSAGE;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0;
		}
	}
}
