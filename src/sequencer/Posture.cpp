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
 * @file Posture.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for abstract class Posture
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <posture.h>
#include <QJsonArray>
#include "robot_defines.h"

using namespace std;
using namespace osa_gui;
using namespace common::osa_msgs_json;
using namespace sequencer;
using namespace Qt;

Posture::Posture(int number_epos_boards) :
	SequenceElement(),
	number_epos_boards_(number_epos_boards_),
	ptr_json_motor_data_array_(NULL)
	//m_msPauseAfter()
{
}

Posture::~Posture()
{
	delete ptr_json_motor_data_array_;
}

int Posture::setJSONMotorDataArray(JSONMotorDataMultiArray* ptr_json_motor_data_array)
{
	//check the value
	if(ptr_json_motor_data_array != 0)
	{
		ptr_json_motor_data_array_ = ptr_json_motor_data_array;

		return 0;
	}
	else
		return -1;
}

void Posture::playElement(rosnode::SequencerNode* sequencerNode)
{
	ROS_INFO("Posture::playElement : Apply a posture.");

	osa_msgs::MotorCmdMultiArray motor_cmd_array;// = new osa_msgs::MotorCmdMultiArray();

	//create the commands multi array
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[0].size = number_epos_boards_; //NUMBER_SLAVE_BOARDS;
	motor_cmd_array.layout.dim[0].stride = number_epos_boards_; //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[0].label = "epos";
/*
	motor_cmd_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array.layout.dim[1].label = "motors";
*/
	motor_cmd_array.layout.data_offset = 0;

	motor_cmd_array.motor_cmd.clear();
	motor_cmd_array.motor_cmd.resize(number_epos_boards_); // NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<number_epos_boards_; i++)
	{
		//motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
		motor_cmd_array.motor_cmd[i].node_id = ptr_json_motor_data_array_->getJSONMotorDataList().at(i)->getNodeID(); //j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
		motor_cmd_array.motor_cmd[i].command = SET_TARGET_POSITION;
		motor_cmd_array.motor_cmd[i].value = ptr_json_motor_data_array_->getJSONMotorDataList().at(i)->getPosition();
	}

	sequencerNode->setMotorCmdArray(motor_cmd_array);

/*//this is done through the command builder
	//pause
	ros::Duration(0.1).sleep();

	//Send a rising edge on the controlword
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0x002F; //63;
		}
	}

	sequencerNode->setMotorCmd_ma(motor_cmd_array);

	ros::Duration(0.1).sleep();

	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			motor_cmd_array.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0x003F; //63;
		}
	}

	sequencerNode->setMotorCmd_ma(motor_cmd_array);
*/
}

void Posture::read(const QJsonObject &json)
{
	SequenceElement::read(json);
	ptr_json_motor_data_array_->read(json);
}

void Posture::write(QJsonObject &json) const
{
	SequenceElement::write(json);
	ptr_json_motor_data_array_->write(json);
}
