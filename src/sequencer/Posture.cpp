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

#include <QJsonArray>
#include "Posture.hpp"
#include "robotDefines.h"

using namespace std;
using namespace osa_gui;
using namespace common::osa_msgs_json;
using namespace sequencer;
using namespace Qt;

//constructors
Posture::Posture() :
		SequenceElement(),
		m_pJSONMotorDataMultiArray(NULL)
		//m_msPauseAfter()
{

}

//destructor
Posture::~Posture()
{
	delete m_pJSONMotorDataMultiArray;
}

//setters
int Posture::setPJSONMotorDataMultiArray(JSONMotorDataMultiArray* pJSONMotorDataMultiArray)
{
	//check the value
	if(pJSONMotorDataMultiArray != 0)
	{
		m_pJSONMotorDataMultiArray = pJSONMotorDataMultiArray;

		return 0;
	}
	else
		return -1;
}

void Posture::playElement(rosnode::SequencerNode* sequencerNode)
{
	ROS_INFO("Posture::playElement : Apply a posture.");

	osa_msgs::MotorCmdMultiArray motorCmd_ma;// = new osa_msgs::MotorCmdMultiArray();

	//create the commands multi array
	motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorCmd_ma.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	motorCmd_ma.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motorCmd_ma.layout.dim[0].label = "slaves";

	motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorCmd_ma.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motorCmd_ma.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motorCmd_ma.layout.dim[1].label = "motors";

	motorCmd_ma.layout.data_offset = 0;

	motorCmd_ma.motorCmd.clear();
	motorCmd_ma.motorCmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_TARGET_POSITION;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = m_pJSONMotorDataMultiArray->getLpJSONMotorData().at(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)->getPosition();
		}
	}

	sequencerNode->setMotorCmd_ma(motorCmd_ma);

/*//this is done through the command builder
	//pause
	ros::Duration(0.1).sleep();

	//Send a rising edge on the controlword
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0x002F; //63;
		}
	}

	sequencerNode->setMotorCmd_ma(motorCmd_ma);

	ros::Duration(0.1).sleep();

	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0x003F; //63;
		}
	}

	sequencerNode->setMotorCmd_ma(motorCmd_ma);
*/
}

void Posture::read(const QJsonObject &json)
{
	SequenceElement::read(json);
	m_pJSONMotorDataMultiArray->read(json);
}

void Posture::write(QJsonObject &json) const
{
	SequenceElement::write(json);
	m_pJSONMotorDataMultiArray->write(json);
}
