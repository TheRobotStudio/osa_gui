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
 * @file JSONMotorCmd.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Implementation file for class JSONMotorCmd
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#include <iostream>
#include <string>
#include "JSONMotorCmd.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace osa_msgs_json;
using namespace Qt;

//constructors
JSONMotorCmd::JSONMotorCmd()
{
}

//destructor
JSONMotorCmd::~JSONMotorCmd()
{
}

//setters
int JSONMotorCmd::setSlaveBoardID(uint8_t slaveBoardID)
{
	//check the value
	if(slaveBoardID >= 1)
	{
		m_slaveBoardID = slaveBoardID;

		return 0;
	}
	else
		return -1;
}

int JSONMotorCmd::setNodeID(uint8_t nodeID)
{
	//check the value
	if(nodeID >= 1)
	{
		m_nodeID = nodeID;

		return 0;
	}
	else
		return -1;
}

int JSONMotorCmd::setCommand(uint8_t command)
{
	//check the value
	//if(command >= 1)
	//{
		m_command = command;

		return 0;
	//}
	//else
		//return -1;
}

int JSONMotorCmd::setValue(int32_t value)
{
	//check the value
	//if(value >= 1)
	//{
		m_value = value;

		return 0;
	//}
	//else
		//return -1;
}

void JSONMotorCmd::read(const QJsonObject &json)
{
	//write attributes
	m_slaveBoardID = json["slaveBoardID"].toDouble();
	m_nodeID = json["nodeID"].toDouble();
	m_command = json["command"].toDouble();
	m_value = json["value"].toDouble();
}

void JSONMotorCmd::write(QJsonObject &json) const
{
	//write attributes
    json["slaveBoardID"] = m_slaveBoardID;
	json["nodeID"] = m_nodeID;
	json["command"] = m_command;
	json["value"] = m_value;
}
