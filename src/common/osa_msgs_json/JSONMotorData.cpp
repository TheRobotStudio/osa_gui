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
 * @file JSONMotorData.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Implementation file for class JSONMotorData
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#include <iostream>
#include <string>
#include "JSONMotorData.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace osa_msgs_json;
using namespace Qt;

//constructors
JSONMotorData::JSONMotorData()
{
}

//destructor
JSONMotorData::~JSONMotorData()
{
}

//setters
int JSONMotorData::setPosition(int32_t position)
{
	m_position = position;

	return 0;
}

int JSONMotorData::setCurrent(int16_t current)
{
		m_current = current;

		return 0;
}

int JSONMotorData::setStatus(uint16_t status)
{
	//check the value
	if(status >= 0)
	{
		m_status = status;

		return 0;
	}
	else
		return -1;
}

void JSONMotorData::read(const QJsonObject &json)
{
	//write attributes
	m_position = json["position"].toDouble();
	m_current = json["current"].toDouble();
	m_status = json["status"].toDouble();
}

void JSONMotorData::write(QJsonObject &json) const
{
	//write attributes
    json["position"] = m_position;
	json["current"] = m_current;
	json["status"] = m_status;
}
