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
 * @file json_motor_data.cpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 0.1.0
 * @brief Implementation file for class JSONMotorData
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#include <iostream>
#include <string>
#include "json_motor_data.h"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace osa_msgs_json;
using namespace Qt;

JSONMotorData::JSONMotorData() : position_(0), velocity_(0), current_(0), following_error_(0), status_(0), mode_of_operation_(0)
{
}

JSONMotorData::~JSONMotorData()
{
}


int JSONMotorData::setPosition(int32_t position)
{
	position_ = position;

	return 0;
}

int JSONMotorData::setCurrent(int16_t current)
{
		current_ = current;

		return 0;
}

int JSONMotorData::setStatus(uint16_t status)
{
	//check the value
	if(status >= 0)
	{
		status_ = status;

		return 0;
	}
	else
		return -1;
}

void JSONMotorData::read(const QJsonObject &json)
{
	//read attributes
	position_ = json["position"].toDouble();
	velocity_ = json["velocity"].toDouble();
	current_ = json["current"].toDouble();
	following_error_ = json["following_error"].toDouble();
	status_ = json["status"].toDouble();
	mode_of_operation_ = json["mode_of_operation"].toDouble();
}

void JSONMotorData::write(QJsonObject &json) const
{
	//write attributes
    json["position"] = position_;
    json["velocity"] = velocity_;
	json["current"] = current_;
	json["following_error"] = following_error_;
	json["status"] = status_;
	json["mode_of_operation"] = mode_of_operation_;
}
