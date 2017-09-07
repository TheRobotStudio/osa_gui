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
 * @file motion_style.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for class MotionStyle.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <motion_style.h>
#include <QJsonArray>
#include "robot_defines.h"

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

MotionStyle::MotionStyle() :
	SequenceElement(),
	mode_of_operation_(CURRENT_MODE),
	profile_acceleration_(1000),
	profile_deceleration_(1000),
	profile_velocity_(850),
	output_current_limit_(800)
{
}

MotionStyle::~MotionStyle()
{
}

int MotionStyle::setModeOfOperation(int8_t mode_of_operation)
{
	mode_of_operation_ = mode_of_operation;

	return 0;
}

int MotionStyle::setProfileAcceleration(uint32_t profile_acceleration)
{
	profile_acceleration_ = profile_acceleration;

	return 0;
}

int MotionStyle::setProfileDeceleration(uint32_t profile_deceleration)
{
	profile_deceleration_ = profile_deceleration;

	return 0;
}

int MotionStyle::setProfileVelocity(uint32_t profile_velocity)
{
	profile_velocity_ = profile_velocity;

	return 0;
}

int MotionStyle::setOutputCurrentLimit(uint16_t output_current_limit)
{
	output_current_limit_ = output_current_limit;

	return 0;
}

void MotionStyle::playElement(rosnode::SequencerNode* sequencerNode)
{

}

void MotionStyle::read(const QJsonObject &json)
{
	SequenceElement::read(json);

	mode_of_operation_ = (int8_t)json["mode_of_operation"].toDouble();
	profile_acceleration_ = (uint32_t)json["profile_acceleration"].toDouble();
	profile_deceleration_ = (uint32_t)json["profile_deceleration"].toDouble();
	profile_velocity_ = (uint32_t)json["profile_velocity"].toDouble();
	output_current_limit_ = (uint16_t)json["output_current_limit"].toDouble();
}

void MotionStyle::write(QJsonObject &json) const
{
	SequenceElement::write(json);

	json["mode_of_operation"] = (double)mode_of_operation_;
	json["profile_acceleration"] = (double)profile_acceleration_;
	json["profile_deceleration"] = (double)profile_deceleration_;
	json["profile_velocity"] = (double)profile_velocity_;
	json["output_current_limit"] = (double)output_current_limit_;
}
