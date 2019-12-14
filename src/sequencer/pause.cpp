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
 * @file pause.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.1.0
 * @brief Implementation file for class Pause
 *
 * Contact: contact@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <pause.h>
#include <ros/ros.h>
#include <QJsonArray>
#include "robot_defines.h"

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

//constructors
Pause::Pause() :
	SequenceElement(),
	ms_duration_(0)
{

}

//destructor
Pause::~Pause()
{

}

//setters
int Pause::setMsDuration(uint32_t ms_duration)
{
	ms_duration_ = ms_duration;

	return 0;
}

void Pause::playElement(rosnode::SequencerNode* sequencerNode)
{
	ROS_INFO("Pause::playElement : Apply a %d ms pause.", ms_duration_);
	double sleep = (double)ms_duration_;
	sleep /= 1000;
	ros::Duration(sleep).sleep();

	//sequencerNode->setPause(ms_duration_);
	//))m_pause.setMsDuration(ms_duration_);
}

void Pause::read(const QJsonObject &json)
{
	SequenceElement::read(json);

	ms_duration_ = (uint32_t)json["ms_duration"].toDouble();
}

void Pause::write(QJsonObject &json) const
{
	SequenceElement::write(json);

	json["ms_duration"] = (double)ms_duration_;
}
