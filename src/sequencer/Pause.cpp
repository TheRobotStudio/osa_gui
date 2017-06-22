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
 * @file Pause.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for class Pause
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <ros/ros.h>
#include <QJsonArray>
#include "Pause.hpp"
#include "robotDefines.h"

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

//constructors
Pause::Pause() :
		SequenceElement(),
		m_msDuration(0)
{

}

//destructor
Pause::~Pause()
{

}

//setters
int Pause::setMsDuration(uint32_t msDuration)
{
	m_msDuration = msDuration;

	return 0;
}

void Pause::playElement(rosnode::SequencerNode* sequencerNode)
{
	ROS_INFO("Pause::playElement : Apply a %d ms pause.", m_msDuration);
	double sleep = (double)m_msDuration;
	sleep /= 1000;
	ros::Duration(sleep).sleep();

	//sequencerNode->setPause(m_msDuration);
	//))m_pause.setMsDuration(m_msDuration);
}

void Pause::read(const QJsonObject &json)
{
	SequenceElement::read(json);

	m_msDuration = (uint32_t)json["msDuration"].toDouble();
}

void Pause::write(QJsonObject &json) const
{
	SequenceElement::write(json);

	json["msDuration"] = (double)m_msDuration;
}
