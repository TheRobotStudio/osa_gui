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
 * @file MotionStyle.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for class MotionStyle
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <QJsonArray>
#include "MotionStyle.hpp"
#include "robotDefines.h"

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

//constructors
MotionStyle::MotionStyle() :
		SequenceElement(),
		m_modeOfOperation(CURRENT_MODE),
		m_profileAcceleration(1000),
		m_profileDeceleration(1000),
		m_profileVelocity(850),
		m_outputCurrentLimit(800)
{

}

//destructor
MotionStyle::~MotionStyle()
{

}

//setters
int MotionStyle::setModeOfOperation(int8_t modeOfOperation)
{
	m_modeOfOperation = modeOfOperation;

	return 0;
}

int MotionStyle::setProfileAcceleration(uint32_t profileAcceleration)
{
	m_profileAcceleration = profileAcceleration;

	return 0;
}

int MotionStyle::setProfileDeceleration(uint32_t profileDeceleration)
{
	m_profileDeceleration = profileDeceleration;

	return 0;
}

int MotionStyle::setProfileVelocity(uint32_t profileVelocity)
{
	m_profileVelocity = profileVelocity;

	return 0;
}

int MotionStyle::setOutputCurrentLimit(uint16_t outputCurrentLimit)
{
	m_outputCurrentLimit = outputCurrentLimit;

	return 0;
}

void MotionStyle::playElement(rosnode::SequencerNode* sequencerNode)
{

}

void MotionStyle::read(const QJsonObject &json)
{
	SequenceElement::read(json);

	m_modeOfOperation = (int8_t)json["modeOfOperation"].toDouble();
	m_profileAcceleration = (uint32_t)json["profileAcceleration"].toDouble();
	m_profileDeceleration = (uint32_t)json["profileDeceleration"].toDouble();
	m_profileVelocity = (uint32_t)json["profileVelocity"].toDouble();
	m_outputCurrentLimit = (uint16_t)json["outputCurrentLimit"].toDouble();
}

void MotionStyle::write(QJsonObject &json) const
{
	SequenceElement::write(json);

	json["modeOfOperation"] = (double)m_modeOfOperation;
	json["profileAcceleration"] = (double)m_profileAcceleration;
	json["profileDeceleration"] = (double)m_profileDeceleration;
	json["profileVelocity"] = (double)m_profileVelocity;
	json["outputCurrentLimit"] = (double)m_outputCurrentLimit;
}
