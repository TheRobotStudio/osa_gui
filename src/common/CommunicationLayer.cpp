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
 * @file CommunicationLayer.cpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Implementation file for class CommunicationLayer
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#include <iostream>
#include "CommunicationLayer.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
CommunicationLayer::CommunicationLayer() :
		m_name(QString("")),
		m_state(false)
{
}

/*
CommunicationLayer::CommunicationLayer(QString name) : m_name(name), m_state(false)
{
}*/
/*
//copy constructor
CommunicationLayer::CommunicationLayer(CommunicationLayer const& communicationLayer) : m_name(communicationLayer.m_name), m_state(communicationLayer.m_state)
{
}
*/
//destructor
CommunicationLayer::~CommunicationLayer()
{
}

//setters
int CommunicationLayer::setName(QString name)
{
	//check the value
	if(!name.isEmpty())
	{
		m_name = name;

		return 0;
	}
	else
		return -1;
}

int CommunicationLayer::setState(bool state)
{
	//check the value
	if((state == true) || (state == false))
	{
		m_state = state;

		return 0;
	}
	else
		return -1;
}

//other methods
int CommunicationLayer::openCommunication()
{
	if(m_state == false)
	{
		setState(true);
		return 0;
	}
	else
		return -1;
}

int CommunicationLayer::closeCommunication()
{
	if(m_state == true)
	{
		setState(false);
		return 0;
	}
	else
		return -1;
}

void CommunicationLayer::display()
{
	cout << endl << "CommunicationLayer: " << endl;
	cout << "Name: " << m_name.toStdString() << endl;
	cout << "State: " << m_state << endl;
}

void CommunicationLayer::read(const QJsonObject &json)
{
	m_name = json["name"].toString();
	m_state = json["state"].toBool();
}

void CommunicationLayer::write(QJsonObject &json) const
{
	json["name"] = m_name;
	json["state"] = m_state;
}
