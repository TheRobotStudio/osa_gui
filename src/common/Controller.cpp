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
 * @file Controller.cpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Implementation file for class Controller
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#include <iostream>
#include "Controller.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Controller::Controller() :
		Hardware(),
		m_status(0),
		m_nodeID(0),
		m_position(0),
		m_current(0),
		m_velocity(0),
		m_pActuator()
		//m_pCommunicationLayer(nullptr),
		//m_pBattery(nullptr)
{
}

//destructor
Controller::~Controller()
{
	delete m_pActuator;
	//delete m_pCommunicationLayer;
	//delete m_pBattery;
}

//setters
int	Controller::setStatus(int status)
{
	//check the value
	if(status > 0)
	{
		m_status = status;

		return 0;
	}
	else
		return -1;
}

int	Controller::setNodeID(unsigned int nodeID)
{
	//check the value
	if(nodeID > 0)
	{
		m_nodeID = nodeID;

		return 0;
	}
	else
		return -1;
}

int Controller::setPosition(int position)
{
	//check the value
	if(position > 0)
	{
		m_position = position;

		return 0;
	}
	else
		return -1;
}

int Controller::setCurrent(int current)
{
	//check the value
	if(current > 0)
	{
		m_current = current;

		return 0;
	}
	else
		return -1;
}

int Controller::setVelocity(int velocity)
{
	//check the value
	if(velocity > 0)
	{
		m_velocity = velocity;

		return 0;
	}
	else
		return -1;
}

int Controller::setPActuator(Actuator* pActuator)
{
	//check the value
	if(pActuator != 0)
	{
		m_pActuator = pActuator;

		return 0;
	}
	else
		return -1;
}
/*
int Controller::setPCommunicationLayer(CommunicationLayer* pCommunicationLayer)
{
	//check the value
	if(pCommunicationLayer != 0)
	{
		m_pCommunicationLayer = pCommunicationLayer;

		return 0;
	}
	else
		return -1;
}

int Controller::setPBattery(Battery* pBattery)
{
	//check the value
	if(pBattery != 0)
	{
		m_pBattery = pBattery;

		return 0;
	}
	else
		return -1;
}
*/

//other methods
void Controller::display()
{
	cout << "Controller:" << endl;
	Hardware::display(); //call the display method from mother class
	cout << "Status: " << m_status << endl;
	cout << "NodeID: " << m_nodeID << endl;
	cout << "Position: " << m_position << endl;
	cout << "Current: " << m_current << endl;
	cout << "Velocity: " << m_velocity << endl;
}

void Controller::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//read attributes
	m_status = (int)json["status"].toDouble();
	m_nodeID = (unsigned int)json["nodeID"].toDouble();
	m_position = (int)json["position"].toDouble();
	m_current = (int)json["current"].toDouble();
	m_velocity = (int)json["velocity"].toDouble();
	m_pActuator->read(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->read(json);
	//m_pBattery->read(json);
}

void Controller::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//write attributes
	json["status"] = (double)m_status;
	json["nodeID"] = (double)m_nodeID;
	json["position"] = (double)m_position;
	json["current"] = (double)m_current;
	json["velocity"] = (double)m_velocity;
	m_pActuator->write(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->write(json);
	//m_pBattery->write(json);
}
