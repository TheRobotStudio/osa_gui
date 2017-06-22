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
 * @file Sensor.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for class Sensor
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <QJsonArray>
#include <iostream>
#include "Sensor.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

Sensor::Sensor() :
		Hardware(),
		m_vValue(),
		m_pCommunicationLayer(NULL),
		m_pComputer(NULL),
		m_pBattery(NULL)
{

}

Sensor::~Sensor()
{
	//clear pointers
	delete m_pCommunicationLayer;
	delete m_pComputer;
	delete m_pBattery;

	//clear vector
	m_vValue.clear();
}

//setters
int Sensor::addValue(double value)
{
		m_vValue.append(value);

		return 0;
}

int Sensor::setValue(unsigned int index, double value)
{
	//check the value
	if(index < m_vValue.size())
	{
		m_vValue[index] = value;

		return 0;
	}
	else
		return -1;
}

int Sensor::setPComputer(Computer* pComputer)
{
	//check the value
	if(pComputer != 0)
	{
		m_pComputer = pComputer;

		return 0;
	}
	else
		return -1;
}

int Sensor::setPCommunicationLayer(CommunicationLayer* pCommunicationLayer)
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

int Sensor::setPBattery(Battery* pBattery)
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


//other methods
void Sensor::display()
{
	cout << "Sensor" << endl;
	Hardware::display();
	cout << "List of values: ";
	for(unsigned int i=0; i<m_vValue.size(); i++)
	{
		cout << "[" << m_vValue[i] << "]";
	}
}

void Sensor::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//write attributes
	//cellVoltage QList
	m_vValue.clear();
	QJsonArray valueArray = json["value"].toArray();

	for(int i = 0; i<valueArray.size(); ++i)
	{
		float value = (float)valueArray[i].toDouble();
		m_vValue.append(value);
	}

	//m_pComputer->read(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->read(json);
	//m_pBattery->read(json);
}

void Sensor::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//write attributes
	//cellVoltage QList
	QJsonArray valueArray;
	foreach(float val, m_vValue)
	{
		valueArray.append(val);
	}
	json["value"] = valueArray;

    //m_pComputer->write(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->write(json);
	//m_pBattery->write(json);
}
