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
 * @file BatteryMonitor.cpp
 * @author Cyril Jourdan
 * @date Dec 13, 2016
 * @version 0.0.1
 * @brief Implementation file for class BatteryMonitor
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 13, 2016
 */

#include <QJsonArray>
#include <iostream>
#include "BatteryMonitor.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
BatteryMonitor::BatteryMonitor() :
		Hardware(),
		m_vCellVoltage(QList<float>()),
		m_totalVoltage(0),
		m_intensity(0),
		m_temperature(0),
		m_pCommunicationLayer(NULL)
{

}

//destructor
BatteryMonitor::~BatteryMonitor()
{
	//clear pointers
	delete m_pCommunicationLayer;

	//clear Qlist
	m_vCellVoltage.clear();
}

//setters
int BatteryMonitor::addCellVoltage(float cellVoltage)
{
	//check the value
	if(cellVoltage > 0)
	{
		m_vCellVoltage.append(cellVoltage);

		return 0;
	}
	else
		return -1;
}

int BatteryMonitor::setCellVoltage(unsigned int cellNumber, float voltage)
{
	//check the value
	if((cellNumber < m_vCellVoltage.size()) && (voltage > 0))
	{
		m_vCellVoltage[cellNumber] = voltage;

		return 0;
	}
	else
		return -1;
}

int	BatteryMonitor::setTotalVoltage(float totalVoltage)
{
	//check the value
	if(totalVoltage > 0)
	{
		m_totalVoltage = totalVoltage;

		return 0;
	}
	else
		return -1;
}

int	BatteryMonitor::setIntensity(float	intensity)
{
	//check the value
	if(intensity > 0)
	{
		m_intensity = intensity;

		return 0;
	}
	else
		return -1;
}

int	BatteryMonitor::setTemperature(float temperature)
{
	//check the value
	if(temperature > -273.15)
	{
		m_temperature = temperature;

		return 0;
	}
	else
		return -1;
}

int BatteryMonitor::setCommunicationLayer(CommunicationLayer* pCommunicationLayer)
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

//getters
float BatteryMonitor::getCellVoltage(unsigned short int cellNumber) const
{
	if((cellNumber >= 0) && (cellNumber <= m_vCellVoltage.size()))
	{
		return m_vCellVoltage[cellNumber];
	}
	else
	{
		return -1;
	}
}

//other methods
void BatteryMonitor::display()
{
	cout << endl << "BatteryMonitor:" << endl;
	Hardware::display(); //call mother class method, Hardware
	cout << "List of cell voltages: ";
	for(unsigned int i=0; i<m_vCellVoltage.size(); i++)
	{
		cout << "[" << m_vCellVoltage[i] << "]";
	}
	cout << endl;
	cout << "TotalVoltage: " << m_totalVoltage << " V" << endl;
	cout << "Intensity: " << m_intensity << " A" << endl;
	cout << "Temperature: " << m_temperature << "°C" << endl;
}

void BatteryMonitor::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//write attributes
	//cellVoltage QList
	m_vCellVoltage.clear();
	QJsonArray cellVoltageArray = json["cellVoltage"].toArray();

	for(int i = 0; i<cellVoltageArray.size(); ++i)
	{
		float cellVoltage = (float)cellVoltageArray[i].toDouble();
		m_vCellVoltage.append(cellVoltage);
	}

	m_totalVoltage = (float)json["totalVoltage"].toDouble();
	m_intensity = (float)json["intensity"].toDouble();
	m_temperature = (float)json["temperature"].toDouble();

	//m_pCommunicationLayer->read(json); //TODO find a way to avoid infinite serialization loop
}

void BatteryMonitor::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//write attributes
	//cellVoltage QList
	QJsonArray cellVoltageArray;
	foreach(float val, m_vCellVoltage)
	{
		cellVoltageArray.append(val);
	}
	json["cellVoltage"] = cellVoltageArray;

    json["totalVoltage"] = (double)m_totalVoltage;
    json["intensity"] = (double)m_intensity;
    json["temperature"] = (double)m_temperature;

    //m_pCommunicationLayer->write(json); //TODO find a way to avoid infinite serialization loop
}
