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
 * @file Battery.cpp
 * @author Cyril Jourdan
 * @date Dec 13, 2016
 * @version 0.0.1
 * @brief Implementation file for class Battery
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 13, 2016
 */

#include <QJsonArray>
#include <iostream>
#include "Battery.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Battery::Battery() :
		Hardware(),
		m_type(QString("")),
		m_numberOfCells(0),
		m_capacity(0),
		m_nominalVoltage(0),
		m_pBatteryMonitor(NULL), //nullptr
		m_vpHardware(QList<Hardware*>())
{

}
/*
Battery::Battery(QString type, unsigned short int numberOfCells, unsigned int capacity, unsigned int nominalVoltage) :
		m_type(type), m_numberOfCells(numberOfCells), m_capacity(capacity), m_nominalVoltage(nominalVoltage), m_pBatteryMonitor(0)
{

}*/

//destructor
Battery::~Battery()
{
	//clear pointers
	delete m_pBatteryMonitor;

	/*
	for(unsigned int i=0; i<m_vpHardware.size(); i++)
	{
		delete m_vpHardware[i];
	}*/

	//clear QList
	m_vpHardware.clear();
}

//setters
int Battery::setType(QString type)
{
	//check the value
	if(!type.isEmpty())
	{
		m_type = type;

		return 0;
	}
	else
		return -1;
}

int Battery::setNumberOfCells(unsigned short int numberOfCells)
{
	//check the value
	if(numberOfCells > 0)
	{
		m_numberOfCells = numberOfCells;

		return 0;
	}
	else
		return -1;
}

int Battery::setCapacity(unsigned int capacity)
{
	//check the value
	if(capacity > 0)
	{
		m_capacity = capacity;

		return 0;
	}
	else
		return -1;
}

int Battery::setNominalVoltage(unsigned int nominalVoltage)
{
	//check the value
	if(nominalVoltage > 0)
	{
		m_nominalVoltage = nominalVoltage;

		return 0;
	}
	else
		return -1;
}

int Battery::setPBatteryMonitor(BatteryMonitor* pBatteryMonitor)
{
	//check the value
	if(pBatteryMonitor != 0)
	{
		m_pBatteryMonitor = pBatteryMonitor;

		return 0;
	}
	else
		return -1;
}

void Battery::addVpHardware(Hardware* pHardware)
{
	m_vpHardware.append(pHardware);
}

//other methods
void Battery::display()
{
	cout << endl << "Battery:" << endl;
	Hardware::display(); //call mother class method, Hardware
	cout << "Type: " << m_type.toStdString() << endl;
	cout << "N umber of cells: " << m_numberOfCells << endl;
	cout << "Capacity: " << m_capacity << " mAh" << endl;
	cout << "Nominal voltage: " << m_nominalVoltage << " V" << endl;
}

void Battery::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//write attributes
	m_type = json["type"].toString();
	m_numberOfCells = json["numberOfCells"].toDouble();
	m_capacity = (unsigned int)json["capacity"].toDouble();
	m_nominalVoltage = (unsigned int)json["nominalVoltage"].toDouble();

	m_pBatteryMonitor->read(json);
/*
	m_vpHardware.clear();
	QJsonArray hardwareArray = json["pHardware"].toArray();

	for(int i = 0; i<hardwareArray.size(); ++i)
	{
		QJsonObject pHardwareObject = hardwareArray[i].toObject();
		Hardware* pHardware;
		pHardware->read(pHardwareObject);
		m_vpHardware.append(pHardware);
	}*/
}

void Battery::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//write attributes
    json["type"] = m_type;
    json["numberOfCells"] = m_numberOfCells;
    json["capacity"] = (double)m_capacity;
    json["nominalVoltage"] = (double)m_nominalVoltage;

	m_pBatteryMonitor->write(json);

	/*
	QJsonArray hardwareArray;
	foreach(const Hardware* pHardware, m_vpHardware)
	{
		QJsonObject pHardwareObject;
		pHardware->write(pHardwareObject);
		hardwareArray.append(pHardwareObject);
	}

	json["pHardware"] = hardwareArray;
	*/
}
