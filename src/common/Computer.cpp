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
 * @file Computer.cpp
 * @author Cyril Jourdan
 * @date Jan 26, 2017
 * @version 0.0.1
 * @brief Implementation file for class Computer
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 14, 2016
 */

#include <QJsonArray>
#include <iostream>
#include "Computer.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Computer::Computer() :
		Hardware(),
		m_pBattery(),
		m_vpCommunicationLayer(QList<CommunicationLayer*>()),
		m_vpUSBDevice(QList<USBDevice*>())
{

}

//destructor
Computer::~Computer()
{
	//clear pointers
	delete m_pBattery;

	for(unsigned int i=0; i<m_vpCommunicationLayer.size(); i++)
	{
		delete m_vpCommunicationLayer[i];
	}

	for(unsigned int i=0; i<m_vpUSBDevice.size(); i++)
	{
		delete m_vpUSBDevice[i];
	}

	//clear vectors
	m_vpCommunicationLayer.clear();
	m_vpUSBDevice.clear();
}

//setters
int Computer::setPBattery(Battery* pBattery)
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

int Computer::addPCommunicationLayer(CommunicationLayer* pCommunicationLayer)
{
	//check the value
	if(pCommunicationLayer != 0)
	{
		m_vpCommunicationLayer.append(pCommunicationLayer);

		return 0;
	}
	else
		return -1;
}

int Computer::addPUSBDevice(USBDevice* pUSBDevice)
{
	//check the value
	if(pUSBDevice != 0)
	{
		m_vpUSBDevice.append(pUSBDevice);

		return 0;
	}
	else
		return -1;
}

//other methods
void Computer::display()
{
	cout << endl << "Computer: " << endl;
	Hardware::display();
	for(unsigned int i=0; i<m_vpCommunicationLayer.size(); i++)
	{
		cout << "- Communication layer [" << i << "]:" << endl;
		m_vpCommunicationLayer[i]->display();
	}
	for(unsigned int i=0; i<m_vpUSBDevice.size(); i++)
	{
		cout << "- USB device [" << i << "]:" << endl;
		m_vpUSBDevice[i]->display();
	}
}

void Computer::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//write attributes
	//m_pBattery->read(json); //TODO find a way to avoid infinite serialization loop

	//CommunicationLayer pointer QList
	m_vpCommunicationLayer.clear();
	QJsonArray communicationLayerArray = json["pCommunicationLayer"].toArray();

	for(int i = 0; i<communicationLayerArray.size(); ++i)
	{
		QJsonObject pCommunicationLayerObject = communicationLayerArray[i].toObject();
		CommunicationLayer* pCommunicationLayer;
		pCommunicationLayer->read(pCommunicationLayerObject);
		m_vpCommunicationLayer.append(pCommunicationLayer);
	}

	//USBDevice pointer QList
	m_vpUSBDevice.clear();
	QJsonArray usbDeviceArray = json["pUSBDevice"].toArray();

	for(int i = 0; i<usbDeviceArray.size(); ++i)
	{
		QJsonObject pUSBDeviceObject = usbDeviceArray[i].toObject();
		USBDevice* pUSBDevice;
		pUSBDevice->read(pUSBDeviceObject);
		m_vpUSBDevice.append(pUSBDevice);
	}
}

void Computer::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//write attributes
	//m_pBattery->write(json); //TODO find a way to avoid infinite serialization loop

	//CommunicationLayer pointer QList
	QJsonArray communicationLayerArray;
	foreach(const CommunicationLayer* pCommunicationLayer, m_vpCommunicationLayer)
	{
		QJsonObject pCommunicationLayerObject;
		pCommunicationLayer->write(pCommunicationLayerObject);
		communicationLayerArray.append(pCommunicationLayerObject);
	}
	json["pCommunicationLayer"] = communicationLayerArray;

	//USBDevice pointer QList
	QJsonArray usbDeviceArray;
	foreach(const USBDevice* pUSBDevice, m_vpUSBDevice)
	{
		QJsonObject pUSBDeviceObject;
		pUSBDevice->write(pUSBDeviceObject);
		usbDeviceArray.append(pUSBDeviceObject);
	}
	json["pUSBDevice"] = usbDeviceArray;
}
