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
 * @file SlaveBoard.cpp
 * @author Cyril Jourdan
 * @date Mar 24, 2017
 * @version 2.0.0
 * @brief Implementation file for class SlaveBoard
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#include <QJsonArray>
#include <iostream>
#include "SlaveBoard.hpp"
//#include "Controller.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
SlaveBoard::SlaveBoard() :
		Hardware(),
		m_boardID(0),
		m_lpController(QList<Controller*>())
		//m_vpCommunicationLayer(QList<CommunicationLayer*>()),
		//m_pBattery(0)
{
}

//destructor
SlaveBoard::~SlaveBoard()
{
	m_lpController.clear();
	//m_vpCommunicationLayer.clear();
	//delete m_pBattery;
}

//setters
int	SlaveBoard::setBoardID(unsigned int boardID)
{
	//check the value
	if(boardID > 0)
	{
		m_boardID = boardID;

		return 0;
	}
	else
		return -1;
}

int SlaveBoard::addPController(Controller* pController)
{
	//check the value
	if(pController != 0)
	{
		m_lpController.append(pController);

		return 0;
	}
	else
		return -1;
}
/*
int SlaveBoard::addCommunicationLayer(CommunicationLayer* pCommunicationLayer)
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

int SlaveBoard::setPBattery(Battery* pBattery)
{
	//check the value
	if(pBattery != 0)
	{
		m_pBattery = pBattery;

		return 0;
	}
	else
		return -1;
}*/

//other methods
void SlaveBoard::display()
{
	cout << "SlaveBoard:" << endl;
	Hardware::display(); //call the display method from mother class

	cout << "BoardID: " << m_boardID << endl;
}

void SlaveBoard::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//Controller pointer QList
	m_lpController.clear();
	QJsonArray controllerArray = json["lpControllerArray"].toArray();

	for(int i = 0; i<controllerArray.size(); ++i)
	{
		QJsonObject pControllerObject = controllerArray[i].toObject();
		Controller *pController = new Controller();
		pController->read(pControllerObject);
		m_lpController.append(pController);
	}

	//read attributes

	//m_pActuator->read(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->read(json);
	//m_pBattery->read(json);
}

void SlaveBoard::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//Controller pointer QList
	QJsonArray controllerArray;
	foreach(const Controller* pController, m_lpController)
	{
		QJsonObject pControllerObject;
		pController->write(pControllerObject);
		controllerArray.append(pControllerObject);
	}
	json["lpControllerArray"] = controllerArray;

	//write attributes

	//m_pActuator->write(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->write(json);
	//m_pBattery->write(json);
}
