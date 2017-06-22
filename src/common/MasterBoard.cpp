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
 * @file MasterBoard.cpp
 * @author Cyril Jourdan
 * @date Mar 24, 2017
 * @version 2.0.0
 * @brief Implementation file for class MasterBoard
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 24, 2017
 */

#include <QJsonArray>
#include <iostream>
#include "MasterBoard.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
MasterBoard::MasterBoard() :
		Hardware(),
		m_lpSlaveBoard(QList<SlaveBoard*>())
		//m_pCommunicationLayer(0),
		//m_pBattery(0)
{
}

//destructor
MasterBoard::~MasterBoard()
{
	m_lpSlaveBoard.clear();
	//delete m_pCommunicationLayer;
	//delete m_pBattery;
}

//setters
int MasterBoard::addPSlaveBoard(SlaveBoard *pslaveBoard)
{
	//check the value
	if(pslaveBoard != 0)
	{
		m_lpSlaveBoard.append(pslaveBoard);

		return 0;
	}
	else
		return -1;
}
/*
int MasterBoard::setPCommunicationLayer(CommunicationLayer* pCommunicationLayer)
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

int MasterBoard::setPBattery(Battery* pBattery)
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
void MasterBoard::display()
{
	cout << "MasterBoard:" << endl;
	Hardware::display(); //call the display method from mother class
}

void MasterBoard::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//SlaveBoard pointer QList
	m_lpSlaveBoard.clear();
	QJsonArray slaveBoardArray = json["lpSlaveBoardArray"].toArray();

	for(int i = 0; i<slaveBoardArray.size(); ++i)
	{
		QJsonObject pSlaveBoardObject = slaveBoardArray[i].toObject();
		SlaveBoard* pSlaveBoard;
		pSlaveBoard->read(pSlaveBoardObject);
		m_lpSlaveBoard.append(pSlaveBoard);
	}

	//read attributes

	//m_pActuator->read(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->read(json);
	//m_pBattery->read(json);
}

void MasterBoard::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//SlaveBoard pointer QList
	QJsonArray slaveBoardArray;
	foreach(const SlaveBoard* pSlaveBoard, m_lpSlaveBoard)
	{
		QJsonObject pSlaveBoardObject;
		pSlaveBoard->write(pSlaveBoardObject);
		slaveBoardArray.append(pSlaveBoardObject);
	}
	json["lpSlaveBoardArray"] = slaveBoardArray;

	//write attributes
	//m_pActuator->write(json); //TODO find a way to avoid infinite serialization loop
	//m_pCommunicationLayer->write(json);
	//m_pBattery->write(json);
}
