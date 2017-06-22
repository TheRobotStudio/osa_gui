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
 * @file SerialCommunication.cpp
 * @author Cyril Jourdan
 * @date Dec 14, 2016
 * @version 0.0.1
 * @brief Implementation file for class SerialCommunication
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 14, 2016
 */

#include <iostream>
#include "SerialCommunication.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
SerialCommunication::SerialCommunication() :
		CommunicationLayer(),
		m_filePath(QString(""))
{
}

//destructor
SerialCommunication::~SerialCommunication()
{
}

//setters
int SerialCommunication::setFilePath(QString filePath)
{
	//check the value
	if(!filePath.isEmpty())
	{
		m_filePath = filePath;

		return 0;
	}
	else
		return -1;
}

//other methods
char* SerialCommunication::read_c()
{
	char* data=0;

	return data;
}

int SerialCommunication::write_c(char* data)
{
	return 0;
}

void SerialCommunication::display()
{
	cout << endl << "SerialCommunication: " << endl;
	CommunicationLayer::display();
	cout << "File path: " << m_filePath.toStdString() << endl;
}

void SerialCommunication::read(const QJsonObject &json)
{
	//call mother class method
	CommunicationLayer::read(json);

	//read attributes
	m_filePath = json["filePath"].toString();
}

void SerialCommunication::write(QJsonObject &json) const
{
	//call mother class method
	CommunicationLayer::write(json);

	//write attributes
    json["filePath"] = m_filePath;
}
