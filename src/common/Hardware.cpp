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
 * @file Hardware.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for class Hardware
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <iostream>
#include "Hardware.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Hardware::Hardware() :
		m_brand(QString("")),
		m_name(QString("")),
		m_partNumber(0),
		m_version(QString(""))
		//m_modelPath(QString(""))
{
}
/*
Hardware::Hardware(QString name) : m_brand(""), m_name(name), m_partNumber(0), m_version(""), m_modelPath("")
{
}

Hardware::Hardware(QString brand, QString name) : m_brand(brand), m_name(name), m_partNumber(0), m_version(""), m_modelPath("")
{
}

Hardware::Hardware(std::QString brand, std::QString name, unsigned int partNumber, std::QString version, std::QString modelPath) :
		m_brand(brand), m_name(name), m_partNumber(partNumber), m_version(version), m_modelPath(modelPath)
{
}
*/
/*
//copy constructor
Hardware::Hardware(Hardware const& hardware) : m_brand(hardware.m_brand), m_name(hardware.m_name),
		m_partNumber(hardware.m_partNumber), m_version(hardware.m_version), m_modelPath(hardware.m_modelPath)
{
}
*/
//destructor
Hardware::~Hardware()
{
}

//setters
int Hardware::setBrand(QString brand)
{
	//check the value
	if(!brand.isEmpty())
	{
		m_brand = brand;

		return 0;
	}
	else
		return -1;
}

int Hardware::setName(QString name)
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

int Hardware::setPartNumber(unsigned int partNumber)
{
	//check the value
	if(partNumber > 0)
	{
		m_partNumber = partNumber;

		return 0;
	}
	else
		return -1;
}

int Hardware::setVersion(QString version)
{
	//check the value
	if(!version.isEmpty())
	{
		m_version = version;

		return 0;
	}
	else
		return -1;
}
/*
int Hardware::setModelPath(QString modelPath)
{
	//check the value
	if(!modelPath.isEmpty())
	{
		m_modelPath = modelPath;

		return 0;
	}
	else
		return -1;
}
*/
//other methods

void Hardware::display()
{
	cout << "Brand: " << m_brand.toStdString() << endl;
	cout << "Name: "<< m_name.toStdString() << endl;
	cout << "PartNumber: " << m_partNumber << endl;
	cout << "Version: " << m_version.toStdString() << endl;
	//cout << "Model path: " << m_modelPath.toStdString() << endl;
}

void Hardware::read(const QJsonObject &json)
{
	m_brand = json["brand"].toString();
	m_name = json["name"].toString();
	m_partNumber = (unsigned int)json["partNumber"].toDouble();
	m_version = json["version"].toString();
	//m_modelPath = json["modelPath"].toString();
}

//TODO solve the issue that it overwrites the labels which have the same name when multiple objects have Hardware as a mother class
void Hardware::write(QJsonObject &json) const
{
	json["brand"] = m_brand;
	json["name"] = m_name;
	json["partNumber"] = (double)m_partNumber;
	json["version"] = m_version;
	//json["modelPath"] = m_modelPath;
}
