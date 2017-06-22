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
 * @file Encoder.cpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Implementation file for class Encoder
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#include <iostream>
#include "Encoder.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Encoder::Encoder() :
		m_countsPerTurn(0),
		m_numberOfChannels(0)
		//m_pMotor(0) //m_name("103_ENX 16 EASY")
{
}
/*
Encoder::Encoder(unsigned int countsPerTurn, unsigned short int numberOfChannels) : m_countsPerTurn(countsPerTurn), m_numberOfChannels(numberOfChannels), m_pMotor(0)
{
}*/
/*
//copy constructor
Encoder::Encoder(Encoder const& encoder) : m_countsPerTurn(encoder.m_countsPerTurn), m_numberOfChannels(encoder.m_numberOfChannels), m_pMotor(0)
{
}
*/
//destructor
Encoder::~Encoder()
{
	//delete m_pMotor;
}

//setters
int Encoder::setCountsPerTurn(unsigned int countsPerTurn)
{
	//check the value
	if(countsPerTurn > 0)
	{
		m_countsPerTurn = countsPerTurn;

		return 0;
	}
	else
		return -1;
}

int Encoder::setNumberOfChannels(unsigned short int numberOfChannels)
{
	//check the value
	if(numberOfChannels > 0)
	{
		m_numberOfChannels = numberOfChannels;

		return 0;
	}
	else
		return -1;
}
/*
int	Encoder::setPMotor(Motor* pMotor)
{
	//check the value
	if(pMotor != 0)
	{
		m_pMotor = pMotor;

		return 0;
	}
	else
		return -1;
}
*/

//other methods
void Encoder::display()
{
	cout << endl << "Encoder: " << endl;
	Hardware::display(); //call the display method from mother class
	cout << "Counts per turn: " << m_countsPerTurn << endl;
	cout << "Number of channels: " << m_numberOfChannels << endl;
}

void Encoder::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//read attributes
	m_countsPerTurn = (unsigned int)json["countsPerTurn"].toDouble();
	m_numberOfChannels = (unsigned short int)json["numberOfChannels"].toDouble();
	//m_pMotor->read(json); //TODO find a way to avoid infinite serialization loop
}

void Encoder::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//write attributes
    json["countsPerTurn"] = (double)m_countsPerTurn;
	json["numberOfChannels"] = (double)m_numberOfChannels;
	// m_pMotor->write(json); //TODO find a way to avoid infinite serialization loop
}
