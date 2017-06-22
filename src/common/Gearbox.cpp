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
 * @file Gearbox.cpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Implementation file for class Gearbox
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#include <iostream>
#include "Gearbox.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Gearbox::Gearbox() :
		m_diameter(0),
		m_reductionNumerator(0),
		m_reductionDenominator(0),
		m_numberOfStages(0)
		//m_pMotor(0) //m_name("91_GPX 22")
{
}

/*
Gearbox::Gearbox(QString reduction) : m_diameter(0), m_reduction(reduction), m_numberOfStages(0), m_pMotor(0)
{
}*/
/*
//copy constructor
Gearbox::Gearbox(Gearbox const& gearbox) : m_diameter(gearbox.m_diameter), m_reduction(gearbox.m_reduction), m_numberOfStages(gearbox.m_numberOfStages), m_pMotor(gearbox.m_pMotor)
{
}*/

//destructor
Gearbox::~Gearbox()
{
	//delete m_pMotor;
}

//setters
int Gearbox::setDiameter(unsigned int diameter)
{
	//check the value
	if(diameter > 0)
	{
		m_diameter = diameter;

		return 0;
	}
	else
		return -1;
}

int Gearbox::setReductionNumerator(unsigned short int reductionNumerator)
{
	//check the value
	if(reductionNumerator > 0)
	{
		m_reductionNumerator = reductionNumerator;

		return 0;
	}
	else
		return -1;
}

int Gearbox::setReductionDenominator(unsigned short int reductionDenominator)
{
	//check the value
	if(reductionDenominator > 0)
	{
		m_reductionDenominator = reductionDenominator;

		return 0;
	}
	else
		return -1;
}

int Gearbox::setNumberOfStages(unsigned short int numberOfStages)
{
	//check the value
	if(numberOfStages > 0)
	{
		m_numberOfStages = numberOfStages;

		return 0;
	}
	else
		return -1;
}
/*
int Gearbox::setPMotor(Motor* pMotor)
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
void Gearbox::display()
{
	cout << "Gearbox: " << endl;
	Hardware::display(); //call the display method from mother class
	cout << "Diameter: " << m_diameter << endl;
	cout << "ReductionNumerator: " << m_reductionNumerator << endl;
	cout << "ReductionDenominator: " << m_reductionDenominator << endl;
	cout << "Number of stages: " << m_numberOfStages << endl;
}

void Gearbox::read(const QJsonObject &json)
{
	//call mother class method
	Hardware::read(json);

	//read attributes
	m_diameter = (unsigned int)json["diameter"].toDouble();
	m_reductionNumerator = json["reductionNumerator"].toDouble();
	m_reductionDenominator = json["reductionDenominator"].toDouble();
	m_numberOfStages = (unsigned short int)json["numberOfStages"].toDouble();

	//m_pMotor->read(json); //TODO find a way to avoid infinite serialization loop
}

void Gearbox::write(QJsonObject &json) const
{
	//call mother class method
	Hardware::write(json);

	//read attributes
    json["diameter"] = (double)m_diameter;
	json["reductionNumerator"] = m_reductionNumerator;
	json["reductionDenominator"] = m_reductionDenominator;
	json["numberOfStages"] = (double)m_numberOfStages;
	//m_pMotor->write(json); //TODO find a way to avoid infinite serialization loop
}
