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
 * @file Motor.cpp
 * @author Cyril Jourdan
 * @date Dec 8, 2016
 * @version 0.0.1
 * @brief Implementation file for class Motor
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 8, 2016
 */

#include <iostream>
#include "Motor.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
/*
Motor::Motor() : Actuator("maxon motor","DCX 22 L"), m_brushType("Graphite Brushes"), m_diameter(22),
	m_nominalVoltage(36), m_nominalSpeed(11400), m_nominalCurrent(1.03), m_thermalTimeConstantWinding(22), m_numberOfPolePairs(1),
	m_gearbox("91_GPX 22"), m_encoder("103_ENX 16 EASY"), m_controller()
{
}
*/

Motor::Motor() :
		Actuator(QString("Motor")),
		m_brushType(QString("")),
		m_diameter(0),
		m_nominalVoltage(0),
		m_nominalSpeed(0),
		m_nominalCurrent(0),
		m_thermalTimeConstantWinding(0),
		m_numberOfPolePairs(0),
		m_pGearbox(NULL),
		m_pEncoder(NULL)
		//m_pController()
{
}

/*
Motor::Motor(QString brand, QString name, QString brushType, unsigned short int diameter, unsigned int partNumber,
		unsigned short int nominalVoltage, unsigned short int nominalSpeed, float nominalCurrent,
		float thermalTimeConstantWinding, unsigned short int numberOfPolePairs) :
		Hardware(brand, name), Actuator("Motor"), m_brushType(brushType), m_diameter(diameter), m_partNumber(partNumber),
		m_nominalVoltage(nominalVoltage), m_nominalSpeed(nominalSpeed), m_nominalCurrent(nominalCurrent),
		m_thermalTimeConstantWinding(thermalTimeConstantWinding), m_numberOfPolePairs(numberOfPolePairs),
		m_gearbox(), m_encoder(), m_pController()
{
}
*/
/*
//copy constructor
Motor::Motor(Motor const& motor) : Actuator(motor), m_brushType(motor.m_brushType), m_diameter(motor.m_diameter),
		m_nominalVoltage(motor.m_nominalVoltage), m_nominalSpeed(motor.m_nominalSpeed), m_nominalCurrent(motor.m_nominalCurrent),
		m_thermalTimeConstantWinding(motor.m_thermalTimeConstantWinding), m_numberOfPolePairs(motor.m_numberOfPolePairs),
		m_pGearbox(motor.m_pGearbox), m_pEncoder(motor.m_pEncoder), m_pController(motor.m_pController)
{

}
*//*
//class operator overloading
Motor& Motor::operator=(Motor const& motor)
{
    if(this != &motor)
    {
    	m_name = motor.m_name;
    	m_nominalVoltage = motor.m_nominalVoltage;
    	m_nominalCurrent = motor.m_nominalCurrent;
    	m_pGearbox = motor.m_pGearbox;
		m_pEncoder = motor.m_pEncoder;
		m_pController = motor.m_pController;
    }

    return *this;
}*/

//destructor
Motor::~Motor()
{
	delete m_pGearbox;
	delete m_pEncoder;
	//delete m_pController;
}

//setters
int Motor::setBrushType(QString brushType)
{
	//check the value
	if(!brushType.isEmpty())
	{
		m_brushType = brushType;

		return 0;
	}
	else
		return -1;
}

int Motor::setDiameter(unsigned short int diameter)
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

int Motor::setNominalVoltage(unsigned short int nominalVoltage)
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

int Motor::setNominalSpeed(unsigned short int nominalSpeed)
{
	//check the value
	if(nominalSpeed > 0)
	{
		m_nominalSpeed = nominalSpeed;

		return 0;
	}
	else
		return -1;
}

int Motor::setNominalCurrent(float nominalCurrent)
{
	//check the value
	if(nominalCurrent > 0)
	{
		m_nominalCurrent = nominalCurrent;

		return 0;
	}
	else
		return -1;
}

int	Motor::setThermalTimeConstantWinding(float thermalTimeConstantWinding)
{
	//check the value
	if(thermalTimeConstantWinding > 0)
	{
		m_thermalTimeConstantWinding = thermalTimeConstantWinding;

		return 0;
	}
	else
		return -1;
}

int Motor::setNumberOfPolePairs(unsigned short int numberOfPolePairs)
{
	//check the value
	if(numberOfPolePairs > 0)
	{
		m_numberOfPolePairs = numberOfPolePairs;

		return 0;
	}
	else
		return -1;
}

int Motor::setPGearbox(Gearbox* pGearbox)
{
	//check the value
	if(pGearbox != 0)
	{
		m_pGearbox = pGearbox;

		return 0;
	}
	else
		return -1;
}

int Motor::setPEncoder(Encoder* pEncoder)
{
	//check the value
	if(pEncoder != 0)
	{
		m_pEncoder = pEncoder;

		return 0;
	}
	else
		return -1;
}

/*
int	Motor::setPController(Controller* pController)
{
	//check the value
	if(pController != 0)
	{
		m_pController = pController;

		return 0;
	}
	else
		return -1;
}
*/

//other methods
void Motor::display()
{
	Actuator::display(); //call the display method from Actuator class
	cout << "Brush type: " << m_brushType.toStdString() << endl;
	cout << "Diameter: " << m_diameter << endl;
	cout << "Nominal voltage: " << m_nominalVoltage << " V" << endl;
	cout << "Nominal Speed: " << m_nominalSpeed << " rpm" << endl;
	cout << "Nominal current: " << m_nominalCurrent << " A" << endl;
	cout << "Thermal time constant winding: " << m_thermalTimeConstantWinding << " s" << endl;
	cout << "Number of pole pairs: " << m_numberOfPolePairs << endl;
	if(m_pGearbox != 0) m_pGearbox->display();
	if(m_pEncoder != 0) m_pEncoder->display();
	//if(m_pController != 0) m_pController->display();
	cout << endl;
}

/*
//other operator overloading
bool operator==(Motor const& motorA, Motor const& motorB)
{

}*/

void Motor::read(const QJsonObject &json)
{
	//call mother class method
	Actuator::read(json);

	//read attributes
	m_brushType = json["brushType"].toString();
	m_diameter = (unsigned short int)json["diameter"].toDouble();
	m_nominalVoltage = (unsigned short int)json["nominalVoltage"].toDouble();
	m_nominalSpeed = (unsigned short int)json["nominalSpeed"].toDouble();
	m_nominalCurrent = (float)json["nominalCurrent"].toDouble();
	m_thermalTimeConstantWinding = (float)json["thermalTimeConstantWinding"].toDouble();
	m_numberOfPolePairs = (unsigned short int)json["numberOfPolePairs"].toDouble();
	m_pGearbox->read(json);
	m_pEncoder->read(json);
	//m_pController->read(json);
}

void Motor::write(QJsonObject &json) const
{
	//call mother class method
	Actuator::write(json);

	//write attributes
	json["brushType"] = m_brushType;
	json["diameter"] = (double)m_diameter;
	json["nominalVoltage"] = (double)m_nominalVoltage;
	json["nominalSpeed"] = (double)m_nominalSpeed;
	json["nominalCurrent"] = (double)m_nominalCurrent;
	json["thermalTimeConstantWinding"] = (double)m_thermalTimeConstantWinding;
	json["numberOfPolePairs"] = (double)m_numberOfPolePairs;
	m_pGearbox->write(json);
	m_pEncoder->write(json);
	//m_pController->write(json);
}
