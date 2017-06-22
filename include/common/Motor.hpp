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
 * @file Motor.hpp
 * @author Cyril Jourdan
 * @date Dec 8, 2016
 * @version 0.0.1
 * @brief Header file for class Motor
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 8, 2016
 */

#ifndef MOTOR_HPP_
#define MOTOR_HPP_

/*! Includes */
#include <QString>
#include <QJsonObject>
#include "Actuator.hpp"
#include "Gearbox.hpp"
#include "Encoder.hpp"
#include "Controller.hpp"

namespace osa_gui
{
	namespace common
	{
		//pre-declaration
		class Gearbox;
		class Encoder;
		//class Controller;

		/*! Motor class */
		class Motor : public Actuator
		{
			//methods
			public:
				//constructors
				Motor();
				/*Motor(QString brand, QString name, QString brushType, unsigned short int diameter,
								unsigned short int nominalVoltage, unsigned short int nominalSpeed, float nominalCurrent, float thermalTimeConstantWinding, unsigned short int numberOfPolePairs);
			*/
				/*	Motor(QString brand, QString name, QString brushType, unsigned short int diameter,
						unsigned short int nominalVoltage, unsigned short int nominalSpeed, float nominalCurrent, float thermalTimeConstantWinding, unsigned short int numberOfPolePairs,
						Gearbox gearbox, Encoder encoder, Controller controller);*/
				//copy constructor
				//Motor(Motor const& motor);
				//operator overloading
				//Motor& operator=(Motor const& motor);

				//destructor
				~Motor();

				//setters
				int setBrushType(QString brushType);
				int setDiameter(unsigned short int diameter);
				int setNominalVoltage(unsigned short int nominalVoltage);
				int setNominalSpeed(unsigned short int nominalSpeed);
				int setNominalCurrent(float nominalCurrent);
				int	setThermalTimeConstantWinding(float thermalTimeConstantWinding);
				int setNumberOfPolePairs(unsigned short int numberOfPolePairs);
				int setPGearbox(Gearbox* pGearbox);
				int setPEncoder(Encoder* pEncoder);
				//int	setPController(Controller* pController);

				//getters
				QString 			getBrushType() 					const { return m_brushType; };
				unsigned short int 	getDiameter() 					const { return m_diameter; };
				unsigned short int 	getNominalVoltage() 			const { return m_nominalVoltage; };
				unsigned short int 	getNominalSpeed() 				const { return m_nominalSpeed; };
				float 				getNominalCurrent() 			const { return m_nominalCurrent; };
				float			 	getThermalTimeConstantWinding() const { return m_thermalTimeConstantWinding; };
				unsigned short int 	getNumberOfPolePairs() 			const { return m_numberOfPolePairs; };
				Gearbox* 			getPGearbox() 					const { return m_pGearbox; };
				Encoder* 			getPEncoder() 					const { return m_pEncoder; };
				//Controller*			getPController() 				const { return m_pController; };

				//other methods
				virtual void display();

				//friends
				//operator overloading
				friend bool operator==(Motor const& motorA, Motor const& motorB);

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			private:
				QString 			m_brushType;
				unsigned short int 	m_diameter;
				unsigned short int 	m_nominalVoltage;
				unsigned short int 	m_nominalSpeed;
				float 				m_nominalCurrent;
				float				m_thermalTimeConstantWinding;
				unsigned short int	m_numberOfPolePairs;
				Gearbox* 			m_pGearbox;	//Linked devices
				Encoder* 			m_pEncoder;
				//Controller*			m_pController; //controller to which the motor is plugged
		};
	}
}
//other
//bool operator==(Motor const& motorA, Motor const& motorB);

#endif /* MOTOR_HPP_ */
