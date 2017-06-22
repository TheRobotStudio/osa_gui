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
 * @file Sensor.hpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Header file for class Sensor
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#ifndef SENSOR_HPP_
#define SENSOR_HPP_

#include <QList>
#include <QJsonObject>
#include "Hardware.hpp"
#include "Computer.hpp"
#include "CommunicationLayer.hpp"
#include "Battery.hpp"

namespace osa_gui
{
	namespace common
	{
		class Sensor : public Hardware
		{
			//methods
			public:
				//constructors
				Sensor();
				//copy constructor
				//Sensor(Sensor const& sensor);
				//operator overloading
				//Sensor& operator=(Sensor const& sensor);

				//destructor
				virtual ~Sensor();

				//setters
				int addValue(double value);
				int setValue(unsigned int index, double value);
				int setPComputer(Computer* );
				int setPCommunicationLayer(CommunicationLayer* );
				int setPBattery(Battery* );

				//getters
				QList<double> 		getValue() const { return m_vValue; };
				Computer* 			getPComputer() const { return m_pComputer; };
				CommunicationLayer* getPCommunicationLayer() const { return m_pCommunicationLayer; };
				Battery* 			getPBattery() const { return m_pBattery; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				QList<double> 		m_vValue;
				Computer* 			m_pComputer;
				CommunicationLayer* m_pCommunicationLayer;
				Battery* 			m_pBattery;
		};
	}
}
#endif /* SENSOR_HPP_ */
