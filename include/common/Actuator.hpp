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
 * @file Actuator.hpp
 * @author Cyril Jourdan
 * @date Jan 17, 2017
 * @version 0.0.1
 * @brief Header file for abstract class Actuator
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#ifndef ACTUATOR_HPP_
#define ACTUATOR_HPP_

#include <QString>
#include <QJsonObject>
#include "Hardware.hpp"


namespace osa_gui
{
	namespace common
	{
		/*! \class Actuator
		 * \brief Abstract class representing an actuator for a robot.
		 *
		 *  This class is used as a mother class for more specific actuators like electric motors.
		 */
		class Actuator : public Hardware
		{
			//methods
			public:
				//constructors
				Actuator();
				Actuator(QString type);
				//copy constructor
				//Actuator(Actuator const& actuator);

				//destructor
				virtual ~Actuator();

				//operator overloading
				//Actuator& operator=(Actuator const& actuator);

				//setters
				int setType(QString type); //will mainly be set by a daughter class like Motor

				//getters
				QString getType() const { return m_type; };

				//other methods
				virtual void display() = 0;

				//JSON serialization
				virtual void read(const QJsonObject &json) = 0;
				virtual void write(QJsonObject &json)  const  = 0;

			//attributes
			protected:
				QString m_type;
				//int power in W ?
				//pRobot ?
		};
	}
}

#endif /* ACTUATOR_HPP_ */
