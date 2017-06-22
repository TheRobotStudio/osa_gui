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
 * @file Encoder.hpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Header file for class Encoder
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include <QJsonObject>
#include <string>
#include "Sensor.hpp"
#include "Motor.hpp"

namespace osa_gui
{
	namespace common
	{
		//pre-declaration
		class Motor;

		class Encoder : public Sensor
		{
			//methods
			public:
				//constructors
				Encoder();
				//Encoder(unsigned int countsPerTurn, unsigned short int numberOfChannels);
				//copy constructor
				//Encoder(Encoder const& encoder);

				//destructor
				virtual ~Encoder();

				//setters
				int setCountsPerTurn(unsigned int countsPerTurn);
				int setNumberOfChannels(unsigned short int numberOfChannels);
				//int	setPMotor(Motor* pMotor);

				//getters
				unsigned int 		getCountsPerTurn() 		const { return m_countsPerTurn; };
				unsigned short int 	getNumberOfChannels() 	const { return m_numberOfChannels; };
				//Motor* 				getPMotor() 			const { return m_pMotor; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			private:
				unsigned int 		m_countsPerTurn;
				unsigned short int 	m_numberOfChannels;
				//Motor*				m_pMotor; //motor reference
		};
	}
}
#endif /* ENCODER_HPP_ */
