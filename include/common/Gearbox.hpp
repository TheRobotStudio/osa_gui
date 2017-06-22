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
 * @file Gearbox.hpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Header file for class Gearbox
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#ifndef GEARBOX_HPP_
#define GEARBOX_HPP_

#include <QString>
#include <QJsonObject>
#include <string>
#include "Hardware.hpp"
#include "Motor.hpp"

namespace osa_gui
{
	namespace common
	{
		//pre-declaration
		class Motor;

		class Gearbox : public Hardware
		{
			//methods
			public:
				//constructors
				Gearbox();
				//Gearbox(QString reduction);
				//copy constructor
				//Gearbox(Gearbox const& gearbox);

				//destructor
				~Gearbox();

				//setters
				int setDiameter(unsigned int diameter);
				int setReductionNumerator(unsigned short int reductionNumerator);
				int setReductionDenominator(unsigned short int reductionDenominator);
				int setNumberOfStages(unsigned short int numberOfStages);
				//int setPMotor(Motor* pMotor);

				//getters
				unsigned int 		getDiameter() 				const { return m_diameter; };
				unsigned short int 	getReductionNumerator() 	const { return m_reductionNumerator; };
				unsigned short int 	getReductionDenominator() 	const { return m_reductionDenominator; };
				unsigned short int 	getNumberOfStages() 		const { return m_numberOfStages; };
				//Motor*				getPMotor()			const { return m_pMotor; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			private:
				unsigned int 		m_diameter;
				unsigned short int 	m_reductionNumerator;
				unsigned short int 	m_reductionDenominator;
				unsigned short int 	m_numberOfStages;
				//Motor*				m_pMotor; //motor reference
		};
	}
}
#endif /* GEARBOX_HPP_ */
