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
 * @file Robot.hpp
 * @author Cyril Jourdan
 * @date Dec 8, 2016
 * @version 0.0.1
 * @brief Header file for class Robot
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 8, 2016
 */

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <QList>
#include <QJsonObject>
#include <vector>
#include "Hardware.hpp"
#include "MasterBoard.hpp"
#include "Actuator.hpp"
#include "Sensor.hpp"
#include "Computer.hpp"
#include "CommunicationLayer.hpp"
#include "Battery.hpp"

namespace osa_gui
{
	namespace common
	{
		class Robot : public Hardware
		{
			//methods
			public:
				//constructors
				Robot();
				//Robot(std::string name, int dof);
				//copy constructor
				//Robot(Robot const& robot);

				//destructor
				virtual ~Robot();

				//setters
				int setDof(unsigned int dof);
				int setPMasterBoard(MasterBoard* pMasterBoard);
				/*
				int addPActuator(Actuator* pActuator);
				int addPSensor(Sensor* pSensor);
				int addPComputer(Computer* pComputer);
				int addPCommunicationLayer(CommunicationLayer* pCommunicationLayer);
				int addPBattery(Battery* pBattery);
		*/

				//getters
				unsigned int 				getDof() 					const { return m_dof; };
				/*
				QList<Actuator*> 			getVpActuator() 			const { return m_vpActuator; };
				QList<Sensor*>				getVpSensor() 				const { return m_vpSensor; };
				QList<Computer*>			getVpComputer() 			const { return m_vpComputer; };
				QList<CommunicationLayer*> 	getVpCommunicationLayer() 	const { return m_vpCommunicationLayer; };
				QList<Battery*> 			getVpBattery() 				const { return m_vpBattery; };
		*/
				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				unsigned int 				m_dof; //Degrees Of Freedom, equal to m_vMotor.size();
				MasterBoard* 				m_pMasterBoard;

				/*
				QList<Actuator*> 			m_vpActuator;
				QList<Sensor*> 				m_vpSensor;
				QList<Computer*> 			m_vpComputer;
				QList<CommunicationLayer*> 	m_vpCommunicationLayer;
				QList<Battery*> 			m_vpBattery;
				*/
		};
	}
}
#endif /* ROBOT_HPP_ */
