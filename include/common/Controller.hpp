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
 * @file Controller.hpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Header file for class Controller
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <QJsonObject>
#include "Hardware.hpp"
#include "Actuator.hpp"
#include "CommunicationLayer.hpp"
#include "Battery.hpp"

namespace osa_gui
{
	namespace common
	{
		class Controller : public Hardware
		{
			//methods
			public:
				//constructor
				Controller();
				//Controller();
				//copy constructor
				//Controller(Controller const& controller);

				//destructor
				virtual ~Controller();

				//setters
				int	setStatus(int status);
				int	setNodeID(unsigned int nodeID);
				int setPosition(int position);
				int setCurrent(int current);
				int setVelocity(int velocity);
				int setPActuator(Actuator* pActuator);
				//int setPCommunicationLayer(CommunicationLayer* pCommunicationLayer);
				//int setPBattery(Battery* pBattery);

				//getters
				int					getStatus() 				const { return m_status; };
				unsigned int		getNodeID() 				const { return m_nodeID; };
				int 				getPosition() 				const { return m_position; };
				int 				getCurrent() 				const { return m_current; };
				int 				getVelocity() 				const { return m_velocity; };
				Actuator* 			getPActuator() 				const { return m_pActuator; };
				//CommunicationLayer* getPCommunicationLayer() 	const { return m_pCommunicationLayer; };
				//Battery* 			getPBattery() 				const { return m_pBattery; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				int					m_status;
				unsigned int		m_nodeID;
				int 				m_position;
				int 				m_current;
				int 				m_velocity;
				Actuator* 			m_pActuator;
				//CommunicationLayer*	m_pCommunicationLayer;
				//Battery*			m_pBattery;
		};
	}
}
#endif /* CONTROLLER_HPP_ */
