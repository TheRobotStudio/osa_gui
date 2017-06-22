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
 * @file SlaveBoard.hpp
 * @author Cyril Jourdan
 * @date Mar 24, 2017
 * @version 2.0.0
 * @brief Header file for class SlaveBoard
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 24, 2017
 */

#ifndef SLAVEBOARD_HPP_
#define SLAVEBOARD_HPP_

#include <QList>
#include <QJsonObject>
#include "Hardware.hpp"
#include "Controller.hpp"
#include "CommunicationLayer.hpp"
#include "Battery.hpp"

namespace osa_gui
{
	namespace common
	{

		//pre-declaration
		class Controller;

		class SlaveBoard : public Hardware
		{
			//methods
			public:
				//constructor
				SlaveBoard();

				//destructor
				virtual ~SlaveBoard();

				//setters
				int	setBoardID(unsigned int nodeID);
				int addPController(Controller* pController);
				//int addCommunicationLayer(CommunicationLayer* pCommunicationLayer);
				//int setPBattery(Battery* pBattery);

				//getters
				unsigned int				getBoardID() 				const { return m_boardID; };
				QList<Controller*> 			getLpController() 			const { return m_lpController; };
				//QList<CommunicationLayer*> 	getVpCommunicationLayer() 	const { return m_vpCommunicationLayer; };
				//Battery* 					getPBattery() 				const { return m_pBattery; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				unsigned int				m_boardID;
				QList<Controller*> 			m_lpController;
				//QList<CommunicationLayer*> 	m_vpCommunicationLayer;
				//Battery*					m_pBattery;
		};
	}
}
#endif /* SLAVEBOARD_HPP_ */
