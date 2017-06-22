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
 * @file MasterBoard.hpp
 * @author Cyril Jourdan
 * @date Mar 24, 2017
 * @version 2.0.0
 * @brief Header file for class MasterBoard
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 24, 2017
 */

#ifndef MASTERBOARD_HPP_
#define MASTERBOARD_HPP_

#include <QList>
#include <QJsonObject>
#include "Hardware.hpp"
#include "SlaveBoard.hpp"
#include "CommunicationLayer.hpp"
#include "Battery.hpp"

namespace osa_gui
{
	namespace common
	{
		class MasterBoard : public Hardware
		{
			//methods
			public:
				//constructor
				MasterBoard();
				//MasterBoard();
				//copy constructor
				//MasterBoard(MasterBoard const& controller);

				//destructor
				virtual ~MasterBoard();

				//setters
				int addPSlaveBoard(SlaveBoard* pSlaveBoard);
				//int setPCommunicationLayer(CommunicationLayer* pCommunicationLayer);
				//int setPBattery(Battery* pBattery);

				//getters
				QList<SlaveBoard*> 	getLpSlaveBoard() 			const { return m_lpSlaveBoard; };
				//CommunicationLayer* getPCommunicationLayer() 	const { return m_pCommunicationLayer; };
				//Battery* 			getPBattery() 				const { return m_pBattery; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				QList<SlaveBoard*> 	m_lpSlaveBoard;
				//CommunicationLayer*	m_pCommunicationLayer;
				//Battery*			m_pBattery;
		};
	}
}
#endif /* MASTERBOARD_HPP_ */
