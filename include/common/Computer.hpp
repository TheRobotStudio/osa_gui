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
 * @file Computer.hpp
 * @author Cyril Jourdan
 * @date Dec 14, 2016
 * @version 0.0.1
 * @brief Header file for class Computer
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 14, 2016
 */

#ifndef COMPUTER_HPP_
#define COMPUTER_HPP_

#include <QList>
#include <QJsonObject>
#include <vector>
#include "Hardware.hpp"
//#include "Robot.hpp"
#include "Battery.hpp"
#include "CommunicationLayer.hpp"
#include "USBDevice.hpp"

namespace osa_gui
{
	namespace common
	{
		class Computer : public Hardware
		{
			//methods
			public:
				//constructors
				Computer();
				//copy constructor
				//Computer(Computer const& computer);
				//operator overloading
				//Computer& operator=(Computer const& computer);

				//destructor
				virtual ~Computer();

				//setters
				int setPBattery(Battery* pBattery);
				int addPCommunicationLayer(CommunicationLayer* pCommunicationLayer);
				int addPUSBDevice(USBDevice* pUSBDevice);

				//getters
				Battery* 							getPBattery() 				const { return m_pBattery; };
				QList<CommunicationLayer*> 	getVpCommunicationLayer() 	const { return m_vpCommunicationLayer; };
				QList<USBDevice*> 			getVpUSBDevice() 			const { return m_vpUSBDevice; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				Battery*							m_pBattery;
				QList<CommunicationLayer*> 	m_vpCommunicationLayer;
				QList<USBDevice*> 			m_vpUSBDevice;
				//Robot*								m_pRobot;
				//HDMI display
				//wifi/ethernet
				//sound speaker
				//microphone
				//serial debugger
		};
	}
}
#endif /* COMPUTER_HPP_ */
