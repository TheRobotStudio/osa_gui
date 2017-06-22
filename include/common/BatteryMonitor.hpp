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
 * @file BatteryMonitor.hpp
 * @author Cyril Jourdan
 * @date Dec 13, 2016
 * @version 0.0.1
 * @brief Header file for class BatteryMonitor
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 13, 2016
 */

#ifndef BATTERYMONITOR_HPP_
#define BATTERYMONITOR_HPP_

#include <QList>
#include <QJsonObject>
#include <vector>
#include "Hardware.hpp"
#include "CommunicationLayer.hpp"

namespace osa_gui
{
	namespace common
	{
		class BatteryMonitor : public Hardware
		{
			//methods
			public:
				//constructor
				BatteryMonitor();

				//destructor
				virtual ~BatteryMonitor();

				//setters
				int addCellVoltage(float cellVoltage);
				int setCellVoltage(unsigned int cellNumber, float voltage);
				int	setTotalVoltage(float totalVoltage);
				int	setIntensity(float	intensity);
				int	setTemperature(float temperature);
				int setCommunicationLayer(CommunicationLayer* pCommunicationLayer);

				//getters
				QList<float>	getVCellVoltage() 								const { return m_vCellVoltage; };
				float				getTotalVoltage() 								const { return m_totalVoltage; };
				float				getIntensity() 									const { return m_intensity; };
				float				getTemperature() 								const { return m_temperature; };
				CommunicationLayer* getPCommunicationLayer() 						const { return m_pCommunicationLayer; };
				float 				getCellVoltage(unsigned short int cellNumber) 	const;

				//other
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			private:
				QList<float> 		m_vCellVoltage;
				float				m_totalVoltage;
				float				m_intensity;
				float				m_temperature;
				CommunicationLayer* m_pCommunicationLayer;

		};
	}
}
#endif /* BATTERYMONITOR_HPP_ */
