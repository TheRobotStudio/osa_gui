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
 * @file Battery.hpp
 * @author Cyril Jourdan
 * @date Dec 13, 2016
 * @version 0.0.1
 * @brief Header file for class Battery
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 13, 2016
 */

#ifndef BATTERY_HPP_
#define BATTERY_HPP_

#include <QString>
#include <QList>
#include <QJsonObject>
#include <vector>
#include "Hardware.hpp"
#include "BatteryMonitor.hpp"

namespace osa_gui
{
	namespace common
	{
		class Battery : public Hardware
		{
			//methods
			public:
				//constructor
				Battery();
				Battery(QString type, unsigned short int numberOfCells, unsigned int capacity, unsigned int nominalVoltage);

				//destructor
				virtual ~Battery();

				//setters
				int setType(QString type);
				int setNumberOfCells(unsigned short int numberOfCells);
				int setCapacity(unsigned int capacity);
				int setNominalVoltage(unsigned int nominalVoltage);
				int setPBatteryMonitor(BatteryMonitor* pBatteryMonitor);
				void addVpHardware(Hardware* pHardware);

				//getters
				QString 				getType() 				const { return m_type; };
				unsigned short int 		getNumberOfCells() 		const { return m_numberOfCells; };
				unsigned int			getCapacity() 			const { return m_capacity; };
				unsigned int			getNominalVoltage() 	const { return m_nominalVoltage; };
				BatteryMonitor*			getPBatteryMonitor() 	const { return m_pBatteryMonitor; };
				QList<Hardware*>	getVpHardware()			const { return m_vpHardware; };

				//other methods
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			protected:
				QString 			m_type;
				unsigned short int 	m_numberOfCells;
				unsigned int		m_capacity;
				unsigned int		m_nominalVoltage;
				BatteryMonitor*		m_pBatteryMonitor;
				QList<Hardware*>	m_vpHardware; //list of hardware powered by this battery
		};
	}
}
#endif /* BATTERY_HPP_ */
