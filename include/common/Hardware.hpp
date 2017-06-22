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
 * @file Hardware.hpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Header file for abstract class Hardware
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#ifndef HARDWARE_HPP_
#define HARDWARE_HPP_

#include <QString>
#include <QJsonObject>
#include <string>

namespace osa_gui
{
	namespace common
	{
		/*** Abstract class ***/
		class Hardware
		{
			//methods
			public:
				//constructor
				Hardware();
				//Hardware(QString name);
				//Hardware(QString brand, QString name);
				//Hardware(QString brand, QString name, unsigned int partNumber, QString version, QString modelPath);

				//copy constructor
				//Hardware(Hardware const& hardware);
				//operator overloading
				//Hardware& operator=(Hardware const& hardware);

				//destructor
				virtual ~Hardware();

				//setters
				int setBrand(QString brand);
				int setName(QString name);
				int setPartNumber(unsigned int partNumber);
				int setVersion(QString version);
				//int setModelPath(QString modelPath);

				//getters
				QString 		getBrand() 		const { return m_brand; };
				QString 		getName() 		const { return m_name; };
				unsigned int 	getPartNumber() const { return m_partNumber; };
				QString 		getVersion() 	const { return m_version; };
				//QString 		getModelPath() 	const { return m_modelPath; };

				//other methods
				virtual void display() = 0;

				//JSON serialization
				virtual void read(const QJsonObject &json) = 0;
				virtual void write(QJsonObject &json) const = 0;

				//friends
				//operator overloading
				//friend bool operator==(Sensor const& sensorA, Sensor const& sensorB);

			//attributes
			protected:
				QString 		m_brand;
				QString 		m_name;
				unsigned int 	m_partNumber;
				QString 		m_version;
				//QString 		m_modelPath; //3D model or picture/drawing
		};
	}
}
#endif /* HARDWARE_HPP_ */
