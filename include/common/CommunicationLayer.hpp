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
 * @file CommunicationLayer.hpp
 * @author Cyril Jourdan
 * @date Dec 9, 2016
 * @version 0.0.1
 * @brief Header file for abstract class CommunicationLayer
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 9, 2016
 */

#ifndef COMMUNICATIONLAYER_HPP_
#define COMMUNICATIONLAYER_HPP_

#include <QString>
#include <QJsonObject>
#include <string>

namespace osa_gui
{
	namespace common
	{
		class CommunicationLayer
		{
			//methods
			public:
				//constructors
				CommunicationLayer();
				//CommunicationLayer(QString name);
				//copy constructor
				//CommunicationLayer(CommunicationLayer const& CommunicationLayer);

				//destructor
				virtual ~CommunicationLayer();

				//setters
				int setName(QString name);
				int setState(bool state);

				//getters
				QString getName() 	const { return m_name; };
				bool 	getState() 	const { return m_state; };

				//other methods
				virtual int openCommunication();
				virtual int closeCommunication();

				virtual void display() = 0;

				//JSON serialization
				virtual void read(const QJsonObject &json) = 0;
				virtual void write(QJsonObject &json) const = 0;

			//attributes
			protected:
				QString m_name;
				bool 	m_state;
		};
	}
}
#endif /* COMMUNICATIONLAYER_HPP_ */
