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
 * @file SerialCommunication.hpp
 * @author Cyril Jourdan
 * @date Dec 14, 2016
 * @version 0.0.1
 * @brief Header file for class SerialCommunication
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 14, 2016
 */

#ifndef SERIALCOMMUNICATION_HPP_
#define SERIALCOMMUNICATION_HPP_

#include <QString>
#include <QJsonObject>
#include "CommunicationLayer.hpp"

namespace osa_gui
{
	namespace common
	{
		class SerialCommunication : public CommunicationLayer
		{
			//methods
			public:
				//constructors
				SerialCommunication();
				//copy constructor
				//SerialCommunication(SerialCommunication const& serialCommunication);

				//destructor
				~SerialCommunication();

				//setters
				int setFilePath(QString filePath);

				//getters
				QString getFilePath() const { return m_filePath; };

				//other methods
				char* read_c();
				int   write_c(char* data);
				virtual void display();

				//JSON serialization
				virtual void read(const QJsonObject &json);
				virtual void write(QJsonObject &json) const;

			//attributes
			private:
				QString m_filePath;
		};
	}
}
#endif /* SERIALCOMMUNICATION_HPP_ */
