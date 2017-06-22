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
 * @file JSONMotorData.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class JSONMotorData
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef JSONMOTORDATA_HPP_
#define JSONMOTORCMD_HPP_

#include <QJsonObject>
#include <stdint.h>

namespace osa_gui
{
	namespace common
	{
		namespace osa_msgs_json
		{
			/*! \class JSONMotorData
			 * \brief
			 *
			 *  This class is
			 */
			class JSONMotorData
			{
				//methods
				public:
					JSONMotorData(); //constructors
					~JSONMotorData(); //destructor

					//setters
					int setPosition(int32_t position);
					int setCurrent(int16_t current);
					int setStatus(uint16_t status);

					//getters
					int32_t getPosition() const { return m_position; };
					int16_t getCurrent() const { return m_current; };
					uint16_t getStatus() const { return m_status; };

					//JSON serialization
					void read(const QJsonObject &json);
					void write(QJsonObject &json)  const;

				//attributes
				protected:
					int32_t m_position;
					int16_t m_current;
					uint16_t m_status;
			};
		}
	}
}

#endif /* JSONMOTORCMD_HPP_ */
