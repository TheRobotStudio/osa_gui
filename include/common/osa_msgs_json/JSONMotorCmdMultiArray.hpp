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
 * @file JSONMotorCmdMultiArray.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class JSONMotorCmdMultiArray
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef JSONMOTORCMDMULTIARRAY_HPP_
#define JSONMOTORCMDMULTIARRAY_HPP_

#include <QString>
#include <QJsonObject>
#include <QList>
#include "JSONMotorCmd.hpp"

namespace osa_gui
{
	namespace common
	{
		namespace osa_msgs_json
		{
			/*! \class JSONMotorCmdMultiArray
			 * \brief
			 *
			 *  This class is
			 */
			class JSONMotorCmdMultiArray
			{
				//methods
				public:
					JSONMotorCmdMultiArray(); //constructors
					~JSONMotorCmdMultiArray(); //destructor

					//setters
					int addPJSONMotorCmd(JSONMotorCmd* pJSONMotorCmd);

					//getters
					QList<JSONMotorCmd*> getLpJSONMotorCmd() const { return m_lpJSONMotorCmd; };

					//JSON serialization
					void read(const QJsonObject &json);
					void write(QJsonObject &json)  const;

				//attributes
				protected:
					QList<JSONMotorCmd*> m_lpJSONMotorCmd;
			};
		}
	}
}

#endif /* JSONMOTORCMDMULTIARRAY_HPP_ */