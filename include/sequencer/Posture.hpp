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
 * @file Posture.hpp
 * @author Cyril Jourdan
 * @date Mar 29, 2017
 * @version 2.0.0
 * @brief Header file for class Posture
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 29, 2016
 */

#ifndef POSTURE_HPP_
#define POSTURE_HPP_

#include <QJsonObject>
#include "SequenceElement.hpp"
#include "JSONMotorDataMultiArray.hpp"
#include <Pause.hpp>

namespace osa_gui
{
	namespace sequencer
	{
		/*! \class Posture
		 *  \brief
		 *
		 *  This class is
		 */
		class Posture : public SequenceElement
		{
			//methods
			public:
				Posture(); //constructors
				~Posture(); //destructor

				//setters
				int setPJSONMotorDataMultiArray(common::osa_msgs_json::JSONMotorDataMultiArray* pJSONMotorDataMultiArray);
				//int setMsPauseAfter(Pause m_msPauseAfter);

				//getters
				common::osa_msgs_json::JSONMotorDataMultiArray* getPJSONMotorDataMultiArray() const { return m_pJSONMotorDataMultiArray; };

				void playElement(rosnode::SequencerNode* sequencerNode);

				//JSON serialization
				void read(const QJsonObject &json);
				void write(QJsonObject &json) const;

			//attributes
			protected:
				common::osa_msgs_json::JSONMotorDataMultiArray *m_pJSONMotorDataMultiArray;
				//Pause m_msPauseAfter;
		};
	}
}

#endif /* POSTURE_HPP_ */
