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
 * @file MotionStyle.hpp
 * @author Cyril Jourdan
 * @date Mar 29, 2017
 * @version 2.0.0
 * @brief Header file for class MotionStyle
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 29, 2016
 */

#ifndef MOTIONSTYLE_HPP_
#define MOTIONSTYLE_HPP_

#include <QJsonObject>
#include "SequenceElement.hpp"

namespace osa_gui
{
	namespace sequencer
	{
		/*! \class MotionStyle
		 *  \brief
		 *
		 *  This class is
		 */
		class MotionStyle : public SequenceElement
		{
			//methods
			public:
				MotionStyle(); //constructors
				~MotionStyle(); //destructor

				//setters
				int setModeOfOperation(int8_t modeOfOperation);
				int setProfileAcceleration(uint32_t profileAcceleration);
				int setProfileDeceleration(uint32_t profileDeceleration);
				int setProfileVelocity(uint32_t profileVelocity);
				int setOutputCurrentLimit(uint16_t outputCurrentLimit);

				//getters
				int8_t getModeOfOperation() 		const { return m_modeOfOperation; };
				uint32_t getProfileAcceleration() 	const { return m_profileAcceleration; };
				uint32_t getProfileDeceleration() 	const { return m_profileDeceleration; };
				uint32_t getProfileVelocity() 		const { return m_profileVelocity; };
				uint16_t getOutputCurrentLimit() 	const { return m_outputCurrentLimit; };

				void playElement(rosnode::SequencerNode* sequencerNode);

				//JSON serialization
				void read(const QJsonObject &json);
				void write(QJsonObject &json)  const;

			//attributes
			protected:
				int8_t m_modeOfOperation;
				uint32_t m_profileAcceleration;
				uint32_t m_profileDeceleration;
				uint32_t m_profileVelocity;
				uint16_t m_outputCurrentLimit;
		};
	}
}

#endif /* MOTIONSTYLE_HPP_ */
