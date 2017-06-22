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
 * @file SequenceElement.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for abstract class SequenceElement
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef SEQUENCEELEMENT_HPP_
#define SEQUENCEELEMENT_HPP_

#include <QJsonObject>
#include <stdint.h>
#include "SequencerNode.hpp"

namespace osa_gui
{
	namespace sequencer
	{
		/*! \class SequenceElement
		 *  \brief Abstract class representing a sequence element for the sequencer.
		 *
		 *  This class is a parent class for all sequence element like Posture, MotionStyle, Pause, Condition...
		 */
		class SequenceElement
		{
			//methods
			public:
				SequenceElement(); //constructors
				virtual ~SequenceElement(); //destructor

				virtual void playElement(rosnode::SequencerNode* sequencerNode) = 0;

				//JSON serialization
				virtual void read(const QJsonObject &json) = 0;
				virtual void write(QJsonObject &json)  const = 0;

			//attributes
			//protected:
		};
	}
}

#endif /* SEQUENCEELEMENT_HPP_ */
