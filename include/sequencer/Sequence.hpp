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
 * @file Sequence.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class Sequence
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef SEQUENCE_HPP_
#define SEQUENCE_HPP_

#include <QJsonObject>
#include <QList>
#include "SequenceElement.hpp"

namespace osa_gui
{
	namespace sequencer
	{
		/*! \class Sequence
		 *  \brief
		 *
		 *  This class is
		 */
		class Sequence
		{
			//methods
			public:
				Sequence(); //constructors
				~Sequence(); //destructor

				//setters
				int addPSequenceElement(SequenceElement* pSequenceElement);

				//getters
				QList<SequenceElement*> getLpSequenceElement() const { return m_lpSequenceElement; };

				void playSequence(rosnode::SequencerNode* sequencerNode);

				//JSON serialization
				void read(const QJsonObject &json);
				void write(QJsonObject &json)  const;

			//attributes
			protected:
				QList<SequenceElement*> m_lpSequenceElement;
		};
	}
}

#endif /* SEQUENCE_HPP_ */
