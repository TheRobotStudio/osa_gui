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
 * @file Sequence.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.0.1
 * @brief Implementation file for abstract class Sequence
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <QJsonArray>
#include "Sequence.hpp"

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

//constructors
Sequence::Sequence() :
		m_lpSequenceElement(QList<SequenceElement*>())
{
}

//destructor
Sequence::~Sequence()
{

}

//setters
int Sequence::addPSequenceElement(SequenceElement* pSequenceElement)
{
	//check the value
	if(pSequenceElement != 0)
	{
		m_lpSequenceElement.append(pSequenceElement);

		return 0;
	}
	else
		return -1;
}

void Sequence::playSequence(rosnode::SequencerNode* sequencerNode)
{
	foreach(SequenceElement* pSequenceElement, m_lpSequenceElement)
	{
		pSequenceElement->playElement(sequencerNode);
	}
}

void Sequence::read(const QJsonObject &json)
{
	//SequenceElement pointer QList
	m_lpSequenceElement.clear();
	QJsonArray sequenceElementArray = json["pSequenceElement"].toArray();

	for(int i = 0; i<sequenceElementArray.size(); ++i)
	{
		QJsonObject pSequenceElementObject = sequenceElementArray[i].toObject();
		SequenceElement* pSequenceElement;
		pSequenceElement->read(pSequenceElementObject);
		m_lpSequenceElement.append(pSequenceElement);
	}
}

void Sequence::write(QJsonObject &json) const
{
	//SequenceElement pointer QList
	QJsonArray sequenceElementArray;
	foreach(const SequenceElement* pSequenceElement, m_lpSequenceElement)
	{
		QJsonObject pSequenceElementObject;
		pSequenceElement->write(pSequenceElementObject);
		sequenceElementArray.append(pSequenceElementObject);
	}
	json["pSequenceElement"] = sequenceElementArray;
}
