/*
 * Copyright (c) 2019, The Robot Studio
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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
 * @file sequence.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.1.0
 * @brief Implementation file for abstract class Sequence
 *
 * Contact: contact@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <sequence.h>
#include <QJsonArray>

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

Sequence::Sequence() :
		sequence_element_list_(QList<SequenceElement*>())
{
}

Sequence::~Sequence()
{

}

//setters
int Sequence::addSequenceElement(SequenceElement* ptr_sequence_element)
{
	//check the argument
	if(ptr_sequence_element != 0)
	{
		sequence_element_list_.append(ptr_sequence_element);

		return 0;
	}
	else
		throw invalid_argument("Argument ptr_sequence_element is NULL.");
		//return -1; //TODO remplace return int with throw
}

void Sequence::playSequence(rosnode::SequencerNode* sequencer_node)
{
	foreach(SequenceElement* ptr_sequence_element, sequence_element_list_)
	{
		ptr_sequence_element->playElement(sequencer_node);
	}
}

void Sequence::read(const QJsonObject &json)
{
	//SequenceElement pointer QList
	sequence_element_list_.clear();
	QJsonArray sequenceElementArray = json["ptr_sequence_element"].toArray();

	for(int i = 0; i<sequenceElementArray.size(); ++i)
	{
		QJsonObject ptr_sequence_elementObject = sequenceElementArray[i].toObject();
		SequenceElement* ptr_sequence_element;
		ptr_sequence_element->read(ptr_sequence_elementObject);
		sequence_element_list_.append(ptr_sequence_element);
	}
}

void Sequence::write(QJsonObject &json) const
{
	//SequenceElement pointer QList
	QJsonArray sequenceElementArray;
	foreach(const SequenceElement* ptr_sequence_element, sequence_element_list_)
	{
		QJsonObject ptr_sequence_elementObject;
		ptr_sequence_element->write(ptr_sequence_elementObject);
		sequenceElementArray.append(ptr_sequence_elementObject);
	}
	json["ptr_sequence_element"] = sequenceElementArray;
}
