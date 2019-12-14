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
 * @file sequence.h
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 0.1.0
 * @brief Header file for class Sequence
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef OSA_GUI_SEQUENCER_SEQUENCE_H
#define OSA_GUI_SEQUENCER_SEQUENCE_H

#include <sequence_element.h>
#include <QJsonObject>
#include <QList>

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
	/** @brief Constructor. */
	Sequence();

	/** @brief Destructor. */
	~Sequence();

	//setters
	int addSequenceElement(SequenceElement* ptr_sequence_element);

	//getters
	QList<SequenceElement*> getSequenceElementList() const { return sequence_element_list_; };

	void playSequence(rosnode::SequencerNode* sequencer_node);

	/** @brief Read method for JSON serialization. */
	void read(const QJsonObject &json);

	/** @brief Write method for JSON serialization. */
	void write(QJsonObject &json)  const;

//attributes
protected:
	QList<SequenceElement*> sequence_element_list_;
};

} // namespace sequencer
} // namespace osa_gui

#endif // OSA_GUI_SEQUENCER_SEQUENCE_H
