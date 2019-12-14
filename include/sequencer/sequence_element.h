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
 * @file sequence_element.h
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 0.1.0
 * @brief Header file for abstract class SequenceElement
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef OSA_GUI_SEQUENCER_SEQUENCEELEMENT_H
#define OSA_GUI_SEQUENCER_SEQUENCEELEMENT_H

#include <sequencer_node.h>
#include <QJsonObject>
#include <stdint.h>

namespace osa_gui
{
namespace sequencer
{

/**
*  @brief Abstract class representing a sequence element for the sequencer.
*
*  Base class for all sequence element like Posture, MotionStyle, Pause, Condition...
*/
class SequenceElement
{
//methods
public:
	/** @brief Constructor. */
	SequenceElement();

	/** @brief Destructor. */
	virtual ~SequenceElement();

	virtual void playElement(rosnode::SequencerNode* sequencerNode) = 0;

	/** @brief Read method for JSON serialization. */
	virtual void read(const QJsonObject &json) = 0;

	/** @brief Write method for JSON serialization. */
	virtual void write(QJsonObject &json)  const = 0;
};

} // namespace sequencer
} // namespace osa_gui

#endif // OSA_GUI_SEQUENCER_SEQUENCEELEMENT_H
