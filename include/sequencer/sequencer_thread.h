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
 * @file sequencer_thread.h
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 0.1.0
 * @brief Header file for class SequencerThread
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef OSA_GUI_SEQUENCER_SEQUENCETHREAD_H
#define OSA_GUI_SEQUENCER_SEQUENCETHREAD_H

#include <posture.h>
#include <sequence.h>
#include <sequencer_node.h>
#include <QObject>
#include <QList>

namespace osa_gui
{
namespace sequencer
{

/*! \class SequencerThread
*  \brief
*
*  This class is
*/
class SequencerThread : public QThread
{
Q_OBJECT
using QThread::run; // This is a final class

public:
	/** @brief Constructor. */
	SequencerThread(QObject * parent = 0);

	/** @brief Destructor. */
	~SequencerThread();

	void init();
	void run();

	int playSelectedPosture(Posture* ptr_posture);

	//setters
	int setPPosture(Posture* ptr_posture);
	int setPSequence(Sequence* ptr_sequence);
	int setPlayPosture(bool play_posture);
	int setPlaySequence(bool play_sequence);
	int setPSequencerNode(rosnode::SequencerNode* ptr_sequencer_node);

	//getters
	Posture* getPosture() const { return ptr_posture_; };
	Sequence* getSequence() const { return ptr_sequence_; };
	bool getPlayPosture() const { return play_posture_; };
	bool getPlaySequence() const { return play_sequence_; };
	rosnode::SequencerNode* getSequencerNode() const { return ptr_sequencer_node_; };

protected:
	Posture* ptr_posture_; //selected posture
	Sequence* ptr_sequence_; //TODO multi sequence Qlist
	bool play_posture_;
	bool play_sequence_;
	rosnode::SequencerNode* ptr_sequencer_node_;
};

} // namespace sequencer
} // namespace osa_gui

#endif // OSA_GUI_SEQUENCER_SEQUENCETHREAD_H
