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
 * @file sequencer_thread.cpp
 * @author Cyril Jourdan
 * @date Dec 12, 2016
 * @version 0.1.0
 * @brief Implementation file for abstract class SequencerThread
 *
 * Contact: contact@therobotstudio.com
 * Created on : Dec 12, 2016
 */

#include <sequencer_thread.h>

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

//constructors
SequencerThread::SequencerThread(QObject * parent) :
	QThread(parent),
	ptr_posture_(new Posture(0)),
	ptr_sequence_(new Sequence()),
	play_posture_(false),
	play_sequence_(false),
	ptr_sequencer_node_(NULL)
{
	//Connect the node to ROS
	//QObject::connect(ptr_sequencer_node_, &rosnode::SequencerNode::motorDataReceived, this, &SequencerGUI::updateLpJSONMotorDataMultiArray);
	//QObject::connect(ptr_sequencer_node_, SIGNAL(rosShutdown()), this, SLOT(close()));
	//ptr_sequencer_node_->init();
}

//destructor
SequencerThread::~SequencerThread()
{
	delete ptr_sequence_;

	//quit();
	//requestInterruption();
	//wait();
}

void SequencerThread::init()
{
	start();
}

void SequencerThread::run() //TODO check the use of exec()
{
	bool loop = true;

	while(loop)
	{
		if(ptr_sequencer_node_ != 0)
		{
			if(play_posture_)
			{
				//ROS_INFO("play posture in thread");	
				ptr_posture_->playElement(ptr_sequencer_node_);
				//posture played
				play_posture_ = false;
			}

			if(play_sequence_)
			{
				ptr_sequence_->playSequence(ptr_sequencer_node_);
				//sequence finished
				play_sequence_ = false;
			}
		}
	}
}

int SequencerThread::playSelectedPosture(Posture* pPosture)
{
	setPlayPosture(true);
	setPPosture(pPosture);

	return 0;
}

int SequencerThread::setPPosture(Posture* pPosture)
{
	ptr_posture_ = pPosture;

	return 0;
}

int SequencerThread::setPSequence(Sequence* pSequence)
{
	ptr_sequence_ = pSequence;

	return 0;
}

int SequencerThread::setPlayPosture(bool playPosture)
{
	play_posture_ = playPosture;

	return 0;
}

int SequencerThread::setPlaySequence(bool playSequence)
{
	play_sequence_ = playSequence;

	return 0;
}

int SequencerThread::setPSequencerNode(rosnode::SequencerNode* pSequencerNode)
{
	ptr_sequencer_node_ = pSequencerNode;

	return 0;
}
