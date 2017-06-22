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

#include "SequencerThread.hpp"

using namespace std;
using namespace osa_gui;
using namespace sequencer;
using namespace Qt;

//constructors
SequencerThread::SequencerThread(QObject * parent) :
		QThread(parent),
		m_pPosture(new Posture()),
		m_pSequence(new Sequence()),
		m_playPosture(false),
		m_playSequence(false),
		m_pSequencerNode(NULL)
{
	//Connect the node to ROS
	//QObject::connect(m_pSequencerNode, &rosnode::SequencerNode::motorDataReceived, this, &SequencerGUI::updateLpJSONMotorDataMultiArray);
	//QObject::connect(m_pSequencerNode, SIGNAL(rosShutdown()), this, SLOT(close()));
	//m_pSequencerNode->init();
}

//destructor
SequencerThread::~SequencerThread()
{
	delete m_pSequence;

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
		if(m_pSequencerNode != 0)
		{
			if(m_playPosture)
			{
				m_pPosture->playElement(m_pSequencerNode);
				//posture played
				m_playPosture = false;
			}

			if(m_playSequence)
			{
				m_pSequence->playSequence(m_pSequencerNode);
				//sequence finished
				m_playSequence = false;
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
	m_pPosture = pPosture;

	return 0;
}

int SequencerThread::setPSequence(Sequence* pSequence)
{
	m_pSequence = pSequence;

	return 0;
}

int SequencerThread::setPlayPosture(bool playPosture)
{
	m_playPosture = playPosture;

	return 0;
}

int SequencerThread::setPlaySequence(bool playSequence)
{
	m_playSequence = playSequence;

	return 0;
}

int SequencerThread::setPSequencerNode(rosnode::SequencerNode* pSequencerNode)
{
	m_pSequencerNode = pSequencerNode;

	return 0;
}
