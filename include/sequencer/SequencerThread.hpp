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
 * @file SequencerThread.hpp
 * @author Cyril Jourdan
 * @date Mar 28, 2017
 * @version 2.0.0
 * @brief Header file for class SequencerThread
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 28, 2016
 */

#ifndef SEQUENCETHREAD_HPP_
#define SEQUENCETHREAD_HPP_

#include <QObject>
#include <QList>
#include "Posture.hpp"
#include "Sequence.hpp"
#include "SequencerNode.hpp"

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

			//methods
			public:
				SequencerThread(QObject * parent = 0); //constructors
				~SequencerThread(); //destructor
				void init();
				void run();

				int playSelectedPosture(Posture* pPosture);

				//setters
				int setPPosture(Posture* pPosture);
				int setPSequence(Sequence* pSequence);
				int setPlayPosture(bool playPosture);
				int setPlaySequence(bool playSequence);
				int setPSequencerNode(rosnode::SequencerNode* pSequencerNode);

				//getters
				Posture* getPPosture() const { return m_pPosture; };
				Sequence* getPSequence() const { return m_pSequence; };
				bool getPlayPosture() const { return m_playPosture; };
				bool getPlaySequence() const { return m_playSequence; };
				rosnode::SequencerNode* getPSequencerNode() const { return m_pSequencerNode; };

			//attributes
			protected:
				Posture* m_pPosture; //selected posture
				Sequence* m_pSequence; //TODO multi sequence Qlist
				bool m_playPosture;
				bool m_playSequence;
				rosnode::SequencerNode* m_pSequencerNode;
		};
	}
}

#endif /* SEQUENCETHREAD_HPP_ */
