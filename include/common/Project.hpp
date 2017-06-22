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
 * @file Project.hpp
 * @author Cyril Jourdan
 * @date Dec 8, 2016
 * @version 0.0.1
 * @brief Header file for class Project
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 8, 2016
 */

#ifndef COMMON_PROJECT_HPP_
#define COMMON_PROJECT_HPP_

#include <QFile>
#include <QJsonObject>
#include "Robot.hpp"
#include "Posture.hpp"
#include "Sequence.hpp"

namespace osa_gui
{
	namespace common
	{
		class Project
		{
			//methods
			public:
				//constructors
				Project();
				//destructor
				~Project();

				//setters
				int setPFile(QFile* pFile);
				int setPRobot(Robot* pRobot);
				int setPSequence(sequencer::Sequence* pSequence);
				int addPPosture(sequencer::Posture* pPosture);

				//getters
				QFile* 						getPFile() 		const { return m_pFile; };
				Robot* 						getPRobot() 	const { return m_pRobot; };
				sequencer::Sequence* 		getPSequence() 	const { return m_pSequence; };
				QList<sequencer::Posture*> 	getLpPosture() 	const { return m_lpPosture; };

				//JSON serialization
				void read(const QJsonObject &json);
				void write(QJsonObject &json) const;

			//attributes
			protected:
				QFile*						m_pFile;
				Robot*						m_pRobot;
				sequencer::Sequence*		m_pSequence; //TODO have a QList of Sequences
				QList<sequencer::Posture*> 	m_lpPosture;
		};
	}
}
#endif /* COMMON_PROJECT_HPP_ */
