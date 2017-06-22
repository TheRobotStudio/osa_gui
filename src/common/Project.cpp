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
 * @file Project.cpp
 * @author Cyril Jourdan
 * @date Dec 8, 2016
 * @version 0.0.1
 * @brief Implementation file for class Project
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Dec 8, 2016
 */

#include <QJsonArray>
#include <iostream>
#include "Project.hpp"

using namespace std;
using namespace osa_gui;
using namespace common;
using namespace Qt;

//constructor
Project::Project() :
		m_pFile(NULL),
		m_pRobot(NULL),
		m_pSequence(NULL),
		m_lpPosture(QList<sequencer::Posture*>())
{
}

//destructor
Project::~Project()
{
	delete m_pFile;
	delete m_pRobot;
	delete m_pSequence;
	//TODO delete Qlist
}

//setters
int Project::setPFile(QFile* pFile)
{
	//check the value
	if(pFile != 0)
	{
		m_pFile = pFile;

		return 0;
	}
	else
		return -1;
}

int Project::setPRobot(Robot* pRobot)
{
	//check the value
	if(pRobot != 0)
	{
		m_pRobot = pRobot;

		return 0;
	}
	else
		return -1;
}

int Project::setPSequence(sequencer::Sequence* pSequence)
{
	//check the value
	if(pSequence != 0)
	{
		m_pSequence = pSequence;

		return 0;
	}
	else
		return -1;
}

int Project::addPPosture(sequencer::Posture* pPosture)
{
	//check the value
	if(pPosture != 0)
	{
		m_lpPosture.append(pPosture);

		return 0;
	}
	else
		return -1;
}

void Project::read(const QJsonObject &json)
{
	m_pRobot->read(json);
	m_pSequence->read(json);

	//TODO posture list
}

void Project::write(QJsonObject &json) const
{
	m_pRobot->write(json);
	m_pSequence->write(json);

	//TODO posture list
}
