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
 * @file SequencerGUI.cpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version 0.0.1
 * @brief Implementation file for class SequencerGUI.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 21, 2017
 */

/*! Include */
#include <stdint.h>
#include <QModelIndex>
#include <QInputDialog>
#include "include/gui/SequencerGUI.hpp"
#include <osa_msgs/MotorData.h>
#include "robotDefines.h"
#include "Pause.hpp"
#include "Condition.hpp"
#include "Posture.hpp"
#include "MotionStyle.hpp"

using namespace std;
using namespace osa_gui;
using namespace gui;
//using namespace common;
//using namespace osa_msgs_json;
using namespace Qt;

/*! \fn
 *  \brief
 *  \param
 */
SequencerGUI::SequencerGUI(QWidget *parent) :
		QMdiSubWindow(parent),
		m_pSequencerNode(new rosnode::SequencerNode()),
		m_pFaceTrackingNode(new rosnode::FaceTrackingNode()),
		m_lpPosture(QList<sequencer::Posture*>()),
		m_pSequencerThread(new sequencer::SequencerThread())
{
	m_ui.setupUi(this); //TODO learn and use the model/view methods for the postures and sequence table

	//init motor data multi array
	m_motorDataMultiArray.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m_motorDataMultiArray.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	m_motorDataMultiArray.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorDataMultiArray.layout.dim[0].label = "slaves";
	m_motorDataMultiArray.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m_motorDataMultiArray.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorDataMultiArray.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorDataMultiArray.layout.dim[1].label = "motors";
	m_motorDataMultiArray.layout.data_offset = 0;
	m_motorDataMultiArray.motorData.clear();
	m_motorDataMultiArray.motorData.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			m_motorDataMultiArray.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].position = 0;
			m_motorDataMultiArray.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].current = 0;
			m_motorDataMultiArray.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].status = 0;
		}
	}

	//Connect the node to ROS
	QObject::connect(m_pSequencerNode, &rosnode::SequencerNode::motorDataReceived, this, &SequencerGUI::updateLpJSONMotorDataMultiArray);
	//Link the Face Tracking node to the Sequencer node, so it transmit the head motor commands when it receives head coordinates
	QObject::connect(m_pFaceTrackingNode, &rosnode::FaceTrackingNode::sendMotorCmd_ma, m_pSequencerNode, &rosnode::SequencerNode::updateMotorCmd_ma);
	//Connect for clean ROS shutdown
	QObject::connect(m_pSequencerNode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(m_pFaceTrackingNode, SIGNAL(rosShutdown()), this, SLOT(close()));

	//Init ROS nodes
	m_pSequencerNode->init();
	m_pFaceTrackingNode->init();

	//Connect the Sequencer
	//QObject::connect(m_pSequence, &sequencer::Sequence::motorDataReceived, this, &SequencerGUI::updateLpJSONMotorDataMultiArray);
	m_pSequencerThread->setPSequencerNode(m_pSequencerNode);
	m_pSequencerThread->init();
}

/*! \fn
 *  \brief
 *  \param
 */
SequencerGUI::~SequencerGUI()
{
	//stop the threads first
	//m_pSequencerThread->

	delete m_pSequencerNode;
	delete m_pFaceTrackingNode;
	delete m_pSequencerThread;
	m_lpPosture.clear();
}

int SequencerGUI::addPPosture(sequencer::Posture* pPosture)
{
	m_lpPosture.append(pPosture);

	return 0;
}

int SequencerGUI::setPSequencerThread(sequencer::SequencerThread *sequencerThread)
{
	m_pSequencerThread = sequencerThread;

	return 0;
}

void SequencerGUI::on_pb_record_clicked()
{
	bool ok;
	QString postureName = QInputDialog::getText(this, tr("Posture name"), tr(""), QLineEdit::Normal, "", &ok);

	if(ok && !postureName.isEmpty())
	{
		//textLabel->setText(postureName);
		m_ui.lw_postures->addItem(postureName);
	}

	//TODO select the line just created
	//m_ui.lw_postures->setItemSelected(m_ui.lw_postures->currentItem(), true);

	//Add the current posture to the array
	common::osa_msgs_json::JSONMotorDataMultiArray *pJSONMotorDataMultiArray = new common::osa_msgs_json::JSONMotorDataMultiArray();

	for(int i=0; i<m_motorDataMultiArray.layout.dim[0].stride; i++)
	{
		common::osa_msgs_json::JSONMotorData *pJSONMotorData = new common::osa_msgs_json::JSONMotorData();

		//int test = (m_pSequencerNode->getMotorData_ma()).motorData[i].position;
		pJSONMotorData->setPosition((m_pSequencerNode->getMotorData_ma()).motorData[i].position);
		pJSONMotorData->setCurrent((m_pSequencerNode->getMotorData_ma()).motorData[i].current);
		pJSONMotorData->setStatus((m_pSequencerNode->getMotorData_ma()).motorData[i].status);
		pJSONMotorDataMultiArray->addPJSONMotorData(pJSONMotorData);
	}

	sequencer::Posture *pPosture = new sequencer::Posture();
	pPosture->setPJSONMotorDataMultiArray(pJSONMotorDataMultiArray);
	addPPosture(pPosture);
}

void SequencerGUI::on_pb_playPosture_clicked()
{
	//get the selected posture index in the list
	int idxList = m_ui.lw_postures->currentIndex().row();

	//play the selected posture
	m_pSequencerThread->playSelectedPosture(m_lpPosture.at(idxList));
}

void SequencerGUI::on_pb_playSequence_clicked()
{
	m_pSequencerThread->setPlaySequence(true);
}

void SequencerGUI::on_pb_add_clicked()
{
	//check which line is selected
	//QModelIndex *idxList = new QModelIndex(m_ui.lw_postures->currentIndex());// m_ui.lw_postures->currentIndex();
	int idxList = m_ui.lw_postures->currentIndex().row();// m_ui.lw_postures->currentIndex();
	//QModelIndexList idxList = m_ui.lw_postures->selectedIndexes();
	//ROS_INFO("add posture %d", idxList);
	m_pSequencerThread->getPSequence()->addPSequenceElement(m_lpPosture.at(idxList));
	//add corresponding item on the table
	m_ui.tw_sequence->setRowCount(m_pSequencerThread->getPSequence()->getLpSequenceElement().size());
	m_ui.tw_sequence->setItem(m_pSequencerThread->getPSequence()->getLpSequenceElement().size()-1, 0, new QTableWidgetItem(m_ui.lw_postures->currentItem()->text()));
	//m_ui.tw_sequence->setItem(m_pSequence->getLpSequenceElement().size()-1, 1, new QTableWidgetItem(m_lpPosture.at(idxList)->));
}

void SequencerGUI::on_pb_remove_clicked()
{

}

void SequencerGUI::on_pb_motionStyle_clicked()
{

}

void SequencerGUI::on_pb_pause_clicked()
{

	bool ok;
	int duration = QInputDialog::getInt(this, tr("Duration in ms"), tr(""), 0, 0, 1000000, 1, &ok);

	sequencer::Pause *pause = new sequencer::Pause();

	if(ok) pause->setMsDuration((uint32_t)duration);

	m_pSequencerThread->getPSequence()->addPSequenceElement(pause);
	//add corresponding item on the table
	m_ui.tw_sequence->setRowCount(m_pSequencerThread->getPSequence()->getLpSequenceElement().size());
	m_ui.tw_sequence->setItem(m_pSequencerThread->getPSequence()->getLpSequenceElement().size()-1, 0, new QTableWidgetItem("Pause"));
	m_ui.tw_sequence->setItem(m_pSequencerThread->getPSequence()->getLpSequenceElement().size()-1, 1, new QTableWidgetItem(QString::number(duration)));

}

void SequencerGUI::on_ch_enablePublish_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		m_pSequencerNode->setEnablePublish(false);
	}
	else if(state == 2) //checked
	{
		m_pSequencerNode->setEnablePublish(true);
	}
}

void SequencerGUI::on_ch_enableFaceTracking_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		m_pFaceTrackingNode->setEnableNode(false);
	}
	else if(state == 2) //checked
	{
		m_pFaceTrackingNode->setEnableNode(true);
	}
}

void SequencerGUI::updateLpJSONMotorDataMultiArray(osa_msgs::MotorDataMultiArray motorData_ma)
{
	for(int i=0; i<motorData_ma.layout.dim[0].stride ; i++)
	{
		m_motorDataMultiArray.motorData[i].position = motorData_ma.motorData[i].position;
		m_motorDataMultiArray.motorData[i].current = motorData_ma.motorData[i].current;
		m_motorDataMultiArray.motorData[i].status = motorData_ma.motorData[i].status;
	}
}
