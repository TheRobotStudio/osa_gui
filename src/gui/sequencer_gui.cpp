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
 * @file sequencer_gui.cpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version 0.1.0
 * @brief Implementation file for class SequencerGUI.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 21, 2017
 */

/*! Include */
#include <condition.h>
#include <motion_style.h>
#include <stdint.h>
#include <QModelIndex>
#include <QInputDialog>
#include <osa_msgs/MotorData.h>
#include <pause.h>
#include <posture.h>
#include <sequencer_gui.h>

#include "robot_defines.h"

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
	ptr_sequencer_node_(new rosnode::SequencerNode()),
	ptr_face_tracking_node_(new rosnode::FaceTrackingNode()),
	posture_list_(QList<sequencer::Posture*>()),
	ptr_sequencer_thread_(new sequencer::SequencerThread())
{
	ui_.setupUi(this); //TODO learn and use the model/view methods for the postures and sequence table

	//Connect the node to ROS
	QObject::connect(ptr_sequencer_node_, &rosnode::SequencerNode::motorDataReceived, this, &SequencerGUI::updateJSONMotorDataMultiArray);
	//Link the Face Tracking node to the Sequencer node, so it transmit the head motor commands when it receives head coordinates
	QObject::connect(ptr_face_tracking_node_, &rosnode::FaceTrackingNode::sendMotorCmd_ma, ptr_sequencer_node_, &rosnode::SequencerNode::updateMotorCmdArray);
	//Connect for clean ROS shutdown
	QObject::connect(ptr_sequencer_node_, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(ptr_face_tracking_node_, SIGNAL(rosShutdown()), this, SLOT(close()));

	//Init ROS nodes
	ptr_sequencer_node_->init();
	ptr_face_tracking_node_->init();

	//init motor data multi array
	motor_data_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_data_array_.layout.dim[0].size = ptr_sequencer_node_->getNumberEPOSBoards();//NUMBER_SLAVE_BOARDS;
	motor_data_array_.layout.dim[0].stride = ptr_sequencer_node_->getNumberEPOSBoards(); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_data_array_.layout.dim[0].label = "epos";
/*
	motor_data_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_data_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_data_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_data_array_.layout.dim[1].label = "motors";
*/
	motor_data_array_.layout.data_offset = 0;
	motor_data_array_.motor_data.clear();
	motor_data_array_.motor_data.resize(ptr_sequencer_node_->getNumberEPOSBoards()); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	int num_nodes = ptr_sequencer_node_->getNumberEPOSBoards();

//	ROS_INFO("num_nodes=%d", num_nodes);

	for(int i=0; i<num_nodes; i++)
	{
		motor_data_array_.motor_data[i].node_id = 0;
		motor_data_array_.motor_data[i].position = 0;
		motor_data_array_.motor_data[i].current = 0;
		motor_data_array_.motor_data[i].status = 0;
	}

	//Connect the Sequencer
	//QObject::connect(m_pSequence, &sequencer::Sequence::motorDataReceived, this, &SequencerGUI::updateLpJSONMotorDataMultiArray);
	ptr_sequencer_thread_->setPSequencerNode(ptr_sequencer_node_);
	ptr_sequencer_thread_->init();
}

/**
 * @brief Destructor.
 */
SequencerGUI::~SequencerGUI()
{
	//stop the threads first
	//ptr_sequencer_thread_->

	delete ptr_sequencer_node_;
	delete ptr_face_tracking_node_;
	delete ptr_sequencer_thread_;
	posture_list_.clear();
}

int SequencerGUI::addPosture(sequencer::Posture* ptr_posture)
{
	posture_list_.append(ptr_posture);

	return 0;
}

int SequencerGUI::setSequencerThread(sequencer::SequencerThread *sequencerThread)
{
	ptr_sequencer_thread_ = sequencerThread;

	return 0;
}

void SequencerGUI::on_pb_record_clicked()
{
	bool ok;
	QString postureName = QInputDialog::getText(this, tr("Posture name"), tr(""), QLineEdit::Normal, "", &ok);

	if(ok && !postureName.isEmpty())
	{
		//textLabel->setText(postureName);
		ui_.lw_postures->addItem(postureName);
	}

	//TODO select the line just created
	//ui_.lw_postures->setItemSelected(ui_.lw_postures->currentItem(), true);

	//Add the current posture to the array
	common::osa_msgs_json::JSONMotorDataMultiArray *pJSONMotorDataMultiArray = new common::osa_msgs_json::JSONMotorDataMultiArray();

	for(int i=0; i<motor_data_array_.layout.dim[0].stride; i++)
	{
		common::osa_msgs_json::JSONMotorData *ptr_json_motor_data = new common::osa_msgs_json::JSONMotorData();

		//int test = (ptr_sequencer_node_->getMotorDataArray()).motor_data[i].position;
		ptr_json_motor_data->setNodeID((ptr_sequencer_node_->getMotorDataArray()).motor_data[i].node_id);
		ptr_json_motor_data->setPosition((ptr_sequencer_node_->getMotorDataArray()).motor_data[i].position);
		ptr_json_motor_data->setCurrent((ptr_sequencer_node_->getMotorDataArray()).motor_data[i].current);
		ptr_json_motor_data->setStatus((ptr_sequencer_node_->getMotorDataArray()).motor_data[i].status);
		pJSONMotorDataMultiArray->addJSONMotorData(ptr_json_motor_data);
	}

//	ROS_INFO("record clicked : nb nodes = %d", ptr_sequencer_node_->getNumberEPOSBoards());
//	int nb_nodes = ptr_sequencer_node_->getNumberEPOSBoards();

	sequencer::Posture *ptr_posture = new sequencer::Posture(ptr_sequencer_node_->getNumberEPOSBoards());
	//sequencer::Posture *ptr_posture = new sequencer::Posture(nb_nodes);
	ptr_posture->setJSONMotorDataArray(pJSONMotorDataMultiArray);
	addPosture(ptr_posture);
}

void SequencerGUI::on_pb_playPosture_clicked()
{
	//get the selected posture index in the list
	int idxList = ui_.lw_postures->currentIndex().row();

	//play the selected posture
	ptr_sequencer_thread_->playSelectedPosture(posture_list_.at(idxList));
}

void SequencerGUI::on_pb_playSequence_clicked()
{
	ptr_sequencer_thread_->setPlaySequence(true);
}

void SequencerGUI::on_pb_add_clicked()
{
	//check which line is selected
	//QModelIndex *idxList = new QModelIndex(ui_.lw_postures->currentIndex());// ui_.lw_postures->currentIndex();
	int idxList = ui_.lw_postures->currentIndex().row();// ui_.lw_postures->currentIndex();
	//QModelIndexList idxList = ui_.lw_postures->selectedIndexes();
	//ROS_INFO("add posture %d", idxList);
	ptr_sequencer_thread_->getSequence()->addSequenceElement(posture_list_.at(idxList));
	//add corresponding item on the table
	ui_.tw_sequence->setRowCount(ptr_sequencer_thread_->getSequence()->getSequenceElementList().size());
	ui_.tw_sequence->setItem(ptr_sequencer_thread_->getSequence()->getSequenceElementList().size()-1, 0, new QTableWidgetItem(ui_.lw_postures->currentItem()->text()));
	//ui_.tw_sequence->setItem(m_pSequence->getSequenceElementList().size()-1, 1, new QTableWidgetItem(posture_list_.at(idxList)->));
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

	ptr_sequencer_thread_->getSequence()->addSequenceElement(pause);
	//add corresponding item on the table
	ui_.tw_sequence->setRowCount(ptr_sequencer_thread_->getSequence()->getSequenceElementList().size());
	ui_.tw_sequence->setItem(ptr_sequencer_thread_->getSequence()->getSequenceElementList().size()-1, 0, new QTableWidgetItem("Pause"));
	ui_.tw_sequence->setItem(ptr_sequencer_thread_->getSequence()->getSequenceElementList().size()-1, 1, new QTableWidgetItem(QString::number(duration)));
}

void SequencerGUI::on_ch_enablePublish_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		ptr_sequencer_node_->setEnablePublish(false);
	}
	else if(state == 2) //checked
	{
		ptr_sequencer_node_->setEnablePublish(true);
	}
}

void SequencerGUI::on_ch_enableFaceTracking_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		ptr_face_tracking_node_->setEnableNode(false);
	}
	else if(state == 2) //checked
	{
		ptr_face_tracking_node_->setEnableNode(true);
	}
}

void SequencerGUI::updateJSONMotorDataMultiArray(osa_msgs::MotorDataMultiArray motorData_ma)
{
	for(int i=0; i<motorData_ma.layout.dim[0].stride ; i++)
	{
		//TODO does copy directly like motor_data_array_.motor_data[i] = motorData_ma.motor_data[i]; works ?
		motor_data_array_.motor_data[i].node_id = motorData_ma.motor_data[i].node_id;
		motor_data_array_.motor_data[i].position = motorData_ma.motor_data[i].position;
		motor_data_array_.motor_data[i].current = motorData_ma.motor_data[i].current;
		motor_data_array_.motor_data[i].status = motorData_ma.motor_data[i].status;
	}
}
