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
 * @file sequencer_gui.h
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version OSA 2.0.0
 * @brief Header file for Qt based gui class SequencerGUI.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Mar 21, 2017
 */

#ifndef OSA_GUI_GUI_SEQUENCER_GUI_H
#define OSA_GUI_GUI_SEQUENCER_GUI_H

#include <QList>
#include <QMdiSubWindow>
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include "ui_SequencerGUI.h"
#include "json_motor_data_multi_array.h"
#include "posture.h"
#include "sequencer_node.h"
#include "face_tracking_node.h"
#include "sequencer_thread.h"
#include "robot_defines.h"

namespace osa_gui
{
namespace gui
{

/**
 * @brief This defines the Qt GUI for the sequencer window.
 */
class SequencerGUI : public QMdiSubWindow
{
Q_OBJECT

public:
	/**
	 * @brief Constructor.
	 */
	SequencerGUI(QWidget *parent = 0);

	/**
	 * @brief Destructor.
	 */
	~SequencerGUI();

	//setters
	int addPosture(sequencer::Posture* ptr_posture);
	int setSequencerThread(sequencer::SequencerThread* ptr_sequence_thread);

	//getters
	QList<sequencer::Posture*> getPostureList() const { return posture_list_; };
	sequencer::SequencerThread* getSequencerThread() const { return ptr_sequencer_thread_; };

public Q_SLOTS:
	/*! Auto-connections (connectSlotsByName()) */
	void on_pb_record_clicked();
	void on_pb_playPosture_clicked();
	void on_pb_playSequence_clicked();
	void on_pb_add_clicked();
	void on_pb_remove_clicked();
	void on_pb_motionStyle_clicked();
	void on_pb_pause_clicked();
	void on_ch_enablePublish_stateChanged(int state);
	void on_ch_enableFaceTracking_stateChanged(int state);
	void updateJSONMotorDataMultiArray(osa_msgs::MotorDataMultiArray motor_data_array);

Q_SIGNALS:
	void sendMotorCmdMultiArray(osa_msgs::MotorCmdMultiArray motor_cmd_multi_array);

private:
	Ui::SequencerGUIClass ui_;
	rosnode::SequencerNode* ptr_sequencer_node_;
	rosnode::FaceTrackingNode* ptr_face_tracking_node_;
	osa_msgs::MotorDataMultiArray motor_data_array_;
	QList<sequencer::Posture*> posture_list_;
	sequencer::SequencerThread* ptr_sequencer_thread_;
};

} // namespace gui
} // namespace osa_gui

#endif // OSA_GUI_GUI_SEQUENCER_GUI_H
