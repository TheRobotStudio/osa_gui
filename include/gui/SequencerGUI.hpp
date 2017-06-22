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
 * @file SequencerGUI.hpp
 * @author Cyril Jourdan
 * @date Mar 21, 2017
 * @version OSA 2.0.0
 * @brief Header file for Qt based gui class SequencerGUI.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 21, 2017
 */

#ifndef SEQUENCERGUI_H
#define SEQUENCERGUI_H

#include <QList>
#include <QMdiSubWindow>
#include "ui_SequencerGUI.h"
#include "JSONMotorDataMultiArray.hpp"
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include "../rosnode/SequencerNode.hpp"
#include "../rosnode/FaceTrackingNode.hpp"
#include "SequencerThread.hpp"
#include "Posture.hpp"
#include "robotDefines.h"

namespace osa_gui
{
	namespace gui
	{
		class SequencerGUI : public QMdiSubWindow
		{
			Q_OBJECT

			public:
				SequencerGUI(QWidget *parent = 0);
				~SequencerGUI();

			//setters
			int addPPosture(sequencer::Posture* pPosture);
			int setPSequencerThread(sequencer::SequencerThread* pSequenceThread);

			//getters
			QList<sequencer::Posture*> getLpPosture() const { return m_lpPosture; };
			sequencer::SequencerThread* getPSequencerThread() const { return m_pSequencerThread; };

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
				void updateLpJSONMotorDataMultiArray(osa_msgs::MotorDataMultiArray motorData_ma);

			Q_SIGNALS:
				void sendMotorCmdMultiArray(osa_msgs::MotorCmdMultiArray motorCmdMultiArray);

			private:
				Ui::SequencerGUIClass m_ui;
				rosnode::SequencerNode *m_pSequencerNode;
				rosnode::FaceTrackingNode *m_pFaceTrackingNode;
				osa_msgs::MotorDataMultiArray m_motorDataMultiArray;
				QList<sequencer::Posture*> m_lpPosture;
				sequencer::SequencerThread *m_pSequencerThread;
		};
	}
}

#endif //SEQUENCERGUI_H
