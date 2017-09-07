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
 * @file basic_control_gui.h
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version OSA 2.0.0
 * @brief Header file for Qt based gui class BasicControlGUI.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#ifndef OSA_GUI_GUI_BASIC_CONTROL_GUI_H
#define OSA_GUI_GUI_BASIC_CONTROL_GUI_H

#include <QList>
#include <QMdiSubWindow>
#include "ui_BasicControlGUI.h"
#include "../gui/basic_control_gui_controller.h"
#include "../rosnode/basic_control_node.h"
#include "robot_defines.h"

#define RANGE_MAX_POSITION 	2147483647
#define RANGE_MAX_VELOCITY 	5000
#define RANGE_MAX_CURRENT 	2000

namespace osa_gui
{
namespace gui
{

/**
 * @brief This defines the Qt GUI for the basic control window.
 */
class BasicControlGUI : public QMdiSubWindow
{
Q_OBJECT

public:
	/**
	 * @brief Constructor.
	 */
	BasicControlGUI(QWidget *parent = 0);

	/**
	 * @brief Destructor.
	 */
	~BasicControlGUI();

	void resetMotorCmdMultiArray();

	//setters
	int addBasicControlGUIController(BasicControlGUIController* ptr_basic_control_gui_controller);
	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/*! Auto-connections (connectSlotsByName()) */
	void on_ch_enable_0_stateChanged(int state);
	void on_ch_selected_0_stateChanged(int state); //Checkbox select
	void on_ch_limit_0_stateChanged(int state);
	void on_sb_minpos_0_valueChanged(int i);
	void on_sb_maxpos_0_valueChanged(int i);
	void on_sb_maxcur_0_valueChanged(int i);
	void on_cb_mode_0_currentIndexChanged(int index);
	void on_pb_send_0_clicked();
	void on_pb_stop_0_clicked();
	void on_ch_enablePublish_stateChanged(int state);

	//slots of callbacks
	//void setMotorDataLabels();

private:
	Ui::BasicControlGUIClass ui_;
	QList<BasicControlGUIController*> basic_control_gui_controller_list_;
	rosnode::BasicControlNode basic_control_node_;
	osa_msgs::MotorCmdMultiArray motor_cmd_array_;
};

} // namespace gui
} // namespace osa_gui

#endif // OSA_GUI_GUI_BASIC_CONTROL_GUI_H
