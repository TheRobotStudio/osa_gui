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
 * @file basic_control_gui_controller.h
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version OSA 2.0.0
 * @brief Header file for Qt based gui class BasicControlGUIController.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#ifndef OSA_GUI_GUI_BASIC_CONTROL_GUI_CONTROLLER_H
#define OSA_GUI_GUI_BASIC_CONTROL_GUI_CONTROLLER_H

#include <QString>
#include <QMdiSubWindow>
#include "ui_BasicControlGUIController.h"
#include <osa_msgs/MotorCmdMultiArray.h>
#include <osa_msgs/MotorDataMultiArray.h>
#include "robot_defines.h"

#define RANGE_MAX_POSITION 	2147483647
#define RANGE_MAX_VELOCITY 	5000
#define RANGE_MAX_CURRENT 	2000

namespace osa_gui
{
namespace gui
{

/**
 * @brief This defines the Qt GUI for one motor controller.
 */
class BasicControlGUIController : public QMdiSubWindow
{
Q_OBJECT

public:
	/**
	 * @brief Constructor.
	 */
	BasicControlGUIController(QWidget *parent = 0, uint8_t dof_idx = 0, QString board_type = "EPOS2", uint8_t node_id = 1, QString name = "Motor name");

	/**
	 * @brief Destructor.
	 */
	~BasicControlGUIController();

	//getter
	Ui::BasicControlGUIControllerClass getUi() const { return ui_; };

	//setter
	void setMotorName(QString motor_name);

public Q_SLOTS:
	/*! Auto-connections (connectSlotsByName()) */
	void on_ch_enable_stateChanged(int state);
	//void on_ch_selected_stateChanged(int state); //Checkbox select
	void on_cb_board_currentIndexChanged(int index);
	void on_ch_limit_stateChanged(int state);
	void on_sb_minpos_valueChanged(int i);
	void on_sb_maxpos_valueChanged(int i);
	void on_sb_maxcur_valueChanged(int i);
	void on_cb_mode_currentIndexChanged(int index);
	void on_sb_accel_valueChanged(int i);
	void on_sb_decel_valueChanged(int i);
	void on_sb_velocity_valueChanged(int i);
	void on_sb_maxoutcurr_valueChanged(int i);
	void on_sb_target_valueChanged(int i);
	void on_hs_target_valueChanged();
	void on_pb_send_clicked();
	void on_pb_stop_clicked();
	void on_pb_halt_clicked();
	void on_pb_enableOperation_clicked();
	void on_pb_faultReset_clicked();

	//slots of callbacks
	void setMotorDataLabels(osa_msgs::MotorDataMultiArray motor_data_array);

Q_SIGNALS:
	void sendMotorCmd(osa_msgs::MotorCmd motor_cmd);

private:
	Ui::BasicControlGUIControllerClass ui_;
	osa_msgs::MotorCmd motor_cmd_;
	uint16_t controlword_;
	uint8_t dof_idx_;
};

} // namespace gui
} // namespace osa_gui

#endif // OSA_GUI_GUI_BASIC_CONTROL_GUI_CONTROLLER_H
