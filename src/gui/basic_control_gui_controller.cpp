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
 * @file basic_control_gui_controller.cpp
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version 0.1.0
 * @brief Implementation file for class BasicControlGUIController.
 *
 * Contact: contact@therobotstudio.com
 * Created on : Feb 20, 2017
 */

/*! Include */
//#include <QtGui>
//#include <QMessageBox>
#include <iostream>
#include "include/gui/basic_control_gui_controller.h"

/*! Namespace */
using namespace std;
using namespace osa_gui;
using namespace gui;
using namespace Qt;

BasicControlGUIController::BasicControlGUIController(QWidget *parent, uint8_t dof_idx, QString board_type, uint8_t node_id, QString name) :
	QMdiSubWindow(parent, Qt::FramelessWindowHint),
	dof_idx_(dof_idx)
{
	//TODO this->showFullScreen();
	ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	//motor_cmd_.slaveBoardID = slaveBoardID;
	motor_cmd_.node_id = node_id;
	motor_cmd_.command = SEND_DUMB_MESSAGE;
	motor_cmd_.value = 0;

	//node_id label
	ui_.lbl_nodeId->setText(QString::number(node_id));

	setMotorName(name);

	//set board type
	if(board_type.compare(QString("EPOS2")) == 0)
	{
		ui_.cb_board->setCurrentIndex(0);
		QStringList modeList = (QStringList() << "POS" << "VEL" << "CUR");
		//mode, default is CUR for current
		ui_.cb_mode->addItems(modeList);
		//set CUR as default
		ui_.cb_mode->setCurrentIndex(2);
		//trigger the change
		on_cb_mode_currentIndexChanged(2);
	}
	else if(board_type.compare(QString("EPOS4")) == 0)
	{
		ui_.cb_board->setCurrentIndex(1);
		QStringList modeList = (QStringList() << "POS" << "VEL" << "TOR");
		//mode, default is CUR for current
		ui_.cb_mode->addItems(modeList);
		//set VEL as default
		ui_.cb_mode->setCurrentIndex(1);
		//trigger the change
		on_cb_mode_currentIndexChanged(1);
	}

	ui_.sb_minpos->setVisible(false);
	ui_.sb_maxpos->setVisible(false);
	ui_.sb_maxcur->setVisible(false);

	//tune position
	//this->setGeometry(16, 28*node_id, 1670, 28);

	controlword_ = 0b0000000000000000;
}

BasicControlGUIController::~BasicControlGUIController()
{
}

void BasicControlGUIController::setMotorName(QString motorName)
{
	ui_.lbl_name->setText(motorName);
}

void BasicControlGUIController::on_ch_enable_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		ui_.ch_selected->setEnabled(false);
		ui_.ch_limit->setEnabled(false);
		ui_.sb_minpos->setEnabled(false);
		ui_.sb_maxpos->setEnabled(false);
		ui_.sb_maxcur->setEnabled(false);
		ui_.cb_mode->setEnabled(false);
		ui_.sb_accel->setEnabled(false);
		ui_.sb_decel->setEnabled(false);
		ui_.sb_velocity->setEnabled(false);
		ui_.sb_maxoutcurr->setEnabled(false);
		ui_.hs_target->setEnabled(false);
		ui_.sb_target->setEnabled(false);
		ui_.pb_send->setEnabled(false);
		ui_.pb_stop->setEnabled(false);
	}
	else if(state == 2) //checked
	{
		ui_.ch_selected->setEnabled(true);
		ui_.ch_limit->setEnabled(true);
		ui_.sb_minpos->setEnabled(true);
		ui_.sb_maxpos->setEnabled(true);
		ui_.sb_maxcur->setEnabled(true);
		ui_.cb_mode->setEnabled(true);
		ui_.sb_accel->setEnabled(true);
		ui_.sb_decel->setEnabled(true);
		ui_.sb_velocity->setEnabled(true);
		ui_.sb_maxoutcurr->setEnabled(true);
		ui_.hs_target->setEnabled(true);
		ui_.sb_target->setEnabled(true);
		ui_.pb_send->setEnabled(true);
		ui_.pb_stop->setEnabled(true);
	}
}

//Individual motors
void BasicControlGUIController::on_cb_board_currentIndexChanged(int index)
{
	if(index == 0) //EPOS2
	{

	}
	else if(index == 1) //EPOS4
	{

	}
}

void BasicControlGUIController::on_ch_limit_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		ui_.sb_minpos->setVisible(false);
		ui_.sb_maxpos->setVisible(false);
		ui_.sb_maxcur->setVisible(false);

		int max = 0;
		//check which mode is active
		if(ui_.cb_mode->currentText()=="POS") max = RANGE_MAX_POSITION;
		else if(ui_.cb_mode->currentText()=="VEL") max = RANGE_MAX_VELOCITY;
		else if(ui_.cb_mode->currentText()=="CUR") max = RANGE_MAX_CURRENT;

		//set min and max of the slider and spinbox to absolute maximums, no limits.
		ui_.hs_target->setRange(0, max);
		ui_.hs_target->setValue(0); //ui_.lbl_curr_0->text().toInt()

		ui_.sb_target->setRange(0, max);
		ui_.sb_target->setValue(0);
	}
	else if(state == 2) //checked
	{
		//check which mode is active
		if(ui_.cb_mode->currentText()=="POS")
		{
			ui_.sb_minpos->setVisible(true);
			ui_.sb_maxpos->setVisible(true);
			ui_.sb_maxcur->setVisible(false);
		}
		else if(ui_.cb_mode->currentText()=="VEL")
		{
			ui_.sb_minpos->setVisible(false);
			ui_.sb_maxpos->setVisible(false);
			ui_.sb_maxcur->setVisible(false);
		}
		else if(ui_.cb_mode->currentText()=="CUR")
		{
			ui_.sb_minpos->setVisible(false);
			ui_.sb_maxpos->setVisible(false);
			ui_.sb_maxcur->setVisible(true);
		}
	}

	on_cb_mode_currentIndexChanged(ui_.cb_mode->currentIndex());
}

void BasicControlGUIController::on_sb_minpos_valueChanged(int i)
{
	ui_.hs_target->setMinimum(i);
	ui_.sb_target->setMinimum(i);
}

void BasicControlGUIController::on_sb_maxpos_valueChanged(int i)
{
	ui_.hs_target->setMaximum(i);
	ui_.sb_target->setMaximum(i);
}

void BasicControlGUIController::on_sb_maxcur_valueChanged(int i)
{
	ui_.hs_target->setMaximum(i);
	ui_.sb_target->setMaximum(i);
}

//Individual motors
void BasicControlGUIController::on_cb_mode_currentIndexChanged(int index)
{
	//std::cout << "Mode ";
	motor_cmd_.command = SET_MODES_OF_OPERATION;

	if(index == 0) //POS
	{
		motor_cmd_.value = PROFILE_POSITION_MODE;

		if(ui_.ch_limit->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			ui_.hs_target->setRange(0, RANGE_MAX_POSITION);
			ui_.hs_target->setValue(0); //ui_.lbl_curr_0->text().toInt()

			ui_.sb_target->setRange(0, RANGE_MAX_POSITION);
			ui_.sb_target->setValue(0);
		}
		else if(ui_.ch_limit->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			ui_.sb_minpos->setVisible(true);
			ui_.sb_maxpos->setVisible(true);
			ui_.sb_maxcur->setVisible(false);

			//set min and max of the slider and spinbox
			ui_.hs_target->setRange(ui_.sb_minpos->value(), ui_.sb_maxpos->value());
			ui_.hs_target->setValue(ui_.sb_minpos->value());

			ui_.sb_target->setRange(ui_.sb_minpos->value(), ui_.sb_maxpos->value());
			ui_.sb_target->setValue(ui_.sb_minpos->value());
		}

		ui_.sb_accel->setVisible(true);
		ui_.sb_decel->setVisible(true);
		ui_.sb_velocity->setVisible(true);

		//std::cout << "position" << endl;
	}
	else if(index == 1) //VEL
	{
		motor_cmd_.value = PROFILE_VELOCITY_MODE;

		//set visibility of the limit QSpinBox
		ui_.sb_minpos->setVisible(false);
		ui_.sb_maxpos->setVisible(false);
		ui_.sb_maxcur->setVisible(false);

		//set min and max of the slider and spinbox to absolute maximums, no limits.
		ui_.hs_target->setRange(-RANGE_MAX_VELOCITY, RANGE_MAX_VELOCITY);
		ui_.hs_target->setValue(0); //ui_.lbl_curr_0->text().toInt()

		ui_.sb_target->setRange(-RANGE_MAX_VELOCITY, RANGE_MAX_VELOCITY);
		ui_.sb_target->setValue(0);

		ui_.sb_accel->setVisible(true);
		ui_.sb_decel->setVisible(true);
		ui_.sb_velocity->setVisible(true);

		//std::cout << "velocity" << endl;
	}
	else if(index == 2) //CUR
	{
		motor_cmd_.value = CURRENT_MODE;

		if(ui_.ch_limit->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			ui_.hs_target->setRange(0, RANGE_MAX_CURRENT);
			ui_.hs_target->setValue(0);

			ui_.sb_target->setRange(0, RANGE_MAX_CURRENT);
			ui_.sb_target->setValue(0);
		}
		else if(ui_.ch_limit->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			ui_.sb_minpos->setVisible(false);
			ui_.sb_maxpos->setVisible(false);
			ui_.sb_maxcur->setVisible(true);

			//set min and max of the slider and spinbox
			ui_.hs_target->setRange(0, ui_.sb_maxcur->value());
			ui_.hs_target->setValue(0);

			ui_.sb_target->setRange(0, ui_.sb_maxcur->value());
			ui_.sb_target->setValue(0);
		}

		ui_.sb_accel->setVisible(false);
		ui_.sb_decel->setVisible(false);
		ui_.sb_velocity->setVisible(true);

		//std::cout << "current" << endl;
	}

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_sb_accel_valueChanged(int i)
{
	motor_cmd_.command = SET_PROFILE_ACCELERATION;
	motor_cmd_.value = i;

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_sb_decel_valueChanged(int i)
{
	motor_cmd_.command = SET_PROFILE_DECELERATION;
	motor_cmd_.value = i;

	Q_EMIT sendMotorCmd(motor_cmd_);
}

//TODO WARNING a change on prof vel also send the command, why ? check mbed code, they are on the same PDO probably
void BasicControlGUIController::on_sb_velocity_valueChanged(int i)
{
	motor_cmd_.command = SET_PROFILE_VELOCITY;
	motor_cmd_.value = i;

	Q_EMIT sendMotorCmd(motor_cmd_);
}

//TODO WARNING a change on max cont curr also send the command, why ? check mbed code, they are on the same PDO probably
void BasicControlGUIController::on_sb_maxoutcurr_valueChanged(int i)
{
	motor_cmd_.command = SET_OUTPUT_CURRENT_LIMIT;
	motor_cmd_.value = i;

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_hs_target_valueChanged()
{
	//change the target value accordingly
	ui_.sb_target->setValue(ui_.hs_target->value());

	if(ui_.cb_mode->currentText()=="POS")
	{
		//motor_cmd_.command = SET_TARGET_POSITION;
	}
	else if(ui_.cb_mode->currentText()=="VEL")
	{
		//motor_cmd_.command = SET_TARGET_VELOCITY;
	}
	else if(ui_.cb_mode->currentText()=="CUR")
	{/*
		motor_cmd_.command = SET_CURRENT_MODE_SETTING_VALUE;
		motor_cmd_.value = ui_.hs_target->value();
		Q_EMIT sendMotorCmd(motor_cmd_);
		*/
	}
}

void BasicControlGUIController::on_sb_target_valueChanged(int i)
{
	//change the slider position
	ui_.hs_target->setValue(i);

	if(ui_.cb_mode->currentText()=="POS")
	{
		//motor_cmd_.command = SET_TARGET_POSITION; //done in command builder
	}
	else if(ui_.cb_mode->currentText()=="VEL")
	{
		//motor_cmd_.command = SET_TARGET_VELOCITY; //done in command builder
	}
	else if(ui_.cb_mode->currentText()=="CUR")
	{
		/*//TODO make it work without being triggered by the mode comboBox when a value changes
		motor_cmd_.command = SET_CURRENT_MODE_SETTING_VALUE;
		motor_cmd_.value = i;

		//std::cout << "SET_CURRENT_MODE_SETTING_VALUE : " << i << endl;

		Q_EMIT sendMotorCmd(motor_cmd_);
		*/
	}

	//motor_cmd_.value = i;

	//Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_pb_send_clicked()
{
	//send a command based on the selected mode
	if(ui_.cb_mode->currentText()=="POS")
	{
		motor_cmd_.command = SET_TARGET_POSITION;
	}
	else if(ui_.cb_mode->currentText()=="VEL")
	{
		motor_cmd_.command = SET_TARGET_VELOCITY;
	}
	else if(ui_.cb_mode->currentText()=="CUR")
	{
		motor_cmd_.command = SET_CURRENT_MODE_SETTING_VALUE;
	}

	motor_cmd_.value = ui_.sb_target->value();

	//if(motor_cmd_.command == SET_CURRENT_MODE_SETTING_VALUE)
		//std::cout << "SET_CURRENT_MODE_SETTING_VALUE : " << motor_cmd_.value << endl;

	/*
	motor_cmd_.command = SET_CONTROLWORD;

	if(ui_.cb_mode->currentText()=="POS")
	{
		//TODO need a rising edge
		motor_cmd_.value = 0x002F;
		motor_cmd_.value = 0x003F;
		controlword_ = motor_cmd_.value; //update the controlword
	}
	else if(ui_.cb_mode->currentText()=="VEL")
	{
		motor_cmd_.value = 0x000F; //63
		controlword_ = motor_cmd_.value; //update the controlword
	}
	else if(ui_.cb_mode->currentText()=="CUR")
	{
		//do nothing, hide the button, just use the slider
	}
	*/

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_pb_stop_clicked()
{
	motor_cmd_.command = SET_CONTROLWORD;
	//motor_cmd_.value = controlword_ & 0b1111111111111011;
	motor_cmd_.value = 0b0000000000001011;
	controlword_ = motor_cmd_.value; //update the controlword

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_pb_halt_clicked()
{
	motor_cmd_.command = SET_CONTROLWORD;
	motor_cmd_.value = controlword_ & 0b1111111011111111;
	controlword_ = motor_cmd_.value; //update the controlword

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_pb_enableOperation_clicked()
{
	motor_cmd_.command = SET_CONTROLWORD;

	if(ui_.pb_enableOperation->isChecked())
	{
		motor_cmd_.value = controlword_ | 0b0000000000001000;
	}
	else
	{
		motor_cmd_.value = controlword_ & 0b1111111111110111;
	}

	controlword_ = motor_cmd_.value; //update the controlword

	Q_EMIT sendMotorCmd(motor_cmd_);
}

void BasicControlGUIController::on_pb_faultReset_clicked()
{
	motor_cmd_.command = SET_CONTROLWORD;
	motor_cmd_.value = controlword_ & 0b1111111101111111;
	controlword_ = motor_cmd_.value; //update the controlword

	//TODO not working // Make the change in the mbed to trigger the other steps automatically

	Q_EMIT sendMotorCmd(motor_cmd_);
}

//void BasicControlGUIController::setMotorDataLabels(int32_t position, int16_t current, uint16_t status)
void BasicControlGUIController::setMotorDataLabels(osa_msgs::MotorDataMultiArray data_ma)
{
	int index = dof_idx_; //motor_cmd_.node_id-1; //(motor_cmd_.slaveBoardID-1)*NUMBER_MAX_EPOS2_PER_SLAVE + motor_cmd_.node_id-1;

	ui_.lbl_position->setText(QString::number(data_ma.motor_data[index].position));
	ui_.lbl_current->setText(QString::number(data_ma.motor_data[index].current));

	//create a string that display the statusword in binary
	ui_.lbl_status->setText(QString::number(data_ma.motor_data[index].status, 2));
}

