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
 * @file BasicControlGUI_controller.cpp
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version 0.0.1
 * @brief Implementation file for class BasicControlGUI_controller.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

/*! Include */
//#include <QtGui>
//#include <QMessageBox>
#include <iostream>
#include <include/gui/BasicControlGUI_controller.hpp>

/*! Namespace */
using namespace std;
using namespace osa_gui;
using namespace gui;
using namespace Qt;

/*! \fn
 *  \brief
 *  \param
 */
BasicControlGUI_controller::BasicControlGUI_controller(QWidget *parent, uint8_t slaveBoardID, QString boardType, uint8_t nodeID, QString name) :
		QMdiSubWindow(parent, Qt::FramelessWindowHint)
{
	//TODO this->showFullScreen();
	m_ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	m_motorCmd.slaveBoardID = slaveBoardID;
	m_motorCmd.nodeID = nodeID;
	m_motorCmd.command = SEND_DUMB_MESSAGE;
	m_motorCmd.value = 0;

	//nodeID label
	m_ui.lbl_nodeId->setText(QString::number(nodeID));

	setMotorName(name);

	//set board type
	if(boardType.compare(QString("EPOS2")) == 0)
	{
		m_ui.cb_board->setCurrentIndex(0);
		QStringList modeList = (QStringList() << "POS" << "VEL" << "CUR");
		//mode, default is CUR for current
		m_ui.cb_mode->addItems(modeList);
		//set CUR as default
		m_ui.cb_mode->setCurrentIndex(2);
		//trigger the change
		on_cb_mode_currentIndexChanged(2);
	}
	else if(boardType.compare(QString("EPOS4")) == 0)
	{
		m_ui.cb_board->setCurrentIndex(1);
		QStringList modeList = (QStringList() << "POS" << "VEL" << "TOR");
		//mode, default is CUR for current
		m_ui.cb_mode->addItems(modeList);
		//set VEL as default
		m_ui.cb_mode->setCurrentIndex(1);
		//trigger the change
		on_cb_mode_currentIndexChanged(1);
	}

	m_ui.sb_minpos->setVisible(false);
	m_ui.sb_maxpos->setVisible(false);
	m_ui.sb_maxcur->setVisible(false);

	//tune position
	this->setGeometry(16, 28*nodeID, 1670, 28);

	m_controlword = 0b0000000000000000;
}

/*! \fn
 *  \brief
 *  \param
 */
BasicControlGUI_controller::~BasicControlGUI_controller()
{

}

void BasicControlGUI_controller::setMotorName(QString motorName)
{
	m_ui.lbl_name->setText(motorName);
}

void BasicControlGUI_controller::on_ch_enable_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		m_ui.ch_selected->setEnabled(false);
		m_ui.ch_limit->setEnabled(false);
		m_ui.sb_minpos->setEnabled(false);
		m_ui.sb_maxpos->setEnabled(false);
		m_ui.sb_maxcur->setEnabled(false);
		m_ui.cb_mode->setEnabled(false);
		m_ui.sb_accel->setEnabled(false);
		m_ui.sb_decel->setEnabled(false);
		m_ui.sb_velocity->setEnabled(false);
		m_ui.sb_maxoutcurr->setEnabled(false);
		m_ui.hs_target->setEnabled(false);
		m_ui.sb_target->setEnabled(false);
		m_ui.pb_send->setEnabled(false);
		m_ui.pb_stop->setEnabled(false);
	}
	else if(state == 2) //checked
	{
		m_ui.ch_selected->setEnabled(true);
		m_ui.ch_limit->setEnabled(true);
		m_ui.sb_minpos->setEnabled(true);
		m_ui.sb_maxpos->setEnabled(true);
		m_ui.sb_maxcur->setEnabled(true);
		m_ui.cb_mode->setEnabled(true);
		m_ui.sb_accel->setEnabled(true);
		m_ui.sb_decel->setEnabled(true);
		m_ui.sb_velocity->setEnabled(true);
		m_ui.sb_maxoutcurr->setEnabled(true);
		m_ui.hs_target->setEnabled(true);
		m_ui.sb_target->setEnabled(true);
		m_ui.pb_send->setEnabled(true);
		m_ui.pb_stop->setEnabled(true);
	}
}

//Individual motors
void BasicControlGUI_controller::on_cb_board_currentIndexChanged(int index)
{
	if(index == 0) //EPOS2
	{

	}
	else if(index == 1) //EPOS4
	{

	}
}

void BasicControlGUI_controller::on_ch_limit_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		m_ui.sb_minpos->setVisible(false);
		m_ui.sb_maxpos->setVisible(false);
		m_ui.sb_maxcur->setVisible(false);

		int max = 0;
		//check which mode is active
		if(m_ui.cb_mode->currentText()=="POS") max = RANGE_MAX_POSITION;
		else if(m_ui.cb_mode->currentText()=="VEL") max = RANGE_MAX_VELOCITY;
		else if(m_ui.cb_mode->currentText()=="CUR") max = RANGE_MAX_CURRENT;

		//set min and max of the slider and spinbox to absolute maximums, no limits.
		m_ui.hs_target->setRange(0, max);
		m_ui.hs_target->setValue(0); //m_ui.lbl_curr_0->text().toInt()

		m_ui.sb_target->setRange(0, max);
		m_ui.sb_target->setValue(0);
	}
	else if(state == 2) //checked
	{
		//check which mode is active
		if(m_ui.cb_mode->currentText()=="POS")
		{
			m_ui.sb_minpos->setVisible(true);
			m_ui.sb_maxpos->setVisible(true);
			m_ui.sb_maxcur->setVisible(false);
		}
		else if(m_ui.cb_mode->currentText()=="VEL")
		{
			m_ui.sb_minpos->setVisible(false);
			m_ui.sb_maxpos->setVisible(false);
			m_ui.sb_maxcur->setVisible(false);
		}
		else if(m_ui.cb_mode->currentText()=="CUR")
		{
			m_ui.sb_minpos->setVisible(false);
			m_ui.sb_maxpos->setVisible(false);
			m_ui.sb_maxcur->setVisible(true);
		}
	}

	on_cb_mode_currentIndexChanged(m_ui.cb_mode->currentIndex());
}

void BasicControlGUI_controller::on_sb_minpos_valueChanged(int i)
{
	m_ui.hs_target->setMinimum(i);
	m_ui.sb_target->setMinimum(i);
}

void BasicControlGUI_controller::on_sb_maxpos_valueChanged(int i)
{
	m_ui.hs_target->setMaximum(i);
	m_ui.sb_target->setMaximum(i);
}

void BasicControlGUI_controller::on_sb_maxcur_valueChanged(int i)
{
	m_ui.hs_target->setMaximum(i);
	m_ui.sb_target->setMaximum(i);
}

//Individual motors
void BasicControlGUI_controller::on_cb_mode_currentIndexChanged(int index)
{
	//std::cout << "Mode ";
	m_motorCmd.command = SET_MODES_OF_OPERATION;

	if(index == 0) //POS
	{
		m_motorCmd.value = PROFILE_POSITION_MODE;

		if(m_ui.ch_limit->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			m_ui.hs_target->setRange(0, RANGE_MAX_POSITION);
			m_ui.hs_target->setValue(0); //m_ui.lbl_curr_0->text().toInt()

			m_ui.sb_target->setRange(0, RANGE_MAX_POSITION);
			m_ui.sb_target->setValue(0);
		}
		else if(m_ui.ch_limit->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			m_ui.sb_minpos->setVisible(true);
			m_ui.sb_maxpos->setVisible(true);
			m_ui.sb_maxcur->setVisible(false);

			//set min and max of the slider and spinbox
			m_ui.hs_target->setRange(m_ui.sb_minpos->value(), m_ui.sb_maxpos->value());
			m_ui.hs_target->setValue(m_ui.sb_minpos->value());

			m_ui.sb_target->setRange(m_ui.sb_minpos->value(), m_ui.sb_maxpos->value());
			m_ui.sb_target->setValue(m_ui.sb_minpos->value());
		}

		m_ui.sb_accel->setVisible(true);
		m_ui.sb_decel->setVisible(true);
		m_ui.sb_velocity->setVisible(true);

		//std::cout << "position" << endl;
	}
	else if(index == 1) //VEL
	{
		m_motorCmd.value = PROFILE_VELOCITY_MODE;

		//set visibility of the limit QSpinBox
		m_ui.sb_minpos->setVisible(false);
		m_ui.sb_maxpos->setVisible(false);
		m_ui.sb_maxcur->setVisible(false);

		//set min and max of the slider and spinbox to absolute maximums, no limits.
		m_ui.hs_target->setRange(-RANGE_MAX_VELOCITY, RANGE_MAX_VELOCITY);
		m_ui.hs_target->setValue(0); //m_ui.lbl_curr_0->text().toInt()

		m_ui.sb_target->setRange(-RANGE_MAX_VELOCITY, RANGE_MAX_VELOCITY);
		m_ui.sb_target->setValue(0);

		m_ui.sb_accel->setVisible(true);
		m_ui.sb_decel->setVisible(true);
		m_ui.sb_velocity->setVisible(true);

		//std::cout << "velocity" << endl;
	}
	else if(index == 2) //CUR
	{
		m_motorCmd.value = CURRENT_MODE;

		if(m_ui.ch_limit->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			m_ui.hs_target->setRange(0, RANGE_MAX_CURRENT);
			m_ui.hs_target->setValue(0);

			m_ui.sb_target->setRange(0, RANGE_MAX_CURRENT);
			m_ui.sb_target->setValue(0);
		}
		else if(m_ui.ch_limit->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			m_ui.sb_minpos->setVisible(false);
			m_ui.sb_maxpos->setVisible(false);
			m_ui.sb_maxcur->setVisible(true);

			//set min and max of the slider and spinbox
			m_ui.hs_target->setRange(0, m_ui.sb_maxcur->value());
			m_ui.hs_target->setValue(0);

			m_ui.sb_target->setRange(0, m_ui.sb_maxcur->value());
			m_ui.sb_target->setValue(0);
		}

		m_ui.sb_accel->setVisible(false);
		m_ui.sb_decel->setVisible(false);
		m_ui.sb_velocity->setVisible(true);

		//std::cout << "current" << endl;
	}

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_sb_accel_valueChanged(int i)
{
	m_motorCmd.command = SET_PROFILE_ACCELERATION;
	m_motorCmd.value = i;

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_sb_decel_valueChanged(int i)
{
	m_motorCmd.command = SET_PROFILE_DECELERATION;
	m_motorCmd.value = i;

	Q_EMIT sendMotorCmd(m_motorCmd);
}

//TODO WARNING a change on prof vel also send the command, why ? check mbed code, they are on the same PDO probably
void BasicControlGUI_controller::on_sb_velocity_valueChanged(int i)
{
	m_motorCmd.command = SET_PROFILE_VELOCITY;
	m_motorCmd.value = i;

	Q_EMIT sendMotorCmd(m_motorCmd);
}

//TODO WARNING a change on max cont curr also send the command, why ? check mbed code, they are on the same PDO probably
void BasicControlGUI_controller::on_sb_maxoutcurr_valueChanged(int i)
{
	m_motorCmd.command = SET_OUTPUT_CURRENT_LIMIT;
	m_motorCmd.value = i;

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_hs_target_valueChanged()
{
	//change the target value accordingly
	m_ui.sb_target->setValue(m_ui.hs_target->value());

	if(m_ui.cb_mode->currentText()=="POS")
	{
		//m_motorCmd.command = SET_TARGET_POSITION;
	}
	else if(m_ui.cb_mode->currentText()=="VEL")
	{
		//m_motorCmd.command = SET_TARGET_VELOCITY;
	}
	else if(m_ui.cb_mode->currentText()=="CUR")
	{/*
		m_motorCmd.command = SET_CURRENT_MODE_SETTING_VALUE;
		m_motorCmd.value = m_ui.hs_target->value();
		Q_EMIT sendMotorCmd(m_motorCmd);
		*/
	}
}

void BasicControlGUI_controller::on_sb_target_valueChanged(int i)
{
	//change the slider position
	m_ui.hs_target->setValue(i);

	if(m_ui.cb_mode->currentText()=="POS")
	{
		//m_motorCmd.command = SET_TARGET_POSITION; //done in command builder
	}
	else if(m_ui.cb_mode->currentText()=="VEL")
	{
		//m_motorCmd.command = SET_TARGET_VELOCITY; //done in command builder
	}
	else if(m_ui.cb_mode->currentText()=="CUR")
	{
		/*//TODO make it work without being triggered by the mode comboBox when a value changes
		m_motorCmd.command = SET_CURRENT_MODE_SETTING_VALUE;
		m_motorCmd.value = i;

		//std::cout << "SET_CURRENT_MODE_SETTING_VALUE : " << i << endl;

		Q_EMIT sendMotorCmd(m_motorCmd);
		*/
	}

	//m_motorCmd.value = i;

	//Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_pb_send_clicked()
{
	//send a command based on the selected mode
	if(m_ui.cb_mode->currentText()=="POS")
	{
		m_motorCmd.command = SET_TARGET_POSITION;
	}
	else if(m_ui.cb_mode->currentText()=="VEL")
	{
		m_motorCmd.command = SET_TARGET_VELOCITY;
	}
	else if(m_ui.cb_mode->currentText()=="CUR")
	{
		m_motorCmd.command = SET_CURRENT_MODE_SETTING_VALUE;
	}

	m_motorCmd.value = m_ui.sb_target->value();

	//if(m_motorCmd.command == SET_CURRENT_MODE_SETTING_VALUE)
		//std::cout << "SET_CURRENT_MODE_SETTING_VALUE : " << m_motorCmd.value << endl;

	/*
	m_motorCmd.command = SET_CONTROLWORD;

	if(m_ui.cb_mode->currentText()=="POS")
	{
		//TODO need a rising edge
		m_motorCmd.value = 0x002F;
		m_motorCmd.value = 0x003F;
		m_controlword = m_motorCmd.value; //update the controlword
	}
	else if(m_ui.cb_mode->currentText()=="VEL")
	{
		m_motorCmd.value = 0x000F; //63
		m_controlword = m_motorCmd.value; //update the controlword
	}
	else if(m_ui.cb_mode->currentText()=="CUR")
	{
		//do nothing, hide the button, just use the slider
	}
	*/

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_pb_stop_clicked()
{
	m_motorCmd.command = SET_CONTROLWORD;
	//m_motorCmd.value = m_controlword & 0b1111111111111011;
	m_motorCmd.value = 0b0000000000001011;
	m_controlword = m_motorCmd.value; //update the controlword

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_pb_halt_clicked()
{
	m_motorCmd.command = SET_CONTROLWORD;
	m_motorCmd.value = m_controlword & 0b1111111011111111;
	m_controlword = m_motorCmd.value; //update the controlword

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_pb_enableOperation_clicked()
{
	m_motorCmd.command = SET_CONTROLWORD;

	if(m_ui.pb_enableOperation->isChecked())
	{
		m_motorCmd.value = m_controlword | 0b0000000000001000;
	}
	else
	{
		m_motorCmd.value = m_controlword & 0b1111111111110111;
	}

	m_controlword = m_motorCmd.value; //update the controlword

	Q_EMIT sendMotorCmd(m_motorCmd);
}

void BasicControlGUI_controller::on_pb_faultReset_clicked()
{
	m_motorCmd.command = SET_CONTROLWORD;
	m_motorCmd.value = m_controlword & 0b1111111101111111;
	m_controlword = m_motorCmd.value; //update the controlword

	//TODO not working // Make the change in the mbed to trigger the other steps automatically

	Q_EMIT sendMotorCmd(m_motorCmd);
}

//void BasicControlGUI_controller::setMotorDataLabels(int32_t position, int16_t current, uint16_t status)
void BasicControlGUI_controller::setMotorDataLabels(osa_msgs::MotorDataMultiArray data_ma)
{
	int index = (m_motorCmd.slaveBoardID-1)*NUMBER_MAX_EPOS2_PER_SLAVE + m_motorCmd.nodeID-1;

	m_ui.lbl_position->setText(QString::number(data_ma.motorData[index].position));
	m_ui.lbl_current->setText(QString::number(data_ma.motorData[index].current));

	//create a string that display the statusword in binary
	m_ui.lbl_status->setText(QString::number(data_ma.motorData[index].status, 2));
}

