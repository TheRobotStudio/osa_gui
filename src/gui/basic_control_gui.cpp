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
 * @file basic_control_gui.cpp
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version 0.1.0
 * @brief Implementation file for class BasicControlGUI.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

/*! Include */
//#include <QtGui>
//#include <QMessageBox>
//#include <iostream>
#include <include/gui/basic_control_gui.h>

/*! Namespace */
using namespace std;
using namespace osa_gui;
using namespace gui;
using namespace rosnode;
using namespace Qt;
//using namespace osa_msgs;

BasicControlGUI::BasicControlGUI(QWidget *parent) :
	QMdiSubWindow(parent),
	//basic_control_node_(),	//, m_basicControlGUI_controller(ui_.gb_sb_2, 1)
	basic_control_gui_controller_list_(QList<BasicControlGUIController*>())
{
	//TODO this->showFullScreen();
	ui_.setupUi(this); //Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	//register meta type for custom ROS messages
	qRegisterMetaType<osa_msgs::MotorDataMultiArray>("osa_msgs::MotorDataMultiArray");
	qRegisterMetaType<osa_msgs::MotorCmdMultiArray>("osa_msgs::MotorCmdMultiArray");
	qRegisterMetaType<osa_msgs::MotorCmd>("osa_msgs::MotorCmd");

	QObject::connect(&basic_control_node_, SIGNAL(rosShutdown()), this, SLOT(close()));
	//QObject::connect(&basic_control_node_, SIGNAL(motorDataReceived()), this, SLOT(setMotorDataLabels()));

	//QObject::connect(&basic_control_node_, &BasicControlNode::motorDataReceived, m_basicControlGUI_controller, &basic_control_gui_controller::setMotorDataLabels);

	//QObject::connect(m_basicControlGUI_controller, &BasicControlGUIController::sendMotorCmd, &basic_control_node_, &BasicControlNode::updateMotorCmd_ma);

	//Connect the node to ROS
	basic_control_node_.init();

	//create the motor command array, it will be used to set the motor command array of the ROS node
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = basic_control_node_.getNumberEPOSBoards(); //NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = basic_control_node_.getNumberEPOSBoards();//NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "epos";
/*
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].label = "motors";
*/
	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(basic_control_node_.getNumberEPOSBoards()); //NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

	//QStringList modeList1({"ALL", "SB1", "SB2"});
	//ui_.cb_slave_board->addItems(modeList1);

	QStringList modeList = (QStringList() << "POS" << "VEL" << "CUR" << "TOR");

	//mode, default is CUR for current
	ui_.cb_mode_0->addItems(modeList);

	//set CUR as default
	ui_.cb_mode_0->setCurrentIndex(2);

	ui_.sb_minpos_0->setVisible(false);
	ui_.sb_maxpos_0->setVisible(false);
	ui_.sb_maxcur_0->setVisible(false);

	//trigger the change
	on_cb_mode_0_currentIndexChanged(2);

	basic_control_gui_controller_list_.clear();

	//Create a line per DOF in the GUI
	for(int i=0; i<basic_control_node_.getNumberEPOSBoards(); i++)
	{
		addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof,
				i,
				QString::fromStdString(basic_control_node_.getDOFControllerList().at(i)),
				basic_control_node_.getDOFNodeIDList().at(i),
				QString::fromStdString(basic_control_node_.getDOFNameList().at(i))));

		basic_control_gui_controller_list_.at(i)->setGeometry(16, 28*i, 1670, 28);
	}
}

BasicControlGUI::~BasicControlGUI()
{
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

//whole robot or selected slave
/*
void BasicControlGUI::on_pb_sendRobotZeroPositionCmd_clicked()
{
	int selectedSB = 0;
	if(ui_.cb_slave_board->currentText()=="ALL") selectedSB = 0;
	else if(ui_.cb_slave_board->currentText()=="SB1") selectedSB = 1;
	else if(ui_.cb_slave_board->currentText()=="SB2") selectedSB = 2;

	basic_control_node_.sendRobotZeroPositionCmd(selectedSB);
}

void BasicControlGUI::on_pb_sendRobotLockPositionCmd_clicked()
{
	int selectedSB = 0;
	if(ui_.cb_slave_board->currentText()=="ALL") selectedSB = 0;
	else if(ui_.cb_slave_board->currentText()=="SB1") selectedSB = 1;
	else if(ui_.cb_slave_board->currentText()=="SB2") selectedSB = 2;

	basic_control_node_.sendRobotLockPositionCmd(selectedSB);
}

void BasicControlGUI::on_pb_sendRobotCurrentCmd_clicked()
{
	int selectedSB = 0;
	if(ui_.cb_slave_board->currentText()=="ALL") selectedSB = 0;
	else if(ui_.cb_slave_board->currentText()=="SB1") selectedSB = 1;
	else if(ui_.cb_slave_board->currentText()=="SB2") selectedSB = 2;

	int robotCurrent = ui_.hslid_robotCurrent->value();
	basic_control_node_.sendRobotCurrentCmd(selectedSB, robotCurrent);

	ui_.lbl_robotCurr->setText(QString::number(robotCurrent));
}
*/

void BasicControlGUI::resetMotorCmdMultiArray()
{
	for(int i=0; i<basic_control_node_.getNumberEPOSBoards(); i++)
	{
		motor_cmd_array_.motor_cmd[i].node_id = 0;
		motor_cmd_array_.motor_cmd[i].command = SEND_DUMB_MESSAGE;
		motor_cmd_array_.motor_cmd[i].value = 0;
	}
}

void BasicControlGUI::on_ch_enable_0_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		foreach(const BasicControlGUIController* ptr_basic_control_gui_controller, basic_control_gui_controller_list_)
		{
			ptr_basic_control_gui_controller->getUi().ch_enable->setCheckState(Qt::Unchecked);
		}
	}
	else if(state == 1) //partially checked
	{
		//do nothing
	}
	else if(state == 2) //checked
	{
		foreach(const BasicControlGUIController* ptr_basic_control_gui_controller, basic_control_gui_controller_list_)
		{
			ptr_basic_control_gui_controller->getUi().ch_enable->setCheckState(Qt::Checked);
		}
	}
}

void BasicControlGUI::on_ch_selected_0_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		foreach(const BasicControlGUIController* ptr_basic_control_gui_controller, basic_control_gui_controller_list_)
		{
			ptr_basic_control_gui_controller->getUi().ch_selected->setCheckState(Qt::Unchecked);
		}
	}
	else if(state == 1) //partially checked
	{
		//do nothing
	}
	else if(state == 2) //checked
	{
		foreach(const BasicControlGUIController* ptr_basic_control_gui_controller, basic_control_gui_controller_list_)
		{
			ptr_basic_control_gui_controller->getUi().ch_selected->setCheckState(Qt::Checked);
		}
	}
}

void BasicControlGUI::on_ch_limit_0_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		ui_.sb_minpos_0->setVisible(false);
		ui_.sb_maxpos_0->setVisible(false);
		ui_.sb_maxcur_0->setVisible(false);
	}
	else if(state == 2) //checked
	{
		ui_.sb_minpos_0->setVisible(true);
		ui_.sb_maxpos_0->setVisible(true);
		ui_.sb_maxcur_0->setVisible(true);
	}
}

void BasicControlGUI::on_sb_minpos_0_valueChanged(int i)
{
	ui_.hs_target_0->setMinimum(i);
	ui_.sb_target_0->setMinimum(i);
}

void BasicControlGUI::on_sb_maxpos_0_valueChanged(int i)
{
	ui_.hs_target_0->setMaximum(i);
	ui_.sb_target_0->setMaximum(i);
}

void BasicControlGUI::on_sb_maxcur_0_valueChanged(int i)
{
	ui_.hs_target_0->setMaximum(i);
	ui_.sb_target_0->setMaximum(i);
}

//TODO special case of this one
void BasicControlGUI::on_cb_mode_0_currentIndexChanged(int index)
{
	if(index == 0) //POS
	{
		if(ui_.ch_limit_0->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			ui_.hs_target_0->setRange(0, RANGE_MAX_POSITION);
			ui_.hs_target_0->setValue(0); //ui_.lbl_curr_0->text().toInt()

			ui_.sb_target_0->setRange(0, RANGE_MAX_POSITION);
			ui_.sb_target_0->setValue(0);
		}
		else if(ui_.ch_limit_0->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			ui_.sb_minpos_0->setVisible(true);
			ui_.sb_maxpos_0->setVisible(true);
			ui_.sb_maxcur_0->setVisible(false);

			//set min and max of the slider and spinbox
			ui_.hs_target_0->setRange(ui_.sb_minpos_0->value(), ui_.sb_maxpos_0->value());
			ui_.hs_target_0->setValue(ui_.sb_minpos_0->value());

			ui_.sb_target_0->setRange(ui_.sb_minpos_0->value(), ui_.sb_maxpos_0->value());
			ui_.sb_target_0->setValue(ui_.sb_minpos_0->value());
		}

		ui_.sb_accel_0->setVisible(true);
		ui_.sb_decel_0->setVisible(true);
		ui_.sb_velocity_0->setVisible(true);




	/*	for(int i=0; i<basic_control_gui_controller_list_.count(); i++)
		{

		}*/
	}
	else if(index == 1) //VEL
	{
		//set min and max of the slider and spinbox to absolute maximums, no limits.
		ui_.hs_target_0->setMinimum(0);
		ui_.hs_target_0->setMaximum(RANGE_MAX_VELOCITY);
		ui_.hs_target_0->setValue(0); //ui_.lbl_curr_0->text().toInt()

		ui_.sb_target_0->setRange(0, RANGE_MAX_VELOCITY);
		ui_.sb_target_0->setValue(0);

		ui_.sb_accel_0->setVisible(true);
		ui_.sb_decel_0->setVisible(true);
		ui_.sb_velocity_0->setVisible(true);
	}
	else if(index == 2) //CUR
	{
		if(ui_.ch_limit_0->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			ui_.hs_target_0->setRange(0, RANGE_MAX_CURRENT);
			ui_.hs_target_0->setValue(0);

			ui_.sb_target_0->setRange(0, RANGE_MAX_CURRENT);
			ui_.sb_target_0->setValue(0);
		}
		else if(ui_.ch_limit_0->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			ui_.sb_minpos_0->setVisible(false);
			ui_.sb_maxpos_0->setVisible(false);
			ui_.sb_maxcur_0->setVisible(true);

			//set min and max of the slider and spinbox
			ui_.hs_target_0->setRange(0, ui_.sb_maxcur_0->value());
			ui_.hs_target_0->setValue(0);

			ui_.sb_target_0->setRange(0, ui_.sb_maxcur_0->value());
			ui_.sb_target_0->setValue(0);
		}

		ui_.sb_accel_0->setVisible(false);
		ui_.sb_decel_0->setVisible(false);
		ui_.sb_velocity_0->setVisible(false);
	}

	//set to selected MODE all the selected nodes in all Slave Boards
	foreach(BasicControlGUIController* ptr_bcg_c, basic_control_gui_controller_list_)
	{
		//check if the node is selected
		if(ptr_bcg_c->getUi().ch_selected->isChecked())
		{
			if((index == 0) || (index == 1))
			{
				ptr_bcg_c->getUi().cb_mode->setCurrentIndex(index);
			}
			else if(index == 2)
			{
				if(ptr_bcg_c->getUi().cb_board->currentText().compare(QString("EPOS2")) == 0)
				{
					ptr_bcg_c->getUi().cb_mode->setCurrentIndex(index);
				}
			}
			else if(index == 3)
			{
				if(ptr_bcg_c->getUi().cb_board->currentText().compare(QString("EPOS4")) == 0)
				{
					ptr_bcg_c->getUi().cb_mode->setCurrentIndex(2);
				}
			}
		}
	}
}

//TODO WARNING a change on max cont curr also send the command, why ? check mbed code, they are on the same PDO probably

void BasicControlGUI::on_pb_send_0_clicked()
{
	for(int i=0; i<basic_control_node_.getNumberEPOSBoards(); i++)
	{
		//TODO node id, not just array nb, as they can be anything now
		motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
		motor_cmd_array_.motor_cmd[i].value = 63;
	}

	basic_control_node_.setMotorCmdArray(motor_cmd_array_);
}

void BasicControlGUI::on_pb_stop_0_clicked()
{
	for(int i=0; i<basic_control_node_.getNumberEPOSBoards(); i++)
	{
		motor_cmd_array_.motor_cmd[i].command = SET_CONTROLWORD;
		motor_cmd_array_.motor_cmd[i].value = 819;
	}

	basic_control_node_.setMotorCmdArray(motor_cmd_array_);
}

void BasicControlGUI::on_ch_enablePublish_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		basic_control_node_.setEnablePublish(false);
	}
	else if(state == 2) //checked
	{
		basic_control_node_.setEnablePublish(true);
	}
}

int BasicControlGUI::addBasicControlGUIController(BasicControlGUIController* ptr_basic_control_gui_controller)
{
	//check the value
	if(ptr_basic_control_gui_controller != 0)
	{
		basic_control_gui_controller_list_.append(ptr_basic_control_gui_controller);

		//register meta type for custom ROS messages
		//qRegisterMetaType<osa_msgs::MotorDataMultiArray>("osa_msgs::MotorDataMultiArray");
		//qRegisterMetaType<osa_msgs::MotorCmdMultiArray>("osa_msgs::MotorCmdMultiArray");
		//qRegisterMetaType<osa_msgs::MotorCmd>("osa_msgs::MotorCmd");

		//Connect the ROS BasicControlNode and the basic_control_gui_controller to display data
		QObject::connect(&basic_control_node_, &BasicControlNode::motorDataReceived, ptr_basic_control_gui_controller, &BasicControlGUIController::setMotorDataLabels);
		//Connect the ROS BasicControlNode and the basic_control_gui_controller to send motor commands
		QObject::connect(ptr_basic_control_gui_controller, &BasicControlGUIController::sendMotorCmd, &basic_control_node_, &BasicControlNode::updateMotorCmdArray);

		return 0;
	}
	else
		return -1;
}

void BasicControlGUI::closeEvent(QCloseEvent *event)
{
	QMdiSubWindow::closeEvent(event);
}

