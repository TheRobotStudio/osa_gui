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
 * @version 0.0.1
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

	//create the motor command array, it will be used to set the motor command array of the ROS node
	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	motor_cmd_array_.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[0].label = "slaves";

	motor_cmd_array_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motor_cmd_array_.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motor_cmd_array_.layout.dim[1].label = "motors";

	motor_cmd_array_.layout.data_offset = 0;

	motor_cmd_array_.motor_cmd.clear();
	motor_cmd_array_.motor_cmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

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

	//TODO do the config with the config GUI
	//HIGH SPEED ANDROID
/*
	//Head
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 1, QString("Neck Right Down")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 2, QString("Neck Left Down")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 3, QString("Neck Right Back")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 4, QString("Neck Left Back")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 5, QString("Eyeball Pitch")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 6, QString("Eyeball Yaw")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 7, QString("Iris")));
*/
/*	//Leg
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 1, QString("Waist")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 2, QString("Pelvis")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 3, QString("Hip Right")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 4, QString("Hip Left")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 5, QString("Knee Right")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 6, QString("Knee Left")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 7, QString("Drive Right")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, 8, QString("Drive Left")));
*/
/*
	//Right Shoulder, Arm, Forearm and Hand
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 1, QString("R Wrist Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 2, QString("R Brachii")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 3, QString("R Triceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 4, QString("R Biceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 5, QString("R Deltoid Anterior")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 6, QString("R Deltoid Posterior")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 7, QString("R Supraspinatus")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 8, QString("R External Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 9, QString("R Internal Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 10, QString("R Shoulder Module Pitch")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 11, QString("R Fore Knuckle Flex 1")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 12, QString("R Fore Knuckle Flex 2")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 13, QString("R Thumb Knuckle In")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_3, 3, QString("EPOS2"), 14, QString("R Thumb Knuckle Down")));

	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 1, QString("R Little Whole Knuckle Out")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 2, QString("R Little Whole Knuckle In")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 3, QString("R Little Whole Tip Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 4, QString("R Little Whole 2nd Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 5, QString("R Little Whole Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 6, QString("R Fore Tip Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 7, QString("R Fore 2nd Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 8, QString("R Fore Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 9, QString("R Thumb Basal Out")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 10, QString("R Thumb Tip Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 11, QString("R Thumb 2nd Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 12, QString("R Thumb Tip Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 13, QString("R Thumb 2nd Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 14, QString("R Wrist In")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 15, QString("R Wrist Up/Back")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_4, 4, QString("EPOS2"), 16, QString("R Wrist Down/Back")));

	//Left Shoulder, Arm, Forearm and Hand
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 1, QString("L Wrist Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 2, QString("L Brachii")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 3, QString("L Triceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 4, QString("L Biceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 5, QString("L Deltoid Anterior")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 6, QString("L Deltoid Posterior")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 7, QString("L Supraspinatus")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 8, QString("L External Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 9, QString("L Internal Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 10, QString("L Shoulder Module Pitch")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 11, QString("L Fore Knuckle Flex 1")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 12, QString("L Fore Knuckle Flex 2")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 13, QString("L Thumb Knuckle In")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_5, 5, QString("EPOS2"), 14, QString("L Thumb Knuckle Down")));

	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 1, QString("L Little Whole Knuckle Out")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 2, QString("L Little Whole Knuckle In")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 3, QString("L Little Whole Tip Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 4, QString("L Little Whole 2nd Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 5, QString("L Little Whole Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 6, QString("L Fore Tip Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 7, QString("L Fore 2nd Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 8, QString("L Fore Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 9, QString("L Thumb Basal Out")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 10, QString("L Thumb Tip Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 11, QString("L Thumb 2nd Flex")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 12, QString("L Thumb Tip Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 13, QString("L Thumb 2nd Extend")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 14, QString("L Wrist In")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 15, QString("L Wrist Up/Back")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_6, 6, QString("EPOS2"), 16, QString("L Wrist Down/Back")));
*/

	//BIBOT
/*	//Right arm
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 1, QString("R Biceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 2, QString("R Supra")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 3, QString("R Subscap")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 4, QString("R Infra")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 5, QString("R Tmin")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 6, QString("R LatDelt")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 7, QString("R AntDelt")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 8, QString("R PostDelt")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 9, QString("R Triceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 10, QString("R Brachi")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 11, QString("R Hand")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 12, QString("R Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 13, QString("R WristIn")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 14, QString("R WristUp")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 15, QString("R WristDown")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS2"), 16, QString("R Thumb")));
	//Base
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, QString("EPOS4"), 1, QString("Right wheel")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.tab_sb_2, 2, QString("EPOS4"), 2, QString("Left wheel")));
*/

	//std::string test = basic_control_node_.getDOFMotorList().at(0);

	for(int i=0; i<basic_control_node_.getNumberEPOSBoards(); i++)
	{
		addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof,
				QString::fromStdString(basic_control_node_.getDOFControllerList().at(i)),
				basic_control_node_.getDOFNodeIDList().at(i),
				QString::fromStdString(basic_control_node_.getDOFNameList().at(i))));
	}

/*
	//Right arm
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 1, QString("R Biceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 2, QString("R Supra")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 3, QString("R Subscap")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 4, QString("R Infra")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 5, QString("R Tmin")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 6, QString("R LatDelt")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 7, QString("R AntDelt")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 8, QString("R PostDelt")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 9, QString("R Triceps")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 10, QString("R Brachi")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 11, QString("R Hand")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 12, QString("R Rotator")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 13, QString("R WristIn")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 14, QString("R WristUp")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 15, QString("R WristDown")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS2"), 16, QString("R Thumb")));
	//Base
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS4"), 17, QString("Right wheel")));
	addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, QString("EPOS4"), 18, QString("Left wheel")));
*/

	//Mobile robot base with 2 wheels
	//addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS4"), 1, QString("Right wheel")));
	//addBasicControlGUIController(new BasicControlGUIController(ui_.sa_dof, 1, QString("EPOS4"), 2, QString("Left wheel")));
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
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			//motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].node_id = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SEND_DUMB_MESSAGE;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0;
		}
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
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 63;
		}
	}

	basic_control_node_.setMotorCmdArray(motor_cmd_array_);
}

void BasicControlGUI::on_pb_stop_0_clicked()
{
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			motor_cmd_array_.motor_cmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 819;
		}
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
		QObject::connect(ptr_basic_control_gui_controller, &BasicControlGUIController::sendMotorCmd, &basic_control_node_, &BasicControlNode::updateMotorCmd_ma);

		return 0;
	}
	else
		return -1;
}

void BasicControlGUI::closeEvent(QCloseEvent *event)
{
	QMdiSubWindow::closeEvent(event);
}

