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
 * @file BasicControlGUI.cpp
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version 0.0.1
 * @brief Implementation file for class BasicControlGUI
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

/*! Include */
//#include <QtGui>
//#include <QMessageBox>
//#include <iostream>
#include <include/gui/BasicControlGUI.hpp>

/*! Namespace */
using namespace std;
using namespace osa_gui;
using namespace gui;
using namespace rosnode;
using namespace Qt;
//using namespace osa_msgs;

/*! \fn
 *  \brief
 *  \param
 */
BasicControlGUI::BasicControlGUI(QWidget *parent) :
		QMdiSubWindow(parent),
		//m_basicControlNode(),	//, m_basicControlGUI_controller(m_ui.gb_sb_2, 1)
		m_lpBasicControlGUI_controller(QList<BasicControlGUI_controller*>())
{
	//TODO this->showFullScreen();
	m_ui.setupUi(this); //Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

	//create the motor command array, it will be used to set the motor command array of the ROS node
	m_motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m_motorCmd_ma.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	m_motorCmd_ma.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorCmd_ma.layout.dim[0].label = "slaves";

	m_motorCmd_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	m_motorCmd_ma.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorCmd_ma.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	m_motorCmd_ma.layout.dim[1].label = "motors";

	m_motorCmd_ma.layout.data_offset = 0;

	m_motorCmd_ma.motorCmd.clear();
	m_motorCmd_ma.motorCmd.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	resetMotorCmdMultiArray();

	//register meta type for custom ROS messages
	qRegisterMetaType<osa_msgs::MotorDataMultiArray>("osa_msgs::MotorDataMultiArray");
	qRegisterMetaType<osa_msgs::MotorCmdMultiArray>("osa_msgs::MotorCmdMultiArray");
	qRegisterMetaType<osa_msgs::MotorCmd>("osa_msgs::MotorCmd");

	QObject::connect(&m_basicControlNode, SIGNAL(rosShutdown()), this, SLOT(close()));
	//QObject::connect(&m_basicControlNode, SIGNAL(motorDataReceived()), this, SLOT(setMotorDataLabels()));

	//QObject::connect(&m_basicControlNode, &BasicControlNode::motorDataReceived, m_basicControlGUI_controller, &BasicControlGUI_controller::setMotorDataLabels);

	//QObject::connect(m_basicControlGUI_controller, &BasicControlGUI_controller::sendMotorCmd, &m_basicControlNode, &BasicControlNode::updateMotorCmd_ma);

	//Connect the node to ROS
	m_basicControlNode.init();

	//QStringList modeList1({"ALL", "SB1", "SB2"});
	//m_ui.cb_slave_board->addItems(modeList1);

	QStringList modeList = (QStringList() << "POS" << "VEL" << "CUR" << "TOR");

	//mode, default is CUR for current
	m_ui.cb_mode_0->addItems(modeList);

	//set CUR as default
	m_ui.cb_mode_0->setCurrentIndex(2);

	m_ui.sb_minpos_0->setVisible(false);
	m_ui.sb_maxpos_0->setVisible(false);
	m_ui.sb_maxcur_0->setVisible(false);

	//trigger the change
	on_cb_mode_0_currentIndexChanged(2);

	m_lpBasicControlGUI_controller.clear();

	//TODO do the config with the config GUI
	//HIGH SPEED ANDROID
/*
	//Head
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 1, QString("Neck Right Down")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 2, QString("Neck Left Down")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 3, QString("Neck Right Back")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 4, QString("Neck Left Back")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 5, QString("Eyeball Pitch")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 6, QString("Eyeball Yaw")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 7, QString("Iris")));
*/
/*	//Leg
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 1, QString("Waist")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 2, QString("Pelvis")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 3, QString("Hip Right")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 4, QString("Hip Left")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 5, QString("Knee Right")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 6, QString("Knee Left")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 7, QString("Drive Right")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, 8, QString("Drive Left")));
*/
/*
	//Right Shoulder, Arm, Forearm and Hand
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 1, QString("R Wrist Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 2, QString("R Brachii")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 3, QString("R Triceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 4, QString("R Biceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 5, QString("R Deltoid Anterior")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 6, QString("R Deltoid Posterior")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 7, QString("R Supraspinatus")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 8, QString("R External Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 9, QString("R Internal Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 10, QString("R Shoulder Module Pitch")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 11, QString("R Fore Knuckle Flex 1")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 12, QString("R Fore Knuckle Flex 2")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 13, QString("R Thumb Knuckle In")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_3, 3, QString("EPOS2"), 14, QString("R Thumb Knuckle Down")));

	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 1, QString("R Little Whole Knuckle Out")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 2, QString("R Little Whole Knuckle In")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 3, QString("R Little Whole Tip Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 4, QString("R Little Whole 2nd Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 5, QString("R Little Whole Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 6, QString("R Fore Tip Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 7, QString("R Fore 2nd Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 8, QString("R Fore Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 9, QString("R Thumb Basal Out")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 10, QString("R Thumb Tip Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 11, QString("R Thumb 2nd Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 12, QString("R Thumb Tip Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 13, QString("R Thumb 2nd Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 14, QString("R Wrist In")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 15, QString("R Wrist Up/Back")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_4, 4, QString("EPOS2"), 16, QString("R Wrist Down/Back")));

	//Left Shoulder, Arm, Forearm and Hand
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 1, QString("L Wrist Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 2, QString("L Brachii")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 3, QString("L Triceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 4, QString("L Biceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 5, QString("L Deltoid Anterior")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 6, QString("L Deltoid Posterior")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 7, QString("L Supraspinatus")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 8, QString("L External Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 9, QString("L Internal Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 10, QString("L Shoulder Module Pitch")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 11, QString("L Fore Knuckle Flex 1")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 12, QString("L Fore Knuckle Flex 2")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 13, QString("L Thumb Knuckle In")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_5, 5, QString("EPOS2"), 14, QString("L Thumb Knuckle Down")));

	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 1, QString("L Little Whole Knuckle Out")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 2, QString("L Little Whole Knuckle In")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 3, QString("L Little Whole Tip Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 4, QString("L Little Whole 2nd Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 5, QString("L Little Whole Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 6, QString("L Fore Tip Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 7, QString("L Fore 2nd Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 8, QString("L Fore Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 9, QString("L Thumb Basal Out")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 10, QString("L Thumb Tip Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 11, QString("L Thumb 2nd Flex")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 12, QString("L Thumb Tip Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 13, QString("L Thumb 2nd Extend")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 14, QString("L Wrist In")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 15, QString("L Wrist Up/Back")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_6, 6, QString("EPOS2"), 16, QString("L Wrist Down/Back")));
*/

	//BIBOT
/*	//Right arm
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 1, QString("R Biceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 2, QString("R Supra")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 3, QString("R Subscap")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 4, QString("R Infra")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 5, QString("R Tmin")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 6, QString("R LatDelt")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 7, QString("R AntDelt")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 8, QString("R PostDelt")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 9, QString("R Triceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 10, QString("R Brachi")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 11, QString("R Hand")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 12, QString("R Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 13, QString("R WristIn")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 14, QString("R WristUp")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 15, QString("R WristDown")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 16, QString("R Thumb")));
	//Base
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, QString("EPOS4"), 1, QString("Right wheel")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_2, 2, QString("EPOS4"), 2, QString("Left wheel")));
*/

	//Right arm
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 1, QString("R Biceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 2, QString("R Supra")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 3, QString("R Subscap")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 4, QString("R Infra")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 5, QString("R Tmin")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 6, QString("R LatDelt")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 7, QString("R AntDelt")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 8, QString("R PostDelt")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 9, QString("R Triceps")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 10, QString("R Brachi")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 11, QString("R Hand")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 12, QString("R Rotator")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 13, QString("R WristIn")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 14, QString("R WristUp")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 15, QString("R WristDown")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS2"), 16, QString("R Thumb")));
	//Base
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS4"), 17, QString("Right wheel")));
	addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS4"), 18, QString("Left wheel")));


	//Mobile robot base with 2 wheels
	//addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS4"), 1, QString("Right wheel")));
	//addPBasicControlGUI_controller(new BasicControlGUI_controller(m_ui.tab_sb_1, 1, QString("EPOS4"), 2, QString("Left wheel")));
}

/*! \fn
 *  \brief
 *  \param
 */
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
	if(m_ui.cb_slave_board->currentText()=="ALL") selectedSB = 0;
	else if(m_ui.cb_slave_board->currentText()=="SB1") selectedSB = 1;
	else if(m_ui.cb_slave_board->currentText()=="SB2") selectedSB = 2;

	m_basicControlNode.sendRobotZeroPositionCmd(selectedSB);
}

void BasicControlGUI::on_pb_sendRobotLockPositionCmd_clicked()
{
	int selectedSB = 0;
	if(m_ui.cb_slave_board->currentText()=="ALL") selectedSB = 0;
	else if(m_ui.cb_slave_board->currentText()=="SB1") selectedSB = 1;
	else if(m_ui.cb_slave_board->currentText()=="SB2") selectedSB = 2;

	m_basicControlNode.sendRobotLockPositionCmd(selectedSB);
}

void BasicControlGUI::on_pb_sendRobotCurrentCmd_clicked()
{
	int selectedSB = 0;
	if(m_ui.cb_slave_board->currentText()=="ALL") selectedSB = 0;
	else if(m_ui.cb_slave_board->currentText()=="SB1") selectedSB = 1;
	else if(m_ui.cb_slave_board->currentText()=="SB2") selectedSB = 2;

	int robotCurrent = m_ui.hslid_robotCurrent->value();
	m_basicControlNode.sendRobotCurrentCmd(selectedSB, robotCurrent);

	m_ui.lbl_robotCurr->setText(QString::number(robotCurrent));
}
*/

void BasicControlGUI::resetMotorCmdMultiArray()
{
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].slaveBoardID = i + 1;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID = j + 1; //i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SEND_DUMB_MESSAGE;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 0;
		}
	}
}

void BasicControlGUI::on_ch_enable_0_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		foreach(const BasicControlGUI_controller* pBasicControlGUI_controller, m_lpBasicControlGUI_controller)
		{
			pBasicControlGUI_controller->getUi().ch_enable->setCheckState(Qt::Unchecked);
		}
	}
	else if(state == 1) //partially checked
	{
		//do nothing
	}
	else if(state == 2) //checked
	{
		foreach(const BasicControlGUI_controller* pBasicControlGUI_controller, m_lpBasicControlGUI_controller)
		{
			pBasicControlGUI_controller->getUi().ch_enable->setCheckState(Qt::Checked);
		}
	}
}

void BasicControlGUI::on_ch_selected_0_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		foreach(const BasicControlGUI_controller* pBasicControlGUI_controller, m_lpBasicControlGUI_controller)
		{
			pBasicControlGUI_controller->getUi().ch_selected->setCheckState(Qt::Unchecked);
		}
	}
	else if(state == 1) //partially checked
	{
		//do nothing
	}
	else if(state == 2) //checked
	{
		foreach(const BasicControlGUI_controller* pBasicControlGUI_controller, m_lpBasicControlGUI_controller)
		{
			pBasicControlGUI_controller->getUi().ch_selected->setCheckState(Qt::Checked);
		}
	}
}

void BasicControlGUI::on_ch_limit_0_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		m_ui.sb_minpos_0->setVisible(false);
		m_ui.sb_maxpos_0->setVisible(false);
		m_ui.sb_maxcur_0->setVisible(false);
	}
	else if(state == 2) //checked
	{
		m_ui.sb_minpos_0->setVisible(true);
		m_ui.sb_maxpos_0->setVisible(true);
		m_ui.sb_maxcur_0->setVisible(true);
	}
}

void BasicControlGUI::on_sb_minpos_0_valueChanged(int i)
{
	m_ui.hs_target_0->setMinimum(i);
	m_ui.sb_target_0->setMinimum(i);
}

void BasicControlGUI::on_sb_maxpos_0_valueChanged(int i)
{
	m_ui.hs_target_0->setMaximum(i);
	m_ui.sb_target_0->setMaximum(i);
}

void BasicControlGUI::on_sb_maxcur_0_valueChanged(int i)
{
	m_ui.hs_target_0->setMaximum(i);
	m_ui.sb_target_0->setMaximum(i);
}

//TODO special case of this one
void BasicControlGUI::on_cb_mode_0_currentIndexChanged(int index)
{
	if(index == 0) //POS
	{
		if(m_ui.ch_limit_0->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			m_ui.hs_target_0->setRange(0, RANGE_MAX_POSITION);
			m_ui.hs_target_0->setValue(0); //m_ui.lbl_curr_0->text().toInt()

			m_ui.sb_target_0->setRange(0, RANGE_MAX_POSITION);
			m_ui.sb_target_0->setValue(0);
		}
		else if(m_ui.ch_limit_0->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			m_ui.sb_minpos_0->setVisible(true);
			m_ui.sb_maxpos_0->setVisible(true);
			m_ui.sb_maxcur_0->setVisible(false);

			//set min and max of the slider and spinbox
			m_ui.hs_target_0->setRange(m_ui.sb_minpos_0->value(), m_ui.sb_maxpos_0->value());
			m_ui.hs_target_0->setValue(m_ui.sb_minpos_0->value());

			m_ui.sb_target_0->setRange(m_ui.sb_minpos_0->value(), m_ui.sb_maxpos_0->value());
			m_ui.sb_target_0->setValue(m_ui.sb_minpos_0->value());
		}

		m_ui.sb_accel_0->setVisible(true);
		m_ui.sb_decel_0->setVisible(true);
		m_ui.sb_velocity_0->setVisible(true);




	/*	for(int i=0; i<m_lpBasicControlGUI_controller.count(); i++)
		{

		}*/
	}
	else if(index == 1) //VEL
	{
		//set min and max of the slider and spinbox to absolute maximums, no limits.
		m_ui.hs_target_0->setMinimum(0);
		m_ui.hs_target_0->setMaximum(RANGE_MAX_VELOCITY);
		m_ui.hs_target_0->setValue(0); //m_ui.lbl_curr_0->text().toInt()

		m_ui.sb_target_0->setRange(0, RANGE_MAX_VELOCITY);
		m_ui.sb_target_0->setValue(0);

		m_ui.sb_accel_0->setVisible(true);
		m_ui.sb_decel_0->setVisible(true);
		m_ui.sb_velocity_0->setVisible(true);
	}
	else if(index == 2) //CUR
	{
		if(m_ui.ch_limit_0->checkState() == 0) //unchecked
		{
			//set min and max of the slider and spinbox to absolute maximums, no limits.
			m_ui.hs_target_0->setRange(0, RANGE_MAX_CURRENT);
			m_ui.hs_target_0->setValue(0);

			m_ui.sb_target_0->setRange(0, RANGE_MAX_CURRENT);
			m_ui.sb_target_0->setValue(0);
		}
		else if(m_ui.ch_limit_0->checkState() == 2) //checked
		{
			//set visibility of the limit QSpinBox
			m_ui.sb_minpos_0->setVisible(false);
			m_ui.sb_maxpos_0->setVisible(false);
			m_ui.sb_maxcur_0->setVisible(true);

			//set min and max of the slider and spinbox
			m_ui.hs_target_0->setRange(0, m_ui.sb_maxcur_0->value());
			m_ui.hs_target_0->setValue(0);

			m_ui.sb_target_0->setRange(0, m_ui.sb_maxcur_0->value());
			m_ui.sb_target_0->setValue(0);
		}

		m_ui.sb_accel_0->setVisible(false);
		m_ui.sb_decel_0->setVisible(false);
		m_ui.sb_velocity_0->setVisible(false);
	}

	//set to selected MODE all the selected nodes in all Slave Boards
	foreach(BasicControlGUI_controller* pBCG_c, m_lpBasicControlGUI_controller)
	{
		//check if the node is selected
		if(pBCG_c->getUi().ch_selected->isChecked())
		{
			if((index == 0) || (index == 1))
			{
				pBCG_c->getUi().cb_mode->setCurrentIndex(index);
			}
			else if(index == 2)
			{
				if(pBCG_c->getUi().cb_board->currentText().compare(QString("EPOS2")) == 0)
				{
					pBCG_c->getUi().cb_mode->setCurrentIndex(index);
				}
			}
			else if(index == 3)
			{
				if(pBCG_c->getUi().cb_board->currentText().compare(QString("EPOS4")) == 0)
				{
					pBCG_c->getUi().cb_mode->setCurrentIndex(2);
				}
			}
		}
	}
}

/*! sb_maxcontcurr */
//TODO WARNING a change on max cont curr also send the command, why ? check mbed code, they are on the same PDO probably

/*! pb_send */
void BasicControlGUI::on_pb_send_0_clicked()
{
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 63;
		}
	}

	m_basicControlNode.setMotorCmd_ma(m_motorCmd_ma);
}

/*! pb_stop */
void BasicControlGUI::on_pb_stop_0_clicked()
{
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].command = SET_CONTROLWORD;
			m_motorCmd_ma.motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value = 819;
		}
	}

	m_basicControlNode.setMotorCmd_ma(m_motorCmd_ma);
}

void BasicControlGUI::on_ch_enablePublish_stateChanged(int state)
{
	if(state == 0) //unchecked
	{
		m_basicControlNode.setEnablePublish(false);
	}
	else if(state == 2) //checked
	{
		m_basicControlNode.setEnablePublish(true);
	}
}

int BasicControlGUI::addPBasicControlGUI_controller(BasicControlGUI_controller* pBasicControlGUI_controller)
{
	//check the value
	if(pBasicControlGUI_controller != 0)
	{
		m_lpBasicControlGUI_controller.append(pBasicControlGUI_controller);

		//register meta type for custom ROS messages
		//qRegisterMetaType<osa_msgs::MotorDataMultiArray>("osa_msgs::MotorDataMultiArray");
		//qRegisterMetaType<osa_msgs::MotorCmdMultiArray>("osa_msgs::MotorCmdMultiArray");
		//qRegisterMetaType<osa_msgs::MotorCmd>("osa_msgs::MotorCmd");

		//Connect the ROS BasicControlNode and the BasicControlGUI_controller to display data
		QObject::connect(&m_basicControlNode, &BasicControlNode::motorDataReceived, pBasicControlGUI_controller, &BasicControlGUI_controller::setMotorDataLabels);
		//Connect the ROS BasicControlNode and the BasicControlGUI_controller to send motor commands
		QObject::connect(pBasicControlGUI_controller, &BasicControlGUI_controller::sendMotorCmd, &m_basicControlNode, &BasicControlNode::updateMotorCmd_ma);

		return 0;
	}
	else
		return -1;
}

void BasicControlGUI::closeEvent(QCloseEvent *event)
{
	QMdiSubWindow::closeEvent(event);
}

