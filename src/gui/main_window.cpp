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
 * @file main_window.cpp
 * @author Cyril Jourdan
 * @date Mar 14, 2017
 * @version 0.1.0
 * @brief Implementation file for class MainWindow.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

/*! Include */
//#include <include/gui/BasicControlGUI.hpp>
//#include <include/gui/ConfigurationGUI.hpp>
//#include <include/gui/SupervisorGUI.hpp>
#include <main_window.h>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
//#include "Motor.hpp"

/*! Define */
/*#define NUMBER_MOTORS_ARM	16
#define NUMBER_MOTORS_DCX22	12
#define NUMBER_MOTORS_RE13	3
#define NUMBER_MOTORS_DCX10	1*/
//#define NUMBER_MOTORS_BASE	2
//#define NUMBER_MAX_EPOS2_PER_SLAVE 16

/*! Namespace */
using namespace std;
using namespace osa_gui;
using namespace gui;
using namespace common;
using namespace Qt;

/*! Implementation of class MainWindow */

/*! \fn MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
 *  \brief constructor
 *  \param argc
 *  \param argv
 *  \param parent
 */
MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
	QMainWindow(parent),
	ptr_robot_(0),
	ptr_saved_file_(0),
	ptr_projet_(0)
{
	ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	//QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	//ReadSettings();
	setWindowIcon(QIcon(":/images/c3po-tux.png"));
	//ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
	//QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
	//QObject::connect(&qnode2, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*! Logging */
	//ui.view_logging->setModel(qnode.loggingModel());
	//QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

	//ui.view_logging->setModel(qnode2.loggingModel());
	//QObject::connect(&qnode2, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

	//Hide some menu and tool bars
	ui_.menu_Robot->setEnabled(false);
	ui_.toolBar_Robot->setEnabled(false);
	ui_.actionSaveProject->setEnabled(false);
	ui_.actionSaveProjectAs->setEnabled(false);

	//Hide all the sub-windows
	ptr_configuration_gui_ = new ConfigurationGUI(ui_.mdiArea);
	ptr_configuration_gui_->setVisible(false);

	ptr_basic_control_gui_ = new BasicControlGUI(ui_.mdiArea);
	ptr_basic_control_gui_->setVisible(false);

	ptr_sequencer_gui_ = new SequencerGUI(ui_.mdiArea);
	ptr_sequencer_gui_->setVisible(false);

	ptr_plot_gui_ = new PlotGUI(ui_.mdiArea);
	ptr_plot_gui_->setVisible(false);

	ptr_supervisor_gui_ = new SupervisorGUI(ui_.mdiArea);
	ptr_supervisor_gui_->setVisible(false);

	//qRegisterMetaType<StatusArray>("StatusArray");
	//QObject::connect(&qnode, SIGNAL(forceChanged(ForceArray)), m_robotSupervisorGui, SLOT(setScene(ForceArray)), Qt::QueuedConnection);
	//QObject::connect(m_robotSupervisorGui->m_cb_playback, SIGNAL(stateChanged(int)), &qnode, SLOT(setPlayback(int)));

	//create a robot project
	//cout << "********* Robot Conductor Project *********" << endl << endl;

	//create a robot
	//*ptr_robot_ = new Robot();
	//ptr_robot_->setBrand("The Robot Studio");
	//ptr_robot_->setName("Bibot");
	//ptr_robot_->setDof(18);

	//create the actuators

	/*** DCX22 ***/
/*
	//create motor
	Motor *pMotor1 = new Motor();
	pMotor1->setBrand("maxon motor");
	pMotor1->setName("DCX 22 L");
	pMotor1->setBrushType("Graphite Brushes");
	pMotor1->setDiameter(22);
	pMotor1->setNominalSpeed(11400);
	pMotor1->setNominalCurrent(1.03);
	pMotor1->setThermalTimeConstantWinding(22);
	pMotor1->setNumberOfPolePairs(1);

	//create gearbox
	Gearbox *pGearbox1 = new Gearbox();
	pGearbox1->setBrand("maxon motor");
	pGearbox1->setName("91_GPX 22");
	pGearbox1->setDiameter(22);
	pGearbox1->setReduction("52:1");
	pGearbox1->setNumberOfStages(3);

	//create encoder
	Encoder *pEncoder1 = new Encoder();
	pEncoder1->setBrand("maxon motor");
	pEncoder1->setName("103_ENX 16 EASY");
	pEncoder1->setCountsPerTurn(256);
	pEncoder1->setNumberOfChannels(3);

	//set references
	pMotor1->setPGearbox(pGearbox1);
	pMotor1->setPEncoder(pEncoder1);
	pGearbox1->setPMotor(pMotor1);
	pEncoder1->setPMotor(pMotor1);
*/
	/*** RE13 ***/
/*
	//create motor
	Motor *pMotor2 = new Motor();
	pMotor2->setBrand("maxon motor");
	pMotor2->setName("RE 13");
	pMotor2->setBrushType("Graphite Brushes");
	pMotor2->setDiameter(13);
	pMotor2->setNominalSpeed(11400);
	pMotor2->setNominalCurrent(1.03);
	pMotor2->setThermalTimeConstantWinding(22);
	pMotor2->setNumberOfPolePairs(1);

	//create gearbox
	Gearbox *pGearbox2 = new Gearbox();
	pGearbox2->setBrand("maxon motor");
	pGearbox2->setName("GPX 13");
	pGearbox2->setDiameter(13);
	pGearbox2->setReduction("16:1");
	pGearbox2->setNumberOfStages(2);

	//create encoder
	Encoder *pEncoder2 = new Encoder();
	pEncoder2->setBrand("maxon motor");
	pEncoder2->setName("ENX 13 EASY");
	pEncoder2->setCountsPerTurn(128);
	pEncoder2->setNumberOfChannels(2);

	//set references
	pMotor2->setPGearbox(pGearbox2);
	pMotor2->setPEncoder(pEncoder2);
	pGearbox2->setPMotor(pMotor2);
	pEncoder2->setPMotor(pMotor2);
*/
	/*** DCX10 ***/
/*
	//create motor
	Motor *pMotor3 = new Motor();
	pMotor3->setBrand("maxon motor");
	pMotor3->setName("DCX 10 L");
	pMotor3->setBrushType("Graphite Brushes");
	pMotor3->setDiameter(10);
	pMotor3->setNominalSpeed(11400);
	pMotor3->setNominalCurrent(1.03);
	pMotor3->setThermalTimeConstantWinding(22);
	pMotor3->setNumberOfPolePairs(1);

	//create gearbox
	Gearbox *pGearbox3 = new Gearbox();
	pGearbox3->setBrand("maxon motor");
	pGearbox3->setName("GPX 10");
	pGearbox3->setDiameter(10);
	pGearbox3->setReduction("32:1");
	pGearbox3->setNumberOfStages(3);

	//create encoder
	Encoder *pEncoder3 = new Encoder();
	pEncoder3->setBrand("maxon motor");
	pEncoder3->setName("ENX 10 EASY");
	pEncoder3->setCountsPerTurn(256);
	pEncoder3->setNumberOfChannels(3);

	//set references
	pMotor3->setPGearbox(pGearbox3);
	pMotor3->setPEncoder(pEncoder3);
	pGearbox3->setPMotor(pMotor3);
	pEncoder3->setPMotor(pMotor3);
	*/
/*
	//create an array of 16 controllers
	Controller *pController[NUMBER_MOTORS_ARM];

	for(int i=0; i<NUMBER_MOTORS_ARM; i++)
	{
		pController[i] = new Controller();
		//pController[i]->setBrand("maxon motor");
		//(*pController[i]).setName("EPOS2 24/5");
	}

	//copy the motors for each in the robot
	//12*DCX22
	Motor *pMotorDCX22[NUMBER_MOTORS_DCX22] = {pMotor1};
	for(int i=0; i<NUMBER_MOTORS_DCX22; i++)
	{
		//set controllers references
		pMotorDCX22[i]->setPController(pController[i]);
		pController[i]->setPActuator(pMotorDCX22[i]);
	}

	//3*RE13
	Motor *pMotorRE13[NUMBER_MOTORS_RE13] = {pMotor2};
	for(int i=0; i<NUMBER_MOTORS_RE13; i++)
	{
		//set controllers references
		pMotorRE13[i]->setPController(pController[i]);
		pController[i]->setPActuator(pMotorRE13[i]);
	}

	//1*DCX10
	Motor *pMotorDCX10[NUMBER_MOTORS_DCX10] = {pMotor3};
	for(int i=0; i<NUMBER_MOTORS_DCX10; i++)
	{
		//set controllers references
		pMotorDCX10[i]->setPController(pController[i]);
		pController[i]->setPActuator(pMotorDCX10[i]);
	}

	for(int i=0; i<12; i++) //DCX22
	{
		ptr_robot_->addPActuator(pMotor1);
	}

	for(int i=0; i<3; i++) //RE13
	{
		ptr_robot_->addPActuator(pMotor2);
	}

	for(int i=0; i<1; i++) //DCX10
	{
		ptr_robot_->addPActuator(pMotor3);
	}
*/

	//cout << "*** Display ***" << endl << endl;

	//ptr_robot_->display();
}

/**
 * @brief Destructor.
 */
MainWindow::~MainWindow()
{
	delete ptr_configuration_gui_;
	delete ptr_basic_control_gui_;
	delete ptr_sequencer_gui_;
	delete ptr_plot_gui_;
	delete ptr_supervisor_gui_;
	delete ptr_robot_;
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::ReadSettings()
{
	QSettings settings("OpenSourceAndroid", "OSA_Studio");
	restoreGeometry(settings.value("geometry").toByteArray());
	restoreState(settings.value("windowState").toByteArray());

	//QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
	//QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
/*
	//QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
	ui.line_edit_master->setText(master_url);
	ui.line_edit_host->setText(host_url);
	//ui.line_edit_topic->setText(topic_name);
	bool remember = settings.value("remember_settings", false).toBool();
	ui.checkbox_remember_settings->setChecked(remember);
	bool checked = settings.value("use_environment_variables", false).toBool();
	ui.checkbox_use_environment->setChecked(checked);
	if ( checked ) {
		ui.line_edit_master->setEnabled(false);
		ui.line_edit_host->setEnabled(false);
		//ui.line_edit_topic->setEnabled(false);
	}
	*/
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::WriteSettings()
{
	QSettings settings("OpenSourceAndroid", "OSA_Studio");
	//settings.setValue("master_url",ui.line_edit_master->text());
	//settings.setValue("host_url",ui.line_edit_host->text());
	//settings.setValue("topic_name",ui.line_edit_topic->text());
	//settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
	settings.setValue("geometry", saveGeometry());
	settings.setValue("windowState", saveState());
	//settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::read(const QJsonObject &json)
{
	ptr_robot_->read(json);
}

void MainWindow::write(QJsonObject &json) const
{
	ptr_robot_->write(json);
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
/*
void MainWindow::showNoMasterMessage()
{
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
	close();
}*/

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
/*
void MainWindow::on_button_connect_clicked(bool check )
{
	if ( ui.checkbox_use_environment->isChecked() )
	{
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else*/
	//{
	/*	if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}*/
	//}
/*
	//test
	if ( !qnode2.init() ) {
		showNoMasterMessage();
	} else {
		ui.button_connect->setEnabled(false);
	}*/

//}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
//void MainWindow::on_btn_rxgraph_clicked()
//{
	//ui.btn_rxgraph->setText("rxgraph clicked !");

	//open a new terminal and launche rxgraph
	//QString cmd = "gnome-terminal -e 'rxgraph'";

	//system(cmd.c_str());

	//if(!fork())// child process
	//int term1(0);

	//term1 = system("gnome-terminal -e gedit");

	//system("gnome-terminal -x bash -c rxgraph");

	//system("gnome-terminal -e ./scripts/rxgraph.sh; cat");
	//system("gnome-terminal -x bash -c ./scripts/rxgraph.sh; cat");
	//system("gnome-terminal -x sh -c ~/./.bashrc && rxgraph");
	//system("gnome-terminal -x sh -c rxgraph");
	//system("./scripts/rxgraph.sh");
	//int i = system("rxgraph");

	//system("echo $ROS_PACKAGE_PATH");
//}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 *//*
void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}*/



void MainWindow::on_actionNewProject_triggered()
{
	//check if a project is already opened by checking whether ptr_robot_ is empty or not
	if(ptr_robot_ == 0) //empty, so create a project
	{
/*
		ptr_robot_ = new Robot();
		ptr_robot_->setBrand("The Robot Studio");
		ptr_robot_->setName("OSA");
		ptr_robot_->setDof(78);
*/
		/*** Hard-coded robot configuration ***/
		//Robot - Hardware
		ptr_robot_ = new Robot();
		ptr_robot_->setBrand("The Robot Studio");
		ptr_robot_->setName("Open Source Android");
		ptr_robot_->setPartNumber(1);
		ptr_robot_->setVersion("0.1.0");

		//Robot - Robot
		ptr_robot_->setDof(77);
/*
		//Create a MasterBoard object
		MasterBoard *masterBoard = new MasterBoard();
		masterBoard->setBrand("The Robot Studio2");
		masterBoard->setName("MasterBoard");
		masterBoard->setPartNumber(2);
		masterBoard->setVersion("1.0.4");
		//link it to the Robot
		ptr_robot_->setPMasterBoard(masterBoard);
	*/
	}
	else
	{
		QMessageBox::StandardButton reply;

		reply = QMessageBox::question(this, "Create a new project", "An OSA project is already opened. Any unsaved modifications will be lost. Do you want to create a new project anyway ?", QMessageBox::Yes|QMessageBox::No);

		if(reply == QMessageBox::Yes)
		{
			delete ptr_saved_file_;
			delete ptr_robot_;


		}
	}

	//Show some menu and tool bars
	ui_.menu_Robot->setEnabled(true);
	ui_.toolBar_Robot->setEnabled(true);
	ui_.actionSaveProject->setEnabled(true);
	ui_.actionSaveProjectAs->setEnabled(true);

	//Show configuration sub-window
	ptr_configuration_gui_->setVisible(true);
}

void MainWindow::on_actionOpenProject_triggered()
{
	if((ptr_robot_ == 0) || (ptr_saved_file_ == 0))
	{
		QString fileName = QFileDialog::getOpenFileName(this, tr("Open OSA project"), "./catkin_ws/src/osa_gui/save", tr("Open Source Android (*.osa)"));

		if(fileName.isEmpty()) return;
		else
		{
			delete ptr_saved_file_;
			ptr_saved_file_ = new QFile(fileName);

			if(!ptr_saved_file_->open(QIODevice::ReadOnly))
			{
				QMessageBox::information(this, tr("Unable to open file"), ptr_saved_file_->errorString());
				return;
			}



			//TODO load Robot data in the GUI

			/*
			contacts.clear();   // clear existing contacts
			in >> contacts;

			if(contacts.isEmpty())
			{
				QMessageBox::information(this, tr("No contacts in file"), tr("The file you are attempting to open contains no contacts."));
			}
			else
			{
				QMap<QString, QString>::iterator i = contacts.begin();
				nameLine->setText(i.key());
				addressText->setText(i.value());
			}
			*/

			//Show some menu and tool bars
			ui_.menu_Robot->setEnabled(true);
			ui_.toolBar_Robot->setEnabled(true);
			ui_.actionSaveProject->setEnabled(true);
			ui_.actionSaveProjectAs->setEnabled(true);
		}
	}
	else
	{

	}
}

void MainWindow::on_actionSaveProject_triggered()
{
	//TODO if never saved, save as, otherwise just overwrite the last opened file.
	if(ptr_saved_file_ == 0)
	{
		on_actionSaveProjectAs_triggered();
	}
	else
	{
		if(!ptr_saved_file_->open(QIODevice::WriteOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"), ptr_saved_file_->errorString());
			return;
		}

		QJsonObject robotObject;
		this->write(robotObject);
		QJsonDocument saveDoc(robotObject);
		ptr_saved_file_->write(saveDoc.toJson()); //or .toBinaryData()
	}
}

void MainWindow::on_actionSaveProjectAs_triggered()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save OSA project as..."), "./catkin_ws/src/osa_gui/save", tr("Open Source Android (*.osa)"));

	if(fileName.isEmpty()) return;
	else
	{
		QFile file(fileName);
		if(!file.open(QIODevice::WriteOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"), file.errorString());
			return;
		}

		QJsonObject robotObject;
		this->write(robotObject);
		QJsonDocument saveDoc(robotObject);
		file.write(saveDoc.toJson()); //or .toBinaryData()
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionOpenTerminal_triggered()
{
	system("gnome-terminal");
}

void MainWindow::on_actionToolBar_triggered()
{
	if(ui_.actionToolBar->isChecked())
	{
		ui_.toolBar_Project->setVisible(true);
		ui_.toolBar_ROS->setVisible(true);
		ui_.toolBar_Robot->setVisible(true);
	}
	else
	{
		ui_.toolBar_Project->setVisible(false);
		ui_.toolBar_ROS->setVisible(false);
		ui_.toolBar_Robot->setVisible(false);
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionStatusBar_triggered()
{
	if(ui_.actionStatusBar->isChecked())
	{
		ui_.statusbar->setVisible(true);
	}
	else
	{
		ui_.statusbar->setVisible(false);
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionOpenConfiguration_triggered()
{
	if(ptr_configuration_gui_ != NULL)
	{
		ptr_configuration_gui_->setVisible(!ptr_configuration_gui_->isVisible()); //change state of visibility
	}
	else
	{
		ptr_configuration_gui_ = new ConfigurationGUI(ui_.mdiArea);
		ptr_configuration_gui_->setVisible(true);
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionOpenControl_triggered()
{
	if(ptr_basic_control_gui_ != NULL)
	{
		ptr_basic_control_gui_->setVisible(!ptr_basic_control_gui_->isVisible()); //change state of visibility

	}
	else
	{
		ptr_basic_control_gui_ = new BasicControlGUI(ui_.mdiArea);
		ptr_basic_control_gui_->setVisible(true);
	}
}

void MainWindow::on_actionOpenSequencer_triggered()
{
	if(ptr_sequencer_gui_ != NULL)
	{
		ptr_sequencer_gui_->setVisible(!ptr_sequencer_gui_->isVisible()); //change state of visibility
	}
	else
	{
		ptr_sequencer_gui_ = new SequencerGUI(ui_.mdiArea);
		ptr_sequencer_gui_->setVisible(true);
	}
}

void MainWindow::on_actionOpenPlot_triggered()
{
	if(ptr_plot_gui_ != NULL)
	{
		ptr_plot_gui_->setVisible(!ptr_plot_gui_->isVisible()); //change state of visibility
	}
	else
	{
		ptr_plot_gui_ = new PlotGUI(ui_.mdiArea);
		ptr_plot_gui_->setVisible(true);
	}
}

void MainWindow::on_actionOpenSupervisor_triggered()
{
	if(ptr_supervisor_gui_ != NULL)
	{
		if(ptr_supervisor_gui_->isVisible())
		{
			ptr_supervisor_gui_->setVisible(false);
		}
		else
		{
			ptr_supervisor_gui_->setVisible(true);
		}
	}
	else
	{
		ptr_supervisor_gui_ = new SupervisorGUI(ui_.mdiArea);
		ptr_supervisor_gui_->setVisible(true);
	}
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

/*! \fn
 *  \brief
 *  \param
 *  \return void
 *//*
void MainWindow::updateLoggingView()
{
		ui.view_logging->scrollToBottom();
}*/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionAbout_triggered()
{
	QMessageBox::about(this, tr("About"),tr("<h2>Open Source Android Studio 2.0.0</h2><p>Copyright The Robot Studio 2017</p><p>Software specially designed for controlling biomimetic tendon-driven compliant robots.</p> <p> <a href=\"http://www.opensourceandroid.org\">Visit Open Source Android website</a></p> <p> <a href=\"http://www.therobotstudio.com\">Visit The Robot Studio website</a></p>"));
}

/*
void MainWindow::on_actionExit_triggered()
{
	//QMainWindow::closeEvent();
}
*/

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

