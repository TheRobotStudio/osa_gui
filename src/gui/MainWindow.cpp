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
 * @file MainWindow.cpp
 * @author Cyril Jourdan
 * @date Mar 14, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for class MainWindow.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

/*! Include */
//#include <include/gui/BasicControlGUI.hpp>
//#include <include/gui/ConfigurationGUI.hpp>
//#include <include/gui/SupervisorGUI.hpp>
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "MainWindow.hpp"
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
		m_pRobot(0),
		m_pSavefile(0),
		m_pProjet(0)
{
	m_ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
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
	m_ui.menu_Robot->setEnabled(false);
	m_ui.toolBar_Robot->setEnabled(false);
	m_ui.actionSaveProject->setEnabled(false);
	m_ui.actionSaveProjectAs->setEnabled(false);

	//Hide all the sub-windows
	m_configurationGui = new ConfigurationGUI(m_ui.mdiArea);
	m_configurationGui->setVisible(false);

	m_basicCtrlGui = new BasicControlGUI(m_ui.mdiArea);
	m_basicCtrlGui->setVisible(false);

	m_sequencerGui = new SequencerGUI(m_ui.mdiArea);
	m_sequencerGui->setVisible(false);

	m_plotGui = new PlotGUI(m_ui.mdiArea);
	m_plotGui->setVisible(false);

	m_supervisorGui = new SupervisorGUI(m_ui.mdiArea);
	m_supervisorGui->setVisible(false);

	//qRegisterMetaType<StatusArray>("StatusArray");
	//QObject::connect(&qnode, SIGNAL(forceChanged(ForceArray)), m_robotSupervisorGui, SLOT(setScene(ForceArray)), Qt::QueuedConnection);
	//QObject::connect(m_robotSupervisorGui->m_cb_playback, SIGNAL(stateChanged(int)), &qnode, SLOT(setPlayback(int)));

	//create a robot project
	//cout << "********* Robot Conductor Project *********" << endl << endl;

	//create a robot
	//*m_pRobot = new Robot();
	//m_pRobot->setBrand("The Robot Studio");
	//m_pRobot->setName("Bibot");
	//m_pRobot->setDof(18);

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
		m_pRobot->addPActuator(pMotor1);
	}

	for(int i=0; i<3; i++) //RE13
	{
		m_pRobot->addPActuator(pMotor2);
	}

	for(int i=0; i<1; i++) //DCX10
	{
		m_pRobot->addPActuator(pMotor3);
	}
*/

	//cout << "*** Display ***" << endl << endl;

	//m_pRobot->display();
}

/*! \fn MainWindow::~MainWindow()
 *  \brief destructor
 */
MainWindow::~MainWindow()
{
	delete m_configurationGui;
	delete m_basicCtrlGui;
	delete m_sequencerGui;
	delete m_plotGui;
	delete m_supervisorGui;
	delete m_pRobot;
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
	m_pRobot->read(json);
}

void MainWindow::write(QJsonObject &json) const
{
	m_pRobot->write(json);
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
	//check if a project is already opened by checking whether m_pRobot is empty or not
	if(m_pRobot == 0) //empty, so create a project
	{
/*
		m_pRobot = new Robot();
		m_pRobot->setBrand("The Robot Studio");
		m_pRobot->setName("OSA");
		m_pRobot->setDof(78);
*/
		/*** Hard-coded robot configuration ***/
		//Robot - Hardware
		m_pRobot = new Robot();
		m_pRobot->setBrand("The Robot Studio");
		m_pRobot->setName("Open Source Android");
		m_pRobot->setPartNumber(1);
		m_pRobot->setVersion("2.0.0");

		//Robot - Robot
		m_pRobot->setDof(74);

		//Create a MasterBoard object
		MasterBoard *masterBoard = new MasterBoard();
		masterBoard->setBrand("The Robot Studio2");
		masterBoard->setName("MasterBoard");
		masterBoard->setPartNumber(2);
		masterBoard->setVersion("1.0.4");
		//link it to the Robot
		m_pRobot->setPMasterBoard(masterBoard);
	}
	else
	{
		QMessageBox::StandardButton reply;

		reply = QMessageBox::question(this, "Create a new project", "An OSA project is already opened. Any unsaved modifications will be lost. Do you want to create a new project anyway ?", QMessageBox::Yes|QMessageBox::No);

		if(reply == QMessageBox::Yes)
		{
			delete m_pSavefile;
			delete m_pRobot;


		}
	}

	//Show some menu and tool bars
	m_ui.menu_Robot->setEnabled(true);
	m_ui.toolBar_Robot->setEnabled(true);
	m_ui.actionSaveProject->setEnabled(true);
	m_ui.actionSaveProjectAs->setEnabled(true);

	//Show configuration sub-window
	m_configurationGui->setVisible(true);
}

void MainWindow::on_actionOpenProject_triggered()
{
	if((m_pRobot == 0) || (m_pSavefile == 0))
	{
		QString fileName = QFileDialog::getOpenFileName(this, tr("Open OSA project"), "./catkin_ws/src/osa_gui/save", tr("Open Source Android (*.osa)"));

		if(fileName.isEmpty()) return;
		else
		{
			delete m_pSavefile;
			m_pSavefile = new QFile(fileName);

			if(!m_pSavefile->open(QIODevice::ReadOnly))
			{
				QMessageBox::information(this, tr("Unable to open file"), m_pSavefile->errorString());
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
			m_ui.menu_Robot->setEnabled(true);
			m_ui.toolBar_Robot->setEnabled(true);
			m_ui.actionSaveProject->setEnabled(true);
			m_ui.actionSaveProjectAs->setEnabled(true);
		}
	}
	else
	{

	}
}

void MainWindow::on_actionSaveProject_triggered()
{
	//TODO if never saved, save as, otherwise just overwrite the last opened file.
	if(m_pSavefile == 0)
	{
		on_actionSaveProjectAs_triggered();
	}
	else
	{
		if(!m_pSavefile->open(QIODevice::WriteOnly))
		{
			QMessageBox::information(this, tr("Unable to open file"), m_pSavefile->errorString());
			return;
		}

		QJsonObject robotObject;
		this->write(robotObject);
		QJsonDocument saveDoc(robotObject);
		m_pSavefile->write(saveDoc.toJson()); //or .toBinaryData()
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
	if(m_ui.actionToolBar->isChecked())
	{
		m_ui.toolBar_Project->setVisible(true);
		m_ui.toolBar_ROS->setVisible(true);
		m_ui.toolBar_Robot->setVisible(true);
	}
	else
	{
		m_ui.toolBar_Project->setVisible(false);
		m_ui.toolBar_ROS->setVisible(false);
		m_ui.toolBar_Robot->setVisible(false);
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionStatusBar_triggered()
{
	if(m_ui.actionStatusBar->isChecked())
	{
		m_ui.statusbar->setVisible(true);
	}
	else
	{
		m_ui.statusbar->setVisible(false);
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionOpenConfiguration_triggered()
{
	if(m_configurationGui != NULL)
	{
		m_configurationGui->setVisible(!m_configurationGui->isVisible()); //change state of visibility
	}
	else
	{
		m_configurationGui = new ConfigurationGUI(m_ui.mdiArea);
		m_configurationGui->setVisible(true);
	}
}

/*! \fn
 *  \brief
 *  \param
 *  \return void
 */
void MainWindow::on_actionOpenControl_triggered()
{
	if(m_basicCtrlGui != NULL)
	{
		m_basicCtrlGui->setVisible(!m_basicCtrlGui->isVisible()); //change state of visibility

	}
	else
	{
		m_basicCtrlGui = new BasicControlGUI(m_ui.mdiArea);
		m_basicCtrlGui->setVisible(true);
	}
}

void MainWindow::on_actionOpenSequencer_triggered()
{
	if(m_sequencerGui != NULL)
	{
		m_sequencerGui->setVisible(!m_sequencerGui->isVisible()); //change state of visibility
	}
	else
	{
		m_sequencerGui = new SequencerGUI(m_ui.mdiArea);
		m_sequencerGui->setVisible(true);
	}
}

void MainWindow::on_actionOpenPlot_triggered()
{
	if(m_plotGui != NULL)
	{
		m_plotGui->setVisible(!m_plotGui->isVisible()); //change state of visibility
	}
	else
	{
		m_plotGui = new PlotGUI(m_ui.mdiArea);
		m_plotGui->setVisible(true);
	}
}

void MainWindow::on_actionOpenSupervisor_triggered()
{
	if(m_supervisorGui != NULL)
	{
		if(m_supervisorGui->isVisible())
		{
			m_supervisorGui->setVisible(false);
		}
		else
		{
			m_supervisorGui->setVisible(true);
		}
	}
	else
	{
		m_supervisorGui = new SupervisorGUI(m_ui.mdiArea);
		m_supervisorGui->setVisible(true);
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

