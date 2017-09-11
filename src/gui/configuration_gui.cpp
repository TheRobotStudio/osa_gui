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
 * @file configuration_gui.cpp
 * @author Cyril Jourdan
 * @date Feb 20, 2017
 * @version 0.1.0
 * @brief Implementation file for class ConfigurationGUI
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Feb 20, 2017
 */

#include "include/gui/configuration_gui.h"

using namespace osa_gui;
using namespace gui;

ConfigurationGUI::ConfigurationGUI(QWidget *parent) :
	QMdiSubWindow(parent)
{
	ui_.setupUi(this);

	QStringList modeList = (QStringList() << "MasterBoard" << "Battery");
	ui_.cb_type->addItems(modeList);

	//this->activateWindow();
	//parent->resize();
}

ConfigurationGUI::~ConfigurationGUI()
{
}

void ConfigurationGUI::on_treev_robot_itemClicked(QTreeWidgetItem *item, int column)
{
	if(item->text(0).toStdString() == "Robot")
	{

	}
	else if(item->text(0).toStdString() == "MasterBoard")
	{

	}
	else if(item->text(0).toStdString() == "Battery")
	{

	}
}

void ConfigurationGUI::on_cb_type_currentIndexChanged(int index)
{
	if(index == 0) //MasterBoard
	{

	}
	else if(index == 1) //Battery
	{

	}
}

void ConfigurationGUI::on_pb_addHardware_clicked()
{
	//Add root node
	addTreeRoot(ui_.cb_type->currentText(), ui_.le_description->text());
}

void ConfigurationGUI::on_pb_wizard_clicked()
{

}

void ConfigurationGUI::addTreeRoot(QString hardware, QString description)
{
    //QTreeWidgetItem(QTreeWidget * parent, int type = Type)
    QTreeWidgetItem *tree_item = new QTreeWidgetItem(ui_.treev_robot);

    //QTreeWidgetItem::setText(int column, const QString & text)
    tree_item->setText(0, hardware);
    tree_item->setText(1, description);
}

void ConfigurationGUI::addTreeChild(QTreeWidgetItem *parent, QString hardware, QString description)
{
    // QTreeWidgetItem(QTreeWidget * parent, int type = Type)
    QTreeWidgetItem *tree_item = new QTreeWidgetItem();

    // QTreeWidgetItem::setText(int column, const QString & text)
    tree_item->setText(0, hardware);
    tree_item->setText(1, description);

    // QTreeWidgetItem::addChild(QTreeWidgetItem * child)
    parent->addChild(tree_item);
}
