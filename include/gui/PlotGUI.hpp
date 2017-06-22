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
 * @file RobotPlotGUI.hpp
 * @author Cyril Jourdan
 * @date Mar 17, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for class RobotPlotGUI.
 * 		  Adapted from QCustomPlot interaction example.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 17, 2017
 */

#ifndef PLOTGUI_H
#define PLOTGUI_H

//#include <QMainWindow>
#include <QMdiSubWindow>
#include <QInputDialog>
#include "qcustomplot/qcustomplot.h"
#include "../rosnode/PlotNode.hpp"
#include "ui_PlotGUI.h"

namespace osa_gui
{
	namespace gui
	{
		class PlotGUI : public QMdiSubWindow //QMainWindow
		{
			Q_OBJECT

			public:
				//explicit RobotPlotGUI(QWidget *parent = 0);
				PlotGUI(QWidget *parent = 0);
				~PlotGUI();

			private Q_SLOTS: //slots:
				void titleDoubleClick(QMouseEvent *event);
				void axisLabelDoubleClick(QCPAxis* axis, QCPAxis::SelectablePart part);
				void legendDoubleClick(QCPLegend* legend, QCPAbstractLegendItem* item);
				void selectionChanged();
				void mousePress();
				void mouseWheel();
				//void addRandomGraph();
				void removeSelectedGraph();
				void removeAllGraphs();
				void contextMenuRequest(QPoint pos);
				void moveLegend();
				void graphClicked(QCPAbstractPlottable *plottable, int dataIndex);

				/*! Auto-connections (connectSlotsByName()) */
				//void on_ch_sel_0_stateChanged(int state);
				void on_pb_record_0_pressed();
				void on_pb_color_0_released();

				//slots of callbacks
				void setCustomPlotData();

			private:
				Ui::PlotGUIClass m_ui;
				rosnode::PlotNode m_plotNode;
				int m_time;
				QVector<double> m_x; //x(5000), y(5000);
				QVector<double> m_y;
				QVector<QColor> m_graphColor;
		};
	}
}

#endif // ROBOTPLOTGUI_H
