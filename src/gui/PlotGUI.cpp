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
 * @file PlotGUI.cpp
 * @author Cyril Jourdan
 * @date Mar 17, 2017
 * @version OSA 2.0.0
 * @brief Implementation file for class PlotGUI.
 * 		  Adapted from QCustomPlot interaction example.
 *
 * Contact: cyril.jourdan@therobotstudio.com
 * Created on : Mar 17, 2017
 */

#include <include/gui/PlotGUI.hpp>
#include <iostream>

using namespace std;
using namespace osa_gui;
using namespace gui;
using namespace Qt;

PlotGUI::PlotGUI(QWidget *parent) :
		QMdiSubWindow(parent),
		//m_ui(new Ui::PlotGUIClass),
		m_time(0),
		m_x(0),
		m_y(0),
		m_graphColor(10)
{
	srand(QDateTime::currentDateTime().toTime_t());

	m_ui.setupUi(this);

	m_ui.customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectAxes |
								  QCP::iSelectLegend | QCP::iSelectPlottables);
	m_ui.customPlot->xAxis->setRange(0, 10);
	m_ui.customPlot->yAxis->setRange(-5, 5);
	m_ui.customPlot->axisRect()->setupFullAxesBox();

	m_ui.customPlot->plotLayout()->insertRow(0);
	QCPTextElement *title = new QCPTextElement(m_ui.customPlot, "Motor data plot", QFont("sans", 17, QFont::Bold));
	m_ui.customPlot->plotLayout()->addElement(0, 0, title);

	m_ui.customPlot->xAxis->setLabel("x Axis - cycles");
	m_ui.customPlot->yAxis->setLabel("y Axis - data");
	m_ui.customPlot->legend->setVisible(true);
	QFont legendFont = font();
	legendFont.setPointSize(10);
	m_ui.customPlot->legend->setFont(legendFont);
	m_ui.customPlot->legend->setSelectedFont(legendFont);
	m_ui.customPlot->legend->setSelectableParts(QCPLegend::spItems); // legend box shall not be selectable, only legend items

	// connect slot that ties some axis selections together (especially opposite axes):
	connect(m_ui.customPlot, SIGNAL(selectionChangedByUser()), this, SLOT(selectionChanged()));
	// connect slots that takes care that when an axis is selected, only that direction can be dragged and zoomed:
	connect(m_ui.customPlot, SIGNAL(mousePress(QMouseEvent*)), this, SLOT(mousePress()));
	connect(m_ui.customPlot, SIGNAL(mouseWheel(QWheelEvent*)), this, SLOT(mouseWheel()));

	// make bottom and left axes transfer their ranges to top and right axes:
	connect(m_ui.customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), m_ui.customPlot->xAxis2, SLOT(setRange(QCPRange)));
	connect(m_ui.customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), m_ui.customPlot->yAxis2, SLOT(setRange(QCPRange)));

	// connect some interaction slots:
	connect(m_ui.customPlot, SIGNAL(axisDoubleClick(QCPAxis*,QCPAxis::SelectablePart,QMouseEvent*)), this, SLOT(axisLabelDoubleClick(QCPAxis*,QCPAxis::SelectablePart)));
	connect(m_ui.customPlot, SIGNAL(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*,QMouseEvent*)), this, SLOT(legendDoubleClick(QCPLegend*,QCPAbstractLegendItem*)));
	connect(title, SIGNAL(doubleClicked(QMouseEvent*)), this, SLOT(titleDoubleClick(QMouseEvent*)));

	// connect slot that shows a message in the status bar when a graph is clicked:
	connect(m_ui.customPlot, SIGNAL(plottableClick(QCPAbstractPlottable*,int,QMouseEvent*)), this, SLOT(graphClicked(QCPAbstractPlottable*,int)));

	// setup policy and connect slot for context menu popup:
	m_ui.customPlot->setContextMenuPolicy(Qt::CustomContextMenu);
	connect(m_ui.customPlot, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(contextMenuRequest(QPoint)));

	QObject::connect(&m_plotNode, SIGNAL(rosShutdown()), this, SLOT(close()));
	QObject::connect(&m_plotNode, SIGNAL(motorDataReceived()), this, SLOT(setCustomPlotData()));

	//Connect the node to ROS
	m_plotNode.init();
}

PlotGUI::~PlotGUI()
{
	//delete m_ui;
}

void PlotGUI::titleDoubleClick(QMouseEvent* event)
{
	Q_UNUSED(event)
	if (QCPTextElement *title = qobject_cast<QCPTextElement*>(sender()))
	{
		// Set the plot title by double clicking on it
		bool ok;
		QString newTitle = QInputDialog::getText(this, "Plot title", "New plot title:", QLineEdit::Normal, title->text(), &ok);
		if (ok)
		{
			title->setText(newTitle);
			m_ui.customPlot->replot();
		}
	}
}

void PlotGUI::axisLabelDoubleClick(QCPAxis *axis, QCPAxis::SelectablePart part)
{
	// Set an axis label by double clicking on it
	if (part == QCPAxis::spAxisLabel) // only react when the actual axis label is clicked, not tick label or axis backbone
	{
		bool ok;
		QString newLabel = QInputDialog::getText(this, "Axis", "New axis name:", QLineEdit::Normal, axis->label(), &ok);
		if (ok)
		{
			axis->setLabel(newLabel);
			m_ui.customPlot->replot();
		}
	}
}

void PlotGUI::legendDoubleClick(QCPLegend *legend, QCPAbstractLegendItem *item)
{
	// Rename a graph by double clicking on its legend item
	Q_UNUSED(legend)
	if (item) // only react if item was clicked (user could have clicked on border padding of legend where there is no item, then item is 0)
	{
		QCPPlottableLegendItem *plItem = qobject_cast<QCPPlottableLegendItem*>(item);
		bool ok;
		QString newName = QInputDialog::getText(this, "Legend", "New legend name:", QLineEdit::Normal, plItem->plottable()->name(), &ok);
		if (ok)
		{
			plItem->plottable()->setName(newName);
			m_ui.customPlot->replot();
		}
	}
}

void PlotGUI::selectionChanged()
{
	/*
	normally, axis base line, axis tick labels and axis labels are selectable separately, but we want
	the user only to be able to select the axis as a whole, so we tie the selected states of the tick labels
	and the axis base line together. However, the axis label shall be selectable individually.

	The selection state of the left and right axes shall be synchronized as well as the state of the
	bottom and top axes.

	Further, we want to synchronize the selection of the graphs with the selection state of the respective
	legend item belonging to that graph. So the user can select a graph by either clicking on the graph itself
	or on its legend item.
	*/
  
	// make top and bottom axes be selected synchronously, and handle axis and tick labels as one selectable object:
	if (m_ui.customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis) || m_ui.customPlot->xAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
	  m_ui.customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spAxis) || m_ui.customPlot->xAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
	{
		m_ui.customPlot->xAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
		m_ui.customPlot->xAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
	}
	// make left and right axes be selected synchronously, and handle axis and tick labels as one selectable object:
	if (m_ui.customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis) || m_ui.customPlot->yAxis->selectedParts().testFlag(QCPAxis::spTickLabels) ||
	  m_ui.customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spAxis) || m_ui.customPlot->yAxis2->selectedParts().testFlag(QCPAxis::spTickLabels))
	{
		m_ui.customPlot->yAxis2->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
		m_ui.customPlot->yAxis->setSelectedParts(QCPAxis::spAxis|QCPAxis::spTickLabels);
	}
  
	// synchronize selection of graphs with selection of corresponding legend items:
	for (int i=0; i<m_ui.customPlot->graphCount(); ++i)
	{
		QCPGraph *graph = m_ui.customPlot->graph(i);
		QCPPlottableLegendItem *item = m_ui.customPlot->legend->itemWithPlottable(graph);
		if (item->selected() || graph->selected())
		{
			item->setSelected(true);
			graph->setSelection(QCPDataSelection(graph->data()->dataRange()));
		}
	}
}

void PlotGUI::mousePress()
{
	// if an axis is selected, only allow the direction of that axis to be dragged
	// if no axis is selected, both directions may be dragged

	if (m_ui.customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
		m_ui.customPlot->axisRect()->setRangeDrag(m_ui.customPlot->xAxis->orientation());
	else if (m_ui.customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
		m_ui.customPlot->axisRect()->setRangeDrag(m_ui.customPlot->yAxis->orientation());
	else
		m_ui.customPlot->axisRect()->setRangeDrag(Qt::Horizontal|Qt::Vertical);
}

void PlotGUI::mouseWheel()
{
	// if an axis is selected, only allow the direction of that axis to be zoomed
	// if no axis is selected, both directions may be zoomed

	if (m_ui.customPlot->xAxis->selectedParts().testFlag(QCPAxis::spAxis))
		m_ui.customPlot->axisRect()->setRangeZoom(m_ui.customPlot->xAxis->orientation());
	else if (m_ui.customPlot->yAxis->selectedParts().testFlag(QCPAxis::spAxis))
		m_ui.customPlot->axisRect()->setRangeZoom(m_ui.customPlot->yAxis->orientation());
	else
		m_ui.customPlot->axisRect()->setRangeZoom(Qt::Horizontal|Qt::Vertical);
}

void PlotGUI::removeSelectedGraph()
{
	//TODO toggle pressed record button of selected graph

	if(m_ui.customPlot->selectedGraphs().size() > 0)
	{
		m_ui.customPlot->removeGraph(m_ui.customPlot->selectedGraphs().first());
		m_ui.customPlot->replot();
	}
}

void PlotGUI::removeAllGraphs()
{
	//TODO toggle pressed record buttons

	m_ui.customPlot->clearGraphs();
	m_ui.customPlot->replot();
}

void PlotGUI::contextMenuRequest(QPoint pos)
{
	QMenu *menu = new QMenu(this);
	menu->setAttribute(Qt::WA_DeleteOnClose);
  
	if(m_ui.customPlot->legend->selectTest(pos, false) >= 0) // context menu on legend requested
	{
		menu->addAction("Move to top left", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignLeft));
		menu->addAction("Move to top center", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignHCenter));
		menu->addAction("Move to top right", this, SLOT(moveLegend()))->setData((int)(Qt::AlignTop|Qt::AlignRight));
		menu->addAction("Move to bottom right", this, SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignRight));
		menu->addAction("Move to bottom left", this, SLOT(moveLegend()))->setData((int)(Qt::AlignBottom|Qt::AlignLeft));
	}
	else  // general context menu on graphs requested
	{
		//menu->addAction("Add random graph", this, SLOT(addRandomGraph()));
		if(m_ui.customPlot->selectedGraphs().size() > 0)
			menu->addAction("Remove selected graph", this, SLOT(removeSelectedGraph()));
		if(m_ui.customPlot->graphCount() > 0)
			menu->addAction("Remove all graphs", this, SLOT(removeAllGraphs()));
	}

	menu->popup(m_ui.customPlot->mapToGlobal(pos));
}

void PlotGUI::moveLegend()
{
	if(QAction* contextAction = qobject_cast<QAction*>(sender())) // make sure this slot is really called by a context menu action, so it carries the data we need
	{
		bool ok;
		int dataInt = contextAction->data().toInt(&ok);

		if(ok)
		{
			m_ui.customPlot->axisRect()->insetLayout()->setInsetAlignment(0, (Qt::Alignment)dataInt);
			m_ui.customPlot->replot();
		}
	}
}

void PlotGUI::graphClicked(QCPAbstractPlottable *plottable, int dataIndex)
{
	// since we know we only have QCPGraphs in the plot, we can immediately access interface1D()
	// usually it's better to first check whether interface1D() returns non-zero, and only then use it.
	double dataValue = plottable->interface1D()->dataMainValue(dataIndex);
	QString message = QString("Clicked on graph '%1' at data point #%2 with value %3.").arg(plottable->name()).arg(dataIndex).arg(dataValue);
	m_ui.statusBar->showMessage(message, 2500);
}

//autoconnected signal/slots
void PlotGUI::on_pb_record_0_pressed()
{
	if(!m_ui.pb_record_0->isChecked())
	{
		//prepare the data array as the size of the duration recording
		//create a new graph with the selected parameters
		m_ui.customPlot->addGraph();
		m_ui.customPlot->graph()->setName(QString("%1-%2 : %3").arg(m_ui.cb_sb_0->currentText(), m_ui.cb_node_0->currentText(), m_ui.cb_data_0->currentText()));

		m_ui.customPlot->graph()->setLineStyle((QCPGraph::LineStyle)(m_ui.cb_sb_0->currentIndex()));
		m_ui.customPlot->graph()->setScatterStyle((QCPScatterStyle::ScatterShape)(m_ui.cb_symbol_0->currentIndex()));

		QPen graphPen;
		graphPen.setColor(m_graphColor[0]);
		graphPen.setWidthF(m_ui.sb_width_0->value());
		m_ui.customPlot->graph()->setPen(graphPen);

		//replot the graph
		m_ui.customPlot->replot();

		//set the x and y data vector size
		m_x.clear();
		m_y.clear();

		m_time = 0;
	}
}

void PlotGUI::on_pb_color_0_released()
{
	m_graphColor[0] = QColorDialog::getColor(Qt::white, this);

	m_ui.pb_color_0->setStyleSheet("background-color: " + m_graphColor[0].name()); //"background-color: #00FF00;"
}

//slots of callbacks
void PlotGUI::setCustomPlotData()
{
	//int n = 1;
	//QVector<double> x(n), y(n);

	//plot if selected and if record has been clicked
	if(m_ui.ch_sel_0->checkState() == 0) //unchecked
	{

	}
	else if(m_ui.ch_sel_0->checkState() == 2) //checked
	{
		if(m_ui.pb_record_0->isChecked())
		{
			m_x.push_back(m_time);

			if(m_ui.cb_data_0->currentIndex() == 0)
				m_y.push_back(m_plotNode.getMotorData_ma().motorData[0].position);
			if(m_ui.cb_data_0->currentIndex() == 1)
				m_y.push_back(m_plotNode.getMotorData_ma().motorData[0].current);
			if(m_ui.cb_data_0->currentIndex() == 2)
				m_y.push_back(m_plotNode.getMotorData_ma().motorData[0].status);

			m_ui.customPlot->graph()->setData(m_x, m_y);
			m_ui.customPlot->replot();
			m_ui.customPlot->rescaleAxes();
		}
	}
/*
	m_x[m_time] = m_time;
	m_y[m_time] = m_plotNode.getMotorData_ma().motorData[0].position;

	m_ui.customPlot->graph()->setData(x, y);

	m_ui.customPlot->replot();
	m_ui.customPlot->rescaleAxes();
*/
	m_time++;
}

