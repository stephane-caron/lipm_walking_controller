# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mc_log_tab_ui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MCLogTab(object):
    def setupUi(self, MCLogTab):
        MCLogTab.setObjectName("MCLogTab")
        MCLogTab.resize(803, 538)
        self.horizontalLayout = QtWidgets.QHBoxLayout(MCLogTab)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.y1SelectorLayout = QtWidgets.QVBoxLayout()
        self.y1SelectorLayout.setObjectName("y1SelectorLayout")
        self.y1Selector = QtWidgets.QTreeWidget(MCLogTab)
        self.y1Selector.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.y1Selector.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectItems)
        self.y1Selector.setColumnCount(1)
        self.y1Selector.setObjectName("y1Selector")
        self.y1Selector.headerItem().setText(0, "1")
        self.y1Selector.header().setVisible(True)
        self.y1SelectorLayout.addWidget(self.y1Selector)
        self.horizontalLayout.addLayout(self.y1SelectorLayout)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.canvas = PlotCanvasWithToolbar(MCLogTab)
        self.canvas.setObjectName("canvas")
        self.verticalLayout.addWidget(self.canvas)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.y2SelectorLayout = QtWidgets.QVBoxLayout()
        self.y2SelectorLayout.setObjectName("y2SelectorLayout")
        self.y2Selector = QtWidgets.QTreeWidget(MCLogTab)
        self.y2Selector.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.y2Selector.setColumnCount(1)
        self.y2Selector.setObjectName("y2Selector")
        self.y2Selector.headerItem().setText(0, "1")
        self.y2Selector.header().setVisible(True)
        self.y2SelectorLayout.addWidget(self.y2Selector)
        self.horizontalLayout.addLayout(self.y2SelectorLayout)

        self.retranslateUi(MCLogTab)
        QtCore.QMetaObject.connectSlotsByName(MCLogTab)

    def retranslateUi(self, MCLogTab):
        _translate = QtCore.QCoreApplication.translate
        MCLogTab.setWindowTitle(_translate("MCLogTab", "MCLogTab"))

from mc_log_ui.mc_log_plotcanvas import PlotCanvasWithToolbar
