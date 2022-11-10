# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mc_log_main_ui.ui'
#
# Created by: PyQt5 UI code generator 5.10.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.centralwidget.sizePolicy().hasHeightForWidth())
        self.centralwidget.setSizePolicy(sizePolicy)
        self.centralwidget.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.tabWidget = MCLogTabWidget(self.centralwidget)
        self.tabWidget.setObjectName("tabWidget")
        self.tab = MCLogTab()
        self.tab.setObjectName("tab")
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.gridLayout_2.addWidget(self.tabWidget, 0, 0, 1, 1)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 40))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuCommonPlots = QtWidgets.QMenu(self.menubar)
        self.menuCommonPlots.setObjectName("menuCommonPlots")
        self.menuUserPlots = QtWidgets.QMenu(self.menubar)
        self.menuUserPlots.setObjectName("menuUserPlots")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionLoad = QtWidgets.QAction(MainWindow)
        self.actionLoad.setObjectName("actionLoad")
        self.actionExit = QtWidgets.QAction(MainWindow)
        self.actionExit.setObjectName("actionExit")
        self.actionCompare = QtWidgets.QAction(MainWindow)
        self.actionCompare.setObjectName("actionCompare")
        self.menuFile.addAction(self.actionLoad)
        self.menuFile.addAction(self.actionCompare)
        self.menuFile.addAction(self.actionExit)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuCommonPlots.menuAction())
        self.menubar.addAction(self.menuUserPlots.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MC Log Plotter"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Plot 1"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "+"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuCommonPlots.setTitle(_translate("MainWindow", "Common plots"))
        self.menuUserPlots.setTitle(_translate("MainWindow", "User plots"))
        self.actionLoad.setText(_translate("MainWindow", "Load..."))
        self.actionExit.setText(_translate("MainWindow", "Exit"))
        self.actionCompare.setText(_translate("MainWindow", "Compare..."))

from mc_log_ui.mc_log_tab import MCLogTab
from mc_log_ui.mc_log_tab_widget import MCLogTabWidget
