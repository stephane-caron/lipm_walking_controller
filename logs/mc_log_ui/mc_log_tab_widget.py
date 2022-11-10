# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtWidgets

class MCLogTabBar(QtWidgets.QTabBar):
  def __init__(self, parent = None):
    super(MCLogTabBar, self).__init__(parent)
  def mouseReleaseEvent(self, event):
    if event.button() == QtCore.Qt.MiddleButton and self.parent().tabsClosable():
      pos = event.pos()
      for i in range(self.count() - 1):
        if self.tabRect(i).contains(pos):
          self.parent().tabCloseRequested.emit(i)
          return
    super(MCLogTabBar, self).mouseReleaseEvent(event)
  def mouseDoubleClickEvent(self, event):
    if event.button() == QtCore.Qt.LeftButton:
      name = QtWidgets.QInputDialog.getText(self,
                                        "New name for current graph",
                                        "New name:",
                                        text = self.tabText(self.currentIndex()))
      if name[1]:
        self.setTabText(self.currentIndex(), name[0])
    super(MCLogTabBar, self).mouseDoubleClickEvent(event)

class MCLogTabWidget(QtWidgets.QTabWidget):
  def __init__(self, parent = None):
    super(MCLogTabWidget, self).__init__(parent)
    self.setTabBar(MCLogTabBar(self))
