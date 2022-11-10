#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtGui, QtWidgets

import functools

# Decorator to wrap a dialog
def InitDialogWithOkCancel(fun = None, Layout = QtWidgets.QFormLayout , apply_ = True):
  def wrap_init(init_fun):
    @functools.wraps(init_fun)
    def wrapped_init(self, parent, *args, **kwargs):
      super(QtWidgets.QDialog, self).__init__(parent)
      self.setModal(True)
      self.layout = Layout(self)
      init_fun(self, parent, *args, **kwargs)
      confirmLayout = QtWidgets.QHBoxLayout()
      okButton = QtWidgets.QPushButton("Ok", self)
      confirmLayout.addWidget(okButton)
      okButton.clicked.connect(self.accept)
      if apply_:
        applyButton = QtWidgets.QPushButton("Apply", self)
        confirmLayout.addWidget(applyButton)
        applyButton.clicked.connect(self.apply)
      cancelButton = QtWidgets.QPushButton("Cancel", self)
      confirmLayout.addWidget(cancelButton)
      cancelButton.clicked.connect(self.reject)
      if Layout is QtWidgets.QFormLayout:
        self.layout.addRow(confirmLayout)
      elif Layout is QtWidgets.QGridLayout:
        colSpan = 2
        if apply_:
          colSpan += 1
        self.layout.addLayout(confirmLayout, self.layout.rowCount(), 1, 1, colSpan)
      else:
        self.layout.addLayout(confirmLayout)
    return wrapped_init
  if fun:
    return wrap_init(fun)
  else:
    return wrap_init
