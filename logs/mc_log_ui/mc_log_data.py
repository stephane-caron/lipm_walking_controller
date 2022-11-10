#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtGui, QtWidgets

class Data(QtCore.QObject):
  data_updated = QtCore.pyqtSignal()
  def __init__(self, data = {}):
    QtCore.QObject.__init__(self)
    self.data = data
  def notify_update(self):
    self.data_updated.emit()
  def __getitem__(self, key):
    return self.data.__getitem__(key)
  def __setitem__(self, key, value):
    self.data.__setitem__(key, value)
  def __len__(self):
    return self.data.__len__()
  def __delitem__(self, key):
    return self.data.__delitem__(key)
  def __contains__(self, key):
    return self.data.__contains__(key)
  def __iter__(self):
    return self.data.__iter__()
  def __next__(self):
    return self.data.__next__()
  def items(self):
    return self.data.items()
  def values(self):
    return self.data.values()
  def keys(self):
    return self.data.keys()
  def __repr__(self):
    return self.data.__repr__()
