#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

import collections
import copy
import csv
import ctypes
import functools
import json
import numpy as np
import os
import re
import signal
import sys
import tempfile

from functools import partial

from PyQt5 import QtCore, QtGui, QtWidgets

from . import ui
from .mc_log_data import Data
from .mc_log_tab import MCLogTab
from .mc_log_types import LineStyle, TextWithFontSize, GraphLabels, ColorsSchemeConfiguration, PlotType
from .mc_log_utils import InitDialogWithOkCancel

try:
  import mc_rbdyn
except ImportError:
  mc_rbdyn = None

UserPlot = collections.namedtuple('UserPlot', ['title', 'x', 'y1', 'y1d', 'y2', 'y2d', 'grid1', 'grid2', 'style', 'style2', 'graph_labels', 'extra', 'type'])

def safe_float(v):
    if len(v):
        return float(v)
    else:
        return None

def read_flat(f, tmp = False):
    def read_size(fd):
        return ctypes.c_size_t.from_buffer_copy(fd.read(ctypes.sizeof(ctypes.c_size_t))).value
    def read_bool(fd):
        return ctypes.c_bool.from_buffer_copy(fd.read(ctypes.sizeof(ctypes.c_bool))).value
    def read_string(fd, size):
        if size == 0:
            return b"".decode('ascii')
        return fd.read(size).decode('ascii')
    def read_array(fd, size):
        return np.frombuffer(fd.read(size * ctypes.sizeof(ctypes.c_double)), np.double)
    def read_string_array(fd, size):
        return [ read_string(fd, read_size(fd)) for i in range(size) ]
    data = {}
    with open(f, 'rb') as fd:
        nrEntries = read_size(fd)
        for i in range(nrEntries):
            is_numeric = read_bool(fd)
            key = read_string(fd, read_size(fd))
            if is_numeric:
                data[key] = read_array(fd, read_size(fd))
            else:
                data[key] = read_string_array(fd, read_size(fd))
    if tmp:
      os.unlink(f)
    return data

def read_csv(fpath, tmp = False):
  data = {}
  string_entries = {}
  with open(fpath) as fd:
    reader = csv.DictReader(fd, delimiter=';')
    for k in reader.fieldnames:
      if not(len(k)):
        continue
      data[k] = []
    for row in reader:
      for k in reader.fieldnames:
        if not(len(k)):
          continue
        try:
          data[k].append(safe_float(row[k]))
        except ValueError:
          data[k].append(row[k].decode('ascii'))
  for k in data:
    if type(data[k][0]) is unicode:
      continue
    data[k] = np.array(data[k])
  if tmp:
    os.unlink(fpath)
  return data

def read_log(fpath, tmp = False):
  if fpath.endswith('.bin'):
    tmpf = tempfile.mkstemp(suffix = '.flat')[1]
    os.system("mc_bin_to_flat {} {}".format(fpath, tmpf))
    return read_log(tmpf, True)
  elif fpath.endswith('.flat'):
    return read_flat(fpath, tmp)
  else:
    return read_csv(fpath, tmp)

def load_UserPlots(fpath):
    if not os.path.exists(fpath):
      return []
    userPlotList = []
    with open(fpath) as f:
      userPlotList = [UserPlot(*x) for x in json.load(f)]
      for i,plt in enumerate(userPlotList):
        for y in plt.style:
          plt.style[y] = LineStyle(**plt.style[y])
        for y in plt.style2:
          plt.style2[y] = LineStyle(**plt.style2[y])
        if not isinstance(plt.graph_labels, GraphLabels):
          for key, value in plt.graph_labels.items():
            plt.graph_labels[key] = TextWithFontSize(**plt.graph_labels[key])
          userPlotList[i] = plt._replace(graph_labels = GraphLabels(**plt.graph_labels))
          plt = userPlotList[i]
        userPlotList[i] = plt._replace(type = PlotType(plt.type))
    return userPlotList

class RobotAction(QtWidgets.QAction):
  def __init__(self, display, parent):
    super(RobotAction, self).__init__(display, parent)
    self._actual = display
  def actual(self, n = None):
    if n is None:
      return self._actual
    else:
      self._actual = n

class CommonStyleDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel
  def __init__(self, parent, name, canvas, style):
    self.name = name
    self.canvas = canvas
    self.style = style

    self.linestyle = QtWidgets.QComboBox()
    styles = ['-', ':', '--', '-.']
    for s in styles:
      self.linestyle.addItem(s)
    self.linestyle.setCurrentIndex(styles.index(style.linestyle))
    self.layout.addRow("Style", self.linestyle)

    self.linewidth = QtWidgets.QLineEdit(str(style.linewidth))
    self.linewidth.setValidator(QtGui.QDoubleValidator(0.01, 1e6, 2))
    self.layout.addRow("Width", self.linewidth)

    self.color = QtGui.QColor(style.color)
    self.colorButton = QtWidgets.QPushButton("")
    self.colorButton.setStyleSheet("background-color: {}; color: {}".format(self.style.color, self.style.color))
    self.colorButton.released.connect(self.selectColor)
    self.layout.addRow("Color", self.colorButton)

  def selectColor(self):
    color = QtWidgets.QColorDialog.getColor(self.color, parent = self)
    if color.isValid():
      self.color = color
      self.colorButton.setStyleSheet("background-color: {}; color: {}".format(self.color.name(), self.color.name()))

  def apply(self):
    self.style.linestyle = self.linestyle.currentText()
    self.style.linewidth = float(self.linewidth.text())
    self.style.color = self.color.name()

  def accept(self):
    super(CommonStyleDialog, self).accept()
    self.apply()

class GridStyleDialog(CommonStyleDialog):
  def __init__(self, parent, name, canvas, style):
    super(GridStyleDialog, self).__init__(parent, name, canvas, style)
    self.setWindowTitle('Edit {} grid style'.format(name))

    self.enabled = QtWidgets.QCheckBox()
    self.enabled.setChecked(style.visible)
    self.layout.insertRow(0, "Visible", self.enabled)

    self.save = QtWidgets.QCheckBox()
    self.layout.insertRow(self.layout.rowCount() - 2, "Save as default", self.save)

  def apply(self):
    super(GridStyleDialog, self).apply()
    self.style.visible = self.enabled.isChecked()
    self.canvas.draw()
    if self.save.isChecked():
      self.parent().gridStyles[self.name] = self.style
      with open(self.parent().gridStyleFile, 'w') as f:
        json.dump(self.parent().gridStyles, f, default = lambda o: o.__dict__)

class LineStyleDialog(CommonStyleDialog):
  def __init__(self, parent, name, canvas, style, set_style_fn):
    super(LineStyleDialog, self).__init__(parent, name, canvas, style)
    self.set_style = set_style_fn

    self.setWindowTitle('Edit {} style'.format(style.label))

    self.labelInput = QtWidgets.QLineEdit(style.label)
    self.layout.insertRow(0, "Label", self.labelInput)

  def apply(self):
    super(LineStyleDialog, self).apply()
    self.style.label = self.labelInput.text()
    self.set_style(self.name, self.style)
    self.setWindowTitle('Edit {} style'.format(self.style.label))
    self.canvas.draw()

class ColorButtonRightClick(QtWidgets.QPushButton):
  rightClick = QtCore.pyqtSignal()
  def __init__(self, parent, color):
    super(ColorButtonRightClick, self).__init__("", parent)
    self.color = QtGui.QColor(color)
    self.setStyleSheet("background-color: {color}; color: {color}".format(color = color))
    self.released.connect(self.selectColor)
  def selectColor(self):
    color = QtWidgets.QColorDialog.getColor(self.color, parent = self)
    if color.isValid():
      self.color = color
      self.setStyleSheet("background-color: {color}; color: {color}".format(color = color.name()))
    self.parent().setCustom()
  def mouseReleaseEvent(self, event):
    if event.button() == QtCore.Qt.RightButton:
      self.parent().removeColorFromSelector(self)
    super(ColorButtonRightClick, self).mouseReleaseEvent(event)

class ColorsSchemeConfigurationDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QFormLayout, apply_ = False)
  def __init__(self, parent, scheme, apply_cb):
    self.scheme = copy.deepcopy(scheme)

    self.setSelector = QtWidgets.QComboBox(self)
    # Qualitative maps, see https://matplotlib.org/3.1.0/gallery/color/colormap_reference.html
    qualitative = ['Pastel1', 'Pastel2', 'Paired', 'Accent', 'Dark2', 'Set1', 'Set2', 'Set3', 'tab10', 'tab20', 'tab20b', 'tab20c']
    [ self.setSelector.addItem(s) for s in qualitative ]
    self.setSelector.addItem('custom')
    self.setSelector.setCurrentText(self.scheme.cm_)
    self.setSelector.currentIndexChanged.connect(self.cmChanged)
    self.layout.addRow("Colormap", self.setSelector)

    self.ncolorsSetter = QtWidgets.QSpinBox(self)
    self.ncolorsSetter.setMinimum(1)
    self.ncolorsSetter.setValue(self.scheme.ncolors_)
    self.ncolorsSetter.valueChanged.connect(self.ncolorsChanged)
    self.layout.addRow("Number of colors", self.ncolorsSetter)

    self.colorSelection = QtWidgets.QGridLayout()
    self.setupColorSelection()
    self.layout.addRow(self.colorSelection)

    self.apply_cb = apply_cb

  def cmChanged(self):
    cm = self.setSelector.currentText()
    if cm != 'custom':
      self.scheme._select_pyplot_set(cm, self.scheme.ncolors_)
      self.ncolorsSetter.setValue(self.scheme.ncolors_)
      self.setupColorSelection()

  def ncolorsChanged(self):
    ncolors = self.ncolorsSetter.value()
    prev = self.scheme.ncolors_
    if self.scheme.cm_ != 'custom':
      self.scheme._select_pyplot_set(self.scheme.cm_, ncolors)
      if self.scheme.ncolors_ != prev:
        self.setupColorSelection()
      if self.scheme.ncolors_ != ncolors:
        self.ncolorsSetter.setValue(self.scheme.ncolors_)
    else:
      if ncolors > prev:
        for i in range(prev, ncolors):
          self.scheme.colors_.append('#000000')
      else:
        self.scheme.colors_ = self.scheme.colors_[:ncolors]
      self.scheme.ncolors_ = len(self.scheme.colors_)
      self.setupColorSelection()


  def setCustom(self):
    self.scheme.cm_ = 'custom'
    self.setSelector.setCurrentText(self.scheme.cm_)

  def setupColorSelection(self):
    itm = self.colorSelection.takeAt(0)
    while itm:
      widget = itm.widget()
      widget.deleteLater()
      itm  = self.colorSelection.takeAt(0)
    ncol = 5
    col = 0
    row = 0
    for color in self.scheme.colors():
      self.colorSelection.addWidget(ColorButtonRightClick(self, color), row, col)
      col += 1
      if col == ncol:
        row += 1
        col = 0

  def removeColorFromSelector(self, btn):
    self.setCustom()
    colors = []
    itm = self.colorSelection.takeAt(0)
    while itm:
      if isinstance(itm.widget(), ColorButtonRightClick) and itm.widget() is not btn:
        colors.append(itm.widget().color.name())
      itm.widget().deleteLater()
      itm = self.colorSelection.takeAt(0)
    self.scheme._select_custom_set(colors)
    self.ncolorsSetter.setValue(self.scheme.ncolors_)
    self.setupColorSelection()

  def accept(self):
    super(ColorsSchemeConfigurationDialog, self).accept()
    if self.scheme.cm_ == 'custom':
      self.removeColorFromSelector(None)
    self.apply_cb(self.scheme)

class MCLogJointDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QVBoxLayout, apply_ = False)
  def __init__(self, parent, rm, name, y1_prefix = None, y2_prefix = None, y1_diff_prefix = None, y2_diff_prefix = None):
    self.setWindowTitle("Select plot joints")
    self.joints = []
    self.name = name
    self.y1_prefix = y1_prefix
    self.y2_prefix = y2_prefix
    self.y1_diff_prefix = y1_diff_prefix
    self.y2_diff_prefix = y2_diff_prefix

    jointsBox = QtWidgets.QGroupBox("Joints", self)
    grid = QtWidgets.QGridLayout(jointsBox)
    margins = grid.contentsMargins()
    margins.setTop(20)
    grid.setContentsMargins(margins)
    self.jointsCBox = []
    row = 0
    col = 0
    if rm is not None:
        for i, j in enumerate(rm.ref_joint_order()):
          cBox = QtWidgets.QCheckBox(j, self)
          cBox.stateChanged.connect(partial(self.checkboxChanged, j))
          grid.addWidget(cBox, row, col)
          self.jointsCBox.append(cBox)
          col += 1
          if col == 4:
            col = 0
            row += 1
    self.layout.addWidget(jointsBox)

    optionsBox = QtWidgets.QGroupBox("Options", self)
    optionsLayout = QtWidgets.QHBoxLayout(optionsBox)
    margins = optionsLayout.contentsMargins()
    margins.setTop(20)
    optionsLayout.setContentsMargins(margins)
    self.selectAllBox = QtWidgets.QCheckBox("Select all", self)
    self.selectAllBox.stateChanged.connect(self.selectAllBoxChanged)
    optionsLayout.addWidget(self.selectAllBox)
    self.onePlotPerJointBox = QtWidgets.QCheckBox("One plot per joint", self)
    optionsLayout.addWidget(self.onePlotPerJointBox)
    self.plotLimits = QtWidgets.QCheckBox("Plot limits", self)
    optionsLayout.addWidget(self.plotLimits)
    self.layout.addWidget(optionsBox)

  def accept(self):
    if len(self.joints):
      plotLimits = self.plotLimits.isChecked()
      if self.onePlotPerJointBox.isChecked():
        [ self.parent().plot_joint_data(self.name + ": {}".format(j), [j], self.y1_prefix, self.y2_prefix, self.y1_diff_prefix, self.y2_diff_prefix, plotLimits) for j in self.joints ]
      else:
        self.parent().plot_joint_data(self.name, self.joints, self.y1_prefix, self.y2_prefix, self.y1_diff_prefix, self.y2_diff_prefix, plotLimits)
    super(MCLogJointDialog, self).accept()

  def checkboxChanged(self, item, state):
    if state:
      self.joints.append(item)
    else:
      self.joints.remove(item)

  def selectAllBoxChanged(self, state):
    for cBox in self.jointsCBox:
      cBox.setChecked(state)
    if state:
      self.selectAllBox.setText("Select none")
    else:
      self.selectAllBox.setText("Select all")

class AllLineStyleDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QGridLayout)
  def __init__(self, parent, name, canvas, plots, style_fn):
    self.name = name
    self.canvas = canvas
    self.plots = plots
    self.style = style_fn

    self.setWindowTitle('Edit {} graph line style'.format(name))

    row = 0
    [ self.layout.addWidget(QtWidgets.QLabel(txt), row, i) for i,txt in enumerate(["Label", "Style", "Width", "Color"]) ]
    row += 1

    self.plotWidgets = {}

    def makeLineStyleComboBox(style):
      ret = QtWidgets.QComboBox()
      [ret.addItem(s) for s in ['-', ':', '--', '-.']]
      ret.setCurrentIndex(['-', ':', '--', '-.'].index(style.linestyle))
      return ret

    def makeLineWidthEdit(style):
      ret = QtWidgets.QLineEdit(str(style.linewidth))
      ret.setValidator(QtGui.QDoubleValidator(0.01, 1e6, 2))
      return ret

    def makeColorButton(self, style):
      ret = QtWidgets.QPushButton("")
      ret.color = QtGui.QColor(style.color)
      ret.setStyleSheet("background-color: {color}; color: {color}".format(color = style.color))
      ret.released.connect(lambda bt=ret: self.selectColor(bt))
      return ret

    def add_plot(self, plot, style):
      self.plotWidgets[plot] = [
        QtWidgets.QLineEdit(style.label),
        makeLineStyleComboBox(style),
        makeLineWidthEdit(style),
        makeColorButton(self, style)
      ]
      [ self.layout.addWidget(w, row, i) for i,w in enumerate(self.plotWidgets[plot]) ]

    for p in self.plots:
      add_plot(self, p, self.style(p))
      row += 1

  def selectColor(self, button):
    color = QtWidgets.QColorDialog.getColor(button.color, parent = self)
    if color.isValid():
      button.color = color
      button.setStyleSheet("background-color: {color}; color: {color}".format(color = color.name()))

  def apply(self):
    for y,widgets in self.plotWidgets.items():
      label = widgets[0].text()
      linestyle = widgets[1].currentText()
      linewidth = float(widgets[2].text())
      color = widgets[3].color.name()
      st = LineStyle(label = label, linestyle = linestyle, linewidth = linewidth, color = color)
      self.style(y, st)
    self.canvas.draw()

  def accept(self):
    super(AllLineStyleDialog, self).accept()
    self.apply()

class DumpSeqPlayDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QGridLayout, apply_ = False)
  def __init__(self, parent):
    self.setWindowTitle("Dump qOut to seqplay")

    row = 0

    row += 1
    self.layout.addWidget(QtWidgets.QLabel("Timestep"), row, 0)
    self.timestepLineEdit = QtWidgets.QLineEdit("0.005")
    validator = QtGui.QDoubleValidator()
    validator.setBottom(1e-6)
    self.timestepLineEdit.setValidator(validator)
    self.layout.addWidget(self.timestepLineEdit, row, 1)

    row += 1
    self.layout.addWidget(QtWidgets.QLabel("Time scale"), row, 0)
    self.timeScaleSpinBox = QtWidgets.QSpinBox()
    self.timeScaleSpinBox.setMinimum(1)
    self.timeScaleSpinBox.setPrefix("x")
    self.layout.addWidget(self.timeScaleSpinBox, row, 1)

    row += 1
    filedialogButton = QtWidgets.QPushButton("Browse...")
    filedialogButton.clicked.connect(self.filedialogButton)
    self.layout.addWidget(filedialogButton, row, 0)
    self.fileLineEdit = QtWidgets.QLineEdit("out.pos")
    self.layout.addWidget(self.fileLineEdit)

  def accept(self):
    fout = self.fileLineEdit.text()
    tScale = self.timeScaleSpinBox.value()
    dt = float(self.timestepLineEdit.text())
    rm = self.parent().rm
    data = self.parent().data
    if os.path.exists(fout):
      overwrite = QtWidgets.QMessageBox.question(self, "Overwrite existing file", "{} already exists, do you want to overwrite it?".format(fout), QtWidgets.QMessageBox.Yes, QtWidgets.QMessageBox.No)
      if overwrite == QtWidgets.QMessageBox.No:
        return
    with open(fout, 'w') as fd:
      rjo_range = range(len(rm.ref_joint_order()))
      i_range = range(len(data['t']))
      t = 0
      for i in i_range:
        q = np.array([data['qOut_{}'.format(jIdx)][i] for jIdx in rjo_range])
        if i == i_range[-1]:
          next_q = q
        else:
          next_q = np.array([data['qOut_{}'.format(jIdx)][i+1] for jIdx in rjo_range])
        for j in range(tScale):
          qOut = map(str, q + j/float(tScale) * (next_q - q))
          fd.write("{} {}\n".format(t, " ".join(qOut)))
          t += dt
    super(DumpSeqPlayDialog, self).accept()

  def filedialogButton(self):
    fpath = QtWidgets.QFileDialog.getSaveFileName(self, "Output file")[0]
    if len(fpath):
      self.fileLineEdit.setText(fpath)

class LabelsTitleEditDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QGridLayout)
  def __init__(self, parent, canvas):
    self.canvas = canvas

    self.setWindowTitle('Edit graph title, labels and style')

    row = 0

    self.titleEdit = QtWidgets.QLineEdit(canvas.title())
    self.titleFontsizeEdit = QtWidgets.QLineEdit(str(canvas.title_fontsize()))
    self.titleFontsizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.layout.addWidget(QtWidgets.QLabel("Title"), row, 0)
    self.layout.addWidget(self.titleEdit, row, 1)
    self.layout.addWidget(self.titleFontsizeEdit, row, 2)
    row += 1


    self.xLabelEdit = QtWidgets.QLineEdit(canvas.x_label())
    self.xLabelFontsizeEdit = QtWidgets.QLineEdit(str(canvas.x_label_fontsize()))
    self.xLabelFontsizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.layout.addWidget(QtWidgets.QLabel("X label"), row, 0)
    self.layout.addWidget(self.xLabelEdit, row, 1)
    self.layout.addWidget(self.xLabelFontsizeEdit, row, 2)
    row += 1

    self.y1LabelEdit = QtWidgets.QLineEdit(canvas.y1_label())
    self.y1LabelFontsizeEdit = QtWidgets.QLineEdit(str(canvas.y1_label_fontsize()))
    self.y1LabelFontsizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.layout.addWidget(QtWidgets.QLabel("Y1 label"), row, 0)
    self.layout.addWidget(self.y1LabelEdit, row, 1)
    self.layout.addWidget(self.y1LabelFontsizeEdit, row, 2)
    row += 1

    self.y2LabelEdit = QtWidgets.QLineEdit(canvas.y2_label())
    self.y2LabelFontsizeEdit = QtWidgets.QLineEdit(str(canvas.y2_label_fontsize()))
    self.y2LabelFontsizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    if canvas._3D:
      self.layout.addWidget(QtWidgets.QLabel("Z label"), row, 0)
    else:
      self.layout.addWidget(QtWidgets.QLabel("Y2 label"), row, 0)
    self.layout.addWidget(self.y2LabelEdit, row, 1)
    self.layout.addWidget(self.y2LabelFontsizeEdit, row, 2)
    row += 1

    self.extraLayout = QtWidgets.QGridLayout()
    extraRow = 0

    self.extraLayout.addWidget(QtWidgets.QLabel("Tick size"), extraRow, 0)
    self.extraLayout.addWidget(QtWidgets.QLabel("Label padding"), extraRow, 1)
    self.extraLayout.addWidget(QtWidgets.QLabel("Top offset"), extraRow, 2)
    self.extraLayout.addWidget(QtWidgets.QLabel("Bottom offset"), extraRow, 3)
    extraRow += 1

    self.tickSizeEdit = QtWidgets.QLineEdit(str(canvas.tick_fontsize()))
    self.tickSizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.labelPaddingEdit = QtWidgets.QLineEdit(str(canvas.labelpad()))
    self.labelPaddingEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.topOffsetEdit = QtWidgets.QLineEdit(str(canvas.top_offset()))
    self.topOffsetEdit.setValidator(QtGui.QDoubleValidator(0, 1, 3))
    self.bottomOffsetEdit = QtWidgets.QLineEdit(str(canvas.bottom_offset()))
    self.bottomOffsetEdit.setValidator(QtGui.QDoubleValidator(0, 1, 3))
    self.extraLayout.addWidget(self.tickSizeEdit, extraRow, 0)
    self.extraLayout.addWidget(self.labelPaddingEdit, extraRow, 1)
    self.extraLayout.addWidget(self.topOffsetEdit, extraRow, 2)
    self.extraLayout.addWidget(self.bottomOffsetEdit, extraRow, 3)
    extraRow += 1

    self.extraLayout.addWidget(QtWidgets.QLabel("Legend size"), extraRow, 0)
    self.extraLayout.addWidget(QtWidgets.QLabel("Legend Y1 columns"), extraRow, 1)
    self.extraLayout.addWidget(QtWidgets.QLabel("Legend Y2 columns"), extraRow, 2)
    extraRow += 1

    self.legendSizeEdit = QtWidgets.QLineEdit(str(canvas.legend_fontsize()))
    self.legendSizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.y1LegendNColEdit = QtWidgets.QLineEdit(str(canvas.y1_legend_ncol()))
    self.y1LegendNColEdit.setValidator(QtGui.QIntValidator(1, 100))
    self.y2LegendNColEdit = QtWidgets.QLineEdit(str(canvas.y2_legend_ncol()))
    self.y2LegendNColEdit.setValidator(QtGui.QIntValidator(1, 100))
    self.extraLayout.addWidget(self.legendSizeEdit, extraRow, 0)
    self.extraLayout.addWidget(self.y1LegendNColEdit, extraRow, 1)
    self.extraLayout.addWidget(self.y2LegendNColEdit, extraRow, 2)
    extraRow += 1

    self.extraLayout.addWidget(QtWidgets.QLabel("Labels legend size"), extraRow, 0, 1, 2)
    self.extraLayout.addWidget(QtWidgets.QLabel("Labels legend top offset"), extraRow, 2, 1, 1)
    extraRow += 1

    self.labelsLegendSizeEdit = QtWidgets.QLineEdit(str(canvas.labels_legend_fontsize()))
    self.labelsLegendSizeEdit.setValidator(QtGui.QDoubleValidator(1, 1e6, 1))
    self.labelsLegendTopOffsetEdit = QtWidgets.QLineEdit(str(canvas.labels_legend_top_offset()))
    self.extraLayout.addWidget(self.labelsLegendSizeEdit, extraRow, 0, 1, 2)
    self.extraLayout.addWidget(self.labelsLegendTopOffsetEdit, extraRow, 2, 1, 2)
    extraRow += 1

    self.layout.addLayout(self.extraLayout, row, 0, extraRow, 3)
    row += extraRow

  def apply(self):
    self.canvas.title(self.titleEdit.text())
    self.canvas.title_fontsize(float(self.titleFontsizeEdit.text()))
    self.canvas.x_label(self.xLabelEdit.text())
    self.canvas.x_label_fontsize(self.xLabelFontsizeEdit.text())
    self.canvas.y1_label(self.y1LabelEdit.text())
    self.canvas.y1_label_fontsize(self.y1LabelFontsizeEdit.text())
    self.canvas.y2_label(self.y2LabelEdit.text())
    self.canvas.y2_label_fontsize(self.y2LabelFontsizeEdit.text())
    self.canvas.tick_fontsize(float(self.tickSizeEdit.text()))
    self.canvas.legend_fontsize(float(self.legendSizeEdit.text()))
    self.canvas.labelpad(float(self.labelPaddingEdit.text()))
    self.canvas.top_offset(float(self.topOffsetEdit.text()))
    self.canvas.bottom_offset(float(self.bottomOffsetEdit.text()))
    self.canvas.y1_legend_ncol(int(self.y1LegendNColEdit.text()))
    self.canvas.y2_legend_ncol(int(self.y2LegendNColEdit.text()))
    self.canvas.labels_legend_fontsize(int(self.labelsLegendSizeEdit.text()))
    self.canvas.labels_legend_top_offset(float(self.labelsLegendTopOffsetEdit.text()))
    self.canvas.draw()

  def accept(self):
    super(LabelsTitleEditDialog, self).accept()
    self.apply()

class MCLogUI(QtWidgets.QMainWindow):
  def __init__(self, parent = None):
    super(MCLogUI, self).__init__(parent)
    self.__init__ui = ui.MainWindow()
    self.ui = ui.MainWindow()

    self.ui.setupUi(self)

    self.tab_re = re.compile('^Plot [0-9]+$')

    self.data = Data()
    self.data.data_updated.connect(self.update_data)

    self.loaded_files = []

    self.gridStyles = {'left': LineStyle(linestyle = '--'), 'right': LineStyle(linestyle = ':') }
    self.gridStyleFile = os.path.expanduser("~") + "/.config/mc_log_ui/grid_style.json"
    if os.path.exists(self.gridStyleFile):
      with open(self.gridStyleFile) as f:
        data = json.load(f)
        for k in self.gridStyles.keys():
          if k in data:
            self.gridStyles[k] = LineStyle(**data[k])
    UserPlot.__new__.__defaults__ = (self.gridStyles['left'], self.gridStyles['right'], {}, {}, GraphLabels(), {}, PlotType(0))

    self.robotFile = os.path.expanduser("~") + "/.config/mc_log_ui/robot"
    self.userPlotFile = os.path.join(os.path.dirname(__file__), "custom_plot.json")
    self.userPlotList = load_UserPlots(self.userPlotFile)
    self.update_userplot_menu()

    self.colorsFile = os.path.expanduser("~") + "/.config/mc_log_ui/colors.json"
    self.colorsScheme = ColorsSchemeConfiguration(self.colorsFile)

    self.polyColorsFile = os.path.expanduser("~") + "/.config/mc_log_ui/poly_colors.json"
    self.polyColorsScheme = ColorsSchemeConfiguration(self.polyColorsFile, 'Pastel1')

    self.activeRobotAction = None
    self.rm = None
    if mc_rbdyn is not None:
      rMenu = QtWidgets.QMenu("Robot", self.ui.menubar)
      rGroup = QtWidgets.QActionGroup(rMenu)
      rCategoryMenu = {}
      rActions = []
      for r in mc_rbdyn.RobotLoader.available_robots():
        rAct = RobotAction(r, rGroup)
        rAct.setCheckable(True)
        rGroup.addAction(rAct)
        if '/' in r:
          category, name = r.split('/', 1)
          if not category in rCategoryMenu:
            rCategoryMenu[category] = rMenu.addMenu(category)
          rAct.setText(name)
          rAct.actual(r)
          rCategoryMenu[category].addAction(rAct)
        else:
          rActions.append(rAct)
      rMenu.addActions(rActions)
      defaultBot = self.getDefaultRobot()
      if defaultBot in mc_rbdyn.RobotLoader.available_robots():
        actionIndex = mc_rbdyn.RobotLoader.available_robots().index(defaultBot)
        defaultBot = rGroup.actions()[actionIndex]
      else:
        defaultBot = rGroup.actions()[0]
      self.activeRobotAction = defaultBot
      self.activeRobotAction.setChecked(True)
      self.setRobot(self.activeRobotAction)
      rGroup.triggered.connect(self.setRobot)
      self.ui.menubar.addMenu(rMenu)

    self.styleMenu = QtWidgets.QMenu("Style", self.ui.menubar)

    # Line style menu
    self.lineStyleMenu = QtWidgets.QMenu("Graph", self.styleMenu)
    def fillLineStyleMenu(self):
      self.lineStyleMenu.clear()
      tab = self.ui.tabWidget.currentWidget()
      canvas = self.getCanvas()
      def makePlotMenu(self, name, plots, style_fn):
        if not len(plots):
          return
        menu = QtWidgets.QMenu(name, self.lineStyleMenu)
        group = QtWidgets.QActionGroup(act)
        action = QtWidgets.QAction("All", group)
        action.triggered.connect(lambda: AllLineStyleDialog(self, name, self.getCanvas(), plots, style_fn).exec_())
        group.addAction(action)
        sep = QtWidgets.QAction(group)
        sep.setSeparator(True)
        group.addAction(sep)
        for y in plots:
          style = style_fn(y)
          action = QtWidgets.QAction(style.label, group)
          action.triggered.connect(lambda checked, yin=y, stylein=style: LineStyleDialog(self, yin, self.getCanvas(), stylein, style_fn).exec_())
          group.addAction(action)
        menu.addActions(group.actions())
        self.lineStyleMenu.addMenu(menu)
      makePlotMenu(self, "Left", canvas._left().plots.keys(), tab.style_left)
      if canvas._right():
        makePlotMenu(self, "Right", canvas._right().plots.keys(), tab.style_right)
    self.lineStyleMenu.aboutToShow.connect(lambda: fillLineStyleMenu(self))
    self.styleMenu.addMenu(self.lineStyleMenu)

    # Grid style menu
    self.gridStyleMenu = QtWidgets.QMenu("Grid", self.styleMenu)
    self.gridDisplayActionGroup = QtWidgets.QActionGroup(self.gridStyleMenu)
    self.gridDisplayActionGroup.setExclusive(True)
    self.leftGridAction = QtWidgets.QAction("Left", self.gridDisplayActionGroup)
    self.leftGridAction.triggered.connect(lambda: GridStyleDialog(self, "left", self.getCanvas(), self.getCanvas()._left().grid).exec_())
    self.gridDisplayActionGroup.addAction(self.leftGridAction)
    self.rightGridAction = QtWidgets.QAction("Right", self.gridDisplayActionGroup)
    self.rightGridAction.triggered.connect(lambda: GridStyleDialog(self, "right", self.getCanvas(), self.getCanvas()._right().grid).exec_())
    self.gridDisplayActionGroup.addAction(self.rightGridAction)
    self.gridStyleMenu.addActions(self.gridDisplayActionGroup.actions())
    self.styleMenu.addMenu(self.gridStyleMenu)

    # Labels
    self.titleAction = QtWidgets.QAction("Labels/Title/Fonts", self.styleMenu)
    self.titleAction.triggered.connect(lambda: LabelsTitleEditDialog(self, self.getCanvas()).exec_())
    self.styleMenu.addAction(self.titleAction)

    # Color scheme selector
    self.colorSchemeAction = QtWidgets.QAction("Colors selection", self.styleMenu)
    self.colorSchemeAction.triggered.connect(lambda: ColorsSchemeConfigurationDialog(self, self.colorsScheme, self.setColorsScheme).exec_())
    self.styleMenu.addAction(self.colorSchemeAction)

    # Polygon color scheme selector
    self.polyColorSchemeAction = QtWidgets.QAction("Polygons colors selection", self.styleMenu)
    self.polyColorSchemeAction.triggered.connect(lambda: ColorsSchemeConfigurationDialog(self, self.polyColorsScheme, self.setPolyColorsScheme).exec_())
    self.styleMenu.addAction(self.polyColorSchemeAction)

    self.ui.menubar.addMenu(self.styleMenu)

    self.toolsMenu = QtWidgets.QMenu("Tools", self.ui.menubar)
    act = QtWidgets.QAction("Dump qOut to seqplay", self.toolsMenu)
    act.triggered.connect(DumpSeqPlayDialog(self).exec_)
    self.toolsMenu.addAction(act)
    self.ui.menubar.addMenu(self.toolsMenu)

    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_O, self.shortcutOpenFile)
    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_W, self.shortcutCloseTab)
    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_PageDown, self.shortcutNextTab)
    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_PageUp, self.shortcutPreviousTab)
    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_T, self.shortcutNewTab)
    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_S, self.save_userplot)
    self.addApplicationShortcut(QtCore.Qt.CTRL + QtCore.Qt.Key_A, self.shortcutAxesDialog)

  def saveUserPlots(self):
    confDir = os.path.dirname(self.userPlotFile)
    if not os.path.exists(confDir):
      os.makedirs(confDir)
    def default_(o):
      if isinstance(o, PlotType):
        return o.value
      else:
        return o.__dict__
    with open(self.userPlotFile, 'w') as f:
        json.dump(self.userPlotList, f, default = default_, indent = 2, separators = (',', ': '))
    self.update_userplot_menu()

  def saveDefaultRobot(self, name):
    confDir = os.path.dirname(self.robotFile)
    if not os.path.exists(confDir):
      os.makedirs(confDir)
    with open(self.robotFile, 'w') as f:
      f.write("{}".format(name))

  def getDefaultRobot(self):
    if os.path.exists(self.robotFile):
      return open(self.robotFile).read().strip()
    else:
      return u""

  def addApplicationShortcut(self, key, callback):
    shortcut = QtWidgets.QShortcut(self)
    shortcut.setKey(key)
    shortcut.setContext(QtCore.Qt.ApplicationShortcut)
    shortcut.activated.connect(lambda: callback())

  def update_userplot_menu(self):
    self.ui.menuUserPlots.clear()
    for p in self.userPlotList:
      act = QtWidgets.QAction(p.title, self.ui.menuUserPlots)
      act.triggered.connect(lambda checked, plot=p: self.plot_userplot(plot))
      self.ui.menuUserPlots.addAction(act)
    act = QtWidgets.QAction("Save current plot", self.ui.menuUserPlots)
    act.triggered.connect(self.save_userplot)
    self.ui.menuUserPlots.addAction(act)
    if len(self.userPlotList):
      rmUserPlotMenu = QtWidgets.QMenu("Remove saved plots", self.ui.menuUserPlots)
      for p in self.userPlotList:
        act = QtWidgets.QAction(p.title, self.ui.menuUserPlots)
        act.triggered.connect(lambda checked, plot=p: self.remove_userplot(plot))
        rmUserPlotMenu.addAction(act)
      self.ui.menuUserPlots.addMenu(rmUserPlotMenu)

  def save_userplot(self):
    tab = self.ui.tabWidget.currentWidget()
    canvas = tab.activeCanvas
    valid = len(canvas._left()) != 0 or len(canvas._right()) != 0
    if not valid:
      err_diag = QtWidgets.QMessageBox(self)
      err_diag.setModal(True)
      err_diag.setText("Cannot save user plot if nothing is shown")
      err_diag.exec_()
      return
    defaultTitle = self.ui.tabWidget.tabText(self.ui.tabWidget.currentIndex())
    if defaultTitle.startswith("Plot"):
      defaultTitle = ""
    title, ok = QtWidgets.QInputDialog.getText(self, "User plot", "Title of your plot:", text = defaultTitle)
    if ok:
      type_ = tab.plotType()
      if type_ is PlotType.TIME:
        y1 = filter(lambda k: k in self.data.keys(), canvas._left().plots.keys())
        y2 = filter(lambda k: k in self.data.keys(), canvas._right().plots.keys())
        y1d = map(lambda sp: "{}_{}".format(sp.name, sp.id), filter(lambda sp: sp.idx == 0, tab.specials.values()))
        y2d = map(lambda sp: "{}_{}".format(sp.name, sp.id), filter(lambda sp: sp.idx == 1, tab.specials.values()))
      else:
        y1 = canvas._left().source.values()
        if canvas._right():
          y2 = canvas._right().source.values()
        else:
          y2 = []
        y1d = []
        y2d = []
      style = { y: canvas.style_left(y) for y in canvas._left().plots.keys() }
      if canvas._right():
        style2 = { y: canvas.style_right(y) for y in canvas._right().plots.keys() }
      else:
        style2 = {}
      grid = canvas._left().grid
      if canvas._right():
        grid2 = canvas._right().grid
      else:
        grid2 = grid
      found = False
      extra = { p: getattr(self.getCanvas(), p)() for p in ["tick_fontsize", "legend_fontsize", "labelpad", "top_offset", "bottom_offset", "y1_legend_ncol", "y2_legend_ncol"] }
      up = UserPlot(title, tab.x_data, y1, y1d, y2, y2d, grid, grid2, style, style2, GraphLabels(title = TextWithFontSize(canvas.title(), canvas.title_fontsize()), x_label = TextWithFontSize(canvas.x_label(), canvas.x_label_fontsize()), y1_label = TextWithFontSize(canvas.y1_label(), canvas.y1_label_fontsize()), y2_label = TextWithFontSize(canvas.y2_label(), canvas.y2_label_fontsize())), extra, type_)
      for i in range(len(self.userPlotList)):
        if self.userPlotList[i].title == title:
          self.userPlotList[i] = up
          found = True
          break
      if not found:
        self.userPlotList.append(up)
      self.saveUserPlots()

  def plot_userplot(self, p):
    if p.type is PlotType.TIME:
      valid = p.x in self.data.keys() and all([y in self.data.keys() for x in [p.y1, p.y2] for y in x])
    elif p.type is PlotType.XY:
      valid = p.x in self.data.keys() and all([x in self.data.keys() and y in self.data.keys() for x,y,l in p.y1 + p.y2])
    else:
      valid = p.x in self.data.keys() and all([x in self.data.keys() and y in self.data.keys() and z in self.data.keys() for x,y,z,l in p.y1 + p.y2])
    if not valid:
      missing_entries = ""
      if not p.x in self.data.keys():
        missing_entries += "- {}\n".format(p.x)
      for x in [p.y1, p.y1d, p.y2, p.y2d]:
        for y in x:
          if not y in self.data.keys():
            missing_entries += "- {}\n".format(y)
      missing_entries = missing_entries[:-1]
      err_diag = QtWidgets.QMessageBox(self)
      err_diag.setModal(True)
      err_diag.setText("Plot {} is not valid for this log file, some data is missing\nMissing entries:\n{}".format(p.title, missing_entries))
      err_diag.exec_()
      return
    plotW = MCLogTab.UserPlot(self, p)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, p.title)
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
    self.updateClosable()

  def remove_userplot(self, p_in):
    for p in self.userPlotList:
      if p.title == p_in.title:
        self.userPlotList.remove(p)
        break
    self.saveUserPlots()

  def setRobot(self, action):
    try:
      self.rm = mc_rbdyn.RobotLoader.get_robot_module(action.actual())
      self.activeRobotAction = action
      for i in range(self.ui.tabWidget.count() - 1):
        tab = self.ui.tabWidget.widget(i)
        assert(isinstance(tab, MCLogTab))
        tab.setRobotModule(self.rm, self.loaded_files)
      self.saveDefaultRobot(action.actual())
    except RuntimeError:
      #QtWidgets.QMessageBox.warning(self, "Failed to get RobotModule", "Could not retrieve Robot Module: {}{}Check your console for more details".format(action.text(), os.linesep))
      action.setChecked(False)
      self.activeRobotAction.setChecked(True)
      self.rm = None

  def getCanvas(self):
    return self.ui.tabWidget.currentWidget().activeCanvas

  @QtCore.Slot()
  def on_actionLoad_triggered(self):
    fpath = QtWidgets.QFileDialog.getOpenFileName(self, "Log file")[0]
    if len(fpath):
      self.load_csv(fpath)

  @QtCore.Slot()
  def on_actionCompare_triggered(self):
    fpath = QtWidgets.QFileDialog.getOpenFileName(self, "Log file")[0]
    if len(fpath):
      self.load_csv(fpath, False)

  @QtCore.Slot()
  def on_actionExit_triggered(self):
    QtWidgets.QApplication.quit()

  @QtCore.Slot(int)
  def on_tabWidget_currentChanged(self, idx):
    if idx == self.ui.tabWidget.count() - 1:
      plotW = MCLogTab(self)
      plotW.setData(self.data)
      plotW.setGridStyles(self.gridStyles)
      plotW.setRobotModule(self.rm, self.loaded_files)
      plotW.setColors(self.colorsScheme.colors())
      plotW.setPolyColors(self.polyColorsScheme.colors())
      j = 1
      for i in range(self.ui.tabWidget.count() -1):
        if self.tab_re.match(self.ui.tabWidget.tabText(i)):
          j += 1
      self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, "Plot {}".format(j))
      self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
      self.updateClosable()

  @QtCore.Slot(int)
  def on_tabWidget_tabCloseRequested(self, idx):
    if self.ui.tabWidget.currentIndex() == idx:
      self.ui.tabWidget.setCurrentIndex(abs(idx - 1))
    self.ui.tabWidget.removeTab(idx)
    j = 1
    for i in range(self.ui.tabWidget.count() - 1):
      if self.tab_re.match(self.ui.tabWidget.tabText(i)):
        self.ui.tabWidget.setTabText(i, "Plot {}".format(j))
        j += 1
    self.updateClosable()

  def updateClosable(self):
    has_closable = self.ui.tabWidget.count() > 2
    self.ui.tabWidget.setTabsClosable(has_closable)
    if has_closable:
      self.ui.tabWidget.tabBar().tabButton(self.ui.tabWidget.count() - 1, QtWidgets.QTabBar.RightSide).hide();

  def shortcutOpenFile(self):
    self.ui.actionLoad.triggered.emit()

  def shortcutCloseTab(self):
    if self.ui.tabWidget.tabsClosable():
      self.ui.tabWidget.tabCloseRequested.emit(self.ui.tabWidget.currentIndex())

  def shortcutPreviousTab(self):
    if self.ui.tabWidget.currentIndex() > 0:
      self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.currentIndex() - 1)

  def shortcutNextTab(self):
    if self.ui.tabWidget.currentIndex() < self.ui.tabWidget.count() - 2:
      self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.currentIndex() + 1)

  def shortcutNewTab(self):
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 1)

  def shortcutAxesDialog(self):
    self.ui.tabWidget.currentWidget().activeCanvas.axesDialog()

  def load_csv(self, fpath, clear = True):
    if clear:
      self.loaded_files = []
    data = read_log(fpath)
    if not 't' in data:
      print("This GUI assumes a time-entry named t is available in the log, failed loading {}".format(fpath))
      return
    if 't' in self.data and not clear:
      dt = self.data['t'][1] - self.data['t'][0]
      ndt = data['t'][1] - data['t'][0]
      if abs(dt - ndt) > 1e-9:
        print("This GUI assumes you are comparing logs with a similar timestep, already loaded dt = {} but attempted to load dt = {} from {}", dt, ndt, fpath)
        return
      pad_left = int(round((self.data['t'][0] - data['t'][0]) / dt))
      pad_right = int(round((data['t'][-1] - self.data['t'][-1]) / dt))
      start_t = min(self.data['t'][0], data['t'][0])
      end_t = max(self.data['t'][-1], data['t'][-1])
    fpath = os.path.basename(fpath).replace('_', '-')
    self.loaded_files.append(fpath)
    i = 0
    while "qIn_{}".format(i) in data and "qOut_{}".format(i) in data:
      data["error_q_{}".format(i)] = data["qOut_{}".format(i)] - data["qIn_{}".format(i)]
      data["qIn_limits_lower_{}".format(i)] = np.full_like(data["qIn_{}".format(i)], 0)
      data["qIn_limits_upper_{}".format(i)] = np.full_like(data["qIn_{}".format(i)], 0)
      data["qOut_limits_lower_{}".format(i)] = data["qIn_limits_lower_{}".format(i)]
      data["qOut_limits_upper_{}".format(i)] = data["qIn_limits_upper_{}".format(i)]
      i += 1
    i = 0
    while "tauIn_{}".format(i) in data:
      data["tauIn_limits_lower_{}".format(i)] = np.full_like(data["tauIn_{}".format(i)], 0)
      data["tauIn_limits_upper_{}".format(i)] = np.full_like(data["tauIn_{}".format(i)], 0)
      i += 1
    while "tauOut_{}".format(i) in data:
      data["tauOut_limits_lower_{}".format(i)] = np.full_like(data["tauOut_{}".format(i)], 0)
      data["tauOut_limits_upper_{}".format(i)] = np.full_like(data["tauOut_{}".format(i)], 0)
      i += 1
    if 'perf_SolverBuildAndSolve' in data and 'perf_SolverSolve' in data:
      data['perf_SolverBuild'] = data['perf_SolverBuildAndSolve'] - data['perf_SolverSolve']
    if len(self.loaded_files) > 1:
      if len(self.loaded_files) == 2:
        keys = self.data.keys()
        for k in keys:
          self.data["{}_{}".format(self.loaded_files[0], k)] = self.data[k]
          del self.data[k]
      def pos_or_zero(i):
        if i > 0:
          return i
        else:
          return 0
      if pad_left > 0 or pad_right > 0:
        pleft = pos_or_zero(pad_left)
        pright = pos_or_zero(pad_right)
        self.data.data = {k: np.concatenate(([float('nan')]*pleft, v, [float('nan')]*pright)) for k,v in self.data.data.items()}
      keys = data.keys()
      for k in keys:
        def abs_or_zero(i):
          if i < 0:
              return -i
          else:
              return 0
        pleft = abs_or_zero(pad_left)
        pright = abs_or_zero(pad_right)
        self.data["{}_{}".format(fpath, k)] = np.concatenate(([float('nan')]*pleft, data[k], [float('nan')]*pright))
      self.data['t'] = np.arange(start_t, end_t, dt)
      # In some cases rounding errors gives us the wrong size so we use the other log timestep
      if len(self.data['t']) != len(self.data['{}_{}'.format(fpath, k)]):
        self.data['t'] = np.arange(start_t, end_t, ndt)
    else:
      self.data.data = data
    self.update_data()
    self.setWindowTitle("MC Log Plotter - {}".format("/".join(self.loaded_files)))

  def setColorsScheme(self, scheme):
    self.colorsScheme = scheme
    for i in range(self.ui.tabWidget.count() - 1):
      self.ui.tabWidget.widget(i).setColors(self.colorsScheme.colors())
    self.colorsScheme.save(self.colorsFile)

  def setPolyColorsScheme(self, scheme):
    self.polyColorsScheme = scheme
    for i in range(self.ui.tabWidget.count() - 1):
      self.ui.tabWidget.widget(i).setPolyColors(self.polyColorsScheme.colors())
    self.polyColorsScheme.save(self.polyColorsFile)

  def update_data(self):
    self.update_menu()
    for i in range(self.ui.tabWidget.count() - 1):
      tab = self.ui.tabWidget.widget(i)
      assert(isinstance(tab, MCLogTab))
      tab.setData(self.data)
      tab.setGridStyles(self.gridStyles)
      tab.setRobotModule(self.rm, self.loaded_files)
      tab.setColors(self.colorsScheme.colors())
      tab.setPolyColors(self.polyColorsScheme.colors())

  def update_menu(self):
    self.ui.menuCommonPlots.clear()
    menuEntries = [
        ("Encoders", "qIn", None, None, None),
        ("Commands", "qOut", None, None, None),
        ("Error", "error_q", None, None, None),
        ("Sensor torques", "tauIn", None, None, None),
        ("Command torques", "tauOut", None, None, None),
        ("Sensor/Command torques", "tauIn", "tauOut", None, None),
        ("Encoders/Commands", "qIn", "qOut", None, None),
        ("Error/Torque", "error_q", "tauIn", None, None),
        ("Encoders velocity", None, None, "qIn", None),
        ("Command velocity", None, None, "qOut", None),
        ("Encoders velocity/Commands velocity", None, None, "qIn", "qOut"),
        ("Encoders/Encoders velocity", "qIn", None, None, "qIn"),
        ("Command/Command velocity", "qOut", None, None, "qOut"),
        ]
    def validEntry(y):
      return any([ y is None or (y is not None and k.startswith(y)) for k in self.data.keys() ])
    menuEntries = [ (n, y1, y2, y1d, y2d) for n, y1, y2, y1d, y2d in menuEntries if all([validEntry(y) for y in [y1, y2, y1d, y2d]]) ]
    for n, y1, y2, y1d, y2d in menuEntries:
      act = QtWidgets.QAction(n, self.ui.menuCommonPlots)
      act.triggered.connect(lambda checked, n_=n, y1_=y1, y2_=y2, y1d_=y1d, y2d_=y2d: MCLogJointDialog(self, self.rm, n_, y1_, y2_, y1d_, y2d_).exec_())
      self.ui.menuCommonPlots.addAction(act)
    fSensors = set()
    for k in self.data:
      if k.find('ForceSensor') != -1:
        fSensors.add(k[:k.find('ForceSensor')])
    if len(fSensors):
      fsMenu = QtWidgets.QMenu("Force sensors", self.ui.menuCommonPlots)
      for f in sorted(fSensors):
        act = QtWidgets.QAction(f, fsMenu)
        act.triggered.connect(partial(self.plot_force_sensor, f))
        fsMenu.addAction(act)
      self.ui.menuCommonPlots.addMenu(fsMenu)

  def plot_force_sensor(self, fs):
    plotW = MCLogTab.ForceSensorPlot(self, fs)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, "FS: {}".format(fs))
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
    self.updateClosable()

  def plot_joint_data(self, name, joints, y1_prefix = None, y2_prefix = None,
                                    y1_diff_prefix = None, y2_diff_prefix = None, plot_limits = False):
    plotW = MCLogTab.JointPlot(self, joints, y1_prefix, y2_prefix, y1_diff_prefix, y2_diff_prefix, plot_limits)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, name)
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
    self.updateClosable()
