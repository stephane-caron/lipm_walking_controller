# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtWidgets

try:
  from . import ui
except ImportError:
  import ui

from .mc_log_plotcanvas import PlotFigure, PlotCanvasWithToolbar
from .mc_log_types import LineStyle, PlotType
from .mc_log_utils import InitDialogWithOkCancel

from functools import partial

import copy
import re

class MCLogTreeWidgetItem(QtWidgets.QTreeWidgetItem):
  def __init__(self, parent, displayText, actualText, hasData):
    super(MCLogTreeWidgetItem, self).__init__(parent, [displayText])
    self._displayText = displayText
    self.originalText = displayText
    self.actualText = actualText
    self.hasData = hasData
  @property
  def displayText(self):
    return self._displayText
  @displayText.setter
  def displayText(self, value):
    self._displayText = value
    self.setText(0, self._displayText)
  @property
  def legendText(self):
    return self.actualText.replace(self.originalText, self.displayText)

class TreeView(object):
  def __init__(self, name = None, parent = None, dataName = None):
    self.name = name
    if dataName is None:
      if parent is not None and parent.dataName is not None:
        self.dataName = parent.dataName + "_" + self.name
      else:
        self.dataName = self.name
    else:
      self.dataName = dataName
    self.hasData = False
    self.leafs = []
    self.parent = parent
    self.modelIdxs = []
    self.widgets = []
  def leaf(self, name):
    for l in self.leafs:
      if l.name == name:
        return l
    self.leafs.append(TreeView(name, self))
    return self.leafs[-1]
  def add(self, key):
    if len(key) == 0:
      self.hasData = True
      return
    self.leaf(key[0]).add(key[1:])
  def simplify(self):
    while len(self.leafs) == 1 and not(self.hasData):
      self.name = self.name + '_' + self.leafs[0].name
      self.dataName = self.leafs[0].dataName
      self.hasData = self.leafs[0].hasData
      self.leafs = self.leafs[0].leafs
      for l in self.leafs:
        l.parent = self
    for l in self.leafs:
      l.simplify()
  def update_y_selector(self, ySelector, parent, baseModelIdx = None):
    row = 0
    needExpand = False
    if all([l.name.isdigit() for l in self.leafs]):
      self.leafs.sort(key = lambda x: x.name)
    for l in self.leafs:
      l.widgets.append(MCLogTreeWidgetItem(parent, l.name, l.dataName, l.hasData))
      if baseModelIdx is not None:
        l.modelIdxs.append(ySelector.model().index(row, 0, baseModelIdx))
      else:
        l.modelIdxs.append(ySelector.model().index(row, 0))
      l.update_y_selector(ySelector, l.widgets[-1], l.modelIdxs[-1])
      row += 1
  def select(self, name, ySelector, idx, fullName = ""):
    if name == fullName:
      selection = ySelector.selectionModel()
      selection.select(self.modelIdxs[idx], QtCore.QItemSelectionModel.Select)
      ySelector.setSelectionModel(selection)
      parent = self.parent
      while parent is not None and idx < len(parent.widgets):
        parent.widgets[idx].setExpanded(True)
        parent = parent.parent
    else:
      for l in self.leafs:
        if len(fullName):
          fName = fullName + "_" + l.name
        else:
          fName = l.name
        if name.startswith(fName):
          l.select(name, ySelector, idx, fName)
  def __print(self, indent):
    ret = "\n"
    if self.name is not None:
      ret = " "*indent + "| " + self.name + '\n'
    for l in self.leafs:
      ret += l.__print(indent + 1)
    return ret
  def __str__(self):
    return self.__print(-1)

class FilterRightClick(QtCore.QObject):
  def __init__(self, parent):
    super(FilterRightClick, self).__init__(parent)

  def eventFilter(self, obj, event):
    if event.type() == QtCore.QEvent.MouseButtonPress:
      if event.button() == QtCore.Qt.RightButton:
        return True
    return False

class SpecialPlot(object):
  def __init__(self, name, figure, idx, special_id, label = None):
    self.figure = figure
    self.idx = idx
    self.name = name
    self.id = special_id
    self.added = []
    if idx == 0:
      self.remove = self.figure.remove_plot_left
    else:
      self.remove = self.figure.remove_plot_right
    if special_id == "diff":
      self.__plot = self.__add_diff
      self.label = " diff"
    elif special_id == "rpy":
      self.__plot = self.__add_rpy
      self.label = " rpy"
    elif special_id == "r":
      self.__plot = self.__add_roll
      self.label = " roll"
    elif special_id == "p":
      self.__plot = self.__add_pitch
      self.label = " pitch"
    elif special_id == "y":
      self.__plot = self.__add_yaw
      self.label = " yaw"
    else:
      print("Cannot handle this special plot: {}".format(special_id))
    if label is not None:
      self.label = label
    else:
      self.label = self.name + self.label
    self.plot()
  def __add_diff(self):
    added = filter(lambda x: re.match("{}($|_.*$)".format(self.name), x) is not None, self.figure.data.keys())
    if self.idx == 0:
      add_fn = self.figure.add_diff_plot_left
    else:
      add_fn = self.figure.add_diff_plot_right
    for a in added:
      label = "{}_diff".format(a)
      if len(added) == 1 and self.label is not None:
        label = self.label
      if add_fn(self.figure.x_data, a, label):
        self.added.append(label)
  def __add_rpy(self):
    if self.idx == 0:
      add_fn = self.figure.add_rpy_plot_left
    else:
      add_fn = self.figure.add_rpy_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, s) for s in ["roll", "pitch", "yaw"] ]
  def __add_roll(self):
    if self.idx == 0:
      add_fn = self.figure.add_roll_plot_left
    else:
      add_fn = self.figure.add_roll_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, "roll") ]
  def __add_pitch(self):
    if self.idx == 0:
      add_fn = self.figure.add_pitch_plot_left
    else:
      add_fn = self.figure.add_pitch_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, "pitch") ]
  def __add_yaw(self):
    if self.idx == 0:
      add_fn = self.figure.add_yaw_plot_left
    else:
      add_fn = self.figure.add_yaw_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, "yaw") ]
  def plot(self):
    self.__plot()

class RemoveSpecialPlotButton(SpecialPlot, QtWidgets.QPushButton):
  def __init__(self, name, logtab, idx, special_id, label = None):
    self.logtab = logtab
    SpecialPlot.__init__(self, name, logtab.ui.canvas, idx, special_id, label)
    QtWidgets.QPushButton.__init__(self, u"Remove {} plot".format(self.label), logtab)
    self.clicked.connect(self.on_clicked)
    if idx == 0:
      self.layout = logtab.ui.y1SelectorLayout
    else:
      self.layout = logtab.ui.y2SelectorLayout
    self.layout.addWidget(self)
    if len(self.added) == 0:
      self.deleteLater()
    else:
      self.logtab.specials["{}_{}".format(name, special_id)] = self
  def plot(self):
    SpecialPlot.plot(self)
    self.logtab.ui.canvas.draw()
  def on_clicked(self):
    for added in self.added:
      self.remove(added)
    self.logtab.ui.canvas.draw()
    del self.logtab.specials["{}_{}".format(self.name, self.id)]
    self.deleteLater()

class RemoveXYDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QVBoxLayout, apply_ = False)
  def __init__(self, parent, remove_cb, style_cb, get_plots_cb):
    self.remove = remove_cb
    self.get_plots = get_plots_cb
    for itm in self.get_plots().keys():
      label = style_cb(itm).label
      btn = QtWidgets.QPushButton("Remove {}".format(label))
      btn.released.connect(lambda l=label,b=btn: self.removePlot(l, b))
      self.layout.addWidget(btn)
  def removePlot(self, label, btn):
    self.remove(label)
    btn.deleteLater()
    if len(self.get_plots()) == 0:
      self.parent().removeButton.hide()

class XYSelectorDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QFormLayout, apply_ = False)
  def __init__(self, parent, data):
    self.setWindowTitle("Add X/Y data to the plot")
    self.data = data
    self.tree_view = TreeView()
    for k in sorted(data.keys()):
      if type(data[k][0]) is not unicode:
        self.tree_view.add(k.split('_'))
    self.tree_view.simplify()
    self.treeSelector = QtWidgets.QTreeWidget(self)
    self.treeSelector.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
    self.treeSelector.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectItems)
    self.treeSelector.setColumnCount(1)
    self.treeSelector.itemClicked.connect(self.itemClicked)
    self.treeSelector.header().setVisible(False)
    self.tree_view.update_y_selector(self.treeSelector, self.treeSelector)
    self.layout.addRow(self.treeSelector)

    self.xLabel = QtWidgets.QLabel(self)
    self.layout.addRow("X", self.xLabel)
    self.yLabel = QtWidgets.QLabel(self)
    self.layout.addRow("Y", self.yLabel)
    self.labelEdit = QtWidgets.QLineEdit(self)
    self.layout.addRow("Label", self.labelEdit)
    self.nextLabel = 0
    self.labels = [self.xLabel, self.yLabel]

  def itemClicked(self, item, col):
    if item.isSelected():
      if not item.hasData:
        childs = [ k for k in self.data.keys() if k.startswith("{}_".format(item.actualText)) ]
        childs.sort()
        if len(childs) == len(self.labels) and all([len(l.text()) == 0 for l in self.labels]):
          [ l.setText(c) for c,l in zip(childs, self.labels) ]
          self.nextLabel = len(self.labels)
        else:
          item.setSelected(False)
      else:
        if self.nextLabel < len(self.labels):
          self.labels[self.nextLabel].setText(item.actualText)
          while self.nextLabel < len(self.labels) and len(self.labels[self.nextLabel].text()) != 0:
            self.nextLabel += 1
        else:
          item.setSelected(False)
    else:
      if not item.hasData:
        [ label.setText("") for label in self.labels ]
        self.nextLabel = 0
        return
      for label in self.labels:
        if label.text() == item.actualText:
          label.setText("")
          break
      for i, label in enumerate(self.labels):
        if len(label.text()) == 0:
          self.nextLabel = i
          return

  def accept(self):
    super(XYSelectorDialog, self).accept()
    x = self.xLabel.text()
    y = self.yLabel.text()
    label = self.labelEdit.text()
    if len(x) and len(y):
      if len(label) == 0:
        label = "{} / {}".format(x, y)
      self.parent().addXYPlot(x, y, label)

class XYSelector(QtWidgets.QWidget):
  def __init__(self, parent, add_plot_cb, remove_plot_cb, style_cb, draw_cb, get_plots_cb):
    super(XYSelector, self).__init__(parent)
    self.layout = QtWidgets.QVBoxLayout(self)
    self.addButton = QtWidgets.QPushButton("Add XY plot")
    self.layout.addWidget(self.addButton)
    self.dialog = XYSelectorDialog
    self.addButton.released.connect(lambda: self.dialog(self, self.parent().data).exec_())
    self.removeButton = QtWidgets.QPushButton("Remove XY plot(s)")
    self.removeButton.released.connect(lambda: RemoveXYDialog(self, self.remove_plot, self.style, self.get_plots).exec_())
    self.removeButton.hide()
    self.layout.addWidget(self.removeButton)
    self.add_plot = add_plot_cb
    self.remove_plot = remove_plot_cb
    self.style = style_cb
    self.draw = draw_cb
    self.get_plots = get_plots_cb
    self.hide()
  def addXYPlot(self, x, y, label):
    self.add_plot(x, y, label)
    self.draw()
    self.removeButton.show()

class XYZSelectorDialog(XYSelectorDialog):
  def __init__(self, parent, data):
    super(XYZSelectorDialog, self).__init__(parent, data)
    self.setWindowTitle("Add X/Y/Z data to the plot")
    self.zLabel = QtWidgets.QLabel(self)
    self.layout.insertRow(3, "Z", self.zLabel)
    self.labels.append(self.zLabel)
  def accept(self):
    super(XYSelectorDialog, self).accept()
    x = self.xLabel.text()
    y = self.yLabel.text()
    z = self.zLabel.text()
    label = self.labelEdit.text()
    if len(x) and len(y) and len(z):
      if len(label) == 0:
        label = "{} / {} / {}".format(x, y, z)
      self.parent().addXYZPlot(x, y, z, label)

class XYZSelector(XYSelector):
  def __init__(self, parent, add_plot_cb, remove_plot_cb, style_cb, draw_cb, get_plots_cb):
    super(XYZSelector, self).__init__(parent, add_plot_cb, remove_plot_cb, style_cb, draw_cb, get_plots_cb)
    self.addButton.setText("Add 3D plot")
    self.removeButton.setText("Remove 3D plot(s)")
    self.dialog = XYZSelectorDialog
  def addXYZPlot(self, x, y, z, label):
    self.add_plot(x, y, z, label)
    self.draw()
    self.removeButton.show()

class MCLogTab(QtWidgets.QWidget):
  canvas_need_update = QtCore.Signal()
  def __init__(self, parent = None, type_ = PlotType.TIME):
    super(MCLogTab, self).__init__(parent)
    self.ui = ui.MCLogTab()
    self.ui.setupUi(self)
    def setupSelector(ySelector):
      ySelector.setHeaderLabels(["Data"])
      ySelector.header().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
      ySelector.viewport().installEventFilter(FilterRightClick(ySelector))
    setupSelector(self.ui.y1Selector)
    setupSelector(self.ui.y2Selector)
    self.ui.y1Selector.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
    self.ui.y2Selector.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
    self.y1Selected = []
    self.y2Selected = []

    self.XYCanvas = PlotCanvasWithToolbar(self, PlotType.XY)
    self.ui.verticalLayout.insertWidget(0, self.XYCanvas)
    self.XYCanvas.hide()
    self.XYSelector1 = XYSelector(self, self.XYCanvas.add_plot_left_xy, self.XYCanvas.remove_plot_left, self.XYCanvas.style_left, self.XYCanvas.draw, lambda: self.XYCanvas._left().plots)
    self.ui.y1SelectorLayout.addWidget(self.XYSelector1)
    self.XYSelector2 = XYSelector(self, self.XYCanvas.add_plot_right_xy, self.XYCanvas.remove_plot_right, self.XYCanvas.style_right, self.XYCanvas.draw, lambda: self.XYCanvas._right().plots)
    self.ui.y2SelectorLayout.addWidget(self.XYSelector2)

    self._3DCanvas = PlotCanvasWithToolbar(self, PlotType._3D)
    self.ui.verticalLayout.insertWidget(0, self._3DCanvas)
    self._3DCanvas.hide()
    self.XYZSelector1 = XYZSelector(self, self._3DCanvas.add_plot_left_xyz, self._3DCanvas.remove_plot_left, self._3DCanvas.style_left, self._3DCanvas.draw, lambda: self._3DCanvas._left().plots)
    self.ui.y1SelectorLayout.addWidget(self.XYZSelector1)

    self.activeCanvas = self.ui.canvas
    self.activeSelectors = [self.ui.y1Selector, self.ui.y2Selector]

    self.modeSelector = QtWidgets.QComboBox(self)
    self.modeSelector.addItem("Time plot")
    self.modeSelector.addItem("X/Y plot")
    self.modeSelector.addItem("3D plot")
    self.ui.y1SelectorLayout.addWidget(self.modeSelector)
    self.modeSelector.currentTextChanged.connect(self.changeCanvasMode)

    self.data = None
    self.rm = None
    self.ui.canvas.x_data = 't'
    self.x_data = 't'

    if parent is not None:
      for c in [self.ui.canvas, self.XYCanvas, self._3DCanvas]:
        c._left().grid = parent.gridStyles['left']
        if c._right() is not None:
          c._right().grid = parent.gridStyles['right']

    self.specials = {}

    self.setPlotType(type_)

  def plotType(self):
    return PlotType(self.modeSelector.currentIndex())

  def setPlotType(self, type_):
    self.modeSelector.setCurrentIndex(type_.value)
    self.changeCanvasMode()

  def changeCanvasMode(self):
    type_ = self.plotType()
    self.activeCanvas.hide()
    [ s.hide() for s in self.activeSelectors ]
    if type_ is PlotType.TIME:
      self.activeCanvas = self.ui.canvas
      self.activeSelectors = [self.ui.y1Selector, self.ui.y2Selector]
    elif type_ is PlotType.XY:
      self.activeCanvas = self.XYCanvas
      self.activeSelectors = [self.XYSelector1, self.XYSelector2]
    else:
      self.activeCanvas = self._3DCanvas
      self.activeSelectors = [self.XYZSelector1]
    self.activeCanvas.show()
    [ s.show() for s in self.activeSelectors ]

  def setData(self, data):
    self.data = data
    for c in [self.ui.canvas, self.XYCanvas, self._3DCanvas]:
      c.setData(data)
    self.update_y_selectors()

  def setGridStyles(self, gridStyles):
    for c in [self.ui.canvas, self.XYCanvas, self._3DCanvas]:
      c._left().grid = copy.deepcopy(gridStyles['left'])
      if c._right():
        c._right().grid = copy.deepcopy(gridStyles['right'])

  def setColors(self, colors):
    for c in [self.ui.canvas, self.XYCanvas, self._3DCanvas]:
      c.setColors(colors)

  def setPolyColors(self, colors):
    for c in [self.ui.canvas, self.XYCanvas, self._3DCanvas]:
      c.setPolyColors(colors)

  def setRobotModule(self, rm, loaded_files):
    self.rm = rm
    if self.rm is None:
      return
    def setQNames(ySelector):
      qList = ySelector.findItems("q", QtCore.Qt.MatchStartsWith | QtCore.Qt.MatchRecursive)
      qList += ySelector.findItems("alpha", QtCore.Qt.MatchStartsWith | QtCore.Qt.MatchRecursive)
      qList += ySelector.findItems("error", QtCore.Qt.MatchStartsWith | QtCore.Qt.MatchRecursive)
      qList += ySelector.findItems("tau", QtCore.Qt.MatchStartsWith | QtCore.Qt.MatchRecursive)
      def update_child_display(items):
        for itm in items:
          cCount = itm.childCount()
          if cCount == 0:
            if itm.originalText.isdigit():
              jIndex = int(itm.originalText)
              if jIndex < len(self.rm.ref_joint_order()):
                itm.displayText = self.rm.ref_joint_order()[jIndex]
          else:
            update_child_display([itm.child(i) for i in range(cCount)])
      update_child_display(qList)
    setQNames(self.ui.y1Selector)
    setQNames(self.ui.y2Selector)
    if self.data is None:
      return
    bounds = self.rm.bounds()
    def setBounds(prefix):
        for i, jn in enumerate(self.rm.ref_joint_order()):
          if "{}qIn_limits_lower_{}".format(prefix, i) in self.data:
            self.data["{}qIn_limits_lower_{}".format(prefix, i)].fill(bounds[0][jn][0])
            self.data["{}qIn_limits_upper_{}".format(prefix, i)].fill(bounds[1][jn][0])
            self.data["{}qOut_limits_lower_{}".format(prefix, i)].fill(bounds[0][jn][0])
            self.data["{}qOut_limits_upper_{}".format(prefix, i)].fill(bounds[1][jn][0])
          if "{}tauIn_limits_lower_{}".format(prefix, i) in self.data:
            self.data["{}tauIn_limits_lower_{}".format(prefix, i)].fill(bounds[4][jn][0])
            self.data["{}tauIn_limits_upper_{}".format(prefix, i)].fill(bounds[5][jn][0])
          if "{}tauOut_limits_lower_{}".format(prefix, i) in self.data:
            self.data["{}tauOut_limits_lower_{}".format(prefix, i)].fill(bounds[4][jn][0])
            self.data["{}tauOut_limits_upper_{}".format(prefix, i)].fill(bounds[5][jn][0])
    if len(loaded_files) > 1:
      for f in loaded_files:
        setBounds("{}_".format(f))
    else:
      setBounds("")


  def on_xSelector_activated(self, canvas, k):
    self.x_data = k
    canvas.x_data = k
    canvas.update_x()
    for _,s in self.specials.items():
      s.plot()

  @QtCore.Slot(QtWidgets.QTreeWidgetItem, int)
  def on_y1Selector_itemClicked(self, item, col):
    self.y1Selected = self.itemSelectionChanged(self.ui.y1Selector, self.y1Selected, 0)

  @QtCore.Slot(QtWidgets.QTreeWidgetItem, int)
  def on_y2Selector_itemClicked(self, item, col):
    self.y2Selected = self.itemSelectionChanged(self.ui.y2Selector, self.y2Selected, 1)

  @QtCore.Slot(QtCore.QPoint)
  def on_y1Selector_customContextMenuRequested(self, point):
    self.showCustomMenu(self.ui.y1Selector, point, 0)

  @QtCore.Slot(QtCore.QPoint)
  def on_y2Selector_customContextMenuRequested(self, point):
    self.showCustomMenu(self.ui.y2Selector, point, 1)

  def itemSelectionChanged(self, ySelector, prevSelected, idx):
    if idx == 0:
      add_fn = self.ui.canvas.add_plot_left
    else:
      add_fn = self.ui.canvas.add_plot_right
    if idx == 0:
      remove_fn = self.ui.canvas.remove_plot_left
    else:
      remove_fn = self.ui.canvas.remove_plot_right
    selected_items = [(i.actualText,i.hasData) for i in ySelector.selectedItems()]
    def is_selected(s, x):
      if s[1]:
        return x == s[0]
      return re.match("{}($|_.*$)".format(s[0]), x) is not None
    selected = sorted(filter(lambda x: any([is_selected(s, x) for s in selected_items]), self.data.keys()))
    def find_item(s):
      itm = QtWidgets.QTreeWidgetItemIterator(ySelector)
      while itm:
        if itm.value().actualText == s:
          return itm.value()
        itm += 1
      return None
    legends = [itm.legendText for itm in [ find_item(s) for s in selected ] ]
    for s,l in zip(selected, legends):
      if s not in prevSelected:
        add_fn(self.x_data, s, l)
    for s in prevSelected:
      if s not in selected:
        remove_fn(s)
    self.ui.canvas.draw()
    return selected

  def update_y_selectors(self):
    canvas = self.ui.canvas
    self.ui.y1Selector.clear()
    self.ui.y2Selector.clear()
    self.tree_view = TreeView()
    for k in sorted(self.data.keys()):
      self.tree_view.add(k.split('_'))
    self.tree_view.simplify()
    def update_y_selector(ySelector):
      self.tree_view.update_y_selector(ySelector, ySelector)
      ySelector.resizeColumnToContents(0)
      cWidth = ySelector.sizeHintForColumn(0)
      ySelector.setMaximumWidth(cWidth + 75)
    update_y_selector(self.ui.y1Selector)
    update_y_selector(self.ui.y2Selector)
    y1 = filter(lambda k: k in self.data.keys(), canvas._left().plots.keys())
    [ self.tree_view.select(y, self.ui.y1Selector, 0) for y in y1 ]
    y2 = filter(lambda k: k in self.data.keys(), canvas._right().plots.keys())
    [ self.tree_view.select(y, self.ui.y2Selector, 1) for y in y2 ]
    poly = filter(lambda k: k in self.data.keys(), canvas._polygons().plots.keys())
    [ self.tree_view.select(y, self.ui.y1Selector, 0) for y in poly ]

  def showCustomMenu(self, ySelector, point, idx):
    item = ySelector.itemAt(point)
    if item is None:
      return
    menu = QtWidgets.QMenu(ySelector)
    addedAction = False
    action = QtWidgets.QAction(u"Plot diff".format(item.actualText), menu)
    action.triggered.connect(lambda: RemoveSpecialPlotButton(item.actualText, self, idx, "diff", item.legendText + "_diff"))
    menu.addAction(action)
    s = re.match('^(.*)_q?[wxyz]$', item.actualText)
    if s is not None:
      for item_label, axis_label in [("RPY angles", "rpy"), ("ROLL angle", "r"), ("PITCH angle", "p"), ("YAW angle", "y")]:
        action = QtWidgets.QAction(u"Plot {}".format(item_label, item.actualText), menu)
        action.triggered.connect(lambda checked, label=axis_label: RemoveSpecialPlotButton(s.group(1), self, idx, label))
        menu.addAction(action)
    else:
      quat_childs = filter(lambda x: x is not None, [ re.match('{}((_.+)*)_q?w$'.format(item.actualText), x) for x in self.data.keys() ])
      for qc in quat_childs:
        for item_label, axis_label in [("RPY angles", "rpy"), ("ROLL angle", "r"), ("PITCH angle", "p"), ("YAW angle", "y")]:
          if len(qc.group(1)):
            action_text = u"Plot {} {}".format(qc.group(1)[1:], item_label)
          else:
            action_text = u"Plot {}".format(item_label)
          action = QtWidgets.QAction(action_text, menu)
          plot_name = item.actualText + qc.group(1)
          action.triggered.connect(lambda checked, name=plot_name, label=axis_label: RemoveSpecialPlotButton(name, self, idx, label))
          menu.addAction(action)
    menu.exec_(ySelector.viewport().mapToGlobal(point))

  def style_left(self, y, styleIn = None):
    return self.activeCanvas.style_left(y, styleIn)

  def style_right(self, y, styleIn = None):
    return self.activeCanvas.style_right(y, styleIn)

  @staticmethod
  def MakeFigure(type_, data, x, y1, y2, y1_label = None, y2_label = None, figure = None):
    def labels(yN):
      if type_ is PlotType.TIME:
        return yN
      elif type_ is PlotType.XY:
        return [l for x,y,l in yN]
      else:
        return [l for x,y,z,l in yN]
    if y1_label is None:
      return MCLogTab.MakeFigure(type_, data, x, y1, y2, labels(y1), y2_label, figure)
    if y2_label is None:
      return MCLogTab.MakeFigure(type_, data, x, y1, y2, y1_label, labels(y2), figure)
    if figure is None:
      return MCLogTab.MakeFigure(type_, data, x, y1, y2, y1_label, y2_label, PlotFigure())
    figure.setData(data)
    if type_ is PlotType.TIME:
      for y,yl in zip(y1, y1_label):
        figure.add_plot_left(x, y, yl)
      for y,yl in zip(y2, y2_label):
        figure.add_plot_right(x, y, yl)
    elif type_ is PlotType.XY:
      for x,y,label in y1:
        figure.add_plot_left_xy(x, y, label)
      for x,y,label in y2:
        figure.add_plot_right_xy(x, y, label)
    else:
      for x,y,z,label in y1:
        figure.add_plot_left_xyz(x, y, z, label)
      for x,y,z,label in y2:
        figure.add_plot_right_xyz(x, y, z, label)
    return figure

  @staticmethod
  def MakePlot(parent, type_, x_data, y1, y2, y1_label = None, y2_label = None):
    def labels(yN):
      if type_ is PlotType.TIME:
        return yN
      elif type_ is PlotType.XY:
        return [l for x,y,l in yN]
      else:
        return [l for x,y,z,l in yN]
    if y1_label is None:
      return MCLogTab.MakePlot(parent, type_, x_data, y1, y2, labels(y1), y2_label)
    if y2_label is None:
      return MCLogTab.MakePlot(parent, type_, x_data, y1, y2, y1_label, labels(y2))
    tab = MCLogTab(parent, type_)
    tab.x_data = x_data
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm, parent.loaded_files)
    if type_ is PlotType.TIME:
      for y,yl in zip(y1, y1_label):
        tab.tree_view.select(y, tab.ui.y1Selector, 0)
      for y,yl in zip(y2, y2_label):
        tab.tree_view.select(y, tab.ui.y2Selector, 1)
      tab.y1Selected = y1
      tab.y2Selected = y2
    elif type_ is PlotType.XY:
      if len(y1_label):
        tab.XYSelector1.removeButton.show()
      if len(y2_label):
        tab.XYSelector2.removeButton.show()
    else:
        tab.XYZSelector1.removeButton.show()

    MCLogTab.MakeFigure(type_, parent.data, x_data, y1, y2, y1_label, y2_label, tab.activeCanvas)
    tab.activeCanvas.x_data = x_data
    return tab

  @staticmethod
  def UserFigure(data, p, figure = None, special = None):
    if figure is None:
      return MCLogTab.UserFigure(data, p, MCLogTab.MakeFigure(p.type, data, p.x, p.y1, p.y2), special)
    if special is None:
      return MCLogTab.UserFigure(data, p, figure, lambda y, idx, id_: UserPlot(y, figure, idx, id_))
    def set_label(label_fn, label_size_fn, label):
      if len(label.text):
        label_fn(label.text)
        label_size_fn(label.fontsize)
    set_label(figure.title, figure.title_fontsize, p.graph_labels.title)
    set_label(figure.x_label, figure.x_label_fontsize, p.graph_labels.x_label)
    set_label(figure.y1_label, figure.y1_label_fontsize, p.graph_labels.y1_label)
    set_label(figure.y2_label, figure.y2_label_fontsize, p.graph_labels.y2_label)
    def handle_yd(yds, idx):
      for yd in yds:
        match = re.match("(.*)_(.*)$", yd)
        if match is None:
          special(yd, idx, "diff")
        elif match.group(2) in ["rpy", "r", "p", "y"]:
          special(match.group(1), idx, match.group(2))
        else:
          special(match.group(1), idx, "diff")
    handle_yd(p.y1d, 0)
    handle_yd(p.y2d, 1)
    if not isinstance(p.grid1, LineStyle):
      figure._left().grid = LineStyle(**p.grid1)
    else:
      figure._left().grid = p.grid1
    if figure._right() is not None:
      if not isinstance(p.grid2, LineStyle):
        figure._right().grid = LineStyle(**p.grid2)
      else:
        figure._right().grid = p.grid2
    for y,s in p.style.items():
      figure.style_left(y, s)
    for y,s in p.style2.items():
      figure.style_right(y, s)
    for param, value in p.extra.items():
      getattr(figure, param)(value)
    return figure

  @staticmethod
  def UserPlot(parent, p):
    tab = MCLogTab.MakePlot(parent, p.type, p.x, p.y1, p.y2)
    MCLogTab.UserFigure(parent.data, p, tab.activeCanvas, lambda y, idx, id_: RemoveSpecialPlotButton(y, tab, idx, id_))
    tab.activeCanvas.draw()
    return tab

  @staticmethod
  def ForceSensorPlot(parent, fs):
    tab = MCLogTab.MakePlot(parent, PlotType.TIME, 't', ['{}ForceSensor_f{}'.format(fs, ax) for ax in ['x', 'y', 'z']], ['{}ForceSensor_c{}'.format(fs, ax) for ax in ['x', 'y', 'z']])
    tab.ui.canvas.title('Force sensor: {}'.format(fs))
    tab.ui.canvas.y1_label('Force')
    tab.ui.canvas.y2_label('Moment')
    tab.ui.canvas.draw()
    return tab

  @staticmethod
  def JointPlot(parent, joints, y1_prefix, y2_prefix, y1_diff_prefix, y2_diff_prefix, plot_limits = False):
    def prefix_to_label(joints, prefix, diff):
      suffix = ''
      if diff:
        suffix += '_velocity'
      if prefix == "qIn":
        return "encoder" + suffix
      if prefix == "qOut":
        return "command" + suffix
      if prefix == "tauIn":
        return "torque" + suffix
      if prefix == "error":
        return "error"+suffix
      return prefix
    y1_label = prefix_to_label(joints, y1_prefix, False)
    y2_label = prefix_to_label(joints, y2_prefix, False)
    y1_diff_label = prefix_to_label(joints, y1_diff_prefix, True)
    y2_diff_label = prefix_to_label(joints, y2_diff_prefix, True)
    rjo = parent.rm.ref_joint_order()
    y_data = [[], []]
    y_data_labels = [[], []]
    y_diff_data = [[], []]
    y_diff_data_labels = [[], []]
    for j in joints:
      jIndex = rjo.index(j)
      if y1_prefix:
        y_data[0] += [ '{}_{}'.format(y1_prefix, jIndex) ]
        y_data_labels[0] += [ '{}_{}'.format(y1_label, j) ]
        if y1_prefix != "error" and plot_limits:
          y_data[0] += [ '{}_limits_lower_{}'.format(y1_prefix, jIndex) ]
          y_data_labels[0] += [ '{}_limits_lower_{}'.format(y1_label, j) ]
          y_data[0] += [ '{}_limits_upper_{}'.format(y1_prefix, jIndex) ]
          y_data_labels[0] += [ '{}_limits_upper_{}'.format(y1_label, j) ]
      if y2_prefix:
        y_data[1] += [ '{}_{}'.format(y2_prefix, jIndex) ]
        y_data_labels[1] += [ '{}_{}'.format(y2_label, j) ]
        if y2_prefix != "error" and plot_limits:
          y_data[1] += [ '{}_limits_lower_{}'.format(y2_prefix, jIndex) ]
          y_data_labels[1] += [ '{}_limits_lower_{}'.format(y2_label, j) ]
          y_data[1] += [ '{}_limits_upper_{}'.format(y2_prefix, jIndex) ]
          y_data_labels[1] += [ '{}_limits_upper_{}'.format(y2_label, j) ]
      if y1_diff_prefix:
        y_diff_data[0] += [ '{}_{}'.format(y1_diff_prefix, jIndex) ]
        y_diff_data_labels[0] += [ '{}_{}'.format(y1_diff_label, j) ]
      if y2_diff_prefix:
        y_diff_data[1] += [ '{}_{}'.format(y2_diff_prefix, jIndex) ]
        y_diff_data_labels[1] += [ '{}_{}'.format(y2_diff_label, j) ]
    tab = MCLogTab.MakePlot(parent, PlotType.TIME, 't', y_data[0], y_data[1], y_data_labels[0], y_data_labels[1])
    for y, y_label in zip(y_diff_data[0], y_diff_data_labels[0]):
      RemoveSpecialPlotButton(y, tab, 0, "diff", y_label)
    for y, y_label in zip(y_diff_data[1], y_diff_data_labels[1]):
      RemoveSpecialPlotButton(y, tab, 1, "diff", y_label)
    title = ''
    def updateTitle(title, nTitle):
      if len(title):
        title += ' / '
      nTitle = nTitle.replace('_', ' ')
      title += nTitle
      return title
    if y1_label:
      title = updateTitle(title, y1_label.title())
      tab.ui.canvas.y1_label(y1_label)
    if y2_label:
      title = updateTitle(title, y2_label.title())
      tab.ui.canvas.y2_label(y2_label)
    if y1_diff_label:
      title = updateTitle(title, y1_diff_label.title())
      tab.ui.canvas.y1_label(y1_diff_label)
    if y2_diff_label:
      title = updateTitle(title, y2_diff_label.title())
      tab.ui.canvas.y2_label(y2_diff_label)
    tab.ui.canvas.title(title)
    tab.ui.canvas.draw()
    return tab
