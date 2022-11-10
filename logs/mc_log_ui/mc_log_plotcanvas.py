#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QComboBox

import copy
import math
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
import numpy as np
matplotlib.use('Qt5Agg')
import matplotlib.pyplot

from matplotlib.animation import FuncAnimation
from matplotlib.figure import Figure
from matplotlib.patches import Patch, Polygon, Rectangle

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas,\
                                               NavigationToolbar2QT as NavigationToolbar

from mpl_toolkits.mplot3d import Axes3D

from collections import OrderedDict
from math import asin, atan2

from .mc_log_types import LineStyle, PlotSide, PlotType
try:
  from . import mc_log_ui
except ImportError:
  import mc_log_ui
from .mc_log_utils import InitDialogWithOkCancel

import sys
if sys.version_info[0] > 2:
    unicode = str


class GenerateRangeDialog(QtWidgets.QDialog):
  @InitDialogWithOkCancel(Layout = QtWidgets.QFormLayout, apply_ = False)
  def __init__(self, parent):
    self.setWindowTitle("Generate time-range(s) based on data range")
    self.data = parent.data
    self.selectedData = QtWidgets.QComboBox(self)
    keys = self.data.keys()
    keys.sort()
    for k in keys:
      self.selectedData.addItem(k)
    self.layout.addRow("Data range", self.selectedData)
    self.stringSelector = QtWidgets.QComboBox(self)
    self.stringSelectorLabel = QtWidgets.QLabel("Select value")
    self.setStringSelector()
    self.layout.addRow(self.stringSelectorLabel, self.stringSelector)
    self.outputName = QtWidgets.QLineEdit()
    self.setDefaultName()
    self.layout.addRow("Name", self.outputName)
    self.selectedData.currentTextChanged.connect(self.setDefaultName)
    self.selectedData.currentTextChanged.connect(self.setStringSelector)
    self.stringSelector.currentTextChanged.connect(self.setDefaultName)

  def setStringSelector(self):
    key = self.selectedData.currentText()
    data = self.data[key]
    self.stringSelector.clear()
    if type(data[0]) is unicode:
      self.stringSelectorLabel.show()
      self.stringSelector.show()
      self.stringSelector.addItem("All")
      self.stringSelector.insertSeparator(1)
      keys = set(data)
      for k in keys:
        self.stringSelector.addItem(k)
    else:
      self.stringSelectorLabel.hide()
      self.stringSelector.hide()

  def setDefaultName(self):
    self.outputName.setText("t_" + self.selectedData.currentText())
    if self.stringSelector.currentIndex() > 0:
      self.outputName.setText(self.outputName.text() + "_" + self.stringSelector.currentText())

  def accept(self):
    key = self.selectedData.currentText()
    data = self.data[key]
    def make_key(i, ranges):
      if len(ranges) == 1:
        return self.outputName.text()
      zeros = len(str(len(ranges)))
      text = self.outputName.text() + "_{}".format(str(i).zfill(zeros))
      i += 1
      return text

    if type(data[0]) is unicode:
      strIdx = self.stringSelector.currentIndex()
      if strIdx == 0:
        def is_valid(idx):
          return len(data[idx]) != 0
      else:
        string = self.stringSelector.currentText()
        def is_valid(idx):
          return data[idx] == string
    else:
      def is_valid(idx):
        return not np.isnan(data[idx])
    def get_range(i0):
      assert(i0 < len(data))
      if i0 == len(data) - 1:
        return None
      while i0 < len(data) and not is_valid(i0):
        i0 += 1
      if i0 == len(data):
        return None
      iN = i0
      while iN < len(data) and is_valid(iN):
        iN += 1
      if iN == len(data):
        iN = len(data) - 1
      return (i0, iN)
    ranges = [get_range(0)]
    while ranges[-1] is not None:
      ranges.append(get_range(ranges[-1][1]))
    ranges = ranges[:-1]
    def make_range(i0, iN):
      out = copy.deepcopy(self.data['t'])
      out[:i0] = np.nan
      out[iN+1:] = np.nan
      return out
    i = 1
    for r in ranges:
      k = make_key(i, ranges)
      self.data[make_key(i, ranges)] = make_range(r[0], r[1])
      i += 1
    self.data.notify_update()
    super(GenerateRangeDialog, self).accept()

def rpyFromMat(E):
    """Same as mc_rbdyn::rpyFromMat."""
    roll = atan2(E[1][2], E[2][2]);
    pitch = -asin(E[0][2]);
    yaw = atan2(E[0][1], E[0][0]);
    return [roll, pitch, yaw]


def rpyFromQuat(quat):
    """Same as mc_rbdyn::rpyFromQuat."""
    import eigen
    return rpyFromMat(list(eigen.Quaterniond(*quat).toRotationMatrix()))

class PlotPolygonAxis(object):
  def __init__(self, parent, axis):
    self.figure = parent
    self._axis = axis.twinx()
    self._axis.set_zorder(-1)
    self._axis.set_yticks([])
    self._axis.set_ylim(0, 1)
    self._axis.get_yaxis().set_visible(False)
    self.data = {}
    self.plots = OrderedDict()
    self.colors = OrderedDict()
  def _legend_fontsize(self):
    return self.figure._labels_legend_fontsize
  def _legend_top_offset(self):
    return self.figure._labels_legend_top_offset
  def _plot_string(self, x, y, y_label, style):
    if y_label in self.plots:
      return False
    self.plots[y_label] = []
    self.data[y_label] = [x, y]
    self.colors[y_label] = {}
    i = 0
    i0 = 0
    label = y[i0]
    while i < len(y):
      if y[i] == label and i + 1 != len(y):
        i += 1
        continue
      if len(label) == 0:
        i += 1
        continue
      if label not in self.colors[y_label]:
        self.colors[y_label][label] = self.figure._next_poly_color()
      color = self.colors[y_label][label]
      if i + 1 < len(y) and not np.isnan(x[i + 1]):
        xi = x[i + 1]
      else:
        xi = x[i]
        if np.isnan(xi):
          xi = x[i - 1]
      self.plots[y_label].append(self._axis.add_patch(Rectangle((x[i0], 0), xi - x[i0], 1, label = label, facecolor = color)))
      i0 = i
      if i < len(y):
        label = y[i0]
      i += 1
    return True
  def legend(self):
    if not len(self.plots):
      self._axis.clear()
      return
    xL = 1.0
    if len(self.figure._right()):
      xL = 1.025
    self._axis.legend(bbox_to_anchor=(xL, self._legend_top_offset(), 0.2, 0.1), mode="expand", borderaxespad=0.5, fontsize=self._legend_fontsize())
  def remove_plot(self, y):
    if y not in self.plots:
      return
    for plt in self.plots[y]:
      plt.remove()
    del self.data[y]
    del self.plots[y]
    del self.colors[y]
    self.figure.draw()
  def update_x(self, x):
    keys = self.data.keys()
    for y_label in keys:
      x, y = self.data[y_label]
      self.remove_plot(y_label)
      self._plot_string(x, y, y_label, None)

class PlotYAxis(object):
  def __init__(self, parent, x_axis = None, poly = None, _3D = False):
    self.figure = parent
    self._3D = _3D
    if x_axis is None:
      self.is_left = True
      if not _3D:
        self._axis = parent.fig.add_subplot(111)
      else:
        self._axis = parent.fig.gca(projection='3d')
        self._axis.set_xlabel('X')
        self._axis.set_ylabel('Y')
        self._axis.set_zlabel('Z')
      self._axis.autoscale(enable = True, axis = 'both', tight = False)
      self._x_axis = self._axis
    else:
      self.is_left = False
      if not _3D:
        self._axis = x_axis.twinx()
      else:
        self._axis = x_axis
      self._x_axis = x_axis
    if poly is None and not _3D:
      self._polyAxis = PlotPolygonAxis(parent, self._axis)
    else:
      self._polyAxis = poly
    try:
      self._axis.set_facecolor((1, 1, 1, 0))
    except AttributeError:
      pass
    box = self._axis.get_position()
    self._axis.autoscale_view(False,True,True)
    self._axis.format_coord = parent.format_coord
    self._axis.get_yaxis().set_visible(False)
    self.grid = LineStyle(linestyle = '--')
    self.plots = OrderedDict()
    self._label_fontsize = 10
    self._z_label_fontsize = 10
    self._legend_ncol = 3
    self.source = {}
    self.data = {}
    self.filtered = {}

  def __len__(self):
    return len(self.plots)

  def _data(self):
    return self.figure.data

  def _legend_fontsize(self):
    return self.figure._legend_fontsize

  def _labelpad(self):
    return self.figure._labelpad

  def _x_label_fontsize(self):
    return self.figure._x_label_fontsize

  def axis(self):
    return self._axis

  def drawGrid(self):
    if len(self.plots):
      self._axis.grid(color = self.grid.color, linestyle = self.grid.linestyle, linewidth = self.grid.linewidth, visible = self.grid.visible, which = 'both')

  def legend(self):
    if not len(self.plots):
      return
    if self.is_left:
      loc = 3
      top_anchor = 1.02
    else:
      loc = 2
      top_anchor = -0.14
      if len(self.figure.x_label()):
        top_anchor = -0.175
    self._axis.legend(bbox_to_anchor=(0., top_anchor, 1., .102), loc=loc, ncol=self._legend_ncol, mode="expand", borderaxespad=0.5, fontsize=self._legend_fontsize())
    if self._polyAxis is not None:
      self._polyAxis.legend()

  def legendNCol(self, n = None):
    if n is None:
      return self._legend_ncol
    self._legend_ncol = n
    self.legend()

  def legendRows(self):
    return math.ceil(len(self.plots) / float(self._legend_ncol))

  def legendOffset(self, offset, sign):
    if self.legendRows() > 3:
      offset = offset + sign * 0.035 * (self.legendRows() - 3)
    return offset

  def getLimits(self, frame, idx):
    if not len(self):
      return None
    min_ = np.nanmin(self.data.values()[0][idx][:frame])
    max_ = np.nanmax(self.data.values()[0][idx][:frame])
    for i in range(1, len(self.data.values())):
      data = self.data.values()[i][idx][:frame]
      min_ = min(np.nanmin(data), min_)
      max_ = max(np.nanmax(data), max_)
    return min_, max_


  def setLimits(self, xlim = None, ylim = None, frame = None, zlim = None):
    if not len(self):
      return xlim
    dataLim = self._axis.dataLim.get_points()
    def setLimit(lim, idx, set_lim):
      if lim is not None:
        min_, max_ = lim
      elif frame is not None:
        min_, max_ = self.getLimits(frame, idx)
      else:
        range_ = dataLim[1][idx] - dataLim[0][idx]
        min_ = dataLim[0][idx] - range_ * 0.01
        max_ = dataLim[1][idx] + range_ * 0.01
      set_lim([min_, max_])
      return min_, max_
    setLimit(ylim, 1, self._axis.set_ylim)
    if self._3D:
      frame = -1
      setLimit(zlim, 2, self._axis.set_zlim)
    return setLimit(xlim, 0, self._x_axis.set_xlim)

  def _label(self, get_label, set_label, l, size):
    if l is None:
      l = get_label()
    set_label(l, fontsize = size, labelpad = self._labelpad())

  def _label_property(self, get_label, set_label, l = None):
    if l is None:
      return get_label()
    set_label(l)

  def _x_label(self, l = None):
    self._label(self.x_label, self._x_axis.set_xlabel, l, self._x_label_fontsize())

  def x_label(self, l = None):
    return self._label_property(self._x_axis.get_xlabel, self._x_label, l)

  def _y_label(self, l = None):
    self._label(self.y_label, self._axis.set_ylabel, l, self._label_fontsize)

  def y_label(self, l = None):
    return self._label_property(self._axis.get_ylabel, self._y_label, l)

  def _z_label(self, l = None):
    self._label(self.z_label, self._axis.set_zlabel, l, self._z_label_fontsize)

  def z_label(self, l = None):
    return self._label_property(self._axis.get_zlabel, self._z_label, l)

  def y_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._label_fontsize
    self._label_fontsize = fontsize
    self._y_label()

  def z_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._z_label_fontsize
    self._z_label_fontsize = fontsize
    self._z_label()

  def animate(self, frame0, frame):
    for y_label in self.plots.keys():
      self.plots[y_label].set_data(self.data[y_label][0][frame0:frame], self.data[y_label][1][frame0:frame])
      if self._3D:
        self.plots[y_label].set_3d_properties(self.data[y_label][2][frame0:frame])
    return self.plots.values()

  def update_x(self, x):
    styles = {}
    for y_label in self.data.keys():
      filter_ = self.data[y_label][-1]
      if filter_ is None:
        self.data[y_label][0] = x
      else:
        filter_ = x
      styles[y_label] = self.style(y_label)
    self.clear()
    for y_label in self.data.keys():
      x = self.data[y_label][0]
      y = self.data[y_label][1]
      z = self.data[y_label][2]
      self._plot(x, y, y_label, styles[y_label], filter_ = filter_, z = z)

  def _filter(self, data, filter_):
    if data is None:
      return None
    out = copy.deepcopy(data)
    for i,d in enumerate(filter_):
      if np.isnan(d):
        out[i] = np.nan
    return out

  def _plot(self, x, y, y_label, style = None, filter_ = None, z = None, source = None):
    if type(y[0]) is unicode:
      if filter_ is not None:
        return False
      return self._polyAxis._plot_string(x, y, y_label, style)
    if style is None:
      return self._plot(x, y, y_label, LineStyle(color = self.figure._next_color()), filter_ = filter_, z = z, source = source)
    if y_label in self.plots:
      return False
    self._axis.get_yaxis().set_visible(True)
    self.data[y_label] = [x, y, z, filter_]
    if filter_ is not None:
      x = self._filter(x, filter_)
      y = self._filter(y, filter_)
      z = self._filter(z, filter_)
      self.filtered[y_label] = [x, y, z]
    else:
      self.filtered[y_label] = None
    self.source[y_label] = source
    if z is None:
      self.plots[y_label] = self._axis.plot(x, y, label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]
    else:
      self.plots[y_label] = self._axis.plot(x, y, z, label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]
    self.legend()
    return True

  def startAnimation(self, i0):
    for y_label in self.plots.keys():
      style = self.style(y_label)
      self.plots[y_label].remove()
      if not self._3D:
        self.plots[y_label] = self._axis.plot(self.data[y_label][0][i0], self.data[y_label][1][i0], label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]
      else:
        self.plots[y_label] = self._axis.plot([self.data[y_label][0][i0]], [self.data[y_label][1][i0]], [self.data[y_label][2][0]], label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]

  def stopAnimation(self):
    for y_label in self.plots.keys():
      style = self.style(y_label)
      self.plots[y_label].remove()
      if self.filtered[y_label] is not None:
        if not self._3D:
          self.plots[y_label] = self._axis.plot(self.filtered[y_label][0], self.filtered[y_label][1], label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]
        else:
          self.plots[y_label] = self._axis.plot(self.filtered[y_label][0], self.filtered[y_label][1], self.filtered[y_label][2], label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]
      else:
        self.plots[y_label] = self._axis.plot(self.data[y_label][0], self.data[y_label][1], label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]

  def add_plot(self, x, y, y_label, style = None):
    return self._plot(self._data()[x], self._data()[y], y_label, style, source = y)

  def add_plot_xy(self, x, y, y_label, t, style = None):
    return self._plot(self._data()[x], self._data()[y], y_label, style, filter_ = self._data()[t], source = [x, y, y_label])

  def add_plot_xyz(self, x, y, z, y_label, t, style = None):
    return self._plot(self._data()[x], self._data()[y], y_label, style, filter_ = self._data()[t], z = self._data()[z], source = [x, y, z, y_label])

  def add_diff_plot(self, x, y, y_label):
    dt = self._data()[x][1] - self._data()[x][0]
    return self._plot(self._data()[x][1:], np.diff(self._data()[y])/dt, y_label)

  def _add_rpy_plot(self, x_label, y, idx):
    assert (idx >= 0 and idx <= 2),"index must be 0, 1 or 2"
    rpy_label = ['roll', 'pitch', 'yaw']
    y_label = "{}_{}".format(y, rpy_label[idx])
    fmt = ""
    if "{}_qw".format(y) in self._data().keys():
      fmt = "q"
    qw, qx, qy, qz = [ self._data()[k] for k in [ "{}_{}{}".format(y, fmt, ax) for ax in ["w", "x", "y", "z"] ] ]
    data = [ rpyFromQuat([w, x, y, z])[idx] for w, x, y, z in zip(qw, qx, qy, qz) ]
    return self._plot(self._data()[x_label], data, y_label)

  def add_roll_plot(self, x, y):
    return self._add_rpy_plot(x, y, 0)

  def add_pitch_plot(self, x, y):
    return self._add_rpy_plot(x, y, 1)

  def add_yaw_plot(self, x, y):
    return self._add_rpy_plot(x, y, 2)

  def add_rpy_plot(self, x, y):
    r = self.add_roll_plot(x, y)
    p = self.add_pitch_plot(x, y)
    y = self.add_yaw_plot(x, y)
    return r or p or y

  def remove_plot(self, y):
    if y not in self.plots:
      self._polyAxis.remove_plot(y)
      for y_label, source in self.source.items():
        if source == y:
          return self.remove_plot(y_label)
      return
    self.plots[y].remove()
    del self.plots[y]
    del self.data[y]
    del self.filtered[y]
    del self.source[y]
    if len(self.plots):
      self._axis.relim()
      self.legend()
    else:
      self._axis.get_yaxis().set_visible(False)
      self._axis.clear()

  def clear(self):
    self.plots = {}
    self._axis.clear()

  # Get or set the style of a given plot
  def style(self, y, style = None):
    if y not in self.plots:
      raise KeyError("No plot named {}".format(y))
    plt = self.plots[y]
    if style is None:
      return LineStyle(plt.get_color(), plt.get_linestyle(), plt.get_linewidth(), label = plt.get_label())
    plt.set_color(style.color)
    plt.set_linestyle(style.linestyle)
    plt.set_linewidth(style.linewidth)
    if len(style.label):
      plt.set_label(style.label)

class PlotFigure(object):
  def __init__(self, type_ = PlotType.TIME, init_canvas = False):
    self.fig = matplotlib.pyplot.figure(figsize=(5, 4), dpi=100)
    if init_canvas:
      self.canvas = FigureCanvas(self.fig)
    self.axes = {}
    self._3D = type_ is PlotType._3D
    self.axes[PlotSide.LEFT] = PlotYAxis(self, _3D = self._3D)
    if not self._3D:
      self.axes[PlotSide.RIGHT] = PlotYAxis(self, self._left().axis(), self._left()._polyAxis, _3D = self._3D)
    else:
      self.axes[PlotSide.RIGHT] = None
    self.animation = None

    self._title_fontsize = 12
    self._x_label_fontsize = 10
    self._labelpad = 10
    self._tick_labelsize = 10
    self._legend_fontsize = 10
    self._labels_legend_fontsize = 10
    self._labels_legend_top_offset = 0.9
    self._top_offset = 0.9
    self._bottom_offset = 0.1
    if self._3D:
      self._bottom_offset = 0

    self.data = None
    self.computed_data = {}

    self.color = 0
    cm = matplotlib.cm.Set1
    self.Ncolor = min(cm.N, 12)
    cm2rgb = (np.array(cm(x)[0:3]) for x in np.linspace(0, 1, self.Ncolor))
    self.colors = ['#%02x%02x%02x' % tuple((255 * rgb).astype(int)) for rgb in cm2rgb]

    self.polyColor = 0
    cm = matplotlib.cm.Pastel1
    cm2rgb = (np.array(cm(x)[0:3] + (0.5,)) for x in np.linspace(0, 1, min(cm.N, 32)))
    self.polyColors = [rgb for rgb in cm2rgb]

    self.x_data = 't'

  # Helper function to call something on all axes, call expects an axis argument
  def _axes(self, call):
    call(self._left())
    if self._right() is not None:
      call(self._right())

  # Shortcut to get the polygon axis
  def _polygons(self):
    return self._left()._polyAxis

  # Shortcut to the left axis
  def _left(self):
    return self.axes[PlotSide.LEFT]

  # Shortcut to the right axis
  def _right(self):
    return self.axes[PlotSide.RIGHT]

  def _drawGrid(self):
    self._axes(lambda axis: axis.drawGrid())

  def _legend(self):
    self._axes(lambda axis: axis.legend())

  def draw(self, x_limits = None, y1_limits = None, y2_limits = None, frame = None):
    if self._3D:
      x_limits = self._left().setLimits(x_limits, y1_limits, frame = frame, zlim = y2_limits)
    else:
      x_limits = self._left().setLimits(x_limits, y1_limits, frame = frame)
      self._right().setLimits(x_limits, y2_limits, frame = frame)
    self._legend()
    self._drawGrid()
    top_offset = self._left().legendOffset(self._top_offset, -1)
    if self._right() is not None:
      bottom_offset = self._right().legendOffset(self._bottom_offset, 1)
    else:
      bottom_offset = self._bottom_offset
    left_offset = 0.125
    right_offset = 0.9
    if self._left()._polyAxis is not None and len(self._left()._polyAxis.plots):
      left_offset, right_offset = 0.05, right_offset - (left_offset - 0.05)
    self.fig.subplots_adjust(left = left_offset, right = right_offset, top = top_offset, bottom = bottom_offset)

  def animate(self, frame0, frame, x_limits = None, y1_limits = None, y2_limits = None):
    ret = self._left().animate(frame0, frame)
    if self._right():
      ret.extend(self._right().animate(frame0, frame))
    PlotFigure.draw(self, x_limits = x_limits, y1_limits = y1_limits, y2_limits = y2_limits, frame = frame)
    return ret

  def stopAnimation(self):
    self._axes(lambda a: a.stopAnimation())

  def setData(self, data):
    self.data = data

  def setColors(self, colors):
    self.colors = colors
    self.Ncolor = len(self.colors)

  def setPolyColors(self, colors):
    self.polyColors = colors

  def show(self):
    self.fig.show()

  def top_offset(self, off = None):
    if off is None:
      return self._top_offset
    else:
      self._top_offset = off

  def bottom_offset(self, off = None):
    if off is None:
      return self._bottom_offset
    else:
      self._bottom_offset = off

  def title(self, title = None):
    if title is None:
      if self.fig._suptitle is None:
        return ""
      return self.fig._suptitle.get_text()
    self.fig.suptitle(title)

  def title_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._title_fontsize
    self._title_fontsize = fontsize
    self.fig.suptitle(self.title(), fontsize = self._title_fontsize)

  def tick_fontsize(self, size = None):
    if size is None:
      return self._tick_labelsize
    self._tick_labelsize = size
    self._axes(lambda a: a.axis().tick_params(labelsize = self._tick_labelsize))

  def labelpad(self, pad = None):
    if pad is None:
      return self._labelpad
    self._labelpad = pad
    self._left()._x_label()
    self._axes(lambda axis: axis._y_label())

  def _x_label(self, label = None):
    self._left()._x_label(label)

  def _y1_label(self, label = None):
    self._left()._y_label(label)

  def _y2_label(self, label = None):
    self._right()._y_label(label)

  def x_label(self, label = None):
    return self._left().x_label(label)

  def x_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._x_label_fontsize
    self._x_label_fontsize = fontsize
    self._x_label()

  def y1_label(self, label = None):
    return self._left().y_label(label)

  def y1_label_fontsize(self, fontsize = None):
    return self._left().y_label_fontsize(fontsize)

  def y2_label(self, label = None):
    if self._3D:
      return self._left().z_label(label)
    return self._right().y_label(label)

  def y2_label_fontsize(self, fontsize = None):
    if self._3D:
      return self._left().z_label_fontsize(fontsize)
    return self._right().y_label_fontsize(fontsize)

  def _next_poly_color(self):
    self.polyColor += 1
    return self.polyColors[ (self.polyColor - 1) % len(self.polyColors) ]

  def _next_color(self):
    self.color += 1
    return self.colors[ (self.color - 1) % self.Ncolor ]

  def legend_fontsize(self, size = None):
    if size is None:
      return self._legend_fontsize
    self._legend_fontsize = size
    self._legend()

  def labels_legend_fontsize(self, size = None):
    if size is None:
      return self._labels_legend_fontsize
    self._labels_legend_fontsize = size
    self._legend()

  def labels_legend_top_offset(self, offset = None):
    if offset is None:
      return self._labels_legend_top_offset
    self._labels_legend_top_offset = offset
    self._legend()

  def y1_legend_ncol(self, n = None):
    return self._left().legendNCol(n)

  def y2_legend_ncol(self, n = None):
    if self._right() is not None:
      return self._right().legendNCol(n)
    else:
      return 0

  def add_plot_left(self, x, y, y_label, style = None):
    return self._left().add_plot(x, y, y_label, style)

  def add_plot_left_xy(self, x, y, y_label, style = None):
    return self._left().add_plot_xy(x, y, y_label, self.x_data, style)

  def add_plot_left_xyz(self, x, y, z, y_label, style = None):
    return self._left().add_plot_xyz(x, y, z, y_label, self.x_data, style)

  def add_plot_right(self, x, y, y_label, style = None):
    return self._right().add_plot(x, y, y_label, style)

  def add_plot_right_xy(self, x, y, y_label, style = None):
    return self._right().add_plot_xy(x, y, y_label, self.x_data, style)

  def add_plot_right_xyz(self, x, y, z, y_label, style = None):
    return self._right().add_plot_xyz(x, y, z, y_label, self.x_data, style)

  def add_diff_plot_left(self, x, y, y_label):
    return self._left().add_diff_plot(x, y, y_label)

  def add_diff_plot_right(self, x, y, y_label):
    return self._right().add_diff_plot(x, y, y_label)

  def add_roll_plot_left(self, x, y):
    return self._left().add_roll_plot(x, y)

  def add_pitch_plot_left(self, x, y):
    return self._left().add_pitch_plot(x, y)

  def add_yaw_plot_left(self, x, y):
    return self._left().add_yaw_plot(x, y)

  def add_roll_plot_right(self, x, y):
    return self._right().add_roll_plot(x, y)

  def add_pitch_plot_right(self, x, y):
    return self._right().add_pitch_plot(x, y)

  def add_yaw_plot_right(self, x, y):
    return self._right().add_yaw_plot(x, y)

  def add_rpy_plot_left(self, x, y):
    return self._left().add_rpy_plot(x, y)

  def add_rpy_plot_right(self, x, y):
    return self._right().add_rpy_plot(x, y)

  def _remove_plot(self, SIDE, y_label):
    self.axes[SIDE].remove_plot(y_label)
    if len(self._left()) == 0 and self._right() and len(self._right()) == 0:
      self.color = 0

  def remove_plot_left(self, y_label):
    self._remove_plot(PlotSide.LEFT, y_label)

  def remove_plot_right(self, y_label):
    self._remove_plot(PlotSide.RIGHT, y_label)

  def format_coord(self, x, y):
    if self._right() is not None:
      display_coord = self.axes[PlotSide.RIGHT].axis().transData.transform((x,y))
      inv = self.axes[PlotSide.LEFT].axis().transData.inverted()
      ax_coord = inv.transform(display_coord)
      if len(self._left()) and len(self._right()):
        return "x: {:.3f}    y1: {:.3f}    y2: {:.3f}".format(x, ax_coord[1], y)
      elif len(self._left()):
        return "x: {:.3f}    y1: {:.3f}".format(x, ax_coord[1])
      elif len(self._right()):
        return "x: {:.3f}    y2: {:.3f}".format(x, y)
      else:
        return "x: {:.3f}".format(x)
    else:
      axis = self.axes[PlotSide.LEFT].axis()
      return type(axis).format_coord(axis, x, y)

  def clear_all(self):
    self.color = 0
    self._axes(lambda a: a.clear())

  def style_left(self, y, styleIn = None):
    return self._left().style(y, styleIn)

  def style_right(self, y, styleIn = None):
    return self._right().style(y, styleIn)

class SimpleAxesDialog(QtWidgets.QDialog):
  def __init__(self, parent):
    QtWidgets.QDialog.__init__(self, parent)
    self.setWindowTitle('Edit axes limits')
    self.setModal(True)
    self.layout = QtWidgets.QGridLayout(self)
    self.layout.addWidget(QtWidgets.QLabel("Min"), 0, 1)
    self.layout.addWidget(QtWidgets.QLabel("Max"), 0, 2)

    self.layout.addWidget(QtWidgets.QLabel("X"), 1, 0)
    x_limits = parent.x_limits
    if x_limits is None:
      x_limits = parent._left().axis().get_xlim()
    self.x_min = QtWidgets.QLineEdit(str(x_limits[0]))
    self.x_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.x_min, 1, 1)
    self.x_max = QtWidgets.QLineEdit(str(x_limits[1]))
    self.x_max.setValidator(QtGui.QDoubleValidator())
    self.x_init = [float(self.x_min.text()), float(self.x_max.text())]
    self.layout.addWidget(self.x_max, 1, 2)

    self.layout.addWidget(QtWidgets.QLabel("Y1"), 2, 0)
    y1_limits = parent.y1_limits
    if y1_limits is None:
      y1_limits = parent._left().axis().get_ylim()
    self.y1_min = QtWidgets.QLineEdit(str(y1_limits[0]))
    self.y1_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.y1_min, 2, 1)
    self.y1_max = QtWidgets.QLineEdit(str(y1_limits[1]))
    self.y1_max.setValidator(QtGui.QDoubleValidator())
    self.y1_init = [float(self.y1_min.text()), float(self.y1_max.text())]
    self.layout.addWidget(self.y1_max, 2, 2)

    y2_label = "Y2"
    if parent._3D:
      y2_label = "Z"
    self.layout.addWidget(QtWidgets.QLabel(y2_label), 3, 0)
    y2_limits = parent.y2_limits
    if y2_limits is None:
      if parent._3D:
        y2_limits = parent._left().axis().get_zlim()
      else:
        y2_limits = parent._right().axis().get_ylim()
    self.y2_min = QtWidgets.QLineEdit(str(y2_limits[0]))
    self.y2_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.y2_min, 3, 1)
    self.y2_max = QtWidgets.QLineEdit(str(y2_limits[1]))
    self.y2_max.setValidator(QtGui.QDoubleValidator())
    self.y2_init = [float(self.y2_min.text()), float(self.y2_max.text())]
    self.layout.addWidget(self.y2_max, 3, 2)

    confirmLayout = QtWidgets.QHBoxLayout()
    okButton = QtWidgets.QPushButton("Ok", self)
    confirmLayout.addWidget(okButton)
    okButton.clicked.connect(self.accept)
    applyButton = QtWidgets.QPushButton("Apply", self)
    confirmLayout.addWidget(applyButton)
    applyButton.clicked.connect(self.apply)
    cancelButton = QtWidgets.QPushButton("Cancel", self)
    confirmLayout.addWidget(cancelButton)
    cancelButton.clicked.connect(self.reject)
    self.layout.addLayout(confirmLayout, 4, 0, 1, 3)

  def apply(self):
    changed = False
    x_limits = [float(self.x_min.text()), float(self.x_max.text())]
    if x_limits != self.x_init:
      changed = True
      self.parent().x_locked.setChecked(True)
      self.parent().x_limits = x_limits
    y1_limits = [float(self.y1_min.text()), float(self.y1_max.text())]
    if y1_limits != self.y1_init:
      changed = True
      self.parent().y1_locked.setChecked(True)
      self.parent().y1_limits = y1_limits
    y2_limits = [float(self.y2_min.text()), float(self.y2_max.text())]
    if y2_limits != self.y2_init:
      changed = True
      self.parent().y2_locked.setChecked(True)
      self.parent().y2_limits = y2_limits
    if changed:
      self.parent().draw()

  def accept(self):
    QtWidgets.QDialog.accept(self)
    self.apply()

class PlotCanvasWithToolbar(PlotFigure, QWidget):
  def __init__(self, parent = None, mode = PlotType.TIME):
    PlotFigure.__init__(self, mode, True)
    QWidget.__init__(self, parent)

    self.canvas.mpl_connect('draw_event', self.on_draw)
    self.toolbar = NavigationToolbar(self.canvas, self)

    self.layout = QVBoxLayout(self)
    self.layout.addWidget(self.canvas)
    self.layout.addWidget(self.toolbar)

    self.setupLockButtons()
    self.setupAnimationButtons()

  def setupLockButtons(self):
    layout = QHBoxLayout()

    self.xSelector = QComboBox(self)
    self.xSelector.activated.connect(lambda: self.parent().on_xSelector_activated(self, self.xSelector.currentText()))
    sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    sizePolicy.setHorizontalStretch(0)
    sizePolicy.setVerticalStretch(0)
    sizePolicy.setHeightForWidth(self.xSelector.sizePolicy().hasHeightForWidth())
    self.xSelector.setSizePolicy(sizePolicy)
    layout.addWidget(self.xSelector)

    self.genRangeButton = QtWidgets.QPushButton(u"Make range")
    self.genRangeButton.released.connect(lambda: GenerateRangeDialog(self).exec_())
    layout.addWidget(self.genRangeButton)

    self.x_locked = QtWidgets.QPushButton(u"🔒X", self)
    self.x_locked.setCheckable(True)
    layout.addWidget(self.x_locked)
    self.x_locked.toggled.connect(self.x_locked_changed)
    self.x_limits = None

    self.y1_locked = QtWidgets.QPushButton(u"🔒 Y1", self)
    self.y1_locked.setCheckable(True)
    layout.addWidget(self.y1_locked)
    self.y1_locked.toggled.connect(self.y1_locked_changed)
    self.y1_limits = None

    self.y2_locked = QtWidgets.QPushButton(u"🔒 Y2", self)
    if self._3D:
      self.y2_locked.setText(u"🔒 Z")
    self.y2_locked.setCheckable(True)
    layout.addWidget(self.y2_locked)
    self.y2_locked.toggled.connect(self.y2_locked_changed)
    self.y2_limits = None

    self.layout.addLayout(layout)

  def setupAnimationButtons(self):
    animationLayout = QtWidgets.QHBoxLayout()
    self.animation = None
    self.animationButton = QtWidgets.QPushButton("Start animation")
    self.animationButton.setCheckable(True)
    self.animationButton.toggled.connect(self.startStopAnimation)
    animationLayout.addWidget(self.animationButton)
    self.lockAxesButton = QtWidgets.QPushButton("Lock axes")
    self.lockAxesButton.released.connect(self.lockAxes)
    animationLayout.addWidget(self.lockAxesButton)
    self.saveAnimationButton = QtWidgets.QPushButton("Save animation")
    self.saveAnimationButton.released.connect(self.saveAnimation)
    animationLayout.addWidget(self.saveAnimationButton)
    self.layout.addLayout(animationLayout)

  def setData(self, data):
    super(PlotCanvasWithToolbar, self).setData(data)
    self.update_x_selector()

  def update_x_selector(self):
    self.xSelector.clear()
    self.xSelector.addItems(sorted(self.data.keys()))
    idx = self.xSelector.findText(self.x_data)
    if idx != -1:
      self.xSelector.setCurrentIndex(idx)

  def startStopAnimation(self):
    if self.animationButton.isChecked():
      if self.startAnimation():
        self.animationButton.setText("Stop animation")
      else:
        self.animationButton.setChecked(False)
    else:
      self.stopAnimation()
      self.animationButton.setText("Start animation")

  def restartAnimation(self):
    if self.animationButton.isChecked():
      self.stopAnimation()
      self.startAnimation()

  def update_x(self):
    self._axes(lambda a: a.update_x(self.data[self.x_data]))
    if self._polygons():
      self._polygons().update_x(self.data[self.x_data])
    self.restartAnimation()
    self.draw()

  def getFrameRange(self):
    if self.data is None or len(self.data) == 0:
      return 0, 0
    x_data = self.data[self.x_data]
    i0 = 0
    while i0 < len(x_data) and np.isnan(x_data[i0]):
      i0 += 1
    iN = i0
    while iN + 1 < len(x_data) and not np.isnan(x_data[iN + 1]):
      iN += 1
    assert(iN > i0 and i0 < len(x_data)),"Strange time range"
    return i0, iN

  def lockAxes(self):
    i0, iN = self.getFrameRange()
    if i0 == iN:
      return
    if self.x_limits is None:
      self.x_limits = self._left().getLimits(iN, 0) or self._right().getLimits(iN, 0)
    if self.x_limits is None:
      return
    self.x_locked.setChecked(True)
    if self.y1_limits is None:
      self.y1_limits = self._left().getLimits(iN, 1)
    if self.y1_limits is not None:
      self.y1_locked.setChecked(True)
    if self.y2_limits is None:
      if self._3D:
        self.y2_limits = self._left().getLimits(iN, 2)
      else:
        self.y2_limits = self._right().getLimits(iN, 1)
    if self.y2_limits is not None:
      self.y2_locked.setChecked(True)
    return i0, iN

  def startAnimation(self):
    interval = 50 # ms
    i0, iN = self.getFrameRange()
    if i0 == iN:
      return False
    x_data = self.data[self.x_data]
    dt = (x_data[i0 + 1] - x_data[i0]) * 1000 # dt in ms
    step = int(math.ceil(interval/dt))
    self.frame0 = i0
    self.animation = FuncAnimation(self.fig, self.animate, frames = range(i0 + 1, iN, step), interval = interval)
    self._axes(lambda a: a.startAnimation(i0))
    self.draw()
    return True

  def animate(self, frame):
    return PlotFigure.animate(self, self.frame0, frame, self.x_limits, self.y1_limits, self.y2_limits)

  def stopAnimation(self):
    self.animation.event_source.stop()
    PlotFigure.stopAnimation(self)
    self.draw()

  def saveAnimation(self):
    fpath = QtWidgets.QFileDialog.getSaveFileName(self, "Output file", filter = "Video (*.mp4)")[0]
    if not len(fpath):
      return
    if self.animationButton.isChecked():
      self.animation.save(fpath)
    else:
      self.startAnimation()
      self.animation.save(fpath)
      self.stopAnimation()

  def axesDialog(self):
    SimpleAxesDialog(self).exec_()

  def on_draw(self, event):
    if self.x_limits is not None:
      self.x_limits = self._left().axis().get_xlim()
    if self.y1_limits is not None:
      self.y1_limits = self._left().axis().get_ylim()
    if self.y2_limits is not None:
      if self._3D:
        self.y2_limits = self._left().axis().get_zlim()
      else:
        self.y2_limits = self._right().axis().get_ylim()

  def draw(self):
    PlotFigure.draw(self, self.x_limits, self.y1_limits, self.y2_limits)
    self.canvas.draw()

  def _y_lock_changed(self, name, cbox, get_lim):
    if cbox.isChecked():
      cbox.setText(u"🔓 {}".format(name))
      return get_lim()
    else:
      cbox.setText(u"🔒{}".format(name))
      return None

  def x_locked_changed(self, status):
    self.x_limits = self._y_lock_changed("X", self.x_locked, self._left().axis().get_xlim)
    if self.x_limits is None:
      self.draw()

  def y1_locked_changed(self, status):
    self.y1_limits = self._y_lock_changed("Y1", self.y1_locked, self._left().axis().get_ylim)
    if self.y1_limits is None:
      self.draw()

  def y2_locked_changed(self, status):
    if self._3D:
      self.y2_limits = self._y_lock_changed("Z", self.y2_locked, self._left().axis().get_zlim)
    else:
      self.y2_limits = self._y_lock_changed("Y2", self.y2_locked, self._right().axis().get_ylim)
    if self.y2_limits is None:
      self.draw()

  def show(self):
    return QWidget.show(self)
