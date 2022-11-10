# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from enum import Enum

import json
import os

import numpy as np

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt

class PlotSide(Enum):
  LEFT = 0
  RIGHT = 1

class PlotType(Enum):
  TIME = 0
  XY = 1
  _3D = 2

class LineStyle(object):
  def __init__(self, color = 'black', linestyle = '-', linewidth = 0.5, visible = False, label = ""):
    self.color = color
    self.linestyle = linestyle
    self.linewidth = linewidth
    self.visible = visible
    self.label = label
  def __repr__(self):
    return "color: {}, linestyle: {}, linewidth: {}, visible: {}, label: {}".format(self.color, self.linestyle, self.linewidth, self.visible, self.label)

class TextWithFontSize(object):
  def __init__(self, text = "", fontsize = 10):
    self.text = text
    self.fontsize = fontsize

class GraphLabels(object):
  def __init__(self, title = TextWithFontSize(fontsize = 12), x_label = TextWithFontSize(), y1_label = TextWithFontSize(), y2_label = TextWithFontSize()):
    self.title = title
    self.x_label = x_label
    self.y1_label = y1_label
    self.y2_label = y2_label

class ColorsSchemeConfiguration(object):
  def __init__(self, f, default = 'Set1'):
    cm = default
    ncolors = 12
    data = {}
    if os.path.exists(f):
      with open(f) as fd:
        data = json.load(fd)
    if 'cm' in data:
      cm = data['cm']
    if 'ncolors' in data:
      ncolors = int(data['ncolors'])
    if cm != 'custom':
      self._select_pyplot_set(cm, ncolors)
    else:
      self._select_custom_set(data['colors'])
  def _select_pyplot_set(self, name, ncolors):
    self.cm_ = name
    cm = plt.cm.get_cmap(name)
    self.ncolors_ = min(cm.N, ncolors)
    cm2rgb = (np.array(cm(x)[0:3]) for x in np.linspace(0, 1, self.ncolors_))
    self.colors_ = ['#%02x%02x%02x' % tuple((255 * rgb).astype(int)) for rgb in cm2rgb]
  def _select_custom_set(self, colors):
    assert(len(colors)),"Cannot create a custom set without colors"
    self.cm_ = 'custom'
    self.colors_ = colors
    self.ncolors_ = len(self.colors_)
  def colors(self):
    return self.colors_
  def save(self, f):
    with open(f, 'w') as fd:
      json.dump({'cm': self.cm_, 'ncolors': self.ncolors_, 'colors': self.colors_}, fd, indent = 2)
