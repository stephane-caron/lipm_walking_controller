#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import signal
import sys

from PyQt5 import QtGui, QtWidgets

from mc_log_ui import MCLogUI, get_icon

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setWindowIcon(QtGui.QIcon(get_icon()))
    gui = MCLogUI()
    files = ["2019-02-21_stair_climbing.flat"]
    if len(sys.argv) > 1:
        files.extend(sys.argv[1:])
    for fpath in files:
        gui.load_csv(fpath, False)
    gui.showMaximized()

    msg = QtWidgets.QMessageBox()
    msg.setWindowTitle("Tutorial on PyQt5")
    msg.setText(
        "Welcome the log GUI from mc_rtc!\n\n"
        "This script has automatically opened a stair climbing log "
        "from the 2019/02/21 demo at the Airbus Saint-Nazaire factory. "
        'Check out the pre-defined plots in the "User plots" menu above. '
        "The ones we use the most are:\n\n"
        "– Tracking DCM-ZMP X/Y\n"
        "– Foot force both\n"
        "– Swing foot Z\n"
    )
    msg.setIconPixmap(
        QtGui.QPixmap("./mc_log_ui/icons/icon.png").scaled(64, 64)
    )
    msg.exec_()

    def sigint_handler(*args):
        gui.close()

    signal.signal(signal.SIGINT, sigint_handler)
    sys.exit(app.exec_())
