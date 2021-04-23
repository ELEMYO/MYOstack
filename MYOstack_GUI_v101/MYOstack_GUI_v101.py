# Graphical interface for signal visualization and interaction with ELEMYO MYOstack sensors
# 2021-04-23 by ELEMYO (https://github.com/ELEMYO/ELEMYO GUI)
# 
# Changelog:
#     2021-04-23 - initial release

# Code is placed under the MIT license
# Copyright (c) 2021 ELEMYO
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
# ===============================================

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import Qt 
import sys
import serial
import pyqtgraph as pg
import numpy as np
import time
from scipy.signal import butter, lfilter
import serial.tools.list_ports
from datetime import datetime
from scipy.fftpack import fft

# Main window
class GUI(QtWidgets.QMainWindow):
    # Initialize constructor
    def __init__(self):
          super(GUI, self).__init__()
          self.initUI()
    # Custom constructor
    def initUI(self): 
        # Values
        COM = '' #Example: COM='COM6'
        baudRate = 1000000 #Serial frequency
        self.f = open(datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".txt", "w")
        self.f.write(datetime.now().strftime("Date: %Y.%m.%d\rTime: %H:%M:%S") + "\r\n")
        self.f.write("File format: \r\nseconds | data1 | data2 | data3 | data4 | data5 | data6 | data7| data8 | data9 \r\n")
        self.l = 0 #Current point
        self.dt = 0.00125 #Updating time s
        self.fs = 1 / self.dt #Updating frequency in Hz
        self.passLowFrec = 10.0 #Low frequency for passband filter
        self.passHighFrec = 200.0 #Low frequency for passband filter
        self.dataWidth = 10000 #Maximum count of data points
        self.xH = [0]*10
        self.Time = [0]*self.dataWidth #Tine array
        self.timeWidth = 10 #Time width of plot
        self.Data = np.zeros((9, self.dataWidth))
        self.DataMovingAverage = np.zeros((9, self.dataWidth))
        self.MA = np.zeros((9, 3))
        self.MA_alpha = 0.95
        self.Y0 = np.zeros(9)
        self.X0 = np.zeros(9)
        self.FFT = 0
        self.msg_end = np.array([0])
        self.setWindowTitle("MYOstack GUI v1.0.1 | ELEMYO" + "    ( COM Port not found )")
        self.setWindowIcon(QtGui.QIcon('img/icon.png'))
        # Menu panel
        startAction = QtGui.QAction(QtGui.QIcon('img/start.png'), 'Start (Enter)', self)
        startAction.setShortcut('Return')
        startAction.triggered.connect(self.start)
        stopAction = QtGui.QAction(QtGui.QIcon('img/pause.png'), 'Stop (Space)', self)
        stopAction.setShortcut('Space')
        stopAction.triggered.connect(self.stop)
        refreshAction = QtGui.QAction(QtGui.QIcon('img/refresh.png'), 'Refresh (R)', self)
        refreshAction.setShortcut('r')
        refreshAction.triggered.connect(self.refresh)
        exitAction = QtGui.QAction(QtGui.QIcon('img/out.png'), 'Exit (Esc)', self)
        exitAction.setShortcut('Esc')
        exitAction.triggered.connect(self.close)
        # Toolbar
        toolbar = self.addToolBar('Tool')
        toolbar.addAction(startAction)
        toolbar.addAction(stopAction)
        toolbar.addAction(refreshAction)
        toolbar.addAction(exitAction)
        
        # Plot widget for 1st sensor
        self.pw1 = pg.PlotWidget(background = (21 , 21, 21, 255))
        self.pw1.showGrid(x = True, y = True, alpha = 0.7) 
        self.p1 = self.pw1.plot()
        self.p1.setPen(color=(100,255,255), width=1)
        # Plot widget for 2nd sensor
        self.pw2 = pg.PlotWidget(background = (13 , 13, 13, 255))
        self.pw2.showGrid(x = True, y = True, alpha = 0.7) 
        self.p2 = self.pw2.plot()
        self.p2.setPen(color=(100,255,255), width=1)
        # Plot widget for 3rd sensor
        self.pw3 = pg.PlotWidget(background = (21 , 21, 21, 255))
        self.pw3.showGrid(x = True, y = True, alpha = 0.7) 
        self.p3 = self.pw3.plot()
        self.p3.setPen(color=(100,255,255), width=1)
        # Plot widget for 4th sensor
        self.pw4 = pg.PlotWidget(background = (13 , 13, 13, 255))
        self.pw4.showGrid(x = True, y = True, alpha = 0.7) 
        self.p4 = self.pw4.plot()
        self.p4.setPen(color=(100,255,255), width=1)
        # Plot widget for 5th sensor
        self.pw5 = pg.PlotWidget(background = (21 , 21, 21, 255))
        self.pw5.showGrid(x = True, y = True, alpha = 0.7) 
        self.p5 = self.pw5.plot()
        self.p5.setPen(color=(100,255,255), width=1)
        # Plot widget for 6th sensor
        self.pw6 = pg.PlotWidget(background = (13 , 13, 13, 255))
        self.pw6.showGrid(x = True, y = True, alpha = 0.7) 
        self.p6 = self.pw6.plot()
        self.p6.setPen(color=(100,255,255), width=1)
        # Plot widget for 7th sensor
        self.pw7 = pg.PlotWidget(background = (21 , 21, 21, 255))
        self.pw7.showGrid(x = True, y = True, alpha = 0.7) 
        self.p7 = self.pw7.plot()
        self.p7.setPen(color=(100,255,255), width=1)
        # Plot widget for 8th sensor
        self.pw8 = pg.PlotWidget(background = (13 , 13, 13, 255))
        self.pw8.showGrid(x = True, y = True, alpha = 0.7) 
        self.p8 = self.pw8.plot()
        self.p8.setPen(color=(100,255,255), width=1)
        # Plot widget for 9th sensor
        self.pw9 = pg.PlotWidget(background = (21 , 21, 21, 255))
        self.pw9.showGrid(x = True, y = True, alpha = 0.7) 
        self.p9 = self.pw9.plot()
        self.p9.setPen(color=(100,255,255), width=1)
        
        # Plot widget for spectral Plot
        self.pw10 = pg.PlotWidget(background = (13 , 13, 13, 255))
        self.pw10.showGrid(x = True, y = True, alpha = 0.7) 
        self.p10 = self.pw10.plot()
        self.p10.setPen(color=(100,255,255), width=1)
        self.pw10.setLabel('bottom', 'Frequency', 'Hz')
        
        # Plot widget for histogram
        self.pbar = pg.PlotWidget(background = (13 , 13, 13, 255))
        self.pbar.showGrid(x = True, y = True, alpha = 0.7)            
        self.pb1 = pg.BarGraphItem(x=np.linspace(1, 2, num=1), height=np.linspace(1, 2, num=1), width=0.3, pen = QtGui.QColor(153,0,0), brush=QtGui.QColor(153,0,0))
        self.pb2 = pg.BarGraphItem(x=np.linspace(2, 3, num=1), height=np.linspace(2, 3, num=1), width=0.3, pen=QtGui.QColor(229, 104, 19), brush=QtGui.QColor(229, 104, 19))
        self.pb3 = pg.BarGraphItem(x=np.linspace(3, 4, num=1), height=np.linspace(3, 4, num=1), width=0.3, pen=QtGui.QColor(221, 180, 10), brush=QtGui.QColor(221, 180, 10))
        self.pb4 = pg.BarGraphItem(x=np.linspace(4, 5, num=1), height=np.linspace(4, 5, num=1), width=0.3, pen=QtGui.QColor(30, 180, 30), brush=QtGui.QColor(30, 180, 30))
        self.pb5 = pg.BarGraphItem(x=np.linspace(5, 6, num=1), height=np.linspace(5, 6, num=1), width=0.3, pen=QtGui.QColor(11, 50, 51), brush=QtGui.QColor(11, 50, 51))
        self.pb6 = pg.BarGraphItem(x=np.linspace(6, 7, num=1), height=np.linspace(6, 7, num=1), width=0.3, pen=QtGui.QColor(29, 160, 191), brush=QtGui.QColor(29, 160, 191))
        self.pb7 = pg.BarGraphItem(x=np.linspace(7, 8, num=1), height=np.linspace(7, 8, num=1), width=0.3, pen=QtGui.QColor(30, 30, 188), brush=QtGui.QColor(30, 30, 188))
        self.pb8 = pg.BarGraphItem(x=np.linspace(8, 9, num=1), height=np.linspace(8, 9, num=1), width=0.3, pen=QtGui.QColor(75, 13, 98), brush=QtGui.QColor(75, 13, 98))
        self.pb9 = pg.BarGraphItem(x=np.linspace(9, 10, num=1), height=np.linspace(9, 10, num=1), width=0.3, pen=QtGui.QColor(139, 0, 55), brush=QtGui.QColor(139, 0, 55))
        self.pbar.addItem(self.pb1)  
        self.pbar.addItem(self.pb2)
        self.pbar.addItem(self.pb3)
        self.pbar.addItem(self.pb4)
        self.pbar.addItem(self.pb5)
        self.pbar.addItem(self.pb6)
        self.pbar.addItem(self.pb7)
        self.pbar.addItem(self.pb8)
        self.pbar.addItem(self.pb9)
        self.pbar.setLabel('bottom', 'Sensor number')
        
        # Styles
        centralStyle = "color: rgb(255, 255, 255); background-color: rgb(13, 13, 13);"
        editStyle = "border-style: solid; border-width: 1px;"
        
        # Settings zone
        filtersText = QtWidgets.QLabel("FILTERS:")
        self.passLowFreq = QtWidgets.QLineEdit(str(self.passLowFrec), self)
        self.passLowFreq.setMaximumWidth(100)
        self.passLowFreq.setStyleSheet(editStyle)
        self.passHighFreq = QtWidgets.QLineEdit(str(self.passHighFrec), self)
        self.passHighFreq.setMaximumWidth(100)
        self.passHighFreq.setStyleSheet(editStyle)
        self.bandpass = QtWidgets.QCheckBox("BANDPASS FILTER:")
        self.bandstop50 = QtWidgets.QCheckBox("NOTCH 50 Hz")
        self.bandstop60 = QtWidgets.QCheckBox("NOTCH 60 Hz")
        self.MovingAverage = QtWidgets.QCheckBox("Signal envelope")
        
        fft1 = QtWidgets.QRadioButton('1')
        fft1.setChecked(True)
        fft1.Value = 1
        fft2 = QtWidgets.QRadioButton('2')
        fft2.Value = 2
        fft3 = QtWidgets.QRadioButton('3')
        fft3.Value = 3
        fft4 = QtWidgets.QRadioButton('4')
        fft4.Value = 4
        fft5 = QtWidgets.QRadioButton('5')
        fft5.Value = 5
        fft6 = QtWidgets.QRadioButton('6')
        fft6.Value = 6
        fft7 = QtWidgets.QRadioButton('7')
        fft7.Value = 7
        fft8 = QtWidgets.QRadioButton('8')
        fft8.Value = 8
        fft9 = QtWidgets.QRadioButton('9')
        fft9.Value = 9
        self.button_group = QtWidgets.QButtonGroup()
        self.button_group.addButton(fft1, 1)
        self.button_group.addButton(fft2, 2)
        self.button_group.addButton(fft3, 3)
        self.button_group.addButton(fft4, 4)
        self.button_group.addButton(fft5, 5)
        self.button_group.addButton(fft6, 6)
        self.button_group.addButton(fft7, 7)
        self.button_group.addButton(fft8, 8)
        self.button_group.addButton(fft9, 9)
        self.button_group.buttonClicked.connect(self._on_radio_button_clicked)
        
        self.l11 = QtWidgets.QLabel("")
        self.l11.setStyleSheet("font-size: 25px; background-color: rgb(21,21,21);")
        self.l13 = QtWidgets.QLabel("")
        self.l13.setStyleSheet("font-size: 25px; background-color: rgb(21,21,21);")
        self.l15 = QtWidgets.QLabel("")
        self.l15.setStyleSheet("font-size: 25px; background-color: rgb(21,21,21);")
        self.l17 = QtWidgets.QLabel("")
        self.l17.setStyleSheet("font-size: 25px; background-color: rgb(21,21,21);")
        self.l19 = QtWidgets.QLabel("")
        self.l19.setStyleSheet("font-size: 25px; background-color: rgb(21,21,21);")
        self.l1 = QtWidgets.QLabel(" 1 ")
        self.l1.setStyleSheet("font-size: 25px; background-color: rgb(153,0,0); border-radius: 14px;")
        self.l2 = QtWidgets.QLabel(" 2")
        self.l2.setStyleSheet("font-size: 25px; background-color: rgb(229, 104, 19); border-radius: 14px;") 
        self.l3 = QtWidgets.QLabel(" 3 ")
        self.l3.setStyleSheet("font-size: 25px; background-color: rgb(221, 180, 10); border-radius: 14px;")
        self.l4 = QtWidgets.QLabel(" 4 ")
        self.l4.setStyleSheet("font-size: 25px; background-color: rgb(30, 180, 30); border-radius: 14px;")
        self.l5 = QtWidgets.QLabel(" 5 ")
        self.l5.setStyleSheet("font-size: 25px; background-color: rgb(11, 50, 51); border-radius: 14px;")
        self.l6 = QtWidgets.QLabel(" 6 ")
        self.l6.setStyleSheet("font-size: 25px; background-color: rgb(29, 160, 191); border-radius: 14px;")
        self.l7 = QtWidgets.QLabel(" 7 ")
        self.l7.setStyleSheet("font-size: 25px; background-color: rgb(30, 30, 188); border-radius: 14px;")
        self.l8 = QtWidgets.QLabel(" 8 ")
        self.l8.setStyleSheet("font-size: 25px; background-color: rgb(75, 13, 98); border-radius: 14px;")
        self.l9 = QtWidgets.QLabel(" 9 ")
        self.l9.setStyleSheet("font-size: 25px; background-color: rgb(139, 0, 55); border-radius: 14px;")
        # Main widget
        centralWidget = QtWidgets.QWidget()
        centralWidget.setStyleSheet(centralStyle)
        # Layout
        vbox = QtWidgets.QVBoxLayout()
        
        layout = QtWidgets.QGridLayout()
        layout.addWidget(self.l11, 0, 1)
        layout.addWidget(self.l1, 0, 1, Qt.AlignVCenter)
        layout.addWidget(self.l2, 1, 1, Qt.AlignVCenter)
        layout.addWidget(self.l13, 2, 1)
        layout.addWidget(self.l3, 2, 1, Qt.AlignVCenter)
        layout.addWidget(self.l4, 3, 1, Qt.AlignVCenter)
        layout.addWidget(self.l15, 4, 1, 4, 1)
        layout.addWidget(self.l5, 4, 1, 4, 1, Qt.AlignVCenter)
        layout.addWidget(self.l6, 8, 1, Qt.AlignVCenter)
        layout.addWidget(self.l17, 9, 1)
        layout.addWidget(self.l7, 9, 1, Qt.AlignVCenter)
        layout.addWidget(self.l8, 10, 1, Qt.AlignVCenter)
        layout.addWidget(self.l19, 11, 1)
        layout.addWidget(self.l9, 11, 1, Qt.AlignVCenter)
        
        layout.addWidget(self.pw1, 0, 2)
        layout.addWidget(self.pw2, 1, 2)
        layout.addWidget(self.pw3, 2, 2)
        layout.addWidget(self.pw4, 3, 2)
        layout.addWidget(self.pw5, 4, 2, 4, 1)
        layout.addWidget(self.pw6, 8, 2)
        layout.addWidget(self.pw7, 9, 2)
        layout.addWidget(self.pw8, 10, 2)
        layout.addWidget(self.pw9, 11, 2, 1, 1)
        layout.addWidget(self.pbar, 0, 3, 4, 10)
        layout.addWidget(self.pw10, 4, 3, 7, 10)
        layout.setColumnStretch(2, 2)
        

        layout.addWidget(fft1,4,10)
        layout.addWidget(fft2,4,11) 
        layout.addWidget(fft3,4,12) 
        layout.addWidget(fft4,5,10)
        layout.addWidget(fft5,5,11)  
        layout.addWidget(fft6,5,12)
        layout.addWidget(fft7,6,10)
        layout.addWidget(fft8,6,11)  
        layout.addWidget(fft9,6,12)      
        layout.addWidget(filtersText,11,3) 
        layout.addWidget(self.bandstop50,11,4) 
        layout.addWidget(self.bandstop60,11,5)
        layout.addWidget(self.bandpass,11,6) 
        layout.addWidget(self.passLowFreq,11,7) 
        layout.addWidget(self.passHighFreq,11,8)
        layout.addWidget(self.MovingAverage,11,9)    
        
        vbox.addLayout(layout)
        centralWidget.setLayout(vbox)
        self.setCentralWidget(centralWidget)  
        self.showMaximized()
        self.show()
        # Serial monitor
        self.monitor = SerialMonitor(COM, baudRate)
        self.monitor.bufferUpdated.connect(self.updateListening, QtCore.Qt.QueuedConnection)
    # Start working
    def start(self):
        self.monitor.running = True
        self.monitor.start()
    # Pause
    def stop(self):
        self.monitor.running = False    
    # Refresh
    def refresh(self):
        self.l = 0 #Current point
        self.Time = [0]*self.dataWidth #Tine array
        self.Data = np.zeros((9, self.dataWidth))
        self.DataMovingAverage = np.zeros((9, self.dataWidth))
        self.Time = [0]*self.dataWidth
        self.msg_end = 0        
    # Update
    def updateListening(self, msg):
        # Update variables
        self.setWindowTitle("MYOstack GUI v1.0.1 | ELEMYO " + 
                            "    ( " + self.monitor.COM + " , " + str(self.monitor.baudRate) + " baud )")
        s = self.passLowFreq.text()
        if s.isdigit():
            self.passLowFrec = float(s)
        s = self.passHighFreq.text()
        if s.isdigit():
            self.passHighFrec = float(self.passHighFreq.text())
        # Parsing data from serial buffer
        msg = msg.decode(errors='ignore')
        if len(msg) >= 2:
            msg_end_n = msg.rfind("\r", 1)
            msg_begin = self.msg_end
            self.msg_end = msg[msg_end_n:len(msg)]
            if(self.l > 2):
                msg = msg_begin + msg[0:msg_end_n]
            for st in msg.split('\r\n'):
                s = st.split(';')
                if (len(s) == 9) :
                    if ( self.l == self.dataWidth):
                        self.l = 0
                    for i in range(9):
                        self.Data[i][self.l] = int(s[i])/1.024*3.3
                        self.DataMovingAverage[i][self.l] = self.movingAverage(i, self.Data[i][self.l], self.MA_alpha)
                        
                    self.Time[self.l] = self.Time[self.l - 1] + self.dt
                    self.f.write(str(round(self.Time[self.l], 3)) + " " + str(self.Data[0][self.l]) + " " + str(self.Data[1][self.l]) + " "
                                 + str(self.Data[2][self.l]) + " " + str(self.Data[3][self.l]) + " "
                                 + str(self.Data[4][self.l]) + " " + str(self.Data[5][self.l]) + " "
                                 + str(self.Data[6][self.l]) + " " + str(self.Data[7][self.l]) + " "
                                 + str(self.Data[8][self.l]) + "\r\n")
                    
                    self.l = self.l + 1
                
        # Filtering
        Data = np.zeros((9, self.dataWidth))
        for i in range(9):
            Data[i] = np.concatenate((self.Data[i][self.l: self.dataWidth], self.Data[i][0: self.l]))
        
        Time = self.Time[self.l + 1: self.dataWidth-1]+(self.Time[0: self.l])
        
        if (self.bandpass.isChecked() == 1 and self.passLowFrec < self.passHighFrec 
            and self.passLowFrec > 0 and self.fs > 2*self.passHighFrec):
            for i in range(9):
                Data[i] = self.butter_bandpass_filter(Data[i], self.passLowFrec, self.passHighFrec, self.fs)
        if self.bandstop50.isChecked() == 1:
            if self.fs > 110: 
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 48, 52, self.fs)
            if self.fs > 210: 
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 98, 102, self.fs)
            if self.fs > 310: 
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 148, 152, self.fs)
            if self.fs > 410:
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 198, 202, self.fs)
        if self.bandstop60.isChecked() == 1:
            if self.fs > 130:
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 58, 62, self.fs)
            if self.fs > 230:
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 118, 122, self.fs)
            if self.fs > 330:
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 158, 162, self.fs)
            if self.fs > 430:
                for i in range(9): Data[i] = self.butter_bandstop_filter(Data[i], 218, 222, self.fs)
        # Shift the boundaries of the graph
        timeCount = self.Time[self.l - 1] // self.timeWidth
        # Update plot
        if (self.l > 3):
            # Main signal graphic
            self.pw1.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw2.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw3.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw4.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw5.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw6.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw7.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw8.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))
            self.pw9.setXRange(self.timeWidth * timeCount, self.timeWidth * ( timeCount + 1))   
 
            
            if self.MovingAverage.isChecked() == 1:
                for i in range(9):
                    Data[i] = np.concatenate((self.DataMovingAverage[i][self.l: self.dataWidth], self.DataMovingAverage[i][0: self.l]))
            
            self.p1.setData(y=Data[0][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p2.setData(y=Data[1][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p3.setData(y=Data[2][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p4.setData(y=Data[3][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p5.setData(y=Data[4][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p6.setData(y=Data[5][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p7.setData(y=Data[6][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p8.setData(y=Data[7][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            self.p9.setData(y=Data[8][0: self.dataWidth-2], x = Time[0: self.dataWidth-1])
            
            if (self.l > 100):
                self.pb1.setOpts(height=self.DataMovingAverage[0][self.l-2])
                self.pb2.setOpts(height=self.DataMovingAverage[1][self.l-2])
                self.pb3.setOpts(height=self.DataMovingAverage[2][self.l-2])
                self.pb4.setOpts(height=self.DataMovingAverage[3][self.l-2])
                self.pb5.setOpts(height=self.DataMovingAverage[4][self.l-2])
                self.pb6.setOpts(height=self.DataMovingAverage[5][self.l-2])
                self.pb7.setOpts(height=self.DataMovingAverage[6][self.l-2])
                self.pb8.setOpts(height=self.DataMovingAverage[7][self.l-2])
                self.pb9.setOpts(height=self.DataMovingAverage[8][self.l-2])
            
            
            # FFT graphic
            if self.l > 1000:
                Y = abs(fft(self.Data[self.button_group.checkedId()-1][self.l-1000: self.l-2])) / 998
                X = 1/self.dt*np.linspace(0, 1, 998)
                self.FFT = (1-0.85)*Y + 0.85*self.FFT
                self.p10.setData(y=self.FFT[2: int(len(self.FFT)/2)], x=X[2:int(len(X)/2)]) 
                    
    # Values for butterworth bandpass filter
    def butter_bandpass(self, lowcut, highcut, fs, order = 4):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype = 'bandpass')
        return b, a
    # Butterworth bandpass filter
    def butter_bandpass_filter(self, data, lowcut, highcut, fs, order = 4):
        b, a = self.butter_bandpass(lowcut, highcut, fs, order=order)
        y = lfilter(b, a, data)
        return y
    # Values for butterworth bandstop filter
    def butter_bandstop(self, lowcut, highcut, fs, order = 2):
        nyq = 0.5 * fs
        low = lowcut / nyq
        high = highcut / nyq
        b, a = butter(order, [low, high], btype = 'bandstop')
        return b, a
    # Butterworth bandstop filter
    def butter_bandstop_filter(self, data, lowcut, highcut, fs, order = 4):
        b, a = self.butter_bandstop(lowcut, highcut, fs, order = order)
        y = lfilter(b, a, data)
        return y
    def movingAverage(self, i, data, alpha):
        wa = 2.0*self.fs*np.tan(3.1416*1/self.fs)
        HPF = (2*self.fs*(data - self.X0[i]) - (wa-2*self.fs)*self.Y0[i])/(2*self.fs+wa)
        self.Y0[i] = HPF
        self.X0[i] = data
        data = HPF
        if data < 0:
            data = -data
        self.MA[i][0] = (1 - alpha)*data + alpha*self.MA[i][0];
        self.MA[i][1] = (1 - alpha)*(self.MA[i][0]) + alpha*self.MA[i][1];
        self.MA[i][2] = (1 - alpha)*(self.MA[i][1]) + alpha*self.MA[i][2];
        return self.MA[i][2]*4
    # Change gain
    def _on_radio_button_clicked(self, button):
        if self.monitor.COM != '':
            self.monitor.ser.write(bytearray([button.Value]))
    # Exit event
    def closeEvent(self, event):
        self.f.close()
        self.monitor.ser.close()
        event.accept()

# Serial monitor class
class SerialMonitor(QtCore.QThread):
    bufferUpdated = QtCore.pyqtSignal(bytes)
    # Custom constructor
    def __init__(self, COM, baudRate):
        QtCore.QThread.__init__(self)
        self.running = False
        self.filter = False
        self.COM = COM
        self.baudRate = baudRate
        self.baudRate = baudRate
        self.checkPort = 1

    # Listening port
    def run(self):
        while self.running is True:
            while self.COM == '': 
                ports = serial.tools.list_ports.comports(include_links=False)
                for port in ports :
                    self.COM = port.device
                if self.COM != '':
                    time.sleep(0.5)
                    self.ser = serial.Serial(self.COM, self.baudRate)
                    self.checkPort = 0
            while self.checkPort:
                ports = serial.tools.list_ports.comports(include_links=False)
                for port in ports :
                    if self.COM == port.device:
                        time.sleep(0.5)
                        self.ser = serial.Serial(self.COM, self.baudRate)
                        self.checkPort = 0
                   
            # Waiting for data
            while (self.ser.inWaiting() == 0):
                pass
            # Reading data
            msg = self.ser.read( self.ser.inWaiting() )
            if msg:
                #Parsing data
                self.bufferUpdated.emit(msg)
                time.sleep(0.1)
                
# Starting program       
if __name__ == '__main__':
    app = QtCore.QCoreApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    window = GUI()
    window.show()
    window.start()
    sys.exit(app.exec_())
