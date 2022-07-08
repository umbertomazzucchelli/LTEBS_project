import struct
import sys
import time
import logging
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks
import scipy.signal as signal
from scipy.signal import butter

from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import * 
from PyQt5.QtCore import *

# We import library dedicated to data plot
import pyqtgraph as pg
from pyqtgraph import PlotWidget

import serial 
import serial.tools.list_ports

import pandas as pd

#Global
CONN_STATUS = False
PORT = ""
TRANSMITTING = False
dataSize = 98
baudRate = 115200 #9600 for USB, 115200 for BT
accData = []
axisSize = dataSize//3
xData = np.full(axisSize,0,dtype=np.int16)
yData = np.full(axisSize,0,dtype=np.int16)
zData = np.full(axisSize,0,dtype=np.int16)

xData_g = np.full(axisSize,0,dtype=np.float16)
yData_g = np.full(axisSize,0,dtype=np.float16)
zData_g = np.full(axisSize,0,dtype=np.float16)

zData_lowpass = np.full(axisSize,0,dtype=np.float16)
zData_bandpass = np.full(axisSize,0,dtype=np.float16)
zData_smoothed = np.full(axisSize,0,dtype=np.float16)
zData_windowed = np.full(axisSize,0,dtype=np.float16)
zData_array = []

clock = np.zeros(axisSize)
count_sec = 0
connectionWait = False
resp_rate=0.0
SECOND = 94
SAMPLE_RATE = 50
LOW_CUT = 0.01   
HIGH_CUT = 0.9
order = 8
cutoff = 5

#Logging config
logging.basicConfig(format="%(message)s", level=logging.INFO)

#########################
# SERIAL_WORKER_SIGNALS #
#########################
class SerialWorkerSignals(QObject):
    """!
    @brief Class that defines the signals available to a serialworker.

    Available signals (with respective inputs) are:
        - device_port:
            str --> port name to which a device is connected
        - status:
            str --> port name
            int --> macro representing the state (0 - error during opening, 1 - success)
    """
    device_port = pyqtSignal(str)
    status = pyqtSignal(str, int)


#################
# SERIAL_WORKER #
#################
class SerialWorker(QRunnable):
    """!
    @brief Main class for serial communication: handles connection with device.
    """
    def __init__(self, serial_port_name):
        """!
        @brief Init worker.
        """
        global baudRate
        self.is_killed = False
        super().__init__()
        #init port, params and signals
        self.port = serial.Serial()
        self.port_name = serial_port_name
        self.baudrate = baudRate 
        self.signals = SerialWorkerSignals()

    @pyqtSlot()
    def run(self):
        """!
        @brief Estabilish connection with desired serial port.
        """
        global CONN_STATUS
        global TRANSMITTING
        global PORT, connectionWait

        if not CONN_STATUS:
            try:
                self.port = serial.Serial(port=self.port_name, baudrate=self.baudrate,
                                        write_timeout=0, timeout=2)                
                if self.port.is_open:
                    self.send('t')
                    time.sleep(1)

                    if(self.read()=="HR/RR sensor"):
                        CONN_STATUS = True
                        self.signals.status.emit(self.port_name, 1)
                        PORT = self.port_name
                        time.sleep(1) #just for compatibility reasons 
                    connectionWait = False   
                        
                    
            except serial.SerialException:
                logging.info("Error with port {}.".format(self.port_name))
                self.signals.status.emit(self.port_name, 0)
                time.sleep(0.01)
                connectionWait=False

    @pyqtSlot()
    def send(self, char):
        """!
        @brief Basic function to send a single char on serial port.
        """
        try:
            self.port.write(char.encode('utf-8'))
            logging.info("Written {} on port {}.".format(char, self.port_name))
        except:
            logging.info("Could not write {} on port {}.".format(char, self.port_name))

    @pyqtSlot()
    def read(self):
        """!
        @brief Basic function to read a single char on serial port.
        """
        testString=''
        try:
            while(self.port.in_waiting>0):
                testString+=self.port.read().decode('utf-8', errors='replace')
                #logging.info(self.port.in_waiting)
            logging.info("Received: {}".format(testString))

            return testString
        except:
            logging.info("Could not receive {} on port {}.".format(testString, self.port_name))
   
    @pyqtSlot()
    def readAcc(self):
        """!
        @brief Basic function to read a single char on serial port.
        """
        testString=''
        try:
            while(self.port.in_waiting>0):
                testString+=self.port.read().decode('utf-8', errors='replace')

            return testString
        except:
            logging.info("Could not receive {} on port {}.".format(testString, self.port_name))

    @pyqtSlot()
    def killed(self):
        """!
        @brief Close the serial port before closing the app.
        """
        global CONN_STATUS

        if self.is_killed and CONN_STATUS:
            self.send('s')
            self.port.close()
            time.sleep(0.01)
            CONN_STATUS = False
            self.signals.device_port.emit(self.port_name)
            
        logging.info("Killing the process")

    @pyqtSlot()
    def readData(self):
        """!
        @brief Function used to read the data and the RR algorithms in order to plot the peaks in a defined window.  
        """
        global TRANSMITTING
        global accData, xData, yData, zData, xData_g, yData_g, zData_g, zData_lowpass, zData_bandpass, zData_array
        global zData_smoothed
        global SAMPLE_RATE,LOW_CUT,HIGH_CUT
        global order, cutoff, count_sec
      
        dataArray = self.port.read(194)
        dataArray = struct.unpack('194B',dataArray)
        dataArray = np.asarray(dataArray,dtype=np.uint8)
        lastIndex = len(dataArray)-1

        if(dataArray[0]==10 and dataArray[lastIndex]==11):
            accData = dataArray[1:193]
            for i in range(axisSize):
            
                xData[i] =  (((accData[i*6+1] & 0xFF)<<8) | (accData[i*6] & 0xFF))>>6
                yData[i] =  (((accData[i*6+3] & 0xFF)<<8) | (accData[i*6+2] & 0xFF))>>6
                zData[i] =  (((accData[i*6+5] & 0xFF)<<8) | (accData[i*6+4] & 0xFF))>>6

                if(xData[i]>511 & xData[i]<1024):
                    xData_g[i] = xData[i]*(-0.0039060665362)+3.99990606654
                else:
                    xData_g[i] = xData[i]*(-0.0039137254902) - 0.0000862745098039
                if(yData[i]>511 & yData[i]<1024):
                    yData_g[i] = yData[i]*(-0.0039060665362)+3.99990606654
                else:
                    yData_g[i] = yData[i]*(-0.0039137254902) - 0.0000862745098039
                if(zData[i]>511 & zData[i]<1024):
                    zData_g[i] = zData[i]*(-0.0039060665362)+3.99990606654
                else:
                    zData_g[i] = zData[i]*(-0.0039137254902) - 0.0000862745098039
                                            
            zData_array = np.append(zData_array, zData_g)
            count_sec+=1

            if (count_sec == SECOND): # about 60 s 
                count_sec = 0

                ### LOWPASS SIGNAL ANALYSIS ###
                self.i_peaks_lp = []
                zData_lowpass = self.butter_lowpass_filter(zData_array, cutoff, SAMPLE_RATE, order)                                                     
                self.threshold_lp=self.calibration_threshold(zData_lowpass)
                self.delta_lp=50
                self.n_peaks_lp, self.i_peaks_lp = self.find_peaks(zData_lowpass, self.threshold_lp, self.delta_lp)
                print('Number of peaks in lowpass', self.n_peaks_lp)

                ### BANDPASS SIGNAL ANALYSIS ###
                self.i_peaks_bp = []
                zData_bandpass = self.butter_bandpass_design(zData_lowpass, LOW_CUT, HIGH_CUT,
                                                                  SAMPLE_RATE)
                self.threshold_bp=self.calibration_threshold(zData_bandpass)
                self.delta_bp=50
                self.n_peaks_bp , self.i_peaks_bp= self.find_peaks(zData_bandpass, self.threshold_bp, self.delta_bp)
                print('Number of peaks in bandpass',self.n_peaks_bp)

                ### LOWPASS + SMOOTHED SIGNAL ANALYSIS ###
                self.i_peaks_sm = []
                self.delta_sm=50
                zData_smoothed = signal.savgol_filter(zData_lowpass, window_length=31, polyorder=3)
                self.threshold_sm=self.calibration_threshold(zData_smoothed)
                self.n_peaks_sm, self.i_peaks_sm = self.find_peaks(zData_smoothed, self.threshold_sm, self.delta_sm)
                print('Number of peaks in smoothed',self.n_peaks_sm)

                ### WINDOWS SIGNAL ANALYSIS ###
                self.i_peaks_w = []
                self.delta_w = 50
                self.window_length_MA = 31
                zData_windowed = self.moving_average(self.window_length_MA, zData_lowpass)
                self.threshold_w=self.calibration_threshold(zData_windowed)
                self.n_peaks_w , self.i_peaks_w= self.find_peaks(zData_windowed, self.threshold_w, self.delta_w)
                print('Number of peaks in windowed',self.n_peaks_w)
                
                ### RR CALCULATION ###

                ## Smoothed signal ##
                self.makesum=0.0
                for i in range(len(self.i_peaks_sm)-1):
                    self.difference = (self.i_peaks_sm[i+1]-self.i_peaks_sm[i])*0.02
                    self.makesum += self.difference
                if (self.n_peaks_sm == 0):
                    self.n_peaks_sm = 1
                    self.makesum = 1
                self.average= self.makesum/self.n_peaks_sm
                self.RR_sm = 60/self.average
                print('RR calculated on smoothed signal',self.RR_sm)

                ## Windowed signal ##
                self.makesum=0.0
                for i in range(len(self.i_peaks_w)-1):
                    self.difference = (self.i_peaks_w[i+1]-self.i_peaks_w[i])*0.02
                    self.makesum += self.difference
                if (self.n_peaks_w == 0):
                    self.n_peaks_w = 1
                    self.makesum = 1
                self.average= self.makesum/self.n_peaks_w
                self.RR_w = 60/self.average
                print('RR calculated on windowed signal',self.RR_w)

                ### PLOT ###

                plt.figure(1)
                plt.plot(zData_lowpass, label = 'zData Lowpass')
                plt.plot(self.i_peaks_lp,zData_lowpass[self.i_peaks_lp], "x")
                plt.axhline(y = self.threshold_lp, color = 'r', linestyle = '-')
                plt.title('zData Lowpass')

                plt.figure(2)
                plt.plot(zData_bandpass, label = 'zData Bandpass')
                plt.plot(self.i_peaks_bp,zData_bandpass[self.i_peaks_bp], "x")
                plt.axhline(y = self.threshold_bp, color = 'r', linestyle = '-')
                plt.title('zData Bandpass')

                plt.figure(3)
                plt.plot(zData_smoothed, label = 'zData Smoothed')
                plt.plot(self.i_peaks_sm,zData_smoothed[self.i_peaks_sm], "x")
                plt.axhline(y = self.threshold_sm, color = 'r', linestyle = '-')
                plt.title('zData Smoothed')

                plt.figure(4)
                plt.plot(zData_windowed, label = 'zData Windowed')
                plt.plot(self.i_peaks_w,zData_windowed[self.i_peaks_w], "x")
                plt.axhline(y = self.threshold_w, color = 'r', linestyle = '-')
                plt.title('zData Windowed')

                plt.show()

    def moving_average(self, window_length, data):
        """!
        @brief Moving average function.   
        """
        smoothed = np.convolve(data, np.ones(window_length)) / window_length
    
        return smoothed
    
    def find_peaks(self, data, threshold, delta):
        """!
        @brief Function used to calculate the peaks.   
        """
        i_peaks, _ = find_peaks(data, height = threshold, distance = delta)
        peaks = len(i_peaks)

        return peaks, i_peaks

    def butter_bandpass_design(self, signal_array, low_cut, high_cut, sample_rate, order=4):
        """
        @brief Defines the Butterworth bandpass filter-design
        :param low_cut: Lower cut off frequency in Hz
        :param high_cut: Higher cut off frequency in Hz
        :param sample_rate: Sample rate of the signal in Hz
        :param order: Order of the filter-design
        :return: b, a : ndarray, ndarray - Numerator (b) and denominator (a) polynomials of the IIR filter. Only returned if output='ba'.
        """
        sos = signal.butter(order, [low_cut, high_cut], btype='band', output ='sos',fs=sample_rate)
        y = signal.sosfiltfilt(sos, signal_array)

        return y
    
    def butter_lowpass(self, cutoff, fs, order):

        return butter(order, cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(self, data, cutoff, fs, order):
        """!
        @brief Defines the Butterworth lowpass filter.
        """
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = signal.filtfilt(b, a, data)

        return y

    def butter_highpass(self, cutoff, fs, order):

        return butter(order, cutoff, fs=fs, btype='high', analog=False)

    def butter_highpass_filter(self, data, cutoff, fs, order):
        """!
        @brief Defines the Butterworth highpass filter.
        """
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = signal.filtfilt(b, a, data)

        return y
    
    def calibration_threshold(self, val):
        """!
        -------- CALIBRATION ---------
        The patient has to breath normally for 10 seconds 
        and the mean value calculated during this calibration window is used for the
        definition of a threshold.
        """
        threshold=0.0
        
        threshold= 0.8 * np.mean(val)
        return threshold

###############
# MAIN WINDOW #
###############
class MainWindow(QMainWindow):

    global clock
    def __init__(self):
        """!
        @brief Init MainWindow.
        """
        # Define worker
        self.serial_worker = SerialWorker(None)

        super(MainWindow, self).__init__()

        # Title and geometry
        self.setWindowTitle("iAcc")

        # Create thread handler
        self.threadpool = QThreadPool()
        
        self.connected = CONN_STATUS
  
        self.initUI()
    
    #####################
    # GRAPHIC INTERFACE #
    #####################
    def initUI(self):
        """!
        @brief Set up the graphical interface structure.
        """
        # Create the plot widget
        self.graphWidget = PlotWidget()
        
        # Plot settings
            # Add grid
        self.graphWidget.showGrid(x=True, y=True)
            # Set background color
        self.graphWidget.setBackground('bbccdd')   #color in exa
            # Add title
        self.graphWidget.setTitle("Accelerometer data",color="b", size="12pt",italic=True)
            # Add axis labels
        styles = {'color':'k', 'font-size':'15px'}
        self.graphWidget.setLabel('left', 'Acc data [m/s2]', **styles)
        self.graphWidget.setLabel('bottom', 'Time [ms]', **styles)
            # Add legend
        self.graphWidget.addLegend()
        self.graphWidget.setMouseEnabled(x=False, y=False)

        # Create the Heart rate plot widget
        self.HR_plot = PlotWidget() 
        self.HR_plot.showGrid(x=True, y=True)
        self.HR_plot.setBackground('bbccdd')   #color in exa
        self.HR_plot.setTitle("Heart rate",color="b", size="12pt",italic=True)
        styles = {'color':'k', 'font-size':'15px'}
        self.HR_plot.setLabel('left', 'Acc data [m/s^2]', **styles)
        self.HR_plot.setLabel('bottom', 'Time [ms]', **styles)
        self.HR_plot.addLegend()
        self.HR_plot.setMouseEnabled(x=False, y=False)

        # Create the Respiratory rate plot widget
        self.RR_plot = PlotWidget() 
        self.RR_plot.showGrid(x=True, y=True)
        self.RR_plot.setBackground('bbccdd')   #color in exa
        self.RR_plot.setTitle("Respiratory rate",color="b", size="12pt",italic=True)
        styles = {'color':'k', 'font-size':'15px'}
        self.RR_plot.setLabel('left', 'Acc data [m/s^2]', **styles)
        self.RR_plot.setLabel('bottom', 'Time [ms]', **styles)
        self.RR_plot.addLegend()
        self.RR_plot.setMouseEnabled(x=False, y=False)
    
        # Display 100 time points
        self.horAxis = list(range(320))  
        self.xGraph = [0]*320
        self.yGraph = [0]*320
        self.zGraph = [0]*320
    
        self.count = 0

        self.draw()
    
        # Plot data
        self.drawGeneralGraph()
        
        # Toolbar
        toolbar = QToolBar("My main toolbar")  
        toolbar.setIconSize(QSize(16, 16))
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        # Exporting files as .csv
        save_action = QAction(QIcon("disk_return_black.png"), "Export to .csv", self)
        save_action.setStatusTip("Export to .csv")
        save_action.triggered.connect(self.save_data)
        toolbar.addAction(save_action)
        self.setStatusBar(QStatusBar(self))
        toolbar.addSeparator()

        menu = self.menuBar()
        file_menu = menu.addMenu("&File")
        file_menu.addAction(save_action)

        self.HR_label = QLabel()
        self.HR_label.setText('')
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        self.HR_label.setFont(font)
        self.HR_label.setStyleSheet("border: 1px solid black")
        self.HR_label.setAlignment(QtCore.Qt.AlignCenter)

        self.RR_label = QLabel()
        self.RR_label.setText('')
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        self.RR_label.setFont(font)
        self.RR_label.setStyleSheet("border: 1px solid black")
        self.RR_label.setAlignment(QtCore.Qt.AlignCenter)

        verticalLayout = QVBoxLayout()
        heart_resp = QHBoxLayout()
        heart_plot = QVBoxLayout()
        resp_plot = QVBoxLayout()
        device_search = QHBoxLayout()
    
        self.conn_btn = QPushButton(
            text=("Device search"), 
            checkable=True,
            toggled=self.on_toggle)

        self.conn_label = QLabel(
            text = ("No device connected")
        )
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        self.conn_label.setFont(font)
        self.conn_label.setStyleSheet("background-color: rgb(255, 0, 0);\n" "border: 1px solid black")
        self.conn_label.setAlignment(QtCore.Qt.AlignCenter)

        device_search.addWidget(self.conn_btn)
        device_search.addWidget(self.conn_label)

        verticalLayout.addLayout(device_search)

        modeSelection = QHBoxLayout()

        self.modeSelect = QComboBox()
        self.modeSelect.addItem("HR & RR")
        self.modeSelect.addItem("HR Only")
        self.modeSelect.addItem("RR Only")
        self.modeSelect.currentIndexChanged.connect(self.selectionchange)

        self.calibrate = QPushButton(
            text = ("Calibration")
)
        self.calibrate.clicked.connect(self.startCalibration)

        self.updateBtn = QPushButton(
            text = "Start", 
            checkable= True, 
            toggled = self.dataUpdate
        )

        self.updateBtn.setIcon(QtGui.QIcon('application-monitor.png'))
        self.updateBtn.setIconSize(QtCore.QSize(25,25))

        modeSelection.addWidget(self.modeSelect)
        modeSelection.addWidget(self.calibrate)
        modeSelection.addWidget(self.updateBtn)
        verticalLayout.addLayout(modeSelection)

        heart_plot.addWidget(self.HR_label)
        heart_plot.addWidget(self.HR_plot)
        resp_plot.addWidget(self.RR_label)
        resp_plot.addWidget(self.RR_plot)

        heart_resp.addLayout(heart_plot)
        heart_resp.addLayout(resp_plot)

        verticalLayout.addWidget(self.graphWidget)
        verticalLayout.addLayout(heart_resp)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(500)

        self.graphTimer= QtCore.QTimer()
        self.graphTimer.setInterval(1000)

        widget = QWidget()
        widget.setLayout(verticalLayout)
        self.setCentralWidget(widget)

        modeSelection.setContentsMargins(5,5,5,5)
        modeSelection.setSpacing(5)

    def save_data(self):
        """!
        @brief Function used to save and export data in a .csv file. 
        """
        print("exporting to csv...")

        global zData_g, xData_g, yData_g
        df = pd.DataFrame({
            'x axis': xData_g,
            'y axis': yData_g,
            'z axis': zData_g,
        })
        df.to_csv('HR_Rate.csv', float_format = '%.2f', index = False)

    def selectionchange(self,i):
        global flag_graph
        flag_graph = i
        a = ['HR & RR','HR only', 'RR only'] 
        # 0 --> HR
        # 1 --> RR
        # 2 --> both
        
        if (flag_graph == 0): #only HR
            self.RR_plot.setBackground('bbccdd')
            self.HR_plot.setBackground('bbccdd')

        elif (flag_graph == 1): #only RR
            self.RR_plot.setBackground('b')
            self.HR_plot.setBackground('bbccdd')

        elif (flag_graph == 2):
            self.HR_plot.setBackground('b')
            self.RR_plot.setBackground('bbccdd')
        
    def drawGeneralGraph(self):
        """!
        @brief Draw the plots.
        """
        global zData_g

        for i in range(len(xData)):

            # Remove the first y element.
            if(self.count<321):
                self.count += 1
            else:
                self.horAxis = self.horAxis[1:]
                self.horAxis.append(self.horAxis[-1] + 1)  # Add a new value 1 higher than the last.
    
            # Z-axis
            self.zGraph = self.zGraph[1:]  # Remove the first 
            self.zGraph.append(zData_g[i])
            self.dataLinez.setData(self.horAxis, self.zGraph)  # Update the data.   

    def draw(self):
        """!
             @brief Draw the plots.
        """
        global zData_g

        self.dataLinez = self.plot(self.graphWidget,clock,zData_g,'z-axis','b')
    
    def plot(self, graph, x, y, curve_name, color):
        """!
        @brief Draw graph.
        """
        pen = pg.mkPen(color=color,width=2)
        line = graph.plot(x, y, name=curve_name, pen=pen)
        return line

    ##################
    # SERIAL SIGNALS #
    ##################

    @pyqtSlot(bool)
    def on_toggle(self, checked):
        global CONN_STATUS,connectionWait
        """!
        @brief Allow connection and disconnection from selected serial port.
        """
        if checked:
            self.conn_btn.setText("Searching device...") 
            self.conn_btn.repaint()
            time.sleep(0.1)
            #acquire list of serial ports
            serial_ports = [
                p.name
                for p in serial.tools.list_ports.comports()
            ]

            for i in range(len(serial_ports)):
                self.port_text=serial_ports[i]

                # Setup reading worker
                self.serial_worker = SerialWorker(self.port_text) 
                connectionWait = True
                print("Active port ", self.port_text)
                # Connect worker signals to functions
                
                self.serial_worker.signals.device_port.connect(self.connected_device)
                # Execute the worker
                self.threadpool.start(self.serial_worker)
    
                while(connectionWait==True):
                    time.sleep(0.5)
                if(CONN_STATUS==True):
                    self.conn_btn.setText(
                    "Disconnect from port {}".format(self.port_text))
                    self.conn_label.setText("Device connected")
                    self.conn_label.setStyleSheet("background-color: rgb(0, 255, 0);\n" "border: 1px solid black")
                    break

            if(CONN_STATUS==False):
                self.conn_btn.setText("Device not found")
                self.conn_btn.repaint()
                time.sleep(0.5)
                self.conn_btn.setChecked(False)
            self.updateBtn.setDisabled(False)
            
        else:
           
            self.updateBtn.setChecked(False)
            # Kill thread
            self.serial_worker.is_killed = True
            self.serial_worker.killed()
            self.conn_label.setText("No device connected")
            self.conn_label.setStyleSheet("background-color: rgb(255, 0, 0);\n" "border: 1px solid black")
            self.conn_btn.setText("Device search")
            self.updateBtn.setDisabled(True)
            
    def connected_device(self, port_name):
        """!
        @brief Checks on the termination of the serial worker.
        """
        logging.info("Port {} closed.".format(port_name))

    def ExitHandler(self):
        """!
        @brief Kill every possible running thread upon exiting application.
        """
        self.serial_worker.is_killed = True
        self.serial_worker.killed()


    def dataUpdate(self,checked):
        global PORT
        global TRANSMITTING

        if checked:
            self.serial_worker.send('a')
            self.updateBtn.setText("Stop")
            TRANSMITTING = True
            self.timer.timeout.connect(lambda: self.serial_worker.readData())
            self.timer.start()
            self.graphTimer.timeout.connect(lambda: self.drawGeneralGraph())
            self.graphTimer.start() 
            self.calibrate.setDisabled(False)
            self.modeSelect.setDisabled(False)

        else:
            self.serial_worker.send('s')
            self.updateBtn.setText("Start")
            TRANSMITTING = False
            self.timer.stop()
            self.graphTimer.stop()
            self.modeSelect.setDisabled(False)
            self.calibrate.setDisabled(True)

    def startCalibration(self):
        global calibration

        calibration = True

    def selectionchange(self,i):
        global flag_graph
        flag_graph = i
        a = ['both','HR only', 'RR only'] 
        # 0 --> HR
        # 1 --> RR
        # 2 --> both
    
        if (flag_graph == 0): #only HR
            self.RR_plot.setBackground('bbccdd')
            self.HR_plot.setBackground('bbccdd')
            
        elif (flag_graph == 1): #only RR
            self.RR_plot.setBackground('r')
            self.HR_plot.setBackground('bbccdd')
        
        elif (flag_graph == 2):
            self.HR_plot.setBackground('y')
            self.RR_plot.setBackground('bbccdd')
    
#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())