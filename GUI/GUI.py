from setuptools import find_packages
import variables as var
from re import L
import struct
import sys
from telnetlib import STATUS
import time
import logging
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks, argrelmin
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

# Global variables
CONN_STATUS = False
STATUS=True
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
sum_data = np.full(axisSize,0,dtype=np.float16)

clock = np.zeros(axisSize)
xData_save = []
yData_save = []
zData_save = []

zData_bandpass_HR = np.full(axisSize,0,dtype=np.float16)
zData_windowed_HR = np.full(axisSize,0,dtype=np.float16)
zData_lowpass_RR = np.full(axisSize,0,dtype=np.float16)
zData_smoothed_RR = np.full(axisSize,0,dtype=np.float16)
zData_lowpass = np.full(axisSize,0,dtype=np.float16)

zData_array_HR = []
zData_array_RR = []

flag_graph = 0
count_sec_HR = 0
count_sec_RR = 0
connectionWait = False
calibration_flag=False
RR_value=0.0
HR_value=0.0
i_peaks_HR = 0
i_peaks_RR = 0
flag_RR = False
flag_HR = False

SAMPLE_RATE = 50
# the respiration range is comprised between 0.2 Hz and 0.33 Hz, during physical activity instead it can be maximum 0.75 Hz
LOW_CUT_RR = 0.01  
HIGH_CUT_RR = 0.9
# the heart rate instead varies between 1 Hz and 1.66 Hz, during physical activity it can reach up to 3 Hz
LOW_CUT_HR = 1    
HIGH_CUT_HR = 5
THRESHOLD = 0.0001
order = 5
cutoff_RR = 3
cutoff_hp = 1 

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
        self.baudrate = baudRate #hard coded but can be a global variable, or an input param
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
                #logging.info(self.port.in_waiting)
            #logging.info("Received: {}".format(testString))
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
        @brief Function used to read the data and to implement the HR and RR algorithms. 
               Moreover we implemented a warning signal for the user in case of losing the connection. 
        """
        global TRANSMITTING, STATUS
        global accData, xData, yData, zData, xData_g, yData_g, zData_g, sum_data,  zData_lowpass
        global zData_array_HR, zData_array_RR, RR_value, xData_save, yData_save, zData_save
        global SAMPLE_RATE, LOW_CUT_RR, HIGH_CUT_RR, LOW_CUT_HR, HIGH_CUT_HR, CONN_STATUS, calibration_flag
        global order, cutoff_RR, HR_value
        global flag_RR, flag_HR
      
        try:
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

                    xData_save = np.append(xData_save, xData_g)
                    yData_save = np.append(yData_save, yData_g)
                    zData_save = np.append(zData_save, zData_g)

                    # Sum_data is used to calculate the HR rate in order to be more robust against movements
                    sum_data[i]=zData_g[i]+yData_g[i]      
            
            # In order to plot a real time graph which is filtered
            zData_lowpass = self.butter_lowpass_filter(zData_g, cutoff_RR, SAMPLE_RATE, order)   

            # Algorithms for HR and RR computation
            HR_value = self.HR_computation(sum_data, calibration_flag)
            RR_value = self.RR_computation(zData_g, calibration_flag)

        except struct.error:
            
            # Warning message in case of connection problems
            self.dlg3 = QMessageBox()
            self.dlg3.setWindowTitle("WARNING")
            self.dlg3.setText("Connection lost, click OK and restart the application")
            self.dlg3.setStandardButtons(QMessageBox.Ok)
            self.dlg3.setIcon(QMessageBox.Critical)
            button=self.dlg3.exec_()
            if(button==QMessageBox.Ok):
                MainWindow.save_data(self)  # save data even if there are connection problems
                sys.exit(app.exec_())
            MainWindow.on_toggle(False)

    def HR_computation(self, data, calibration_flag):
        """!
        @brief Algorithm for Heart Rate computation after the calibration. 

        The pre-processing steps applied to the data are the following: 
            1) Data are Band-Pass filtered in a range between 1 Hz to 5 Hz;
            2) Then we apply a Savitzky-Golay filter;
            3) Finally a Moving Average.

        Then we proceed calculating the Heart Rate based on a threshold and on a time window in order to be
        more robust against noise and more precise in the computation. 
        """
        global zData_array_HR, count_sec_HR, SAMPLE_RATE, zData_bandpass_HR
        global zData_windowed_HR, cutoff_hp, i_peaks_HR
        global character, HR_value, flag_HR
        HR_value = 0.0

        # We proceed with the computation of the Heart Rate only after the calibration is finished
        if (calibration_flag):
            count_sec_HR+=1   
            zData_array_HR = np.append(zData_array_HR, data)
           
            if (count_sec_HR==16):     # The HR is updated after about 10 s
            
                zData_bandpass_HR = self.butter_bandpass_design(np.abs(zData_array_HR), LOW_CUT_HR, HIGH_CUT_HR,
                                                                  SAMPLE_RATE)
                zData_array_HR=[]    
                zData_windowed_HR = signal.savgol_filter(np.abs(zData_bandpass_HR), window_length=15, polyorder=3)
                self.window_length_MA = 15
                zData_windowed_HR = self.moving_average(self.window_length_MA, np.abs(zData_windowed_HR))  
                
                # Threshold computation
                self.threshold_wi=self.calibration_threshold(np.abs(zData_windowed_HR))
                self.delta_HR = 25

                # Number of peaks detected in the defined time window
                self.n_peaks_HR, i_peaks_HR  = self.find_peaks(np.abs(zData_windowed_HR), self.threshold_wi, 
                                                                                            self.delta_HR)
                self.makesum=0.0

                # HR calculation
                for i in range(len(i_peaks_HR)-1):
                    self.difference= (i_peaks_HR[i+1]-i_peaks_HR[i])*0.02
                    self.makesum += self.difference
                #self.average= self.makesum/self.n_peaks_HR
                #HR_value = 60/self.average
                HR_value= (self.n_peaks_HR * 60)/ (count_sec_HR*32*0.02)
                count_sec_HR = 0
                # In case there is a too high HR, we set an alarm window warning about possible fibrillation
                if (HR_value > 180):
                    self.fibrillazione()

                '''
                # Plot showing the data after the filtering steps
                plt.figure(1)
                plt.plot(np.abs(zData_windowed_HR), label = 'zData_windowed')
                plt.plot(i_peaks_HR,np.abs(zData_windowed_HR[i_peaks_HR]), "x")
                plt.axhline(y = self.threshold_wi, color = 'r', linestyle = '-')
                plt.title('zData_windowed')

                plt.show()
                '''

                flag_HR = True

        return HR_value

    def RR_computation(self, data, calibration_flag):
        """!
        @brief Algorithm for Respiratory Rate computation after the calibration. 

        The pre-processing steps applied to the data are the following: 
            1) Firstly, data are Low-Pass filtered with a cut-off frequency of 3 Hz;
            2) Then we apply a Savitzky-Golay filter;

        Then we proceed calculating the Respiratory Rate based on a threshold and on a time window in order to be
        more robust against noise and more precise in the computation. 
        """

        global count_sec_RR, zData_smoothed_RR, zData_lowpass_RR, i_peaks_RR, zData_array_RR
        global RR_value, cutoff_RR, flag_RR, THRESHOLD

        # We proceed with the computation of the Heart Rate only after the calibration is finished
        if (calibration_flag):
            count_sec_RR+=1    
            zData_array_RR = np.append(zData_array_RR, data)
            
            if (count_sec_RR==16):     # The HR is updated after about 10 s
                
                zData_lowpass_RR = self.butter_lowpass_filter(zData_array_RR, cutoff_RR, SAMPLE_RATE, order)
                zData_array_RR=[]    
                zData_smoothed_RR = signal.savgol_filter(zData_lowpass_RR, window_length=31, polyorder=3)   
                
                # Threshold computation
                self.threshold_RR=self.calibration_threshold(zData_smoothed_RR)
                self.delta_RR = 50

                # Number of peaks detected in the defined time window
                self.n_peaks_bp_RR, i_peaks_RR = self.find_peaks(zData_smoothed_RR, self.threshold_RR, self.delta_RR)
                self.i_min_RR= signal.argrelmin(zData_smoothed_RR)

                min = zData_smoothed_RR[self.i_min_RR]
                max = zData_smoothed_RR[i_peaks_RR]
                sum_min = sum(min)
                sum_max = sum(max)
                avg_min = sum_min/len(min)
                avg_max = sum_max/len(max)
                THRESHOLD_OLD=THRESHOLD
                THRESHOLD = avg_max - avg_min
                
                if (THRESHOLD < THRESHOLD_OLD /2):
                    RR_value = 0.0
                else: 
                
                    self.makesum=0.0

                    # RR calculation
                    for i in range(len(i_peaks_RR)-1):
                        self.difference= (i_peaks_RR[i+1]-i_peaks_RR[i])*0.02
                        self.makesum += self.difference
                    #self.average= self.makesum/self.n_peaks_bp_RR
                    #RR_value = 60/self.average
                    RR_value= (self.n_peaks_bp_RR* 60)/ (count_sec_RR*32*0.02)
                    count_sec_RR = 0

                    # In case there is a too low RR, we set an alarm window warning about possible apnea
                    if (RR_value < 5.0):
                        self.apnea()


                    '''
                    # Plot showing the data after the filtering steps
                    plt.figure(2)
                    plt.plot(zData_smoothed_RR, label = 'zData smoothed RR')
                    plt.plot(i_peaks_RR,zData_smoothed_RR[i_peaks_RR], "x")
                    plt.axhline(y = self.threshold_RR, color = 'r', linestyle = '-')
                    plt.title('zData_smoothed RR')

                    plt.show()
                    '''

                flag_RR = True

                return RR_value

    def apnea (self):
        """!
        @brief Warning message for the user in case of apnea.  
        """
        self.dlg3 = QMessageBox()
        self.dlg3.setWindowTitle("WARNING")
        self.dlg3.setText("Apnea alert!\nRespiration rate below 5 resp/min")
        self.dlg3.setStandardButtons(QMessageBox.Ok)
        self.dlg3.setIcon(QMessageBox.Critical)
        button=self.dlg3.exec_()
        if(button==QMessageBox.Ok):
            self.dlg3.accept()
        MainWindow.on_toggle(False)
    
    def fibrillazione (self):
        """!
        @brief Warning message for the user in case of fibrillation.  
        """
        self.dlg3 = QMessageBox()
        self.dlg3.setWindowTitle("WARNING")
        self.dlg3.setText("Fibrillation alert!\nHeart rate above 180 beats/min")
        self.dlg3.setStandardButtons(QMessageBox.Ok)
        self.dlg3.setIcon(QMessageBox.Critical)
        button=self.dlg3.exec_()
        if(button==QMessageBox.Ok):
            self.dlg3.accept()
        MainWindow.on_toggle(False)

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

        return peaks , i_peaks

    def butter_bandpass_design(self, signal_array, low_cut, high_cut, sample_rate, order=4):
        """
        @brief Defines the Butterworth bandpass filter-design.
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
        global RR_value, HR_value

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
        self.horAxis = list(range(320))  #100 time points
        self.xGraph = [0]*320
        self.yGraph = [0]*320
        self.zGraph = [0]*320
        self.zGraph_windowed_HR = [0]*320
        self.zGraph_lowpass = [0]*320
        self.zGraph_smoothed_RR = [0]*320

        self.count = 0

        self.draw()
    
        # Plot data
        self.drawGeneralGraph()

        # Update the value of HR and RR in the GUI
        self.updateValue()
        
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

        # Label used to display HR values
        self.HR_label = QLabel()
        text = 'Instant heart rate value: {} bpm'
        a = text.format(HR_value)
        self.HR_label.setText(a)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        font = QFont("Lucida", 12, QFont.Bold)
        self.HR_label.setFont(font)
        self.HR_label.setStyleSheet("border: 1px solid black")
        self.HR_label.setAlignment(QtCore.Qt.AlignCenter)

        # Label used to display RR values
        self.RR_label = QLabel()
        text = 'Instant respiratory rate value: {} bpm'
        a = text.format(RR_value)
        self.RR_label.setText(a)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        font = QFont("Lucida", 12, QFont.Bold)
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

        # ComboBox for the HR and RR selection
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

        self.statusTimer = QtCore.QTimer()
        self.statusTimer.setInterval(2000)

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

        global zData_save, xData_save, yData_save
        df = pd.DataFrame({
            'x axis': xData_save,
            'y axis': yData_save,
            'z axis': zData_save, 
        })
        df.to_csv('HR_Rate.csv', float_format = '%.2f', index = False)

        
    def selectionchange(self,i):
        """!
        @brief Allows the selection of different graphs in the bottom panels of the GUI's window.
        # 0 --> HR
        # 1 --> RR
        # 2 --> both
        """
        global flag_graph
        flag_graph = i
        a = ['both','HR only', 'RR only'] 
        if (flag_graph == 0): # Both RR and HR
            self.RR_plot.setBackground('bbccdd')
            self.HR_plot.setBackground('bbccdd')
        
        elif (flag_graph == 1): #only HR
            self.RR_plot.setBackground('b')
            self.HR_plot.setBackground('bbccdd')
    
        elif (flag_graph == 2): #only RR
            self.HR_plot.setBackground('b')
            self.RR_plot.setBackground('bbccdd')
        
    def drawGeneralGraph(self):
        """!
        @brief Draw the plots.
        """
        global xData_g, yData_g, zData_g, flag_graph, zData_lowpass

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

            # Z-axis low pass FILTERED
            self.zGraph_lowpass = self.zGraph_lowpass[1:]  # Remove the first 
            self.zGraph_lowpass.append(zData_lowpass[i])
            self.dataLinez_lowpass.setData(self.horAxis, self.zGraph_lowpass)  # Update the data.

            if (flag_graph ==0 or flag_graph == 1):
                # Heart Rate
                self.zGraph_windowed_HR = self.zGraph_windowed_HR[1:]  # Remove the first 
                self.zGraph_windowed_HR.append(zData_windowed_HR[i])
                self.dataLinez_windowed_HR.setData(self.horAxis, np.abs(self.zGraph_windowed_HR))  # Update the data.

            if (flag_graph ==0 or flag_graph == 2):
                # Respiratory Rate
                self.zGraph_smoothed_RR = self.zGraph_smoothed_RR[1:]  # Remove the first 
                self.zGraph_smoothed_RR.append(zData_smoothed_RR[i])
                self.dataLinez_smoothed_RR.setData(self.horAxis, np.abs(self.zGraph_smoothed_RR))  # Update the data.
            
    def updateValue(self):
        """!
        @brief Update the values of HR and RR on the GUI.
        """
        global HR_value, RR_value, flag_RR, flag_HR

        text = 'Instant {} value: {:0.2f}'

        if (flag_RR == True):

            RR_text = text.format('RR', RR_value)
            print(RR_text + 'bpm')
            self.RR_label.setText(RR_text+' bpm')

            flag_RR = False

        if (flag_HR == True):
                
            HR_text = text.format('HR', HR_value)
            print(HR_text + 'bpm')
            self.HR_label.setText(HR_text+' bpm')

            flag_HR = False
            
    def draw(self):
        """!
             @brief Draw the plots.
        """
        global xData_g, yData_g, zData_g, zData_smoothed_RR, zData_lowpass_RR, zData_windowed_HR

        #self.dataLinex = self.plot(self.graphWidget,clock,xData_g,'x-axis','r')
        #self.dataLiney = self.plot(self.graphWidget,clock,yData_g,'y-axis','g')
        self.dataLinez = self.plot(self.graphWidget,clock,zData_g,'Z-axis','b')
        self.dataLinez_lowpass = self.plot(self.graphWidget,clock,zData_lowpass,'Z-axis Low-Pass Filtered','r')
        self.dataLinez_smoothed_RR = self.plot(self.RR_plot,clock,np.abs(zData_smoothed_RR),'Respiratory wave','b')
        self.dataLinez_windowed_HR = self.plot(self.HR_plot,clock,np.abs(zData_windowed_HR),'Heart beat','b')
    
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
            # Acquire list of serial ports
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
                    self.statusTimer.timeout.connect(self.checkStatus)
                    self.statusTimer.start()
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
            #self.com_list_widget.setDisabled(False) # enable the possibility to change port
            self.conn_btn.setText("Device search")
            self.updateBtn.setDisabled(True)
            self.statusTimer.stop()
            
    def connected_device(self, port_name):
        """!
        @brief Checks on the termination of the serial worker.
        """
        logging.info("Port {} closed.".format(port_name))


    def checkStatus(self):
        """!
        @brief Handle the status of the serial port connection.
        Available status:
            - 0  --> Error during opening of serial port
            - 1  --> Serial port opened correctly
        """
        if self.serial_worker.port.is_open == False:
            self.conn_btn.setChecked(False)
            self.dlg3 = QMessageBox(self)
            self.dlg3.setWindowTitle("WARNING")
            self.dlg3.setText("Connection lost, reconnect the device before proceeding")
            self.dlg3.setStandardButtons(QMessageBox.Ok)
            self.dlg3.setIcon(QMessageBox.Critical)
            button=self.dlg3.exec_()
            if(button==QMessageBox.Ok):
                self.dlg3.accept()
        
    def ExitHandler(self):
        """!
        @brief Kill every possible running thread upon exiting application.
        """
        self.serial_worker.is_killed = True
        self.serial_worker.killed()

    def dataUpdate(self,checked):
        """!
        @brief Update the data.
        """
        global PORT, TRANSMITTING

        if checked:
            self.serial_worker.send('a')
            self.updateBtn.setText("Stop")
            TRANSMITTING = True
            self.timer.timeout.connect(lambda: self.serial_worker.readData())
            self.timer.timeout.connect(lambda: self.updateValue())
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
        """!
        @brief After the button "Calibration" is pressed. 
        """
        global calibration_flag

        calibration_flag = True
        self.calibrate.setDisabled(True) # Deactivate the button after first calibration (the next ones will be authomatic)

#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())