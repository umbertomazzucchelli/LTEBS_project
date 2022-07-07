from setuptools import find_packages
import variables as var
from audioop import findmax
from multiprocessing import connection
from pydoc import source_synopsis
from re import L
import struct
import sys
from telnetlib import STATUS
import time
import logging
from typing_extensions import Self
from matplotlib.pyplot import connect, fill_between
import matplotlib.pyplot as plt
#import matplotlib #.axis import XAxis
import numpy as np
#import matplotlib 
#import peakutils
from scipy.signal import find_peaks
from scipy import interpolate
from scipy.fft import fftfreq
import scipy.signal as signal
from scipy.fftpack import fft
from scipy.signal import butter, lfilter, freqz, blackman


from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import * 
from PyQt5.QtCore import *
from numpy.core.numeric import indices 

# We import library dedicated to data plot
import pyqtgraph as pg
from pyqtgraph import PlotWidget

import serial 
import serial.tools.list_ports

import pandas as pd

#Global
CONN_STATUS = False
STATUS=True
PORT = ""
TRANSMITTING = False
STARTED = False
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

zData_highpass_HR = np.full(axisSize,0,dtype=np.float16)
zData_windowed_HR = np.full(axisSize,0,dtype=np.float16)
zData_lowpass_RR = np.full(axisSize,0,dtype=np.float16)
zData_bandpass_RR = np.full(axisSize,0,dtype=np.float16)

zData_array = []
sum_data = np.full(axisSize,0,dtype=np.float16)
zData_array_LP = []
index_increment = 0
newZero = np.zeros(3)

clock = np.zeros(axisSize)
count_sec_HR = 0
count_sec_RR = 0
k = 0
p = 0
xavg = 0
yavg = 0
zavg = 0
connectionWait = False
calibration = False
calibration_flag=False
start_threshold=0
RR_value=0.0
HR_value=0.0
flag_time_RR = False
flag_time_HR = False
time_difference_HR=0
time_difference_RR=0
#character = ''
start_HR = 0
start_RR = 0
stop_HR = 0
stop_RR = 0
i_peaks_HR = 0
i_peaks_RR = 0
xData_save = []
yData_save = []
zData_save = []
#start_time=-1
#delta_time=0.5   #deve essere circa 2 secondi

SAMPLE_RATE = 50
LOW_CUT_RR = 0.01  #il range del respiro normale è tra 0.2 Hz e 0.33 Hz, sottosforzo invece è max 0.75 Hz
HIGH_CUT_RR = 0.9
LOW_CUT_HR = 1    #da paper
HIGH_CUT_HR = 5
#per la HR il range è tra 1 Hz e 1.66 Hz, sottosforzo invece è max 3 Hz
order = 5
cutoff = 3
cutoff_hp = 1 
f_bw=0.25 #Hz for normal activities, put 0.50 Hz for sport activities

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
        global TRANSMITTING
        global STATUS
        global accData, xData, yData, zData, xData_g, yData_g, zData_g, zData_array
        global sum_data, zData_array_LP,k, start_threshold, RR_value, xData_save, yData_save, zData_save
        global SAMPLE_RATE,LOW_CUT_RR,HIGH_CUT_RR,LOW_CUT_HR,HIGH_CUT_HR, CONN_STATUS, calibration_flag
        global order, cutoff, p, HR_value
      
        #self.serial_worker = SerialWorker(PORT)
        try:
            dataArray = self.port.read(194)
            dataArray = struct.unpack('194B',dataArray)
            dataArray = np.asarray(dataArray,dtype=np.uint8)
            lastIndex = len(dataArray)-1
            if(dataArray[0]==10 and dataArray[lastIndex]==11):
                accData = dataArray[1:193]
                for i in range(axisSize):
                    '''
                    xData[i] =  (accData[i*6] | (accData[i*6+1]<<8))>>6
                    yData[i] =  (accData[i*6+2] | (accData[i*6+3]<<8))>>6
                    zData[i] =  (accData[i*6+4] | (accData[i*6+5]<<8))>>6
                    '''
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

                    sum_data[i]=zData_g[i]+yData_g[i]      #vediamo se usare solo z o la somma dei due

            #self.new_zero=self.calibration(sum_data) 
            #sum_data=sum_data-self.new_zero

            HR_value = self.HR_computation(sum_data, calibration_flag)

            RR_value = self.RR_computation(sum_data, calibration_flag)

        except struct.error:
            #MainWindow.conn_btn.setChecked(False)
            self.dlg3 = QMessageBox()
            self.dlg3.setWindowTitle("WARNING")
            self.dlg3.setText("Connection lost, click OK and restart the application")
            self.dlg3.setStandardButtons(QMessageBox.Ok)
            self.dlg3.setIcon(QMessageBox.Critical)
            button=self.dlg3.exec_()
            if(button==QMessageBox.Ok):
                #self.dlg3.accept()
                MainWindow.save_data(self)  # save even if there are connection problems
                sys.exit(app.exec_())
            MainWindow.on_toggle(False)

    def HR_computation(self, data, calibration_flag):
        global zData_array, time_difference_HR, count_sec_HR, flag_time_HR, start_HR, stop_HR, SAMPLE_RATE, zData_highpass_HR
        global zData_windowed_HR, cutoff_hp, i_peaks_HR
        #global character, HR value
        HR_value = 0.0
        if (calibration_flag):
            count_sec_HR+=1    #chiamarlo count_sec
            zData_array = np.append(zData_array, data)
            '''
            if (flag_time_HR):
                #start_HR = time.time()
                flag_time_HR = False
            '''
            if (count_sec_HR==40):     #vogliamo 10 secondi
                #self.zData_array_HR = zData_array
                count_sec_HR = 0
                #self.new_zero=self.calibration(self.zData_array_HR)  
                #self.zData_array_HR=self.zData_array_HR-self.new_zero
                # Calcoliamo i dati low pass dopo aver creato un array di tot secondi e dopo aver calibrato a zero
                zData_highpass_HR = self.butter_bandpass_design(np.abs(zData_array), LOW_CUT_HR, HIGH_CUT_HR,
                                                                  SAMPLE_RATE)
                #self.zData_array_HR=[]    
                zData_windowed_HR = signal.savgol_filter(np.abs(zData_highpass_HR), window_length=15, polyorder=3)
                self.window_length_MA = 15
                zData_windowed_HR = self.moving_average(self.window_length_MA, np.abs(zData_windowed_HR))  
                
                #Calcolo la threshold ogni tot secondi
                self.threshold_wi=self.calibration_threshold(np.abs(zData_windowed_HR))
                self.delta_HR = 25
                self.n_peaks_HR, i_peaks_HR  = self.find_peaks(np.abs(zData_windowed_HR), self.threshold_wi, self.delta_HR)
                self.makesum=0.0
                for i in range(len(i_peaks_HR)-1):
                    self.difference= (i_peaks_HR[i+1]-i_peaks_HR[i])*0.02
                    self.makesum += self.difference
                self.average= self.makesum/len(i_peaks_HR)
                HR_value = 60/self.average

                print('numero picchi window',self.n_peaks_HR)

                #stop_HR = time.time()
                #time_difference_HR = stop_HR - start_HR
                #HR_value = (self.n_peaks_HR * 60) / time_difference_HR

                print('heart rate: ', HR_value)
                plt.figure(1)
                plt.plot(np.abs(zData_windowed_HR), label = 'zData_windowed')
                plt.plot(i_peaks_HR,np.abs(zData_windowed_HR[i_peaks_HR]), "x")
                plt.axhline(y = self.threshold_wi, color = 'r', linestyle = '-')
                plt.title('zData_windowed')

                plt.show()
                #print('time delta HR: ', time_difference_HR)
                #stop_HR = 0
                #start_HR = 0
                #flag_time_HR = True
                #character = 'HR'
                MainWindow.updateValue(self, HR_value, character = 'HR')

        return HR_value



    def RR_computation(self, data, calibration_flag):

        global zData_array, time_difference_RR, count_sec_RR, flag_time_RR, start_RR, stop_RR
        global zData_bandpass_RR, zData_lowpass_RR, i_peaks_RR
        #global character, RR value
        if (calibration_flag):
            count_sec_RR+=1    #chiamarlo count_sec
            zData_array = np.append(zData_array, data)
            '''
            if (flag_time_RR):
                #start_RR = time.time()
                flag_time_RR = False
            '''
            if (count_sec_RR==40):     #vogliamo 10 secondi
                #self.zData_array_RR = zData_array
                count_sec_RR = 0
                #self.new_zero=self.calibration(self.zData_array_RR)  
                #self.zData_array_RR=self.zData_array_RR-self.new_zero
                # Calcoliamo i dati low pass dopo aver creato un array di tot secondi e dopo aver calibrato a zero
                zData_lowpass_RR = self.butter_lowpass_filter(zData_array, cutoff, SAMPLE_RATE, order)
                #self.zData_array_RR=[]    
                zData_bandpass_RR = self.butter_bandpass_design(np.abs(zData_lowpass_RR), LOW_CUT_RR, HIGH_CUT_RR,
                                                                    SAMPLE_RATE)   
                #Calcolo la threshold ogni tot secondi
                self.threshold_RR=self.calibration_threshold(np.abs(zData_bandpass_RR))

                self.delta_RR = 60
                self.n_peaks_bp_RR, i_peaks_RR = self.find_peaks(np.abs(zData_bandpass_RR), self.threshold_RR, self.delta_RR)
                self.makesum=0.0
                for i in range(len(i_peaks_RR)-1):
                    self.difference= (i_peaks_RR[i+1]-i_peaks_RR[i])*0.02
                    self.makesum += self.difference
                self.average= self.makesum/len(i_peaks_RR)
                RR_value = 60/self.average
                print('numero picchi bp RR',self.n_peaks_bp_RR)
                #stop_RR = time.time()
                #time_difference_RR = stop_RR - start_RR
                #RR_value = (self.n_peaks_bp_RR * 60) / time_difference_RR

                print('resp rate: ', RR_value)
                plt.figure(2)
                plt.plot(np.abs(zData_bandpass_RR), label = 'zData bandpass RR')
                plt.plot(i_peaks_RR,np.abs(zData_bandpass_RR[i_peaks_RR]), "x")
                plt.axhline(y = self.threshold_RR, color = 'r', linestyle = '-')
                plt.title('zData_bandpass RR')

                plt.show()
                #print('time delta RR: ', time_difference_RR)
                #stop_RR = 0
                #start_RR = 0
                flag_time_RR = True
                #character = 'RR'
                MainWindow.updateValue(self,RR_value, character = 'RR')

                return RR_value
                                
    ### moving average ###
    def moving_average(self, window_length, data):
        
        smoothed = np.convolve(data, np.ones(window_length)) / window_length
    
        return smoothed
    
    ### peak finding ###
    def find_peaks(self, data, threshold, delta):
        
        i_peaks, _ = find_peaks(data, height = threshold, distance = delta)
        #plt.plot(data)
        #plt.plot(i_peaks,data[i_peaks], "x")
        #plt.axhline(y = threshold, color = 'r', linestyle = '-')
        #plt.show()
        # returns the indeces of the peaks --> use them to find the respiration peaks
        peaks = len(i_peaks)
        return peaks , i_peaks

    def butter_bandpass_design(self, signal_array, low_cut, high_cut, sample_rate, order=4):
        """
        Defines the Butterworth bandpass filter-design
        :param low_cut: Lower cut off frequency in Hz
        :param high_cut: Higher cut off frequency in Hz
        :param sample_rate: Sample rate of the signal in Hz
        :param order: Order of the filter-design
        :return: b, a : ndarray, ndarray - Numerator (b) and denominator (a) polynomials of the IIR filter. Only returned if output='ba'.
        """
        sos = signal.butter(order, [low_cut, high_cut], btype='band', output ='sos',fs=sample_rate)
        y = signal.sosfiltfilt(sos, signal_array)
        return y

    def butter_lowpass(self, cutoff, fs, order):#=5):
        return butter(order, cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(self, data, cutoff, fs, order):#=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        #y = lfilter(b, a, data)
        y = signal.filtfilt(b, a, data, padlen=len(data)-1) #uso filtfilt anzichè lfilter per rimanere in fase
        return y

    def butter_highpass(self, cutoff, fs, order):#=5):
        return butter(order, cutoff, fs=fs, btype='high', analog=False)

    def butter_highpass_filter(self, data, cutoff, fs, order):#=5):
        b, a = self.butter_highpass(cutoff, fs, order=order)
        #y = lfilter(b, a, data)
        y = signal.filtfilt(b, a, data) #uso filtfilt anzichè lfilter per rimanere in fase
        return y

    def calibration_threshold(self, val):
        """
        -------- CALIBRATION ---------
        The patient has to breath normally for 10 seconds 
        and the maximum value reached in this calibration window is used for the
        definition of a threshold.
        """
        threshold=0.0
        
        threshold= 0.8 * np.mean(val[500:2500])
        return threshold

    def calibration(self, array):
        global calibration,xData_g,yData_g,zData_g,calibration_flag,newZero, start_threshold, zData_array_LP, zData_array

        zSum = 0.0
        
        zSum = sum(array)
        zAvg = zSum/len(array)
        newZero = zAvg

        return newZero  

###############
# MAIN WINDOW #
###############
class MainWindow(QMainWindow):

    global clock
    def __init__(self):
        """!
        @brief Init MainWindow.
        """
        #Define worker
        self.serial_worker = SerialWorker(None)

        super(MainWindow, self).__init__()

        # title and geometry
        self.setWindowTitle("Respiratory / Heart Rate Measurement")
        width = 1280
        height = 720
        #self.setMaximumSize(width, height)

        #create thread handler
        self.threadpool = QThreadPool()
        
        self.connected = CONN_STATUS
        #self.serialscan()
  
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
        self.zGraph_bandpass_RR = [0]*320

        self.count = 0

        self.draw()
    
        # Plot data: x, y values
        self.drawGeneralGraph()
        
        # Toolbar
        toolbar = QToolBar("My main toolbar")   #my toolbar
        toolbar.setIconSize(QSize(16, 16))
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        #button_action = QAction("File", self)       # name of the toolbar

        save_action = QAction(QIcon("disk_return_black.png"), "Export to .csv", self)
        save_action.setStatusTip("Export to .csv")
        save_action.triggered.connect(self.save_data)
        toolbar.addAction(save_action)
        self.setStatusBar(QStatusBar(self))
        toolbar.addSeparator()

        #home_action = QAction(QIcon("home.png"), "Home",self)
        #toolbar.addAction(home_action)
        menu = self.menuBar()
        file_menu = menu.addMenu("&File")
        file_menu.addAction(save_action)
        #file_menu = menu.addMenu("&Home")
        #file_menu.addAction(home_action)

        self.HR_label = QLabel()
        text = 'Instant heart rate value: {} bpm'
        a = text.format(HR_value)
        self.HR_label.setText(a)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        self.HR_label.setFont(font)
        self.HR_label.setStyleSheet("border: 1px solid black")
        self.HR_label.setAlignment(QtCore.Qt.AlignCenter)

        self.RR_label = QLabel()
        text = 'Instant respiratory rate value: {} rpm'
        a = text.format(RR_value)
        self.RR_label.setText(a)
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
            #text=("Connect to port {}".format(self.port_text)), 
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

        # top horizontal layout
        device_search.addWidget(self.conn_btn)
        device_search.addWidget(self.conn_label)

        verticalLayout.addLayout(device_search)

        modeSelection = QHBoxLayout()

        self.modeSelect = QComboBox()
        self.modeSelect.addItem("both")
        self.modeSelect.addItem("HR Only")
        self.modeSelect.addItem("RR Only")
        self.modeSelect.currentIndexChanged.connect(self.selectionchange)

        self.calibrate = QPushButton(
            text = ("calibration")
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

        self.save_btn = QPushButton(
            text = ("Save status")
            # once clicked save FS and So ?
        )

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
        #calibrationSelection.setContentsMargins(20,20,20,20)
        #calibrationSelection.setSpacing(20)

    def save_data(self):
        print("exporting to csv...")

        global zData_save, xData_save, yData_save
        df = pd.DataFrame({
            'x axis': xData_save,
            'y axis': yData_save,
            'z axis': zData_save, 
        })
        df.to_csv('HR_Rate.csv', float_format = '%.2f', index = False)

        
    def selectionchange(self,i):
        global flag_graph
        flag_graph = i
        a = ['both','HR only', 'RR only'] 
        # 0 --> HR
        # 1 --> RR
        # 2 --> both
        #print("Plotting: ", a[flag_graph])
        ### FLAG GRAPH per plottare solo ciò che interessa --- disattivo cio che non voglio
        if (flag_graph == 0): #only HR
            self.RR_plot.setBackground('bbccdd')
            self.HR_plot.setBackground('bbccdd')
            #zData_lowpass = np.full(axisSize,10,dtype=np.float16)
            #self.dataLinez_lowpass = self.plot(self.graphWidget,clock,zData_lowpass,'z-axis low-pass filtered','r')
        elif (flag_graph == 1): #only RR
            self.RR_plot.setBackground('r')
            self.HR_plot.setBackground('bbccdd')
            #zData_lowpass = np.full(axisSize,5,dtype=np.float16)
            #self.dataLinez_lowpass = self.plot(self.HR_plot,clock,zData_lowpass,'z-axis low-pass filtered','g')
        elif (flag_graph == 2):
            self.HR_plot.setBackground('y')
            self.RR_plot.setBackground('bbccdd')
        #    zData_bandpass = np.full(axisSize,5,dtype=np.float16)
        #    self.dataLinez_bandpass = self.plot(self.RR_plot,clock,zData_bandpass,'z-axis band-pass filtered','b')

        
    def drawGeneralGraph(self):
        """!
        @brief Draw the plots.
        """
        global xData, yData, zData, xData_g, yData_g, zData_g

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
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez.setData(self.horAxis, self.zGraph)  # Update the data.

            # Z-axis low pass FILTERED
            self.zGraph_lowpass = self.zGraph_lowpass[1:]  # Remove the first 
            self.zGraph_lowpass.append(zData_lowpass_RR[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_lowpass_RR.setData(self.horAxis, self.zGraph_lowpass)  # Update the data.
            '''
                CONTROLLA COSA C'è DA DEFINIRE DI STE ROBE
            '''
            # Heart Rate
            self.zGraph_windowed_HR = self.zGraph_windowed_HR[1:]  # Remove the first 
            self.zGraph_windowed_HR.append(zData_windowed_HR[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_windowed_HR.setData(self.horAxis, np.abs(self.zGraph_windowed_HR))  # Update the data.

            # Respiratory Rate
            self.zGraph_bandpass_RR = self.zGraph_bandpass_RR[1:]  # Remove the first 
            self.zGraph_bandpass_RR.append(zData_bandpass_RR[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_bandpass_RR.setData(self.horAxis, np.abs(self.zGraph_bandpass_RR))  # Update the data.

    def updateValue(self, data, character):

        #global HR_value, RR_value, character

        text = 'Instant {} value: {}'
        R_text = text.format(character, data)

        if (character == 'RR'):
            print(R_text + 'rpm')
            self.RR_label.setText(R_text+'rpm')
        elif (character == 'HR'):
            print(R_text + 'bpm')
            self.HR_label.setText(R_text+'bpm')
            
    def draw(self):
        """!
             @brief Draw the plots.
        """
        global accData, xData, yData, zData, xData_g, yData_g, zData_g
        global zData_bandpass_RR, zData_lowpass_RR, zData_windowed_HR

        #self.dataLinex = self.plot(self.graphWidget,clock,xData_g,'x-axis','r')
        #self.dataLiney = self.plot(self.graphWidget,clock,yData_g,'y-axis','g')
        self.dataLinez = self.plot(self.graphWidget,clock,zData_g,'z-axis','b')
        self.dataLinez_lowpass_RR = self.plot(self.graphWidget,clock,zData_lowpass_RR,'z-axis low-pass filtered','r')
        self.dataLinez_bandpass_RR = self.plot(self.RR_plot,clock,np.abs(zData_bandpass_RR),'Respiratory wave','b')
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
            #acquire list of serial ports
            serial_ports = [
                p.name
                for p in serial.tools.list_ports.comports()
            ]

            for i in range(len(serial_ports)):
                self.port_text=serial_ports[i]

                #setup reading worker
                self.serial_worker = SerialWorker(self.port_text) #needs to be re defined
                connectionWait = True
                print("Active port ", self.port_text)
                # connect worker signals to functions
                
                self.serial_worker.signals.device_port.connect(self.connected_device)
                # execute the worker
                self.threadpool.start(self.serial_worker)
                #time.sleep(2)
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
            
            #self.checkToggle = bool(True)
            
        else:
           
            self.updateBtn.setChecked(False)
            # kill thread
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
        global PORT
        global TRANSMITTING

        if checked:
            self.serial_worker.send('a')
            self.updateBtn.setText("Stop")
            #self.modeSelect.setDisabled(True)
            #self.FSR_Select.setDisabled(True)
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
            #self.graphWidget.clear()
            TRANSMITTING = False
            self.timer.stop()
            self.graphTimer.stop()
            
            self.modeSelect.setDisabled(False)
            #self.FSR_Select.setDisabled(False)
            self.calibrate.setDisabled(True)

    def startCalibration(self): 
    # set calibration_flag = True when calibration button is pressed
        global calibration_flag

        calibration_flag = True
        self.calibrate.setDisabled(True) # deactivate the button after first calibration (the next ones will be authomatic)

#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())