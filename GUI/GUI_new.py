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

zData_lowpass = np.full(axisSize,0,dtype=np.float16)
zData_bandpass = np.full(axisSize,0,dtype=np.float16)
zData_BP_FT = np.full(axisSize,0,dtype=np.float16)
zData_smoothed = np.full(axisSize,0,dtype=np.float16)
zData_interp = np.full(axisSize,0,dtype=np.float16)
zData_array = []
zData_array_smoothed = []
sum_data = np.full(axisSize,0,dtype=np.float16)
zData_array_LP = []
index_increment = 0
newZero = np.zeros(3)

clock = np.zeros(axisSize)
j = 0
k = 0
p = 0
xavg = 0
yavg = 0
zavg = 0
connectionWait = False
calibration = False
calibration_flag=-1
start_threshold=0
max_ipo=0
count_max=0
RR_value=0.0
flag_time = False
time_difference=0
#start_time=-1
#delta_time=0.5   #deve essere circa 2 secondi

SAMPLE_RATE = 50
LOW_CUT = 0.01
HIGH_CUT = 1.0

order = 8
cutoff = 3

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
        global accData, xData, yData, zData, xData_g, yData_g, zData_g, j, zData_lowpass, zData_bandpass, zData_array
        global zData_BP_FT, sum_data, zData_array_LP,k, start_threshold, zData_smoothed, zData_array_smoothed, RR_value, flag_time
        global SAMPLE_RATE,LOW_CUT,HIGH_CUT
        global order, cutoff, p
      
        #self.serial_worker = SerialWorker(PORT)
        
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
                
                #sum_data[i]=zData_g[i]+yData_g[i]      #vediamo se usare solo z o la somma dei due

            if (calibration_flag):
                j+=1    #chiamarlo count_sec
                zData_array = np.append(zData_array, zData_g)
                if (flag_time):
                    start = time.time()
                    flag_time = False
                
                if (j==20):     #vogliamo 10 secondi

                    self.new_zero=self.calibration()   #azzero j in calibration
                    self.new_zero_z=self.new_zero[2]        #new_zero[ZAXIS] , ZAXIS = 2
                    zData_array=zData_array-self.new_zero_z 
                    # Calcoliamo i dati low pass dopo aver creato un array di tot secondi e dopo aver calibrato a zero
                    zData_lowpass = self.butter_lowpass_filter(zData_array, cutoff, SAMPLE_RATE, order)
            
                    zData_bandpass = self.butter_bandpass_design(zData_lowpass, LOW_CUT, HIGH_CUT,
                                                                        SAMPLE_RATE)   
                    #Calcolo la threshold ogni tot secondi
                    self.threshold=self.calibration_threshold(zData_bandpass)

                    self.delta1=40
                    self.n_peaks_bp = self.find_peaks(zData_bandpass, self.threshold, self.delta1)
                    print('numero picchi bp',self.n_peaks_bp)
                    stop = time.time()
                    self.time_difference = stop - start
                    RR_value = (self.n_peaks_bp * 60) / self.time_difference
                    stop = 0
                    start = 0
                    flag_time = True
            
    ### moving average ###

    def moving_average(self, window_length, data):
        
        ### with rolling function ###
        #series = pd.Series(data)
        #series = data.to_frame()
        #Mov_avg = series.rolling(window = window_length, center = True).mean()
        #smoothed = Mov_avg.to_numpy # optional: (dtype = 'float32') # dipende cosa ci serve
        smoothed = np.convolve(data, np.ones(window_length)) / window_length
    
        return smoothed
    
    ### peak finding ###
    
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.find_peaks.html#scipy.signal.find_peaks
    # calculate the relativa maxima of the 1D array with find_peaks

    def find_peaks(self, data, threshold, delta):
        
        i_peaks, _ = find_peaks(data, height = threshold, distance = delta)
        plt.plot(data)
        plt.plot(i_peaks,data[i_peaks], "x")
        plt.show()
        # returns the indeces of the peaks --> use them to find the respiration peaks
        peaks = len(i_peaks)
        return peaks 

    def butter_bandpass_design(self, signal_array, low_cut, high_cut, sample_rate, order=4):
        """
        Defines the Butterworth bandpass filter-design
        :param low_cut: Lower cut off frequency in Hz
        :param high_cut: Higher cut off frequency in Hz
        :param sample_rate: Sample rate of the signal in Hz
        :param order: Order of the filter-design
        :return: b, a : ndarray, ndarray - Numerator (b) and denominator (a) polynomials of the IIR filter. Only returned if output='ba'.
        """
        nyq = 0.5 * sample_rate
        #low = low_cut / nyq
        #high = high_cut / nyq
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

    def fast_fourier_transformation(self, signal_array, sample_rate):
        """
        Apply's the Fast Furier Transformation. This transforms the signal into an power spectrum in frequency domain.
        :param signal_array: signal as ndarray
        :param sample_rate: Sample rate of the signal in Hz
        :return: yf : complex ndarray - Results of the FFT
                 xf : ndarray - frequency parts in an equally interval
        """
        N = int(len(signal_array))  # number of sample points
        #N=320
        T = 1 / sample_rate  # sample spacing
        yf = fft(signal_array)
        w=blackman(N)
        ywf= fft(signal_array*w)
        #xf = fftfreq(N, T)  # for all frequencies
        #xf = np.linspace(0.0, 1.0 / (2.0 * T), N, endpoint=False)  # for positive frequencies only
        xf=fftfreq(N,T)[:N//2]
        #plt.semilogy(xf[1:N//2], 2.0/N * np.abs(yf[1:N//2]), '-b')
        #plt.semilogy(xf[1:N//2], 2.0/N * np.abs(ywf[1:N//2]), '-r')
        #plt.legend(['FFT', 'FFT w. window'])
        
        #plt.grid()
        #plt.show()

        return ywf, xf

    def calibration_threshold(self, val):
        """
        -------- CALIBRATION ---------
        The patient has to breath normally for 10 seconds 
        and the maximum value reached in this calibration window is used for the
        definition of a threshold.
        """
        global calibration_flag #azzerata dentro calibration
        self.max_calibration=0
        threshold=0.0

        #self.second=time.time()
        
        if(calibration_flag==0):    
            for i in range(len(val)):
                #si può accendere il LED qui durante calibrazione
                if (val[i]>self.max_calibration):
                    self.max_calibration=val[i]
                #Defining a calibration threshold on the 50% of the maximum   
            calibration_flag=1
            threshold=self.max_calibration*0.5
            
        return threshold

    def calibration(self):
        global calibration,xData_g,yData_g,zData_g,calibration_flag,newZero, start_threshold, j, zData_array_LP, zData_array
        calibration_flag=0
    
        j=0
        xSum = 0.0
        ySum = 0.0
        zSum = 0.0
        '''
        for i in range(len(xData_g)):  #SAREBBERO DA FARE SUI DATI FILTRATI
            xSum = xSum + xData_g[i]
        xAvg = xSum/len(xData_g)
        newZero[0] = xAvg

        for i in range(len(yData_g)):
            ySum = ySum + yData_g[i]
        yAvg = ySum/len(yData_g)
        newZero[1] = yAvg
        '''
        newZero[0] = xSum
        newZero[1] = ySum
        
        for i in range(len(zData_array)):
            zSum = zSum + zData_array[i]
        zAvg = zSum/len(zData_array)
        newZero[2] = zAvg

    
        zData_array=[]    #lo azzero quando clicco calibrazione
        #start_threshold=1
        #calibration = False            #DA RISISTEMARE SE NON SI VUOLE CALIBRAZIONE AUTOMATICA A 0 OGNI 10 SECONDI
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
        global RR_value
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
        self.zGraph_smoothed = [0]*320
        self.zGraph_lowpass = [0]*320
        self.zGraph_bandpass = [0]*320
        self.zGraph_BP_FT = [0]*320
        self.zGraph_interp = [0]*320
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
        HR_value = 100 # this value is update with the algorithm
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
        text = 'Instant respiratory rate value: {}'
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

        '''
        #mostra dialog di errore se il psoc è stato disconnesso per sbaglio
        self.dlg3 = QMessageBox(self)
        self.dlg3.setWindowTitle("WARNING")
        self.dlg3.setText("Connection lost, reconnect the device before proceeding")
        self.dlg3.setStandardButtons(QMessageBox.Ok)
        self.dlg3.setIcon(QMessageBox.Critical)
        button=self.dlg3.exec_()
        if(button==QMessageBox.Ok):
            self.dlg3.accept()
        '''

        widget = QWidget()
        widget.setLayout(verticalLayout)
        self.setCentralWidget(widget)

        modeSelection.setContentsMargins(5,5,5,5)
        modeSelection.setSpacing(5)
        #calibrationSelection.setContentsMargins(20,20,20,20)
        #calibrationSelection.setSpacing(20)

    def save_data(self):
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
        a = ['both','HR only', 'RR only'] 
        # 0 --> HR
        # 1 --> RR
        # 2 --> both
        print("Plotting: ", a[flag_graph])
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
        global xData, yData, zData, xData_g, yData_g, zData_g, zData_lowpass, zData_bandpass, zData_BP_FT, zData_smoothed, zData_interp

        for i in range(len(xData)):

            # Remove the first y element.
            if(self.count<321):
                
                self.count += 1
            else:
                self.horAxis = self.horAxis[1:]
                self.horAxis.append(self.horAxis[-1] + 1)  # Add a new value 1 higher than the last.
            '''
            # X-axis
            self.xGraph = self.xGraph[1:]  # Remove the first
            self.xGraph.append(xData_g[i])  #  Add a new random value. 
            #self.xGraph.append(xData_g[i])  #  Add a new random value.
            self.dataLinex.setData(self.horAxis, self.xGraph)  # Update the data.
            # Y-axis
            self.yGraph = self.yGraph[1:]  # Remove the first 
            self.yGraph.append(yData_g[i])
            #self.yGraph.append(yData_g[i])  #  Add a new random value.
            self.dataLiney.setData(self.horAxis, self.yGraph)  # Update the data.
            '''
            # Z-axis
            self.zGraph = self.zGraph[1:]  # Remove the first 
            self.zGraph.append(zData_g[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez.setData(self.horAxis, self.zGraph)  # Update the data.
            '''
            # Z-axis interp
            self.zGraph_interp = self.zGraph_interp[1:]  # Remove the first 
            self.zGraph_interp.append(zData_interp[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_interp.setData(self.horAxis, self.zGraph_interp)  # Update the data.
            '''

            # Z-axis low pass FILTERED
            self.zGraph_lowpass = self.zGraph_lowpass[1:]  # Remove the first 
            self.zGraph_lowpass.append(zData_lowpass[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_lowpass.setData(self.horAxis, self.zGraph_lowpass)  # Update the data.

            # Z-axis sgavigonk FILTERED
            self.zGraph_smoothed = self.zGraph_smoothed[1:]  # Remove the first 
            self.zGraph_smoothed.append(zData_smoothed[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_smoothed.setData(self.horAxis, self.zGraph_smoothed)  # Update the data.

            # Z-axis band pass FILTERED
            self.zGraph_bandpass = self.zGraph_bandpass[1:]  # Remove the first 
            self.zGraph_bandpass.append(zData_bandpass[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_bandpass.setData(self.horAxis, self.zGraph_bandpass)  # Update the data.
            
            # Z-axis band pass AFTER FT
            self.zGraph_BP_FT = self.zGraph_BP_FT[1:]  # Remove the first 
            self.zGraph_BP_FT.append(zData_BP_FT[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_BP_FT.setData(self.horAxis, self.zGraph_BP_FT)  # Update the data.
            
    def draw(self):
        """!
             @brief Draw the plots.
        """
        global accData, xData, yData, zData, xData_g, yData_g, zData_g, zData_lowpass, zData_bandpass, zData_BP_FT, zData_smoothed, zData_interp

        #self.dataLinex = self.plot(self.graphWidget,clock,xData_g,'x-axis','r')
        #self.dataLiney = self.plot(self.graphWidget,clock,yData_g,'y-axis','g')
        self.dataLinez = self.plot(self.graphWidget,clock,zData_g,'z-axis','b')
        self.dataLinez_lowpass = self.plot(self.graphWidget,clock,zData_lowpass,'z-axis low-pass filtered','r')
        #self.dataLinez_interp = self.plot(self.graphWidget,clock,zData_interp,'z-axis interp','black')
        #self.dataLinez_lowpass = self.plot(self.HR_plot,clock,zData_lowpass,'z-axis low-pass filtered','r')
        self.dataLinez_smoothed = self.plot(self.HR_plot,clock,zData_smoothed,'z-axis smoothed','b')
        self.dataLinez_bandpass = self.plot(self.HR_plot,clock,zData_bandpass,'z-axis band-pass filtered','g')
        self.dataLinez_BP_FT = self.plot(self.RR_plot,clock,zData_BP_FT,'z-axis band-pass after FT','black')
    
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
        

        if self.serial_worker.port.isOpen() == False:
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
        global calibration_flag

        calibration_flag = True

    def selectionchange(self,i):
        global flag_graph
        flag_graph = i
        a = ['both','HR only', 'RR only'] 
        # 0 --> HR
        # 1 --> RR
        # 2 --> both
        print("Plotting: ", a[flag_graph])
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

#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())