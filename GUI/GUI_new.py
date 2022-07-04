from multiprocessing import connection
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
xavg = 0
yavg = 0
zavg = 0
connectionWait = False
calibration = False
calibration_flag=-1
start_threshold=0
max_ipo=0
count_max=0
resp_rate=0.0
time_max=0
start_time=-1
delta_time=0.5   #deve essere circa 2 secondi

FSR_index= 0
SAMPLE_RATE = 50
LOW_CUT = 0.00001
HIGH_CUT = 1.0

order = 8
fs = 50.0       # sample rate, Hz
cutoff = 3

f_bw=0.25 #Hz for normal activities, put 0.50 Hz for sport activities

'''
xData = np.zeros(axisSize)
xData = xData.astype("int16")
yData = np.zeros(axisSize)
yData = yData.astype("int16")
zData = np.zeros(axisSize)
zData = zData.astype("int16")
dataBuffer = np.zeros(96+3)#96 data + 3 separator values
'''
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
        global zData_BP_FT, sum_data, zData_array_LP,k, start_threshold, zData_smoothed, zData_array_smoothed, zData_interp
        global FSR_index
        global SAMPLE_RATE,LOW_CUT,HIGH_CUT
        global order, fs, cutoff
      
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

            '''                
            NON SO CHE FILTRO USEREMO, MA NEL PAPER CONSIGLIA UN BAND PASS [0,1]Hz, in particolare con BUTTERWORTH e calcolando la frequenza dominante nel range
            '''  
            
            if (start_threshold==1): #j inizia ad aumentare solo dopo che ho calibrato
                j=j+1
            k=k+1
            #xData_g = self.butter_lowpass_filter(xData_g, cutoff, fs, order)
            #yData_g = self.butter_lowpass_filter(yData_g, cutoff, fs, order)
            zData_lowpass = self.butter_lowpass_filter(zData_g, cutoff, fs, order)
            #xData_g = self.butter_bandpass_filter(xData_g, LOW_CUT, HIGH_CUT,
            #                                                    SAMPLE_RATE)
            #yData_g = self.butter_bandpass_filter(yData_g, LOW_CUT, HIGH_CUT,
            #                                                    SAMPLE_RATE)
            zData_bandpass = self.butter_bandpass_filter(zData_lowpass, LOW_CUT, HIGH_CUT,
                                                              SAMPLE_RATE)   

            #zData_smoothed = self.smooth_signal(zData_lowpass, SAMPLE_RATE, window_length=5, polyorder=3)
            zData_smoothed = signal.savgol_filter(zData_lowpass, window_length=5, polyorder=3)

            self.new_zero=self.calibration()   
            self.new_zero_y=self.new_zero[1]
            zData_smoothed=zData_smoothed-self.new_zero_y     #FORSE QUESTA CALIBRAZIONE FA FATTA DURARE PIù DI 32 DATI            
            
            for i in range(len(zData_lowpass)):
                zData_array_LP.append(zData_lowpass[i])

                for i in range(len(zData_smoothed)):
                    zData_array_smoothed.append(zData_smoothed[i])
            
            self.RRalgorithm(zData_array_LP)

                        
            if (k==10):
                
                zData_ty,zData_tx= self.fast_fourier_transformation(zData_array_LP,fs)
                fmax=self.calcolamax(zData_ty,zData_tx) #trovo fmax per filtro
                #print(fmax)
                k = 0
                #print("J resettato")
                f_low=fmax-f_bw
                f_high=fmax+f_bw
                if (f_low <= 0):
                    f_low = 0.0001
                # We apply again a bandpass filter over the characteristic frequency
                zData_BP_FT = self.butter_bandpass_filter(zData_bandpass, f_low, f_high,
                                                              SAMPLE_RATE)  
                for i in range(len(zData_BP_FT)):
                    zData_array.append(zData_BP_FT[i])

    ### moving average ###
    '''
    window_length = 3
    def moving average(self, window_length, xData_g, yData_g, zData_g)
        
        ### with rolling function ###
        data = xData_g #yData_g zData_g
        series = pd.Series(data)
        Mov_avg = series.rolling(window = WL, center = True).mean()
        smoothed = Mov_avg.to_numpy # optional: (dtype = 'float32') # dipende cosa ci serve
    
        return smoothed
    '''

    ### interpolation ###
    '''
    from scipy import interpolate

    x = np.arange(0, len(zData))

    y = np.cos(x)

    f = interpolate.interp1d(x, y, kind = 'cubic')

    '''

    ### acceleration Ai + PCA###
    '''
    deltaAi[i] = ([xData[i+1]-xDAta[i])**2 + (yData[i+1]-yData[i])**2 + (zData[i+1]-zData[i])**2
    # sopra da ripetere per 1 minuto
    A_body = sum(deltaAi) #in one minute --> this is the EE
    # low EE --> 0.2-0.4Hz
    ...
    ### PCA ###
    # sigma = matrix of 3 coordinates accelerations:
    sigma = [xData][yData][zData]

    # eigenvectors and eigenvalues using PCA --> lambda
    # subtract the mean of each variable (x, y, z):
    sigma = [xData - xavg][yData - yavg][zData - zavg]

    # covariance matrix
    cov_sigma = np.cov(sigma, rowvar = False)

    # eigenvalues and eigenvectors
    eigen_values , eigen_vectors = np.linalg.eigh(cov_sigma)

    # sorting
    # sort the eigenvalues in descending order
    sorted_index = np.argsort(eigen_values)[::-1]

    sorted_eigenvalue = eigen_values[sorted_index] # we need the 3 eigenvalues
    
    # similarly sort the eigenvectors 
    sorted_eigenvectors = eigen_vectors[:,sorted_index]

    # once we have the 3 eigenvalues we can calculate the weights:
    eta[i] = (sorted_eigenvalue[i])/(sum(eigenvalues)) # i = 1,2,3

    # respiratory time sequence, x0:
    x0 = eta[1]*xData + eta[2]*yData + eta[3]*zData

    ### SPECTRUM ANALYSIS OVER 1 MIN ###
    # power spectrm over 1 min x0 --> highest peak is resp rate

    sampling_rate = 50.0
    data = x0 
    fourier_transform = np.fft.rfft(data)
    abs_fourier_transform = np.abs(fourier_transform) # absolute value of fourier transform
    power_spectrum = np.square(abs_fourier_transform) # square of fourier transform --> power spectrum
    frequency = np.linspace(0, sampling_rate/2, len(power_spectrum))
    plt.plot(frequency, power_spectrum)

    # then find the max of the frequency and estimate the respiration rate

    max = argrelmax(power_spectrum, np.greater) # returns indeces of local maxima of the power spectrum
    max = max(power_spectrum) # returns the absolute maxima of the power spectrum
    '''

    ### digital filtering ###
    '''
    def filtering(self, order, cut_freq, type, fsample, data):
        # cut_freq can be either a scalar or an array type (if band pass filter)
        sos = butter(N = order, Wn = cut_freq, btype = type, output='sos', fs = fsample)
        # sos contains the num and den of coefficient for the IIR filter
        # y is the filtered signal
        y = sosfiltfilt(sos, data)

        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.filtfilt.html#scipy-signal-filtfilt
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.sosfiltfilt.html#scipy-signal-sosfiltfilt
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.sosfilt.html#scipy-signal-sosfilt+

        return y        
    '''
    

    ### peak finding ###
    '''
    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.find_peaks.html#scipy.signal.find_peaks
    # calculate the relativa maxima of the 1D array with find_peaks
    def find_peaks(self, data, threshold, delta):
        from scipy.signal import find_peaks
        peaks, _ = find_peaks(x, height = threshold, distance = delta)
        plt.plot(x)
        plt.plot(peaks, x[peaks], "x")
        plt.show()
        # returns the indeces of the peaks --> use them to find the respiration peaks
        return peaks 

    '''

    ### heart rate computation ###
    '''
    # bandpass 30,35 Hz with 4th butterworth
    HP_filtered_signal = self.filtering(4, [20, 35], 'bandpass', zData_g)
    
    # envelope detection with HILBERT TRANSFORM:
    import matplotlib.pyplot as plt
    from scipy.signal import hilbert, chirp
    envelope = np.abs(hilbert(HP_filtered_signal))

    # search for envelope local maxima in disjoint windows of duration 0.8s = 40 samples
    # - false peaks might be detected
    peaks = self.find_peaks(envelope, threshold = 0, 0.8s)
    maxima = envelope[peaks]

    # compute the dominant freq in the range 0.16, 2 Hz

    # Selecting the first maximum which satisfies the following condition:
    # abs(2xm(i+1)−m(i)−m(i+2))   < c1&abs(2xm(i+2)−m(i+1)−m(i+3))  <  c2&abs((m(i+1)−m(i))−to)  <

    # Propagate information from the first maximum to find other peaks by an adaptive threshold:
    # t∗=floor(1/3x(m(i)−m(i−3)))

    # Finally, refine the peak time positions on the [20,80] Hz band-pass filtered envelope.

    '''


                


    def butter_bandpass_design(self, low_cut, high_cut, sample_rate, order=4):
        """
        Defines the Butterworth bandpass filter-design
        :param low_cut: Lower cut off frequency in Hz
        :param high_cut: Higher cut off frequency in Hz
        :param sample_rate: Sample rate of the signal in Hz
        :param order: Order of the filter-design
        :return: b, a : ndarray, ndarray - Numerator (b) and denominator (a) polynomials of the IIR filter. Only returned if output='ba'.
        """
        nyq = 0.5 * sample_rate
        low = low_cut / nyq
        high = high_cut / nyq
        b, a = signal.butter(order, [low, high], btype='band')

        return b, a
    def butter_bandpass_filter(self, signal_array, low_cut, high_cut, sample_rate, order=4):
        """
        Apply's the filter design on the signal_array.
        :param signal_array: signal, which should get filtered - as ndarray
        :param low_cut: Lower cut off frequency in Hz
        :param high_cut: Higher cut off frequency in Hz
        :param sample_rate: Sample rate of the signal in Hz
        :param order: Order of the filter-design
        :return: ndarray - The filtered output, an array of type numpy.float64 with the same shape as signal_array.
        """
        b, a = self.butter_bandpass_design(low_cut, high_cut, sample_rate, order=order)
        y = signal.filtfilt(b, a, signal_array)

        return y

    def butter_lowpass(self, cutoff, fs, order):#=5):
        return butter(order, cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(self, data, cutoff, fs, order):#=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        #y = lfilter(b, a, data)
        y = signal.filtfilt(b, a, data) #uso filtfilt anzichè lfilter per rimanere in fase
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
        plt.semilogy(xf[1:N//2], 2.0/N * np.abs(yf[1:N//2]), '-b')
        plt.semilogy(xf[1:N//2], 2.0/N * np.abs(ywf[1:N//2]), '-r')
        plt.legend(['FFT', 'FFT w. window'])
        
        plt.grid()
        plt.show()

        return ywf, xf

    def calcolamax(self, yf, xf):
        yf_max=0
        index_max=0
        for i in range(len(xf)):
            if (yf[i] > yf_max):
                yf_max=yf[i]
                index_max=i

        f_max=xf[index_max]
        return f_max

    def smooth_signal(data, sample_rate, window_length=None, polyorder=3):
        '''
        smooths given signal using savitzky-golay filter
        Function that smooths data using savitzky-golay filter using default settings.
        Functionality requested by Eirik Svendsen. Added since 1.2.4
        Parameters
        '''
        if window_length == None:
            window_length = sample_rate // 10

        if window_length % 2 == 0 or window_length == 0: window_length += 1

        smoothed = signal.savgol_filter(data, window_length = window_length,
                                 polyorder = polyorder)

            
        return smoothed    

    def RRalgorithm(self, data):
        global calibration_flag, time_max, delta_time, j, max_ipo, start_time, count_max, resp_rate, index_increment
        
        '''
        ------ PEAK DETECTION ---------
        The algorithm to detect the peak is based on the simultaneous usage
        of a time threshold (to prevent 'false respiration') and a numerical
        threshold (determined during the calibration phase)
        '''
        if (j==15): #così ho un array dato da 9.6 secondi di registrazione
            threshold=self.calibration_threshold(data)  #in this way I can calculate a new threshold every tot sec
            print('thr', threshold)

        #Calibration has finished. Research for maxima begins
        if (calibration_flag==1):
            for i in range(index_increment,len(data)-2):
                
                if(data[i+2]<data[i+1] and data[i+1]>data[i] and data[i+1]>threshold): 

                    
                    max_ipo=data[i+1]
                    print('indice:',i+1)
                    flag_max_ipo_found=1
                    print('max_ipo',max_ipo)
                        
                    if (flag_max_ipo_found):
                        stop_time=time.time()
                        print('stop time', stop_time)
                        flag_max_ipo_found=0
                        time_max=stop_time - start_time
                        print('time max', time_max)

                        if (time_max >= delta_time):
                            max_real=max_ipo
                            stop_time=0
                            start_time=0
                            start_time=time.time()  #mi riparte quando ho trovato un massimo vero
                            print('start time ', start_time)
                            print('max-real',max_real)
                    
                            count_max+=1
                            print('count_max', count_max)
                            if (count_max>=2):  #we need to detect 2 absolute maxima before starting to provide a respiratory frequency
                                #resp_rate= 60/(time_max*0.02)  
                                resp_rate= 60/(time_max)    #gio dice che non ci va la frequenza di campionamento
                                print('Resp rate:',resp_rate)
                        #time_max=0
                        #count_max=0
                        #flag_max_found=0    
            index_increment=len(data)      
            print('index_increment',index_increment)     
            #j=0
            

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
        global calibration,xData_g,yData_g,zData_g,calibration_flag,newZero, start_threshold, j, zData_array_LP, zData_array_smoothed
        calibration_flag=0
        
        if(calibration):
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
            for i in range(len(zData_array_smoothed)):
                ySum = ySum + zData_array_smoothed[i]
            yAvg = ySum/len(zData_array_smoothed)
            newZero[1] = yAvg
            newZero[0] = xSum
            #newZero[1] = ySum
            for i in range(len(zData_array_LP)):
                zSum = zSum + zData_array_LP[i]
            zAvg = zSum/len(zData_array_LP)
            newZero[2] = zAvg

            zData_array_smoothed=[] #lo azzero quando clicco calibrazione
            zData_array_LP=[]    #lo azzero quando clicco calibrazione
            start_threshold=1
        calibration = False
        return newZero  

    """
    def RRalgortithm(self):
        global xData_g,yData_g,zData_g
        nSamples = 200 #a caso, dovrà essere il numero di sample in 2s
        max = 0
        index1 = 0
        index2 = 0

        xDataWindow = xDataWindow + xData_g
        if(np.size(xDataWindow)==nSamples):
            for i in len(xDataWindow):
                if(xDataWindow[i]>max):
                    max = xDataWindow[i]
                    index1 = i
    """

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
        RR_value = resp_rate
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

        self.FSR_Select = QComboBox()
        self.FSR_Select.setEditable(False)
        self.FSR_Select.addItem("FS: ±2 g")
        self.FSR_Select.addItem("FS: ±4 g")
        #self.FSR_Select.activated.connect(self.calibration(1)) #con .activated "Used when an item is selected by the user."
        self.FSR_Select.currentIndexChanged.connect(self.change_graph)  #quando l'item nella lista viene cambiato mi porta a change_graph

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

        
    def change_graph (self):#,index
        """
        @brief Curve calibration
        """
        global FSR_index, old_calibration
        self.dataLinex.clear()
        self.dataLiney.clear()
        self.dataLinez.clear()
        #self.graphWidget.clear()
        FSR_index = self.FSR_Select.currentIndex()  #indica l'indice del combo box selezionato, valore di default = -1

        #self.draw()
        #self.static_canvas1.draw()

        # LA PARTE COMMENTATA SAREBBE PER PULIRE IL GRAFICO MA LO FA SOLO UNA VOLTA
        #old_calibration = calibration_index
        #calibration_index = self.FSR_Select.itemData(index)
        #if (old_calibration != calibration_index):
        #    self.graphWidget.clear()
        
        
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
            # Z-axis interp
            self.zGraph_interp = self.zGraph_interp[1:]  # Remove the first 
            self.zGraph_interp.append(zData_interp[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_interp.setData(self.horAxis, self.zGraph_interp)  # Update the data.
            # Z-axis low pass FILTERED
            self.zGraph_lowpass = self.zGraph_lowpass[1:]  # Remove the first 
            self.zGraph_lowpass.append(zData_lowpass[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_lowpass.setData(self.horAxis, self.zGraph_lowpass)  # Update the data.
            # Z-axis low pass FILTERED
            self.zGraph_smoothed = self.zGraph_smoothed[1:]  # Remove the first 
            self.zGraph_smoothed.append(zData_smoothed[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_smoothed.setData(self.horAxis, self.zGraph_smoothed)  # Update the data.
            '''
            # Z-axis band pass FILTERED
            self.zGraph_bandpass = self.zGraph_bandpass[1:]  # Remove the first 
            self.zGraph_bandpass.append(zData_bandpass[i])
            #self.zGraph.append(zData_g[i])  #  Add a new random value.
            self.dataLinez_bandpass.setData(self.horAxis, self.zGraph_bandpass)  # Update the data.
            '''
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
        self.dataLinez_interp = self.plot(self.graphWidget,clock,zData_interp,'z-axis interp','black')
        self.dataLinez_lowpass = self.plot(self.HR_plot,clock,zData_lowpass,'z-axis low-pass filtered','r')
        self.dataLinez_smoothed = self.plot(self.HR_plot,clock,zData_smoothed,'z-axis smoothed','b')
        self.dataLinez_bandpass = self.plot(self.RR_plot,clock,zData_bandpass,'z-axis band-pass filtered','g')
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
            self.modeSelect.setDisabled(True)
            self.FSR_Select.setDisabled(True)
            TRANSMITTING = True
            self.timer.timeout.connect(lambda: self.serial_worker.readData())
            self.timer.start()
            self.graphTimer.timeout.connect(lambda: self.drawGeneralGraph())
            self.graphTimer.start() 
            self.calibrate.setDisabled(False)

        else:
            self.serial_worker.send('s')
            self.updateBtn.setText("Start")
            #self.graphWidget.clear()
            TRANSMITTING = False
            self.timer.stop()
            self.graphTimer.stop()
            self.modeSelect.setDisabled(False)
            self.FSR_Select.setDisabled(False)
            self.calibrate.setDisabled(True)

    def startCalibration(self):
        global calibration

        calibration = True
        #SerialWorker.calibration(self)
        
      

#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())