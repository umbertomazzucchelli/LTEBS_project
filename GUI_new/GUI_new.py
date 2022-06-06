import struct
import sys
from telnetlib import STATUS
import time
import logging
#import matplotlib #.axis import XAxis
import numpy as np
#import matplotlib 
import peakutils
import scipy.signal as signal
from scipy.fftpack import fft

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

#Global
CONN_STATUS = False
STATUS=True
PORT = ""
TRANSMITTING = False
STARTED = False
dataSize = 98
baudRate = 9600 #9600 for USB, 115200 for BT
accData = []
axisSize = dataSize//3
xData = np.full(axisSize,0,dtype=np.int16)
yData = np.full(axisSize,0,dtype=np.int16)
zData = np.full(axisSize,0,dtype=np.int16)
clock = np.zeros(axisSize)
FSR_index= 0
old_FSR=0

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
        global PORT

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
                        
                    
            except serial.SerialException:
                logging.info("Error with port {}.".format(self.port_name))
                self.signals.status.emit(self.port_name, 0)
                time.sleep(0.01)


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
        global accData, xData, yData, zData
        global FSR_index

        #self.serial_worker = SerialWorker(PORT)
        
        dataArray = self.port.read(194)
        dataArray = struct.unpack('194B',dataArray)
        lastIndex = len(dataArray)-1
        if(dataArray[0]==10 and dataArray[lastIndex]==11):
            accData = dataArray[1:193]
            for i in range(axisSize):
                xData[i] =  (accData[i*6] | (accData[i*6+1]<<8))>>6
                yData[i] =  (accData[i*6+2] | (accData[i*6+3]<<8))>>6
                zData[i] =  (accData[i*6+4] | (accData[i*6+5]<<8))>>6
                if FSR_index==0:    #+-2g   
                    xData[i]=xData[i]/256 -2
                    yData[i]=yData[i]/256 -2
                    zData[i]=zData[i]/256 -2
                    print(xData)
                elif FSR_index==1:  #+- 4g
                    xData[i]=xData[i]/128 -4
                    yData[i]=yData[i]/128 -4
                    zData[i]=zData[i]/128 -4
        
        '''
        
        dataString = self.readAcc()
        #print("dataString type:" ,type(dataString))
        #print(len(dataString))
        dataArray = dataString.split(',')
        if(dataArray[0]==-32768):
            print("dataArray len",len(dataArray))
            print(dataArray)

        print('clock data:')
        print(clock)
        print(dataArray)
        print(accData)
        print("X data:")
        print(xData)
        print("Y data:")
        print(yData)
        print("Z data:")   
        print(zData)
        #print(dataArray)
        '''

        '''
        while(TRANSMITTING==True):
    
            header = self.port.read(1)
            header = struct.unpack('1B',header)[0]
            print("Header ",header)
            if(header==0x0A):
                dataArray = self.port.read(192)
                dataArray = struct.unpack('192B',dataArray)
                if

            dataArray = self.port.read(194)
            dataArray = struct.unpack('194B',dataArray)
            print(dataArray)
        '''

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
        self.setMaximumSize(width, height)

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
        self.graphWidget.setLabel('left', 'Acc data m/s2', **styles)
        self.graphWidget.setLabel('bottom', 'Time [ms]', **styles)
            # Add legend
        self.graphWidget.addLegend()
        self.graphWidget.setMouseEnabled(x=False, y=False)
        
        
    
        # Display 100 time points
        self.horAxis = list(range(320))  #100 time points
        self.xGraph = [0]*320
        self.yGraph = [0]*320
        self.zGraph = [0]*320
        self.count = 0

        self.draw()

        '''
        self.dataLinex = self.graphWidget.plot(self.h,self.x)
        self.dataLiney = self.graphWidget.plot(self.h,self.y)
        self.dataLinez = self.graphWidget.plot(self.h,self.z)
        '''
    
        # Plot data: x, y values
        self.drawGeneralGraph()
        
        # Toolbar
        toolbar = QToolBar("My main toolbar")   #my toolbar
        toolbar.setIconSize(QSize(16, 16))
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        #button_action = QAction("File", self)       # name of the toolbar

        button_action=QAction(QIcon("disk_return_black.png"),"Save Data", self)  #home icon
        button_action.setStatusTip("My botton")
        button_action.setCheckable(True)    #now I can press it
        toolbar.addAction(button_action)
        toolbar.addSeparator()
        button_action2 = QAction(QIcon("home.png"), "Your &button2",self)
        toolbar.addAction(button_action2)
        menu = self.menuBar()
        file_menu = menu.addMenu("&File")
        file_menu.addAction(button_action)

        gridlayout = QGridLayout()
        verticalLayout = QVBoxLayout()
        device_search = QHBoxLayout()
    
        self.conn_btn = QPushButton(
            #text=("Connect to port {}".format(self.port_text)), 
            text=("Device search"), 
            checkable=True,
            toggled=self.on_toggle)

        self.conn_label = QLabel(
            text = ("No Device Connected")
        )
        font = QFont()
        font.setBold(True)
        font.setWeight(75)
        self.conn_label.setFont(font)
        self.conn_label.setStyleSheet("background-color: rgb(255, 0, 0);\n"
"border: 1px solid black")
        self.conn_label.setAlignment(QtCore.Qt.AlignCenter)

        # top horizontal layout
        device_search.addWidget(self.conn_btn)
        device_search.addWidget(self.conn_label)

        verticalLayout.addLayout(device_search)

        modeSelection = QHBoxLayout()

        self.modeSelect = QComboBox()
        self.modeSelect.addItem("HR Only")
        self.modeSelect.addItem("RR Only")
        self.modeSelect.addItem("both")

        self.calibrate = QPushButton(
            text = ("calibration")
        )

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

        saveStatus = QHBoxLayout()

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

        saveStatus.addWidget(self.save_btn) 
        saveStatus.addWidget(self.FSR_Select)
        verticalLayout.addLayout(saveStatus)
        #plot = QHBoxLayout
        #plot.addWidget(self.graphWidget)

        verticalLayout.addWidget(self.graphWidget)


        # ModeSelect Combo-Box
       # self.modeSelect = QComboBox()
       # self.modeSelect.setEditable(False)
       # self.modeSelect.addItems(["HR only", "RR only","Both"])

        # CalibrationSelect Combo-Box
        #self.calibrationSelect = QComboBox()
        #self.calibrationSelect.setEditable(False)
        #self.calibrationSelect.addItems(["Digit","Calibration +-2g", "Calibration +-4g"])
        #self.calibrationSelect.activated.connect(self.calibration)

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

        # layout
        #serialButton = QHBoxLayout()
        ##serialButtons.addWidget(self.com_list_widget)
        #serialButton.addWidget(self.conn_btn)
        #modeSelection = QHBoxLayout()
        #modeSelection.addWidget(self.modeSelect)
        #modeSelection.addWidget(self.updateBtn)
        #calibrationSelection = QHBoxLayout()
        #calibrationSelection.addWidget(self.modeSelect)
        #calibrationSelection.addWidget(self.updateBtn)
        #dataGraph = QHBoxLayout()
        #dataGraph.addWidget(self.graphWidget)
        #
        #RRHRgraphs = QHBoxLayout()
        #RRHRgraphs.addWidget(self.graphWidget)
        #RRHRgraphs.addWidget(self.graphWidget)
        #vlay = QVBoxLayout()
        #vlay.addLayout(serialButton)
        #vlay.addLayout(modeSelection)
        #vlay.addLayout(calibrationSelection)
        #vlay.addLayout(dataGraph)
        #vlay.addLayout(RRHRgraphs)

        widget = QWidget()
        widget.setLayout(verticalLayout)
        self.setCentralWidget(widget)

        modeSelection.setContentsMargins(5,5,5,5)
        modeSelection.setSpacing(5)
        #calibrationSelection.setContentsMargins(20,20,20,20)
        #calibrationSelection.setSpacing(20)
        
        
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
        global xData, yData, zData

        for i in range(len(xData)):

            # Remove the first y element.
            if(self.count<321):
                
                self.count += 1
            else:
                self.horAxis = self.horAxis[1:]
                self.horAxis.append(self.horAxis[-1] + 1)  # Add a new value 1 higher than the last.

            # X-axis
            self.xGraph = self.xGraph[1:]  # Remove the first 
            self.xGraph.append(xData[i])  #  Add a new random value.
            self.dataLinex.setData(self.horAxis, self.xGraph)  # Update the data.
            # Y-axis
            self.yGraph = self.yGraph[1:]  # Remove the first 
            self.yGraph.append(yData[i])  #  Add a new random value.
            self.dataLiney.setData(self.horAxis, self.yGraph)  # Update the data.
            # Z-axis
            self.zGraph = self.zGraph[1:]  # Remove the first 
            self.zGraph.append(zData[i])  #  Add a new random value.
            self.dataLinez.setData(self.horAxis, self.zGraph)  # Update the data.
        
        '''
        def update_plot_data(self):
        global clock

        clock=clock[1:]
        self.clock.append(clock[-1] + 1)

        self.xData = self.xData[1:]
        self.xData.append(self.xData)

        self.line.setData(clock,self.xData)
        '''

    def draw(self):
        """!
             @brief Draw the plots.
        """
        global accData, xData, yData, zData

        self.dataLinex = self.plot(self.graphWidget,clock,xData,'x-axis','r')
        self.dataLiney = self.plot(self.graphWidget,clock,yData,'y-axis','g')
        self.dataLinez = self.plot(self.graphWidget,clock,zData,'z-axis','b')

    
    def plot(self, graph, x, y, curve_name, color):
        """!
        @brief Draw graph.
        """
        pen = pg.mkPen(color=color,width=2)
        line = graph.plot(x, y, name=curve_name, pen=pen)
        return line

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

    def fast_fourier_transformation(self, signal_array, sample_rate):
        """
        Apply's the Fast Furier Transformation. This transforms the signal into an power spectrum in frequency domain.
        :param signal_array: signal as ndarray
        :param sample_rate: Sample rate of the signal in Hz
        :return: yf : complex ndarray - Results of the FFT
                 xf : ndarray - frequency parts in an equally interval
        """
        N = signal_array.size  # number of sample points
        T = 1 / sample_rate  # sample spacing
        yf = fft(signal_array)

        # xf = fftfreq(N, T)  # for all frequencies
        xf = np.linspace(0.0, 1.0 / (2.0 * T), N / 2)  # for positive frequencies only

        return yf, xf


    ##################
    # SERIAL SIGNALS #
    ##################
    '''
    def port_changed(self):
        """!
        @brief Update conn_btn label based on selected port.
        """
        self.port_text = self.com_list_widget.currentText()
        #self.conn_btn.setText("Connect to port {}".format(self.port_text))
        self.conn_btn.setText("Device search")
    '''

    @pyqtSlot(bool)
    def on_toggle(self, checked):
        global CONN_STATUS
        """!
        @brief Allow connection and disconnection from selected serial port.
        """
        if checked:
            self.conn_btn.setText("Searching device...") 
            #acquire list of serial ports
            serial_ports = [
                p.name
                for p in serial.tools.list_ports.comports()
            ]

            for i in range(len(serial_ports)):
                self.port_text=serial_ports[i]

                #setup reading worker
                self.serial_worker = SerialWorker(self.port_text) #needs to be re defined
                print("Porta attiva ", self.port_text)
                # connect worker signals to functions
                self.serial_worker.signals.status.connect(self.check_serialport_status)
                self.serial_worker.signals.device_port.connect(self.connected_device)
                # execute the worker
                self.threadpool.start(self.serial_worker)
                time.sleep(2)
                if(CONN_STATUS==True):
                    break
            self.updateBtn.setDisabled(False)
            
            #self.checkToggle = bool(True)
            
        else:
           
            self.updateBtn.setChecked(False)
            # kill thread
            self.serial_worker.is_killed = True
            self.serial_worker.killed()
            #self.com_list_widget.setDisabled(False) # enable the possibility to change port
            self.conn_btn.setText("Device search")
            self.updateBtn.setDisabled(True)
            

    def check_serialport_status(self, port_name, status):
        """!
        @brief Handle the status of the serial port connection.

        Available status:
            - 0  --> Error during opening of serial port
            - 1  --> Serial port opened correctly
        """
        if status == 0:
            self.conn_btn.setChecked(False)
        elif status == 1:
            # enable all the widgets on the interface
            #self.com_list_widget.setDisabled(True) # disable the possibility to change COM port when already connected
            self.conn_btn.setText(
                "Disconnect from port {}".format(port_name)
            )

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

        else:
            self.serial_worker.send('s')
            self.updateBtn.setText("Start")
            #self.graphWidget.clear()
            TRANSMITTING = False
            self.timer.stop()
            self.graphTimer.stop()
            self.modeSelect.setDisabled(False)
            self.FSR_Select.setDisabled(False)

#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())