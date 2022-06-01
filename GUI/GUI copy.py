import struct
import sys
from telnetlib import STATUS
import time
import logging
import numpy as np

from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 
from PyQt5 import QtCore, QtGui, QtWidgets


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

dataSize = 192
accData = []
axisSize = dataSize//6
xData = np.full(axisSize,0,dtype=np.int16)
yData = np.full(axisSize,0,dtype=np.int16)
zData = np.full(axisSize,0,dtype=np.int16)
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
        self.is_killed = False
        super().__init__()
        #init port, params and signals
        self.port = serial.Serial()
        self.port_name = serial_port_name
        #self.baudrate = 9600 #hard coded but can be a global variable, or an input param
        self.baudrate = 115200 #BT baudrate
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
                    time.sleep(0.5)

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
    def readArray(self,size):
        retArray=[]
        try:
            while(self.port.in_waiting>0):
                retArray += self.port.read(size).decode('utf-8', errors='replace')
                if(len(retArray)==size):
                    break
                logging.info(self.port.in_waiting)
            logging.info("Array received")
            return retArray
        except:
            logging.info("Could not receive the array")
   
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

  
        
        #self.serial_worker = SerialWorker(PORT)
        try:
            dataArray = self.port.read(194)
            dataArray = struct.unpack('194B',dataArray)
            lastIndex = len(dataArray)-1
            if(dataArray[0]==10 and dataArray[lastIndex]==11):
                accData = dataArray[1:193]
                for i in range(axisSize):
                    xData[i] =  (accData[i*6] | (accData[i*6+1]<<8))>>6
                    yData[i] =  (accData[i*6+2] | (accData[i*6+3]<<8))>>6
                    zData[i] =  (accData[i*6+4] | (accData[i*6+5]<<8))>>6
            
            print(dataArray)
            print(accData)
            print("X data:")
            print(xData)
            print("Y data:")
            print(yData)
            print("Z data:")   
            print(zData)
            #print(dataArray)
            
        except:
            self.killed()
            self.dlg3 = QMessageBox(self)
            self.dlg3.setWindowTitle("WARNING")
            self.dlg3.setText("Connection lost, reconnect the device before proceeding")
            self.dlg3.setStandardButtons(QMessageBox.Ok)
            self.dlg3.setIcon(QMessageBox.Critical)
            button=self.dlg3.exec_()
            if(button==QMessageBox.Ok):
                self.dlg3.accept()

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

class Ui_UserWindow(object):
    def setupUi(self, UserWindow):

        """!
        @brief Init MainWindow - Graphical User Interface.
        """
        #Define worker
        self.serial_worker = SerialWorker(None)

        #create thread handler
        self.threadpool = QThreadPool()
 
        self.connected = CONN_STATUS
        #self.serialscan()

        # Some random data
        self.x = [] 
        self.y = []

        UserWindow.setObjectName("UserWindow")
        UserWindow.resize(1440, 900)
        self.centralwidget = QtWidgets.QWidget(UserWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        self.gridLayout_3 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_3.setObjectName("gridLayout_3")
        
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")

        self.device_search = QtWidgets.QHBoxLayout()

        self.conn_btn = QtWidgets.QPushButton(self.centralwidget, clicked = lambda: self.on_toggle)
        self.conn_btn.setText("Device Search")
        self.conn_btn.setStyleSheet("")
        self.conn_btn.setCheckable(True)
        self.conn_btn.setObjectName("conn_btn")

        self.conn_label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.conn_label.setFont(font)
        self.conn_label.setStyleSheet("background-color: rgb(255, 0, 0);\n"
"border: 1px solid black")
        self.conn_label.setScaledContents(False)
        self.conn_label.setAlignment(QtCore.Qt.AlignCenter)
        self.conn_label.setWordWrap(False)
        self.conn_label.setOpenExternalLinks(False)
        self.conn_label.setObjectName("conn_label")
        #self.modeSelection.addWidget(self.conn_label)

        self.device_search.addWidget(self.conn_btn)
        self.device_search.addWidget(self.conn_label)
        dataGraph = QHBoxLayout()

        self.verticalLayout.addLayout(self.device_search)

        self.modeSelection = QtWidgets.QHBoxLayout()
        self.modeSelection.setObjectName("modeSelection")
        
        self.modeSelect = QtWidgets.QComboBox(self.centralwidget)
        self.modeSelect.setObjectName("modeSelect")
        self.modeSelect.addItem("")
        self.modeSelect.addItem("")
        self.modeSelect.addItem("")
        self.modeSelection.addWidget(self.modeSelect)

        self.calibrate = QtWidgets.QPushButton(self.centralwidget)
        self.calibrate.setText("Calibration")

        self.updateBtn = QtWidgets.QPushButton(self.centralwidget)
        self.updateBtn.setCheckable(True)
        self.updateBtn.setObjectName("updateBtn")
        self.updateBtn.toggled = self.dataUpdate
        self.modeSelection.addWidget(self.updateBtn)
        self.modeSelection.addWidget(self.calibrate)
        self.verticalLayout.addLayout(self.modeSelection)
        
        # horizontal layout with save button and FS selection
        self.saveStatus = QtWidgets.QHBoxLayout()
        self.FSR_Select = QtWidgets.QComboBox(self.centralwidget)
        self.FSR_Select.setEditable(False)
        self.FSR_Select.setObjectName("FSR_Select")
        self.FSR_Select.addItem("FS: ±2 g")
        self.FSR_Select.addItem("FS: ±4 g")
        self.FSR_Select.addItem("FS: ±8 g")
        self.FSR_Select.addItem("FS: ±16 g")
        self.saveStatus.addWidget(self.FSR_Select)

        self.save_btn = QtWidgets.QPushButton(self.centralwidget)
        self.save_btn.setObjectName("save_btn")
        self.saveStatus.addWidget(self.save_btn) 
        self.verticalLayout.addLayout(self.saveStatus)

        self.acc_plot = PlotWidget(self.centralwidget)
        self.acc_plot.setMinimumSize(QtCore.QSize(50, 30))
        #self.acc_plot.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.acc_plot.setLineWidth(2)
        self.acc_plot.setObjectName("acc_plot")
        self.acc_plot.showGrid(x=True, y=True)
        self.acc_plot.setBackground("w")
        self.acc_plot.setTitle("Acceleration data")
        styles = {'color':'k', 'font-size':'15px'}
        self.acc_plot.setLabel('left','Acceleartion [g]', **styles)
        self.acc_plot.setLabel('bottom','Time [s]', **styles)
        self.verticalLayout.addWidget(self.acc_plot)
        
        self.acc_label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        self.acc_label.setFont(font)
        self.acc_label.setStyleSheet("background-color: rgb(0, 255, 127);\n"
"border: 1px solid black")
        self.acc_label.setAlignment(QtCore.Qt.AlignCenter)
        self.acc_label.setObjectName("acc_label")
        self.verticalLayout.addWidget(self.acc_label)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")

        self.RR_plot = PlotWidget(self.centralwidget)
        #self.RR_plot.setStyleSheet("background-color: rgb(255, 255, 255)")
        self.RR_plot.setObjectName("RR_plot")
        self.RR_plot.showGrid(x=True, y=True)
        self.RR_plot.setBackground("w")
        self.RR_plot.setTitle("Respiratory rate")
        self.RR_plot.setLabel('left','Acceleration [g]', **styles)
        self.RR_plot.setLabel('bottom','Time [s]', **styles)
        self.gridLayout.addWidget(self.RR_plot, 0, 0, 1, 1)

        self.HR_label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        self.HR_label.setFont(font)
        self.HR_label.setStyleSheet("border: 1px solid black")
        self.HR_label.setAlignment(QtCore.Qt.AlignCenter)
        self.HR_label.setObjectName("HR_label")
        self.gridLayout.addWidget(self.HR_label, 1, 1, 1, 1)
        self.RR_label = QtWidgets.QLabel(self.centralwidget)
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setBold(True)
        font.setWeight(75)
        self.RR_label.setFont(font)
        self.RR_label.setStyleSheet("border: 1px solid black")
        self.RR_label.setAlignment(QtCore.Qt.AlignCenter)
        self.RR_label.setOpenExternalLinks(False)
        self.RR_label.setObjectName("RR_label")
        self.gridLayout.addWidget(self.RR_label, 1, 0, 1, 1)
        
        self.HR_plot = PlotWidget(self.centralwidget)
        self.HR_plot.setObjectName("HR_plot")
        self.HR_plot.showGrid(x=True, y=True)
        self.HR_plot.setBackground("w")
        self.HR_plot.setTitle("Heart rate")
        self.HR_plot.setLabel('left','Acceleration [g]', **styles)
        self.HR_plot.setLabel('bottom','Time [s]', **styles)
        self.gridLayout.addWidget(self.RR_plot, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.HR_plot, 0, 1, 1, 1)

        self.verticalLayout.addLayout(self.gridLayout)
        self.gridLayout_3.addLayout(self.verticalLayout, 0, 0, 1, 1)
        UserWindow.setCentralWidget(self.centralwidget)

        self.toolBar = QtWidgets.QToolBar(UserWindow)
        self.toolBar.setObjectName("toolBar")
    
        UserWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        UserWindow.insertToolBarBreak(self.toolBar)

        self.tool_save_data = QtWidgets.QAction(UserWindow)
        self.tool_save_data = QtWidgets.QAction(QIcon("disk_return_black.png"),"Save data")
        
        self.tool_save_status = QtWidgets.QAction(UserWindow)
        self.tool_save_status.setObjectName("actionSave_status")
        self.tool_save_status.setText("Save status")

        self.toolBar.setIconSize(QSize(16,16))
        
        self.toolBar.setMovable(False)

        self.toolBar.addAction(self.tool_save_data)
        #self.toolBar.addAction(self.tool_save_status)

        self.retranslateUi(UserWindow)
        QtCore.QMetaObject.connectSlotsByName(UserWindow)

         # Some random data
        self.hour = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.temperature1 = [30, 32, 34, 32, 33, 31, 29, 32, 35, 45]
        self.temperature2 = [16, 20, 17, 23, 30, 25, 28, 26, 22, 32]
        # Plot data: x, y values
        self.draw()

    def retranslateUi(self, UserWindow):
        _translate = QtCore.QCoreApplication.translate
        UserWindow.setWindowTitle(_translate("UserWindow", "HR_RR_Monitoring"))
        self.conn_btn.setText(_translate("UserWindow", "Device Search"))
        self.modeSelect.setItemText(0, _translate("UserWindow", "HR Only"))
        self.modeSelect.setItemText(1, _translate("UserWindow", "RR Only"))
        self.modeSelect.setItemText(2, _translate("UserWindow", "Both"))
        self.conn_label.setText(_translate("UserWindow", "NO DEVICE CONNECTED"))
        self.updateBtn.setText(_translate("UserWindow", "Start"))
        self.save_btn.setText(_translate("UserWindow", "SAVE STATUS"))
        self.acc_label.setText(_translate("UserWindow", "Accelerometer data"))
        self.HR_label.setText(_translate("UserWindow", "Instant HR value:"))
        self.RR_label.setText(_translate("UserWindow", "Instant RR value:"))
        

    def draw(self):
        """!
        @brief Draw the plots.
        """
        self.temp1line = self.plot(self.acc_plot, self.hour, self.temperature1, 'Temp 1', 'r')
        self.temp2line = self.plot(self.acc_plot, self.hour, self.temperature2, 'Temp 2', 'b')

    def plot(self, graph, x, y, curve_name, color):
        """!
        @brief Draw graph.
        """
        pen = pg.mkPen(color=color)
        line = graph.plot(x, y, name=curve_name, pen=pen)
        return line

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
                time.sleep(1)
                if(CONN_STATUS==True):
                    break
            self.conn_btn.setText("Searching device...") 
            
            #self.checkToggle = bool(True)
            
        else:
            self.updateBtn.setChecked(False)
            # kill thread
            self.serial_worker.is_killed = True
            self.serial_worker.killed()
            #self.com_list_widget.setDisabled(False) # enable the possibility to change port
            self.conn_btn.setText("Device search")
            self.updateBtn.setDisabled
            

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
            TRANSMITTING = True
            self.timer.timeout.connect(lambda: self.serial_worker.readData())
            self.timer.start()
            

        else:
            self.serial_worker.send('s')
            self.updateBtn.setText("Start")
            TRANSMITTING = False
            self.timer.stop()


#############
#  RUN APP  #
#############
if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    UserWindow = QtWidgets.QMainWindow()
    ui = Ui_UserWindow()
    ui.setupUi(UserWindow)
    UserWindow.show()
    sys.exit(app.exec_())

#if __name__ == '__main__':
#    app = QApplication(sys.argv)
#    w = Ui_UserWindow()
#    app.aboutToQuit.connect(w.ExitHandler)
#    w.show()
#    sys.exit(app.exec_())