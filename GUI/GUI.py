import sys
import time
import logging
import numpy as np

from PyQt5.QtWidgets import * 
from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import * 
from PyQt5.QtCore import * 

# We import library dedicated to data plot
import pyqtgraph as pg
from pyqtgraph import PlotWidget

import serial 
import serial.tools.list_ports

#Global
CONN_STATUS = False
PORT = ""
TRANSMITTING = False
STARTED = False
dataSize = 32
xData = np.zeros(dataSize)
yData = np.zeros(dataSize)
zData = np.zeros(dataSize)
dataBuffer = np.zeros(96+3)#96 data + 3 separator values

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
        # init port, params and signals
        self.port = serial.Serial()
        self.port_name = serial_port_name
        self.baudrate = 9600 # hard coded but can be a global variable, or an input param
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
                        time.sleep(0.5) #just for compatibility reasons    
                        
                    
            except serial.SerialException:
                logging.info("Error with port {}.".format(self.port_name))
                self.signals.status.emit(self.port_name, 0)
                time.sleep(0.01)

        if CONN_STATUS:
            print("Prova, entro in conn status")
            if TRANSMITTING:
                print("prova, entro in transmitting")
                self.readData()

    def readData(self):
        global STARTED
        global xData, yData, zData
        count = 0
        
        if(self.read()=="Data ready\n"):
            STARTED = True
        
        if STARTED:
            dataBuffer[count]=self.read()
            count += count
            if(count==99):
                STARTED = False
                for i in len(dataBuffer):
                    print(dataBuffer[i])

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
                logging.info( self.port.in_waiting )
            logging.info("Received: {}".format(testString))
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
            self.port.close()
            time.sleep(0.01)
            CONN_STATUS = False
            self.signals.device_port.emit(self.port_name)


        logging.info("Killing the process")

class UpdateGraphicSignals(QObject):
    plot_values = pyqtSignal(int,int)
    data_ready_signal = pyqtSignal(object)


###############
# MAIN WINDOW #
###############
class MainWindow(QMainWindow):
    global TRANSMITTING
    TRANSMITTING = False

    def __init__(self):
        """!
        @brief Init MainWindow.
        """

        #Define worker
        self.serial_worker = SerialWorker(None)

        super(MainWindow, self).__init__()

        # title and geometry
        self.setWindowTitle("GUI")
        width = 1280
        height = 720
        self.setFixedSize(width, height)

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
        # Define buttons
        self.clear_btn = QPushButton(
            text="Clear",
            clicked=self.graphWidget.clear # .clear() is a method of the PlotWidget class
        )
        self.draw_btn = QPushButton(
            text="Draw",
            clicked=self.draw
        )

        self.conn_btn = QPushButton(
            #text=("Connect to port {}".format(self.port_text)), 
            text=("Device search"), 
            checkable=True,
            toggled=self.on_toggle)

        self.updateBtn = QPushButton(
            text = "Start", checkable= True, toggled = self.dataUpdate
        )

        self.modeSelect = QComboBox()
        self.modeSelect.setEditable(False)
        self.modeSelect.addItems(["HR only", "RR only","Both"])


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
        serialButton = QHBoxLayout()
        #serialButtons.addWidget(self.com_list_widget)
        serialButton.addWidget(self.conn_btn)
        modeSelection = QHBoxLayout()
        modeSelection.addWidget(self.modeSelect)
        modeSelection.addWidget(self.updateBtn)
        dataGraph = QHBoxLayout()
        dataGraph.addWidget(self.graphWidget)
        RRHRgraphs = QHBoxLayout()
        RRHRgraphs.addWidget(self.graphWidget)
        RRHRgraphs.addWidget(self.graphWidget)
        vlay = QVBoxLayout()
        vlay.addLayout(serialButton)
        vlay.addLayout(modeSelection)
        vlay.addLayout(dataGraph)
        vlay.addLayout(RRHRgraphs)
        widget = QWidget()
        widget.setLayout(vlay)
        self.setCentralWidget(widget)

        modeSelection.setContentsMargins(20,20,20,20)
        modeSelection.setSpacing(20)

        # Some random data
        self.hour = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        self.temperature1 = [30, 32, 34, 32, 33, 31, 29, 32, 35, 45]
        self.temperature2 = [16, 20, 17, 23, 30, 25, 28, 26, 22, 32]

        # Plot settings
            # Add grid
        self.graphWidget.showGrid(x=True, y=True)
            # Set background color
        self.graphWidget.setBackground('w')
            # Add title
        self.graphWidget.setTitle("Temperature measurement")
            # Add axis labels
        styles = {'color':'k', 'font-size':'15px'}
        self.graphWidget.setLabel('left', 'Temperature [°C]', **styles)
        self.graphWidget.setLabel('bottom', 'Time [h]', **styles)
            # Add legend
        self.graphWidget.addLegend()

        # Plot data: x, y values
        self.draw()
        

    def draw(self):
        """!
        @brief Draw the plots.
        """
        self.temp1line = self.plot(self.graphWidget, self.hour, self.temperature1, 'Temp 1', 'r')
        self.temp2line = self.plot(self.graphWidget, self.hour, self.temperature2, 'Temp 2', 'b')

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

                # connect worker signals to functions
                self.serial_worker.signals.status.connect(self.check_serialport_status)
                self.serial_worker.signals.device_port.connect(self.connected_device)
                # execute the worker
                self.threadpool.start(self.serial_worker)
            self.conn_btn.setText("Searching device...") 
            
            #self.checkToggle = bool(True)
            
        else:
            self.updateBtn.setChecked(False)
            # kill thread
            self.serial_worker.is_killed = True
            self.serial_worker.killed()
            #self.com_list_widget.setDisabled(False) # enable the possibility to change port
            self.conn_btn.setText("Device search")
            

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

        else:
            self.serial_worker.send('s')
            self.updateBtn.setText("Start")
            TRANSMITTING = False


#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())