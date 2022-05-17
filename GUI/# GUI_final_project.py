# GUI interface file
import sys

import time

import logging      #to print messages

from PyQt5 import QtCore
from PyQt5.QtCore import (
    QObject,
    QThreadPool, 
    QRunnable, 
    pyqtSignal, 
    pyqtSlot
)

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QComboBox,
    QHBoxLayout,
    QWidget,
)

import serial       #perché ci serve una connessione seriale
import serial.tools.list_ports


# Globals
CONN_STATUS = False     #to keep track of the connection status of the device


# Logging config
logging.basicConfig(format="%(message)s", level=logging.INFO)   #configura in maniera semplice il loggin module, handles different types of messages


#########################
# SERIAL_WORKER_SIGNALS #
#########################
class SerialWorkerSignals(QObject): #signals perché è un segnale, worker, e serial perchè è seriale 
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
class SerialWorker(QRunnable):  #serve Qrunnable object o lo subclassiamo chiamando SerialWorker
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

        if not CONN_STATUS:
            try:
                self.port = serial.Serial(port=self.port_name, baudrate=self.baudrate,
                                        write_timeout=0, timeout=2)                
                if self.port.is_open:
                    CONN_STATUS = True
                    self.signals.status.emit(self.port_name, 1) #connection between the gui tread e l'altra tread
                    time.sleep(0.01)     
            except serial.SerialException:
                logging.info("Error with port {}.".format(self.port_name))  #non diciamo print (error) ma logging -> si può usare anche logging.critical se l'errore è grave
                self.signals.status.emit(self.port_name, 0) #questa volta passiamo 0, non 1
                time.sleep(0.01)

    @pyqtSlot()
    def send(self, char):       #funzione per mandare i segnali, ad esempio accendere e spegnere LED
        """!
        @brief Basic function to send a single char on serial port.
        """
        try:
            self.port.write(char.encode('utf-8'))
            logging.info("Written {} on port {}.".format(char, self.port_name))
        except:
            logging.info("Could not write {} on port {}.".format(char, self.port_name))

   
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
            self.signals.device_port.emit(self.port_name)   #mi dice che è stata chiusa la porta

        logging.info("Killing the process")


###############
# MAIN WINDOW #
###############
class MainWindow(QMainWindow):
    def __init__(self):
        """!
        @brief Init MainWindow.
        """
        # define worker
        self.serial_worker = SerialWorker(None) #creiamo il serial worker, questa volta passiamo un none perché devo passare
        #una porta ma essendo un'inizializzazione ancora non so il nome

        super(MainWindow, self).__init__()  #

        # title and geometry
        self.setWindowTitle("GUI")
        width = 400
        height = 320
        self.setMinimumSize(width, height)

        # create thread handler
        self.threadpool = QThreadPool() #ci serve il thread pool

        self.connected = CONN_STATUS
        self.serialscan() #check if c'è qualcosa nella serial port
        self.initUI()


    #####################
    # GRAPHIC INTERFACE #
    #####################
    def initUI(self):
        """!
        @brief Set up the graphical interface structure.
        """
        # layout
        button_hlay = QHBoxLayout()
        button_hlay.addWidget(self.com_list_widget)
        button_hlay.addWidget(self.conn_btn)
        widget = QWidget()
        widget.setLayout(button_hlay)
        self.setCentralWidget(widget)


    ####################
    # SERIAL INTERFACE #
    ####################
    def serialscan(self):
        """!
        @brief Scans all serial ports and create a list.
        """
        # create the combo box to host port list
        self.port_text = ""
        self.com_list_widget = QComboBox()  #cambiare il testo nel bottone quando l'user seleziona la porta
        self.com_list_widget.currentTextChanged.connect(self.port_changed)
        
        # create the connection button
        self.conn_btn = QPushButton(
            text=("Connect to port {}".format(self.port_text)), 
            checkable=True,
            toggled=self.on_toggle  #con il toggle fa tipo interruttore, se premo va giù per tornare su 
            #devo ripremere, invece se mettessi .click fa da pulsante
        )

        # acquire list of serial ports and add it to the combo box
        serial_ports = [
                p.name
                for p in serial.tools.list_ports.comports()
            ]
        self.com_list_widget.addItems(serial_ports)


    ##################
    # SERIAL SIGNALS #
    ##################

    #ultima cosa: definire i metodi che accettano il segnale

    def port_changed(self):
        """!
        @brief Update conn_btn label based on selected port.
        """
        self.port_text = self.com_list_widget.currentText()
        self.conn_btn.setText("Connect to port {}".format(self.port_text))

    @pyqtSlot(bool)
    def on_toggle(self, checked):
        """!
        @brief Allow connection and disconnection from selected serial port.
        """
        if checked:         
            # setup reading worker
            self.serial_worker = SerialWorker(self.port_text) # needs to be re defined --> è la stessa che avevamo messo nel main con "none" ma ora sappiamo il nome della porta
            # connect worker signals to functions
            self.serial_worker.signals.status.connect(self.check_serialport_status) #che sia andata a buon fine o meno 
            self.serial_worker.signals.device_port.connect(self.connected_device)
            # execute the worker
            self.threadpool.start(self.serial_worker)
        else:
            # kill thread
            self.serial_worker.is_killed = True
            self.serial_worker.killed()
            self.com_list_widget.setDisabled(False) # enable the possibility to change port
            self.conn_btn.setText(
                "Connect to port {}".format(self.port_text)
            )

    def check_serialport_status(self, port_name, status):
        """!
        @brief Handle the status of the serial port connection.

        Available status:
            - 0  --> Error during opening of serial port
            - 1  --> Serial port opened correctly
        """
        if status == 0:
            self.conn_btn.setChecked(False) #se è 0 la connessione non è andata a buon fine quindi non dobbiamo cambiare nulla nell'applicazione
            #avremo ancora scritto che dobbiamo connettere la porta
        elif status == 1:
            # enable all the widgets on the interface
            self.com_list_widget.setDisabled(True) # disable the possibility to change COM port when already connected
            self.conn_btn.setText(
                "Disconnect from port {}".format(port_name)
            )

    def connected_device(self, port_name):
        """!
        @brief Checks on the termination of the serial worker.
        """
        logging.info("Port {} closed.".format(port_name))   #display che la porta chiamata port_name è chiusa


    def ExitHandler(self):  #viene chiamata quando chiudimamo l'applicazione quindi con la x in alto
        """!
        @brief Kill every possible running thread upon exiting application.
        """
        self.serial_worker.is_killed = True
        self.serial_worker.killed()




#############
#  RUN APP  #
#############
if __name__ == '__main__':
    app = QApplication(sys.argv)
    w = MainWindow()
    app.aboutToQuit.connect(w.ExitHandler)
    w.show()
    sys.exit(app.exec_())