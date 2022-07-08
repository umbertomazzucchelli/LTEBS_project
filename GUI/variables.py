import numpy as np

# Global variables
CONN_STATUS = False
PORT = ""
dataSize = 98
baudRate = 115200 #9600 for USB, 115200 for BT
axisSize = dataSize//3
SECONDI = 16
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

zData_windowed_HR = np.full(axisSize,0,dtype=np.float16)
zData_lowpass_RR = np.full(axisSize,0,dtype=np.float16)
zData_smoothed_RR = np.full(axisSize,0,dtype=np.float16)
zData_lowpass = np.full(axisSize,0,dtype=np.float16)

zData_array_HR = []
zData_array_RR = []
HR_array = []
RR_array= []

flag_graph = 0
flag_graph_RR = 0
flag_graph_HR = 0
count_sec_HR = 0
count_sec_RR = 0
connectionWait = False
calibration_flag=False
RR_value=0.0
HR_value=0.0
RR_old=0.0
HR_old=0.0
count_RR = 0
count_HR = 0
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