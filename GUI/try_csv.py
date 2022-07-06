import numpy
import pandas
import matplotlib.pyplot as plt
import scipy.signal as signal
from scipy.signal import butter, find_peaks


def butter_lowpass( cutoff, fs, order):#=5):
        return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order):#=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    #y = lfilter(b, a, data)
    y = signal.filtfilt(b, a, data, padlen=len(data)-1) #uso filtfilt anzichè lfilter per rimanere in fase
    return y

def butter_bandpass_design(signal_array, low_cut, high_cut, sample_rate, order=4):
        """
        Defines the Butterworth bandpass filter-design
        :param low_cut: Lower cut off frequency in Hz
        :param high_cut: Higher cut off frequency in Hz
        :param sample_rate: Sample rate of the signal in Hz
        :param order: Order of the filter-design
        :return: b, a : ndarray, ndarray - Numerator (b) and denominator (a) polynomials of the IIR filter. Only returned if output='ba'.
        """
        #nyq = 0.5 * sample_rate
        #low = low_cut / nyq
        #high = high_cut / nyq
        sos = signal.butter(order, [low_cut, high_cut], btype='band', output ='sos',fs=sample_rate)
        y = signal.sosfiltfilt(sos, signal_array)
        return y

SAMPLE_RATE = 50
LOW_CUT = 0.01
HIGH_CUT = 1.0

order = 8
cutoff = 3

z_axis = []
df = pandas.read_csv('HR_Rate.csv')
#print(df)
#print(type(df['z axis']))
df1 = df['z axis']
print(df1)
z_axis = df1.to_numpy()
z_axis_try = z_axis[:(len(z_axis)//2)]
zData_lowpass = butter_lowpass_filter(z_axis_try, cutoff, SAMPLE_RATE, order)
zData_lowpass2 = butter_lowpass_filter(z_axis[len(z_axis)//2:], cutoff, SAMPLE_RATE, order)
zData_bandpass = butter_bandpass_design(zData_lowpass, LOW_CUT, HIGH_CUT, SAMPLE_RATE)
zData_bandpass2 = butter_bandpass_design(zData_lowpass, LOW_CUT, HIGH_CUT, SAMPLE_RATE)
sumData = 0
th_arr = []
max = -10
for i in range(len(zData_bandpass)):
    if (zData_bandpass[i] > max):
        max = zData_bandpass[i]
        sumData += zData_bandpass

threshold = 0.5*max
delta = 50
i_peaks, _ = find_peaks(zData_bandpass, height = threshold, distance = delta)

# returns the indeces of the peaks --> use them to find the respiration peaks
peaks = len(i_peaks)
print('num picchi: ', peaks)
plt.plot(z_axis, "-b")
plt.plot(numpy.append(zData_lowpass, zData_lowpass2), "-r")
plt.plot(numpy.append(zData_bandpass, zData_lowpass2), "-y")
plt.plot(i_peaks,zData_bandpass[i_peaks], "x")
x = numpy.arange(len(zData_bandpass))

plt.plot( threshold, "--k")
plt.show()                                                  


