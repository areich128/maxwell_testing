import matplotlib.pyplot as plt
import numpy as np
import cv2
from scipy.fft import fft, ifft, fftfreq
from scipy.interpolate import interp1d

import datetime as dt
import csv

from scipy.signal import butter, sosfilt

def kf():
    return

def lowpass(data, cutoff, order=5):
    sos = butter(order, cutoff, 'low', output='sos')
    return sosfilt(sos, data)

testnum = input("Enter test number: ")

time = []
angleseries = []
with open (f'OpenCV/CSS_6-6-25/test_data{testnum}.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        datetime_data = dt.datetime.strptime(row[0], "%H:%M:%S.%f")
        time.append(datetime_data)
        angleseries.append(float(row[1]))

angleseries = np.array(angleseries)
angleseries_smooth = np.array(angleseries)
offset = 0
for i in range(1, len(angleseries)):
    if angleseries_smooth[i-1] - angleseries_smooth[i] > 300:
        offset += 360
        print("INCREASING OFFSET")
    elif angleseries_smooth[i-1] - angleseries_smooth[i] < -300:
        offset += -360
        print("DECREASING OFFSET")
    angleseries_smooth[i:] = angleseries[i:] + offset

# angleseries = lowpass(angleseries, 0.05, order=5)

# plt.plot(time, angleseries)
# plt.xlabel('Time')
# plt.ylabel('Angle')
# plt.savefig(f'images/anglevstime{testnum}.png')
# plt.show()

omegaseries = []
omegaseries_avg = np.zeros(len(angleseries))
omegaseries.append(0)
for i in range(1, len(angleseries)):
    diff = time[i] - time[i-1]
    diff_secs = diff.total_seconds()
    # if angleseries[i] - angleseries[i-1] > 355:
    #     omegaseries.append((angleseries[i] - angleseries[i-1] - 360) / diff_secs)
    omegaseries.append((angleseries_smooth[i] - angleseries_smooth[i-1]) / diff_secs)
    if i < len(angleseries_smooth)-10 | i > 10:
        omegaseries_avg[i] = sum(omegaseries[i-10:i+10]) / 10
    omegaseries_lowpass = lowpass(omegaseries, 0.01, order=5)

time = [(t - time[0]).total_seconds() for t in time]
time = np.array(time)

# plt.plot(time, omegaseries)
plt.plot(time, omegaseries_lowpass)
# plt.plot(time, fft(omegaseries_lowpass))
# plt.plot(time, omegaseries_avg)
# angleseries = angleseries + 180
# plt.plot(time, angleseries)
# plt.plot(time, angleseries_smooth)
plt.plot(time, np.zeros(len(time)))
# plt.legend(['Omega Lowpass','Angle Smooth'])
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.title('Angular Velocity vs Time')
plt.savefig(f'anglevstime{testnum}.png')
plt.show()

t_uniform = np.linspace(time.min(), time.max(), len(time))  # new uniform time vector
interp_func = interp1d(time, omegaseries, kind='linear', fill_value="extrapolate")
x_uniform = interp_func(t_uniform)

# Number of sample points
N = len(x_uniform)
T = t_uniform[1] - t_uniform[0]  # sample spacing
yf = fft(x_uniform)
xf = fftfreq(N, T)[:N // 2]  # frequency bins

# Amplitude spectrum (normalized)
amplitude = 2.0/N * np.abs(yf[:N//2])

plt.figure()
plt.loglog(xf, amplitude)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.grid(True, which='both')
plt.title('FFT of Angular Velocity without Lowpass')
plt.show()

t_uniform = np.linspace(0, 1400, 1400)  # new uniform time vector for test signals

x_test100 = np.sin(2*np.pi*t_uniform/100)
x_test10 = np.sin(2*np.pi*t_uniform/10)
x_test700 = np.sin(2*np.pi*t_uniform/700)

N = len(x_test700)
T = t_uniform[1] - t_uniform[0]
yf700 = fft(x_test700)
xf700 = fftfreq(N, T)[:N // 2]
amplitude700 = 2.0/N * np.abs(yf700[:N//2])

N = len(x_test100)
yf100 = fft(x_test100)
xf100 = fftfreq(N, T)[:N // 2]
amplitude100 = 2.0/N * np.abs(yf100[:N//2])

N = len(x_test10)
yf10 = fft(x_test10)
xf10 = fftfreq(N, T)[:N // 2]
amplitude10 = 2.0/N * np.abs(yf10[:N//2])

plt.figure()
plt.loglog(xf700, amplitude700)
plt.loglog(xf100, amplitude100)
plt.loglog(xf10, amplitude10)
plt.legend(['700 Hz', '100 Hz', '10 Hz'])
plt.ylim(1e-6, 1e1)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Magnitude')
plt.grid(True, which='both')
plt.title(f'Test FFT over {max(t_uniform)} seconds')
plt.show()