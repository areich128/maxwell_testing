import matplotlib.pyplot as plt
import numpy as np
import cv2

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
with open (f'CSS_4-4-25/test_data{testnum}.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        datetime_data = dt.datetime.strptime(row[0], "%H:%M:%S.%f")
        time.append(datetime_data)
        angleseries.append(float(row[1]))

angleseries = np.array(angleseries)
angleseries_smooth = np.array(angleseries)
# offset = 0
# for i in range(1, len(angleseries)):
#     if angleseries_smooth[i-1] - angleseries_smooth[i] > 300:
#         offset += 360
#         print("INCREASING OFFSET")
#     elif angleseries_smooth[i-1] - angleseries_smooth[i] < -300:
#         offset += -360
#         print("DECREASING OFFSET")
#     angleseries_smooth[i:] = angleseries[i:] + offset

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
    omegaseries_lowpass = lowpass(omegaseries, 0.1, order=5)

# plt.plot(time, omegaseries)
# plt.plot(time, omegaseries_lowpass)
# plt.plot(time, omegaseries_avg)
angleseries = angleseries + 180
plt.plot(time, angleseries)
# plt.plot(time, angleseries_smooth)
plt.plot(time, np.zeros(len(time)))
# plt.legend(['Omega Lowpass','Angle Smooth'])
plt.xlabel('Time')
plt.ylabel('Angle')
plt.savefig(f'CSSimages/anglevstime{testnum}.png')
plt.show()