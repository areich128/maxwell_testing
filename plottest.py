import matplotlib.pyplot as plt
import numpy as np
import cv2

import datetime as dt
import csv

from scipy.signal import butter, sosfilt

def lowpass(data, cutoff, order=5):
    sos = butter(order, cutoff, 'low', output='sos')
    return sosfilt(sos, data)

time = []
angleseries = []
with open ('data/angle.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        datetime_data = dt.datetime.strptime(row[0], "%H:%M:%S.%f")
        time.append(datetime_data)
        angleseries.append(float(row[1]))

# plt.plot(time, angleseries)
# plt.xlabel('Time')
# plt.ylabel('Angle')
# # plt.savefig('images/anglevstime.png')
# plt.show()

omegaseries = []
omegaseries_avg = np.zeros(len(angleseries))
omegaseries.append(0)
for i in range(1, len(angleseries)):
    diff = time[i] - time[i-1]
    diff_secs = diff.total_seconds()
    omegaseries.append((angleseries[i] - angleseries[i-1]) / diff_secs)
    if i < len(angleseries)-10 | i > 10:
        omegaseries_avg[i] = sum(omegaseries[i-10:i+10]) / 10
    omegaseries_lowpass = lowpass(omegaseries, 0.1, order=2)

plt.plot(time, omegaseries)
plt.plot(time, omegaseries_lowpass)
plt.plot(time, omegaseries_avg)
plt.legend(['Omega', 'Omega Lowpass', 'Omega Average'])
plt.xlabel('Time')
plt.ylabel('Angular Velocity')
plt.savefig('images/omegavstime.png')
plt.show()