import matplotlib.pyplot as plt
import numpy as np
import cv2

import datetime as dt
import csv

time = []
angleseries = []
with open ('data/angle.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        datetime_data = dt.datetime.strptime(row[0], "%H:%M:%S.%f")
        time.append(datetime_data)
        angleseries.append(float(row[1]))

plt.plot(time, angleseries)
plt.xlabel('Time')
plt.ylabel('Angle')
plt.savefig('images/anglevstime.png')
plt.show()

omegaseries = []
omegaseries.append(0)
for i in range(1, len(angleseries)):
    diff = time[i] - time[i-1]
    diff_secs = diff.total_seconds()
    omegaseries.append((angleseries[i] - angleseries[i-1]) / diff_secs)

plt.plot(time, omegaseries)
plt.xlabel('Time')
plt.ylabel('Angular Velocity')
plt.savefig('images/omegavstime.png')
plt.show()