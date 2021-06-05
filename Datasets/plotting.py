import csv
import matplotlib.pyplot as plt
import numpy as np
  

filename = open('stable_data1.csv', 'r')
  

file = csv.DictReader(filename)

gyr_x = []
gyr_y = []
acc_x = []
acc_y = []
kal_x = []
kal_y = []
cfil_x = []
cfil_y = [] 
 

for col in file:
    gyr_x.append(float(col['gyrX']))
    gyr_y.append(float(col['gyrY']))
    acc_x.append(float(col['accX']))
    acc_y.append(float(col['accY']))
    kal_x.append(float(col['kalX']))
    kal_y.append(float(col['kalY']))
    cfil_x.append(float(col['cfilX']))
    cfil_y.append(float(col['cfilY']))

accX = np.array(acc_x[0:200])
accY = np.array(acc_y)
gyrX = np.array(gyr_x[0:200])
kalX = np.array(kal_x)
cfilX = np.array(cfil_x)

print(np.var(accX[0:200]))
print(np.var(accY[0:200]))

# create data
y_count = []
for i in range(0, 200):
    y_count.append(i)

y = np.array(y_count)

# plot lines
plt.plot(y, accX, label = "acc_x")
plt.plot(y, gyrX, label = "gyr_x")
# plt.plot(y, kalX, label = "kal_x")
# plt.plot(y, cfilX, label = "cfil_x")
plt.grid(True)
plt.legend()
plt.show()
