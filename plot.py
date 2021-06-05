import matplotlib.pyplot as plt
import csv

import matplotlib.pyplot as plt
import csv
  
x1 = []
y1= []
with open('stable_data.csv','r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines:
        x.append(row[0])
        y.append(int(row[1]))

plt.plot(x, y, color = 'g', linestyle = 'dashed',
         marker = 'o',label = "Stable Data")
  
plt.xticks(rotation = 25)
plt.xlabel('Dates')
plt.ylabel('Temperature(°C)')
plt.title('Weather Report', fontsize = 20)
plt.grid()
plt.legend()
plt.show()


from pandas import *
  
# reading CSV file
data = read_csv("company_sales_data.csv")
  
# converting column data to list
month = data['month_number'].tolist()
fc = data['facecream'].tolist()
fw = data['facewash'].tolist()
tp = data['toothpaste'].tolist()
sh = data['shampoo'].tolist()
  
# printing list data
print('Facecream:', fc)
print('Facewash:', fw)
print('Toothpaste:', tp)
print('Shampoo:', sh)



import matplotlib.pyplot as plt
  
# create data
x = [10,20,30,40,50]
y = [30,30,30,30,30]
  
# plot lines
plt.plot(x, y, label = "line 1")
plt.plot(y, x, label = "line 2")
plt.legend()
plt.show()