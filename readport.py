import serial

INCOMINGDATAPORT = "/dev/pts/2"
DATARATE = 9600

ser = serial.Serial(port = INCOMINGDATAPORT, baudrate = DATARATE, timeout = 1)

ser.xonxoff = False     #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False       #disable hardware (DSR/DTR) flow control

input()


while(ser.isOpen()):
    try:
        line = ser.readline().strip()
        values = line.decode('ascii').split(", ")
        if(values != ['']):
            print("x:" + str(values[0]))
            print("y:" + str(values[1]))
            print("z:" + str(values[2]))





    except Exception as e1:
        ser.close()
        print ("error communicating...: " + str(e1)) 


ser.close()    

