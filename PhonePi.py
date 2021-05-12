import time
from flask import Flask
from flask_sockets import Sockets
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socket



import struct
import serial
#initialization and open the port

#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

INCOMINGDATAPORT = "COM1"
DATARATE = 115200

ser = serial.Serial()
ser.port = INCOMINGDATAPORT             # COM Port to write to

ser.baudrate = DATARATE                 # Bits transfered per second
ser.bytesize = serial.EIGHTBITS         # Number of bits per bytes
ser.parity = serial.PARITY_NONE         # Set parity check: no parity

ser.stopbits = serial.STOPBITS_ONE      # Number of stop bits
ser.timeout = 0.1                         # Time to read between 2 values

ser.xonxoff = False                     #disable software flow control
ser.rtscts = False                      #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False                      #disable hardware (DSR/DTR) flow control
ser.writeTimeout = 0.1                    #timeout for write


try: 
    ser.open()
except Exception as e:
    print("error open serial port: " + str(e))
    exit()

def get_ip():



    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

hostname = socket.gethostname()
IPAddr = get_ip()
print("Your Computer Name is: " + hostname)
print("Your Computer IP Address is: " + IPAddr)
print("Enter {}:5000 in the app and select the sensors to stream.".format(IPAddr))

app = Flask(__name__)
sockets = Sockets(app)

hasA = False
hasG = False

gx = None
gy = None
gz = None
ax = None
ay = None
az = None

def writeToPort(x, y, z, i):
    global hasA, hasG, gx, gy, gz, ax, ay, az
    try:
        #ser.flushInput() 
        #ser.flushOutput()
        
        #write data
        if(i == 'a'):
            ax = bytearray(struct.pack("f", x))
            ay = bytearray(struct.pack("f", y))
            az = bytearray(struct.pack("f", z))
            hasA = True
        elif (i == 'g'):
            gx = bytearray(struct.pack("f", x))
            gy = bytearray(struct.pack("f", y))
            gz = bytearray(struct.pack("f", z))
            hasG = True

        if(hasA and hasG):
            print("sent A: ", ax, " ",  ay, " ", az)
            print("sent G: ", gx, " ",  gy, " ", gz)
    
            ser.write(gx)
            ser.write(gy)
            ser.write(gz)

            ser.write(ax)
            ser.write(ay)
            ser.write(az)

            hasA = False
            hasG = False

        # time.sleep(0.5)  #give the serial port sometime to receive the data

    except Exception as e1:
        ser.close()
        print ("error communicating...: " + str(e1))


@sockets.route('/gyroscope')
def echo_socket(ws):
    # f = open("accelerometer.txt", "a")
    while ser.isOpen():
        t = time.time()

        message = ws.receive()
        values = message.split(",")
        x = float(values[0])
        y = float(values[1]) 
        z = float(values[2]) 

        writeToPort(x, y, z, 'g')

        #while(time.time() - t < 0.5):
        #    ws.receive()
        
        ws.send(message)
        # print(message, file=f)
        # time.sleep(0.8)


        

    #f.close()


@sockets.route('/accelerometer')
def echo_socket(ws):
    # f = open("accelerometer.txt", "a")
    while ser.isOpen():
        t = time.time()

        message = ws.receive()
        values = message.split(",")
        x = float(values[0])
        y = float(values[1]) 
        z = float(values[2]) 

        writeToPort(x, y, z, 'a')

        #while(time.time() - t < 0.5):
        #    ws.receive()
        
        ws.send(message)
        # print(message, file=f)
        # time.sleep(0.8)

# @sockets.route('/gyroscope')
# def echo_socket(ws):
#     f = open("gyroscope.txt", "a")
#     while True:
#         message = ws.receive()
# 
#         time.sleep(2)
# 
#         print(message)
#         ws.send(message)
#         print(message, file=f)
#     f.close()



@app.route('/')
def hello():
    return 'Hello World!'


server = pywsgi.WSGIServer(('0.0.0.0', 5000), app, handler_class=WebSocketHandler)
server.serve_forever()