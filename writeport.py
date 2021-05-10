import serial, time
#initialization and open the port

#possible timeout values:
#    1. None: wait forever, block call
#    2. 0: non-blocking mode, return immediately
#    3. x, x is bigger than 0, float allowed, timeout block call

INCOMINGDATAPORT = "/dev/pts/3"
DATARATE = 9600

ser = serial.Serial()
ser.port = INCOMINGDATAPORT             # COM Port to write to

ser.baudrate = DATARATE                 # Bits transfered per second
ser.bytesize = serial.EIGHTBITS         # Number of bits per bytes
ser.parity = serial.PARITY_NONE         # Set parity check: no parity

ser.stopbits = serial.STOPBITS_ONE      # Number of stop bits
ser.timeout = 1                         # Time to read between 2 values

ser.xonxoff = False                     #disable software flow control
ser.rtscts = False                      #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False                      #disable hardware (DSR/DTR) flow control
#ser.writeTimeout = 2                   #timeout for write

try: 
    ser.open()
except Exception as e:
    print("error open serial port: " + str(e))
    exit()

input()



while(ser.isOpen()):

    try:
        ser.flushInput() #flush input buffer, discarding all its contents
        ser.flushOutput()#flush output buffer, aborting current output 
                        #and discard all that is in buffer

        #write data
        print("Enter the string to the write to the port: ")
        b = bytes(input(), 'ascii')

        ser.write(b)
        
        time.sleep(0.5)  #give the serial port sometime to receive the data


    except Exception as e1:
        ser.close()
        print ("error communicating...: " + str(e1))


ser.close()