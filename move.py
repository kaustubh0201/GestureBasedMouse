import pyautogui as pg
import math 
import time

import numpy as n

# case x1 > x2, y1 > y2 ✔️✔️✔️ (1008, 567) -> (960, 540)
# case x1 > x2, y1 < y2 ✔️✔️✔️ (1008, 513) -> (960, 540)
# case x1 < x2, y1 > y2 ✔️✔️✔️(912, 567) -> (960, 540)
# case x1 < x2, y1 < y2 ✔️✔️✔️ (960, 540) -> (1008, 567)

# with open("./test_inputs") as file:
#     line = file.readline()

#correctly-implemented
def midPoint(ix0,iy0, ix1,iy1):
    path = []
    isReverse = False


    if(ix0 > ix1 and iy0 > iy1):
        ix0, ix1 = ix1, ix0
        iy0, iy1 = iy1, iy0
        isReverse = True
        
    elif(ix0 < ix1 and iy0 > iy1):
        ix0, ix1 = ix1, ix0
        iy0, iy1 = iy1, iy0
        isReverse = True

    xIncr = 1
    compare = lambda x0, x1: x0 < x1
    if(ix0 > ix1 and iy0 < iy1):
        xIncr = -1
        compare = lambda x0, x1: x0 > x1

    # calculate dx & dy
    dx = abs(ix1 - ix0)
    dy = abs(iy1 - iy0)
 
    # initial value of decision parameter d
    d = dy - (dx/2)
    x = ix0
    y = iy0
 
    # Plot initial given point
    # putpixel(x,y) can be used to print pixel
    # of line in graphics
    path.append((x, y))
    # print(x,",",y)
    # iterate through value of X

    while (compare(x, ix1)):
        x=x + xIncr 
        # E or East is chosen
        if(d < 0):
            d = d + dy
 
        # NE or North East is chosen
        else:
            d = d + (dy - dx)
            y=y+1
     
 
        # Plot intermediate points
        # putpixel(x,y) is used to print pixel
        # of line in graphics
        # print(x,",",y)
        
        path.append((x, y))

    if(isReverse):
        path.reverse()

    return path
        



def clamp(n, smallest, largest): 
    return max(smallest, min(n, largest))


# Frame per second
FPS = 600.
maxFrameTime = 1.0/FPS * 1000

# Dimension of the screen
WIDTH, HEIGHT = pg.size()


def normaliseToScreenCoords(x, y):

    x_new = clamp (math.floor((x+1) / 2 * WIDTH), 0, WIDTH - 1)
    y_new = clamp (HEIGHT - math.floor((y+1) / 2 * HEIGHT), 0, HEIGHT - 1)
    
    return x_new, y_new


def screenCoordsToNormalise(x, y):
    x_new = clamp(2 * x / float(WIDTH) - 1, -1, 1) 
    y_new = clamp(-2 * y / float(HEIGHT) + 1, -1, 1)

    return x_new, y_new
    
    
# pg.moveTo(x_new, y_new, 5)

# x0, y0 = normaliseToScreenCoords(-0.05, 0.05)
# x1, y1 = normaliseToScreenCoords(0, 0)

# x0, y0 = normaliseToScreenCoords(0.05, -0.05)
# x1, y1 = normaliseToScreenCoords(0, 0)
 
# x0, y0 = normaliseToScreenCoords(-0.05, -0.05)
# x1, y1 = normaliseToScreenCoords(0, 0)
 
x0, y0 = normaliseToScreenCoords(0.95, 0.95)
x1, y1 = normaliseToScreenCoords(0, 0)

print(x0)
print(y0)
print(x1)
print(y1)
# points = midPoint(x0, y0, x1, y1)

#print(points)

# 0 1 2 3

class State:
    k = 0
    path = []

    def __init__ (self, path):
        self.path = path
    
    def nextPoint(self):
        return self.path[self.k]
    
    def updateState(self):
        if(self.k + 1 < len(self.path)):
            self.k += 1

            return True
        else:
            return False

def move(s):
    x, y = s.nextPoint()   
    pg.moveTo(x, y)
    if(s.updateState()):
        return True
    else:
        return False


def cleanPath(path):
    v = 200
    totalTime = (len(path) - 2) * 0.1
    x0, y0 = path[0]
    x1, y1 = path[-1]

    dist = math.sqrt( (x0 - x1)**2 + (y0 - y1)**2 )    
    t = dist/v

    remPoints = t/0.1
    
    print(remPoints)
    print(len(path))

    incre = math.ceil((len(path) - 2)/float(remPoints))

    p = [path[0]]

    for i in range(1, len(path) - 1, incre):
        p.append(path[i])
    
    p.append(path[-1])

    print(len(p))
    return p


def calcPositionVelocity(s, v, a):
    newV = v + a * frameTime
    newP = v * frameTime + 0.5 * a * frameTime**2

    return newV, newP

#  v = 0
v = n.array([[0], [0]])

# inverse transform on cur position
cusX, cusY = pg.position()
posX, posY = screenCoordsToNormalise(cusX, cusY)
s = n.array([[0], [0]])
a = n.array([[2], [2]])


isPathComplete = True
b = False

s = None
path = []

while(True):
    # cur unix time
    prevTime = time.time() * 1000
    
    # check if there is new a then get a 
    # calculate new s, v

    # calculate path
    if(isPathComplete and not b):        
        path = cleanPath(midPoint(x0, y0, x1, y1))
        
        # print(path)
        
        s = State(path)
        isPathComplete = False

        #print('a')
        
        b = True
    
    # move
    if(not isPathComplete):
        isPathComplete = not move(s)
    else:
        print("as")
        x0, y0 = normaliseToScreenCoords(0.00, 0.0)
        x1, y1 = normaliseToScreenCoords(0.95, -0.95)
        path = cleanPath(midPoint(x0, y0, x1, y1))
        s = State(path)

        isPathComplete = False

    

    # cur unixtime
    curTime = time.time() * 1000
    frameTime = curTime - prevTime

    if(frameTime < maxFrameTime):
        time.sleep((maxFrameTime - frameTime)/1000.0)
    # time
    # sleep