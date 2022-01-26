
import sys
from croblink import *
import math
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14
prev_out=0
cur_out=0
prev_x=0
prev_y=0
cur_x=0
cur_y=0
cur_teta=0
prev_teta=0

currentCompass = None
GPSX = None
GPSY = None
class MyRob(CRobLinkAngs):


    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        global state, ang, currentDirection
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'walk'
        ang = 0
        currentDirection = ''

        while True:
            self.readSensors()
            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()
            if currentCompass == None:
                self.saveCurrentCompass()
            if GPSX == None and GPSY == None:
                self.saveCurrentGPS()
            if state == 'walk' and self.measures.start:
                self.moveOne()
            if state == 'rot_left':
                self.rot_left(ang, currentDirection)
            if state == 'rot_right':
                self.rot_right(ang, currentDirection)
            if state == 'chooseNext':
                self.chooseNext()
            if state == 'stop':
                quit()


    def chooseNext(self):
        global state, ang, currentDirection, cur_x, cur_y
        print(GPSX,GPSY )
        if currentDirection == 'N':
            if self.measures.irSensor[0] < 1.5:
                state = 'walk'
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] < 1.5:
                state = 'rot_left'
                ang = 90
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[2] < 1.5:
                state = 'rot_right'
                ang = 90
            else:
                state = 'rot_right'
                ang = 180
        if currentDirection == 'S':
            if self.measures.irSensor[0] < 1.5:
                state = 'walk'
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] < 1.5:
                state = 'rot_left'
                ang = 90
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[2] < 1.5:
                state = 'rot_right'
                ang = 90
            else:
                state = 'rot_right'
                ang = 180
        if currentDirection == 'O':
            if self.measures.irSensor[0] < 1.5:
                state = 'walk'
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] < 1.5:
                state = 'rot_left'
                ang = 90
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[2] < 1.5:
                state = 'rot_right'
                ang = 90
            else:
                state = 'rot_right'
                ang = 180
        if currentDirection == 'E':
            if self.measures.irSensor[0] < 1.5:
                state = 'walk'
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] < 1.5:
                state = 'rot_left'
                ang = 90
            elif self.measures.irSensor[0] >= 1.5 and self.measures.irSensor[1] >= 1.5 and self.measures.irSensor[2] < 1.5:
                state = 'rot_right'
                ang = 90
            else:
                state = 'rot_right'
                ang = 180

    def moveOne(self):
        global cur_x, cur_y, cur_out, prev_out, prev_x, prev_y, prev_teta, state
        if self.measures.irSensor[0] >= 2.4:
            aux_x = int(abs(cur_x)) if ((int(abs(cur_x)) % 2) == 0) else int(abs(cur_x)) +1
            aux_y = int(abs(cur_y)) if ((int(abs(cur_y)) % 2) == 0) else int(abs(cur_y)) +1
            if cur_x < 0:
                cur_x = 0 - aux_x
            else:
                cur_x = aux_x
            if cur_y < 0:
                cur_y = 0 - aux_y
            else:
                cur_y = aux_y
            prev_x = cur_x
            prev_y = cur_y
            self.saveCurrentGPS()
            self.driveMotors(0.0,0.0)
            prev_out = 0
            state = 'chooseNext'
        elif (abs(math.hypot(GPSX-cur_x, GPSY-cur_y)) < 1.5) and self.measures.irSensor[0] < 2.4:
            if self.measures.irSensor[1] >= 3:
                cur_out = (0.1 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.1,0.0975)
            elif self.measures.irSensor[2] >= 3:
                cur_out = (0.1 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.0975,0.1)
            else:
                if self.measures.compass in range(-5, 6, 1):
                    if self.measures.compass < 0:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.095,0.1)
                    else:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.1,0.095)
                elif self.measures.compass in range(86, 96, 1):
                    if self.measures.compass < 90:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.095,0.1)
                    else:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.1,0.095)
                elif self.measures.compass in range(-95, -84, 1):
                    if self.measures.compass < -90:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.095,0.1)
                    else:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.1,0.095)
                elif self.measures.compass in range(-180, -169, 1) or self.measures.compass in range(170, 181, 1):
                    if self.measures.compass < 0 and self.measures.compass > -180:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.1,0.095)
                    elif self.measures.compass > 0 and self.measures.compass < 180:
                        cur_out = (0.1 + prev_out)/2
                        prev_out = cur_out
                        lin = (cur_out + cur_out)/2
                        prev_teta = cur_teta
                        prev_teta = math.radians(prev_teta)
                        cur_x = prev_x + lin * math.cos(prev_teta)
                        cur_y = prev_y + lin * math.sin(prev_teta)
                        prev_x = cur_x
                        prev_y = cur_y
                        self.driveMotors(0.095,0.1)
            #print(cur_x, cur_y)
        elif (abs(math.hypot(GPSX-cur_x, GPSY-cur_y)) < 1.7) and self.measures.irSensor[0] < 2.4:
            if self.measures.irSensor[1] >= 3:
                cur_out = (0.05 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.05,0.0475)
            elif self.measures.irSensor[2] >= 3:
                cur_out = (0.05 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.0475,0.05)
            else:
                cur_out = (0.05 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.05,0.05)
        elif (abs(math.hypot(GPSX-cur_x, GPSY-cur_y)) < 1.8) and self.measures.irSensor[0] < 2.4:
            if self.measures.irSensor[1] >= 3:
                cur_out = (0.03 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.03,0.0275)
            elif self.measures.irSensor[2] >= 3:
                cur_out = (0.03 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.0275,0.03)
            else:
                cur_out = (0.03 + prev_out)/2
                prev_out = cur_out
                lin = (cur_out + cur_out)/2
                prev_teta = cur_teta
                prev_teta = math.radians(prev_teta)
                cur_x = prev_x + lin * math.cos(prev_teta)
                cur_y = prev_y + lin * math.sin(prev_teta)
                prev_x = cur_x
                prev_y = cur_y
                self.driveMotors(0.03,0.03)
        else:
            self.saveCurrentGPS()
            self.driveMotors(0.0,0.0)
            prev_out = 0
            state = 'chooseNext'


    def rot_left(self, delta_ang, cardinal):
        global state, currentDirection, cur_teta
        if cardinal == "N":
            if self.measures.compass >= 89 and self.measures.compass <= 91:
                self.saveCurrentGPS()
                self.driveMotors(0.0, 0.0)
                self.saveCurrentCompass()
                cur_teta = 90
                state = "walk"
            elif (75 < self.measures.compass < 89):
                self.driveMotors(-0.005, 0.005)
                state = "rot_left"
            elif (50 < self.measures.compass <= 75):
                self.driveMotors(-0.05, 0.05)
                state = "rot_left"
            else:
                self.driveMotors(-0.1, 0.1)
                state = "rot_left"
        if cardinal == "S":
            if self.measures.compass >= -91 and self.measures.compass <= -89:
                self.driveMotors(0.0, 0.0)
                self.saveCurrentCompass()
                cur_teta = -90
                state = "walk"
            elif (-105 < self.measures.compass < -91):
                self.driveMotors(-0.005, 0.005)
                state = "rot_left"
            elif (-125 < self.measures.compass <= -105):
                self.driveMotors(-0.05, 0.05)
                state = "rot_left"
            else:
                self.driveMotors(-0.1, 0.1)
                state = "rot_left"
        if cardinal == "O":
            if self.measures.compass >= 179 or self.measures.compass <= -179:
                self.driveMotors(0.0, 0.0)
                self.saveCurrentCompass()
                cur_teta = 180
                state = "walk"
            elif (165 < self.measures.compass < 179):
                self.driveMotors(-0.005, 0.005)
                state = "rot_left"
            elif (135 < self.measures.compass <= 165):
                self.driveMotors(-0.05, 0.05)
                state = "rot_left"
            else:
                self.driveMotors(-0.1, 0.1)
                state = "rot_left"
        if cardinal == "E":
            if self.measures.compass >= -1 and self.measures.compass <= 1:
                self.driveMotors(0.0, 0.0)
                cur_teta = 0
                self.saveCurrentCompass()
                state = "walk"
            elif (-15 < self.measures.compass < -1):
                self.driveMotors(-0.005, 0.005)
                state = "rot_left"
            elif (-45 < self.measures.compass <= -15):
                self.driveMotors(-0.05, 0.05)
                state = "rot_left"
            else:
                self.driveMotors(-0.1, 0.1)
                state = "rot_left"


    def rot_right(self, delta_ang, cardinal): # def
        global state, currentDirection, cur_teta
        if delta_ang == 180:
            if cardinal == "N":
                if self.measures.compass >= 179 or self.measures.compass <= -179:
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = -180
                    state = "walk"
                elif (-179 < self.measures.compass <= -165):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (-165 < self.measures.compass < -110):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
            if cardinal == "S":
                if self.measures.compass >= -1 and self.measures.compass <= 1:
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = 0
                    state = "walk"
                elif (1 < self.measures.compass <= 15):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (15 < self.measures.compass < 65):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
            if cardinal == "O":
                if self.measures.compass >= -91 and self.measures.compass <= -89 :
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = -90
                    state = "walk"
                elif (-89 < self.measures.compass <= -65):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (-65 < self.measures.compass < -15):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
            if cardinal == "E":
                if self.measures.compass >= 89 and self.measures.compass <= 91:
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = 90
                    state = "walk"
                elif (91 < self.measures.compass <= 115):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (115 < self.measures.compass < 165):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
        else:
            if cardinal == "N":
                if self.measures.compass >= -91 and self.measures.compass <= -89 :
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = -90
                    state = "walk"
                elif (-89 < self.measures.compass <= -65):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (-65 < self.measures.compass < -35):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
            if cardinal == "S":
                if self.measures.compass >= 89 and self.measures.compass <= 91:
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = 90
                    state = "walk"
                elif (91 < self.measures.compass <= 115):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (115 < self.measures.compass < 145):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
            if cardinal == "O":
                if self.measures.compass >= -1 and self.measures.compass <= 1:
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = 0
                    state = "walk"
                elif (1 < self.measures.compass <= 15):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (15 < self.measures.compass < 45):
                    self.driveMotors(0.05, -0.05)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"
            if cardinal == "E":
                if self.measures.compass >= 179 or self.measures.compass <= -179:
                    self.driveMotors(0.0, 0.0)
                    self.saveCurrentCompass()
                    cur_teta = -180
                    state = "walk"
                elif (-179 < self.measures.compass <= -165):
                    self.driveMotors(0.005, -0.005)
                    state = "rot_right"
                elif (-165 < self.measures.compass < -130):
                    self.driveMotors(0.015, -0.015)
                    state = "rot_right"
                else:
                    self.driveMotors(0.1, -0.1)
                    state = "rot_right"


    def saveCurrentCompass(self):
        global currentCompass, currentDirection
        currentCompass = self.measures.compass
        if self.measures.compass < 45 and self.measures.compass > -45:
            currentDirection = 'N'
        elif self.measures.compass > 45 and self.measures.compass < 135:
            currentDirection = 'O'
        elif self.measures.compass < -45 and self.measures.compass > -135:
            currentDirection = 'E'
        else:
            currentDirection = 'S'

    def saveCurrentGPS(self):
        global GPSX, GPSY
        GPSX = cur_x
        GPSY = cur_y


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None

           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
