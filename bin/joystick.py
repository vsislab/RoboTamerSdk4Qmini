import pygame
import struct
import time
import json

class JoyStick:
    #按键定义
    LaxiX = 0.0    #左摇杆X轴，axis[0]
    LaxiY = 0.0    #左摇杆Y轴，axis[1]
    RaxiX = 0.0    #右摇杆X轴，axis[2]
    RaxiY = 0.0    #右摇杆Y轴，axis[3]
    hatX = 0       #方向键X轴，hat[0]
    hatY = 0       #方向键Y轴，hat[1]
    butA = 0       #A键，but[0]
    butB = 0       #B键，but[1]
    butX = 0       #X键，but[3]
    butY = 0       #Y键，but[4]
    L1 = 0         #L1键，but[6]
    R1 = 0         #R1键，but[7]
    L2 = 0         #L2键，but[8]
    R2 = 0         #R2键，but[9]
    SELECT = 0     #SELECT键，but[10]
    START = 0      #START键，but[11]
    
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
    def initjoystick(self):
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

    def getjoystickstates(self):
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        for event in pygame.event.get():  # User did something
            if event.type == pygame.JOYAXISMOTION:
                self.LaxiX = self.joystick.get_axis(0)
                self.LaxiY = self.joystick.get_axis(1)
                self.RaxiX = self.joystick.get_axis(2)
                self.RaxiY = self.joystick.get_axis(3)

            if event.type == pygame.JOYHATMOTION:
                hat = self.joystick.get_hat(0)
                self.hatX = hat[0]
                self.hatY = hat[1]

            if event.type == pygame.JOYBUTTONDOWN:
                self.butA = self.joystick.get_button(0)
                self.butB = self.joystick.get_button(1)
                self.butX = self.joystick.get_button(3)
                self.butY = self.joystick.get_button(4)
                self.L1 = self.joystick.get_button(6)
                self.R1 = self.joystick.get_button(7)
                self.L2 = self.joystick.get_button(8)
                self.R2 = self.joystick.get_button(9)
                self.SELECT = self.joystick.get_button(10)
                self.START = self.joystick.get_button(11)

            if event.type == pygame.JOYBUTTONUP:
                self.butA = self.joystick.get_button(0)
                self.butB = self.joystick.get_button(1)
                self.butX = self.joystick.get_button(3)
                self.butY = self.joystick.get_button(4)
                self.L1 = self.joystick.get_button(6)
                self.R1 = self.joystick.get_button(7)
                self.L2 = self.joystick.get_button(8)
                self.R2 = self.joystick.get_button(9)
                self.SELECT = self.joystick.get_button(10)
                self.START = self.joystick.get_button(11)
        
    def display(self):
        print('================')
        print('Axies:')
        print('LaxiX: {}'.format(self.LaxiX))
        print('LaxiY: {}'.format(self.LaxiY))
        print('RaxiX: {}'.format(self.RaxiX))
        print('RaxiY: {}'.format(self.RaxiY))
        print('----------------')
        print('Hat:')
        print('hatX: {}'.format(self.hatX))
        print('hatY: {}'.format(self.hatY))
        print('----------------')
        print('button:')
        print('butA: {}'.format(self.butA))
        print('butB: {}'.format(self.butB))
        print('butX: {}'.format(self.butX))
        print('butY: {}'.format(self.butY))
        print('L1: {}'.format(self.L1))
        print('R1: {}'.format(self.R1))
        print('L2: {}'.format(self.L2))
        print('R2: {}'.format(self.R2))
        print('SELECT: {}'.format(self.SELECT))
        print('START: {}'.format(self.START))
        print('================')
                
joy = JoyStick()

def init_joystick():
    global joy
    joy.initjoystick()

def read_joystick():
    global joy
    joy.getjoystickstates()
    
    result = {
        "LaxiX": 0.,
        "LaxiY": 0.,
        "RaxiX": 0.,
        "RaxiY": 0.,
        "hatX": 0,
        "hatY": 0,
        "butA": 0,
        "butB": 0,
        "butX": 0,
        "butY": 0,
        "L1": 0,
        "R1": 0,
        "L2": 0,
        "R2": 0,
        "SELECT": 0,
        "START": 0,
    }

    result["LaxiX"] = joy.LaxiX
    result["LaxiY"] = joy.LaxiY
    result["RaxiX"] = joy.RaxiX
    result["RaxiY"] = joy.RaxiY
    
    result["hatX"] = joy.hatX
    result["hatY"] = joy.hatY
    
    result["butA"] = joy.butA
    result["butB"] = joy.butB
    result["butX"] = joy.butX
    result["butY"] = joy.butY
    
    result["L1"] = joy.L1
    result["R1"] = joy.R1
    result["L2"] = joy.L2
    result["R2"] = joy.R2
    
    result["SELECT"] = joy.SELECT
    result["START"] = joy.START
    
    return json.dumps(result)