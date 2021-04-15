import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

from threading import Thread
import serial
import collections
import struct
import sys

import vpython as vp
from BB8 import *
import PySimpleGUI as sg

import pandas as pd

g = 9.81


def filtro_sma(signal,M):
    return np.mean(signal[-M:])

class BYTE(object):
    """Informação do carater recebido organizada em forma de um objeto com os atributos
        axis: 0 (Button), 1 (Eixo dos ZZ), 2 (Eixo dos YY), 3 (Eixo dos XX)
        msb:  1 (Most significant byte), 0 (Less significant byte), 
        data: Informação contida no byt (Informação de 5 bits)
    """
    def __init__(self, axis,msb,data):
        self.axis = axis
        self.msb = msb
        self.data = data


class Char(object):
    """Reconstrução da informação a partir do numero de bytes lidos"""
    def __init__(self, byte):
        self.__byte = byte
        self.decodeData()

    def decodeData(self):
        M,L = struct.unpack('BB', self.__byte)
        val = np.array([M,L])
        axis = val//2**6
        msb = (val-axis*2**6)//2**5
        data = (val-axis*2**6-msb*2**5)
        self.__byte1 = BYTE(axis[0],msb[0],data[0])
        self.__byte2 = BYTE(axis[1],msb[1],data[1])
        self.__axis = axis[0]

    def verify_data(self):
        axis = self.__byte1.axis == self.__byte2.axis
        msb = self.__byte1.msb == 1 and self.__byte2.msb == 0
        if axis and msb:
            self.calc_value()
        return axis and msb

    def change_data(self,byte):
        self.__byte = byte
        self.decodeData()

    def calc_value(self):
        self.__data=self.__byte1.data*2**5+self.__byte2.data

    def get_data(self):
        return self.__data

    def get_axis(self):
        return self.__axis





class Serial_Comunication(object):
    def __init__(self, serialPort = 'COM5', serialBaud = 9600, plotLength = 100, dataNumBytes = 2, num_calibrate = 500, sensivity = 9.81/0.8, convertFactor = 3.3/(2**10-1)):
        self.__port = serialPort
        self.__baud = serialBaud
        self.__dataNumBytes = dataNumBytes
        self.__rawData = bytearray(dataNumBytes)

        self.__aceleration = []
        self.__aceleration_LPF = []
        self.__aceleration_SMA = []

        self.__isRun = True
        self.__isReceiving = False
        self.__thread = None
        self.__Enable = False
        self.__Program = 0
        self.__NumPrograms = 2
        self.__calibrated = False

        self.__plotMaxLength = plotLength
        self.__sensivity = sensivity
        self.__num_calibrate = num_calibrate
        self.__convertFactor = convertFactor
        self.__numPoints = np.array([0,0,0])

        self.__datos = [[],[],[]]
        self.__alpha = 130/(2*np.pi*10+130)

        for i in range(3):
            self.__aceleration.append(collections.deque([0]*2*plotLength, maxlen=2*plotLength))
            self.__aceleration_LPF.append(collections.deque([0]*2*plotLength, maxlen=2*plotLength))
            self.__aceleration_SMA.append(collections.deque([0]*2*plotLength, maxlen=2*plotLength))

    def conect_SerialPort(self):
        print('Trying to connect to: ' + str(self.__port) + ' at ' + str(self.__baud) + ' BAUD.')
        try:
            self.__serialConnection = serial.Serial(self.__port, self.__baud, timeout=4)
            print('Connected to ' + str(self.__port) + ' at ' + str(self.__baud) + ' BAUD.')
            return True
        except:
            print("Failed to connect with " + str(self.__port) + ' at ' + str(self.__baud) + ' BAUD.')
            return False

    def readSerialStart(self,*args):
        self.__args = [*args]
        if self.__thread == None:
            self.__thread = Thread(target=self.backgroundThread)
            self.__thread.start()
            # Block till we start receiving values
            while self.__isReceiving != True:
                time.sleep(0.1)

    def calc_Byte(self):
        self.__serialConnection.readinto(self.__rawData)
        char = Char(self.__rawData)
        while not char.verify_data():
            self.__serialConnection.readinto(bytearray(1))
            self.__serialConnection.readinto(self.__rawData)
            char.change_data(self.__rawData)
        return char.get_axis(),char.get_data()


    def backgroundThread(self):  
        time.sleep(1.0) 
        self.__serialConnection.reset_input_buffer()
        print("\nReading in background enabled\n")
        while self.__isRun:
            axis, N = self.calc_Byte()
            if axis != 0 and self.__calibrated:
                value = N*self.__convertFactor-self.__V0[3-axis]
                #self.__datos[3-axis].append(value) #Raw values for analisis

                #Filtragem Digital do sinal
                value = self.__alpha*self.__aceleration_LPF[3-axis][-1]+(1-self.__alpha)*value
                self.__aceleration_LPF[3-axis].append(value)
                if np.abs(value) <= self.__std[3-axis]:
                    value = 0
                self.__aceleration_SMA[3-axis].append(value)
                value = filtro_sma(np.array(self.__aceleration_SMA[3-axis]),15)

                value *= self.__sensivity
                self.__numPoints[3-axis] += 1
                self.__aceleration[3-axis].append(value)
                
            else:
                if N == 1:
                    self.__Enable = not self.__Enable
                    if self.__Enable:
                        self.calibrate()
                        self.__calibrated = True
                    else:
                        self.__isRun = False

            self.__isReceiving = True

    def calibrate(self):

        i = 0
        v0 = [[],[],[]]

        print('\nCalibration of axis, enabled\n')
        axis = 'x'
        while(i<=self.__num_calibrate):
            progress = np.round(i*100/self.__num_calibrate,1)
            string = f'Calibrating {progress}/100%, Axis={axis}'
            aux = ' '*len(string)
            sys.stdout.write(f'\r{aux}\r{string}')
            sys.stdout.flush()
            axis, N = self.calc_Byte()
            data = N*self.__convertFactor
            v0[3-axis].append(data)
            i += 1

        print('\nCalibration complete\n')
        self.__V0 = []
        self.__std = []
        strings = 'xyz'
        pd.DataFrame(v0[2]).to_csv('Z0.csv')
        for i,v in enumerate(v0):
            self.__V0.append(np.mean(v))
            self.__std.append(np.std(v)*3/2)
            print(f'\nV{strings[i]}_Value: {self.__V0[i]} V, {self.__std[i]}')
        
    def get_data(self):
        res = []
        for a in self.__aceleration:
            res.append(a[-1])
        return res

    def get_lenData(self):
        return self.__numPoints

    def get_All(self):
        return self.__aceleration

    def isEnable(self):
        return self.__Enable

    def get_plotLength(self):
        return self.__plotMaxLength

    def get_sensivity(self):
        return self.__sensivity

    def save_dataFrame(self):
        dataframe = pd.DataFrame(np.array(self.__datos))
        dataframe.to_csv('Data.csv') 
        labels = 'xyz'
        head = []
        info = []
        for i,v in enumerate(self.__V0):
            head.append('V0'+labels[i])
            head.append('std'+labels[i])
            info.append(v)
            info.append(self.__std[i])
        pd.DataFrame([head,info]).to_csv('Values.csv')

    def close(self):
        #self.save_dataFrame()
        self.__isRun = False
        self.__thread.join()
        self.__serialConnection.close()
        print('\nDisconnected...')

        

class Data_Visualization(Serial_Comunication):

    def __init__(self,*args):
        super(Data_Visualization, self).__init__(*args)
        self.__data = []
        self.__position = collections.deque([np.array([0,0,0])]*self.get_plotLength(), maxlen=self.get_plotLength())
        self.__velocity = collections.deque([np.array([0,0,0])]*self.get_plotLength(), maxlen=self.get_plotLength())
        self.__numPoints_Anim = np.array([0,0,0])
        for i in range(3):
            self.__data.append(collections.deque([0]*self.get_plotLength(), maxlen=self.get_plotLength()))

    def calc_position(self):
        acelerations = np.array(self.get_data())
        alpha,beta,azz = self.calc_orientation()
        ox = np.round(np.sin(alpha),2)
        oy = np.round(np.sin(beta),2)
        oz = np.round(np.cos(alpha)*np.cos(beta),2)
        acelerations += np.array([0,0,azz-oz*g])
        if np.abs(ox) < 0.5 and oy < np.abs(0.5):
            acelerations += np.array([ox,oy,0])*g 
        dt = 0.01
        v0 = self.__velocity[-1]
        r0 = self.__position[-1]
        acs = np.array(list(self.__velocity)+list([acelerations*dt+v0]))
        
        r = r0+dt*v0
        v = []
        for i,k in enumerate(v0+dt*acelerations):
            if np.round(np.abs(k),2) == np.round(np.abs(np.mean(acs[-20:,i])),2):
                k = 0
            v.append(k)
        v = np.array(v)

        self.__position.append(r)
        self.__velocity.append(v)
        return v

    def calc_orientation(self):
        res = []
        acelerations = self.get_data()
        for i,a in enumerate(acelerations):
            if i<2:
                v = -a/g
                if np.abs(v) > 1:
                    v = np.pi/2 * v/np.abs(v)
                else:
                    v=(np.arcsin(v))
                res.append(v)
            else:
                res.append((a/self.get_sensivity()+0.8)*self.get_sensivity())
        return res
    
    def plot_SerialData(self, frame, line1,line2,line3):
        lines = [line1,line2,line3]
        acelerations = self.get_data()
        self.__numPoints_Anim += 1
        for i,line in enumerate(lines):
            self.__data[i].append(acelerations[i])
            line.set_data(range(self.get_plotLength()), self.__data[i])
        Data = self.get_lenData()
        sys.stdout.write(f'\rBackground {Data} Animacion:{self.__numPoints_Anim} Ratio: {Data/self.__numPoints_Anim}')
        sys.stdout.flush()

    def draw_SerialData(self):
        alpha,beta,azz = self.calc_orientation()
        ox = np.round(np.sin(alpha),2)
        oy = np.round(np.sin(beta),2)
        oz = np.round(np.cos(alpha)*np.cos(beta),2)
        shot = False
        if azz > 1.2*g:
            shot = True

        if np.abs(ox) >= 0.93 or np.abs(oy) >= 0.93:
            return ([ox,oy,0],shot)
        if azz >= 0:
            return ([ox,oy,oz],shot)
        else:
            return ([ox,oy,-oz],shot)

    def create_scene(self,width,height,vmax):
        self.__scene = vp.scene
        self.__scene.width = width
        self.__scene.height = height
        self.__scene.range = 15
        self.__scene.camera.pos = vp.vector(-30,0,10)
        self.__scene.camera.axis = vp.vector(30,0,-10)
        self.__scene.camera.up = vp.vector(0,0,1)
        axis = [vp.vector(1,0,0),vp.vector(0,1,0),vp.vector(0,0,1)]
        self.__floor = []
        w = 100
        l = w*1.7
        N = 3
        for i in range(N):
            for j in range(N):
                b = vp.box(pos = vp.vector(-w+w*i,-l+l*j,-0.5),height=1,width=l,length=w,up=vp.vector(0,0,1))
                b.texture =  'Resources\\floor.jpg'
                self.__floor.append(b)
        for ax in axis:
            vp.arrow(pos=vp.vector(0,0,0),  axis=ax, shaftwidth=0.05, color=ax)
        self.__dt = 0.1
        self.__frame = 0
        self.__vmax = vmax

    def tracker_enable(self):
        self.__acelerometer = vp.sphere(color=color.blue, make_trail=True, retain=200)
        v = vp.vector(0,0,0)
        while  self.isEnable():
            vp.rate(60)
            string = f'Runing, Frame: {self.__frame} Position: {self.__acelerometer.pos}     {v}'
            aux = ' '*len(string)*2
            sys.stdout.write(f'\r{aux}\r{string}')
            sys.stdout.flush()
            [x,y,z] = self.calc_position() 
            v =   vp.vector(x,y,z)
            self.__acelerometer.pos += v*1e-1
            self.__scene.camera.pos += v*1e-1
            self.__frame += 1
        self.close() 

    def control_enable(self):
        import scene
        orientation = [0,0,1]
        self.__acelerometer = BB8(self.__vmax)
        theta = 0
        calc_R = True
        while self.isEnable():
            vp.rate(60)
            string = f'Runing, Frame: {self.__frame} Orientation: {orientation} Position: {self.__acelerometer.get_pos()}'
            aux = ' '*len(string)*2
            sys.stdout.write(f'\r{aux}\r{string}')
            sys.stdout.flush()
            ([ox,oy,oz],shot) = self.draw_SerialData()
            orientation = np.array([ox,oy,oz])
            if shot:
                self.__acelerometer.shot()
            if oz > -0.1:
                if calc_R:
                    R = np.array([[np.cos(theta),np.sin(theta),0],[-np.sin(theta),np.cos(theta),0],[0,0,1]])
                    calc_R = False
                orientation =  np.dot(R,orientation)
                [ox,oy,oz] = orientation
                self.__acelerometer.orientation(vp.vector(ox,oy,oz))
                self.__acelerometer.move(vp.vector(ox,oy,0),self.__dt)
                self.__scene.camera.pos += vp.vector(ox,oy,0)*self.__vmax
            elif oz < -0.7:
                self.__acelerometer.orientation(vp.vector(0,0,1))
                r = (self.__scene.camera.pos-self.__acelerometer.get_pos())
                self.__scene.camera.pos += vp.vector(-r.y,r.x,0)*oy/vp.mag(r)*0.5
                r_ = (self.__scene.camera.pos-self.__acelerometer.get_pos())
                self.__scene.camera.axis = vp.vector(-r_.x,-r_.y,self.__scene.camera.axis.z-ox*0.2)
                theta_ = theta
                theta = np.arccos(np.round(r_.dot(-vp.vector(1,0,0)/np.sqrt(r_.x**2+r_.y**2)),2))
                self.__acelerometer.rotate_head(theta_-theta)
                calc_R = True
            self.__acelerometer.move_projectiles(self.__dt)    
            self.__frame += 1
        self.close()

    def create_plot(self):
        pltInterval = 50    # Period at which the plot animation updates [ms]
        xmin = 0
        xmax = self.get_plotLength()
        ymin = -1.2*g
        ymax = 1.2*g

        fig, (ax1,ax2,ax3) = plt.subplots(3,1)
        axs = (ax1,ax2,ax3)
        line1, = ax1.plot([],[],label=r'$a_x$',color='#1BA39B',lw=1.2)
        line2, = ax2.plot([],[],label=r'$a_y$',color='#2E8D17',lw=1.2)
        line3, = ax3.plot([],[],label=r'$a_z$',color='#A92F11',lw=1.2)

        for i,ax in enumerate(axs):
            ax.set_ylim(ymin, ymax)
            ax.set_xlim(xmin, xmax)
            ax.legend()
            ax.grid()

        anim = animation.FuncAnimation(fig, self.plot_SerialData, fargs=(line1,line2,line3), interval=pltInterval) 
        plt.show()
        self.close()


