#!/usr/bin/env python
import rospy
import rosparam

import numpy as np
from scipy import optimize

#from sub8_msgs.srv import Sonar, SonarResponse

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from std_msgs.msg import UInt16
#from pinger_tracker.msg import *

from advantech_pci1714.srv import *
from advantech_pci1714.msg import *

import time
import scipy.fftpack
import operator

class plotter():
    def plot_ping(self,data):       

        channels = data.channels
        samples = float(data.samples/channels)
        #print samples
        sample_rate = data.sample_rate
        #print sample_rate
        adc_bit = data.adc_bit
        data = data.data

        time = (samples/float(sample_rate))#*10**6

        #print("data received from service")

        length = samples*channels
        self.x_axis_length = samples*(1.0/sample_rate)
        starting_sample = 0
        distance = (float(length)/float(samples))*time

        #self.x = np.arange(starting_sample*(time/samples),distance, time/samples)
        #print self.x
        self.a = data[starting_sample + 0:int(length):channels]
        #print self.a
        self.b = data[starting_sample + 1:int(length):channels]
        self.c = data[starting_sample + 2:int(length):channels]
        self.d = data[starting_sample + 3:int(length):channels]

        signal = 'bad' 

        self.N = len(self.a)   #number of samples in channel1
        Fs = sample_rate
        Ts = 1.0/Fs    
        #x = np.linspace(0.0, N*Ts, N)    
        self.yf = scipy.fftpack.fft(self.a)
        index, value = max(enumerate(abs(self.yf)), key=operator.itemgetter(1))
        '''print "**************************"
        print "**************************"
        print "**************************"
        print "**************************"
        print "length of whole list: ", len(self.yf)
        print "index in whole list: ", index
        print "max value in whole list: ", abs(value)
        
        front_half = self.a[:len(self.a)/2]
        yf_1 = scipy.fftpack.fft(front_half)
        indexf, value = max(enumerate(abs(yf_1)), key=operator.itemgetter(1))
        print "**************************"
        print "length of front_half: ", len(yf_1)
        print "index in front_half: ", index
        print "max value in front_half: ", abs(value)            
        
        back_half  = self.a[len(self.a)/2:]
        yf_2 = scipy.fftpack.fft(back_half )
        indexb, value = max(enumerate(abs(yf_2)), key=operator.itemgetter(1))
        print "**************************"
        print "length of back_half: ", len(yf_2)
        print "index in back_half: ", index
        print "max value in back_half: ", abs(value)

        if indexf != indexb and index >= 25:
            signal = 'good'
            print "GOOD SIGNAL GOOD SIGNAL GOOD SIGNAL!!!!!!!"'''

        #if signal == "good":
        self.xf = np.linspace(0.0, 1.0/(2.0*Ts), self.N/2)


        legends = [None]*channels  #set up ledgends for x channels
        wave = [None]*len(self.a)  #make empty list

        n = len(self.a)   #length of samples (samplecount/4 from AdvanTech driver c++)

        self.t = np.arange(0,n*Ts,Ts)  #resolution of sampling, ie 1 MS/s = 1*10^-6

        if len(self.t) > n:
            #print t
            self.t = self.t[:-1]  #make sure len(t) is = to len(n), shave the last number off            


    def __init__(self):
        rospy.init_node('ping_plotter')
        #rospy.Subscriber('/hydrophones/ping', Ping, self.signal_plotter)

        rate = rospy.Rate(10)

        #self.x = []
        self.a = []
        self.b = []
        self.c = []
        self.d = []
        self.trigger = False
        self.t  = []
        self.x_axis_length = 0
        self.xf = 0
        self.N = 1
        self.yf = [0]

        plt.ion()
        fig, self.ax = plt.subplots(2, 1)         

        while not rospy.is_shutdown():
            #ping_service = rospy.ServiceProxy('hydrophones/ping', Ping)
            #ping = ping_service()

            rospy.Subscriber('/hydrophones/pingmsg', Pingdata, self.plot_ping)
            #rospy.Subscriber('hydrophones/ping', Ping, self.actual_position)

            self.ax[0].cla()
            self.ax[0].plot(self.t,self.a, linewidth=2.0, label='Hydrophone A')
            self.ax[0].plot(self.t,self.b, linewidth=2.0, label='Hydrophone B')
            self.ax[0].plot(self.t,self.c, linewidth=2.0, label='Hydrophone C')
            self.ax[0].plot(self.t,self.d, linewidth=2.0, label='Hydrophone D')

            #self.ax[0].legend(loc="upper left", fontsize=25)
            self.ax[0].set_title("Actual Received Signals", weight = 'bold', size = 37, x = 0.5, y = 1.02, horizontalalignment='center')
            self.ax[0].set_xlabel('Time (seconds)', size = 25, weight = 'bold', x = 0.5, y = 0)
            self.ax[0].set_ylabel('Amplitude', size = 25, weight = 'bold', x = 0, y = 0.5)
            self.ax[0].set_ylim(-5,5)
            self.ax[0].set_xlim(0,self.x_axis_length)
            self.ax[0].tick_params(axis='both', which='major', labelsize=25, pad=20)
            self.ax[0].tick_params(axis='both', which='minor', labelsize=25, pad=20)
            self.ax[0].xaxis.labelpad = 20
            self.ax[0].yaxis.labelpad = 20


            self.ax[1].cla()
            #self.ax[2].set_title("FFT On Channel One")
            #self.ax[1].plot(frq,abs(Y),'r') # plotting the FFT spectrum
            if self.yf[0] != 0:
                self.ax[1].plot(self.xf,2.0/self.N * np.abs(self.yf[:self.N//2]),'r') # plotting the FFT spectrum
            #print abs(Y)
            self.ax[1].set_xlim(5000,50000)
            #plt.xticks(np.arange(5000, 50000+1, 500.0))
            #self.ax[1].set_ylim(0,n/10)
            self.ax[1].set_xlabel('Freq (Hz)')
            self.ax[1].set_ylabel('|Y(freq)|')

            plt.pause(0.05)

            rate.sleep()

        plt.close('all')

def main():
    rospy.init_node('ping_plotter', anonymous=False)

    plotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()