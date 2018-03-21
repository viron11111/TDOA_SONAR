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
from pinger_tracker.srv import *

import time
import scipy.fftpack
import operator

import math
import pygame

class plotter():

    def received(self,data):
        rospy.loginfo("Ping received")

        ref = rospy.ServiceProxy('/hydrophones/crane_srv', Crane_pos_service)
        ref = ref()
        #print ref

        self.crane_x = ref.x
        self.crane_y = ref.y
        self.crane_z = ref.z

        crane_heading = np.arctan2(self.crane_y,self.crane_x) + np.pi

        crane_horizontal_distance = np.sqrt(self.crane_x**2+self.crane_y**2)
        
        if crane_horizontal_distance != 0:
            calculated_declination = np.arctan(self.crane_z/crane_horizontal_distance)
        else:
            calculated_declination = 0.0

        crane_heading = math.degrees(crane_heading)
        calculated_declination = math.degrees(calculated_declination)

        rospy.loginfo("heading: %f deg, declination: %f deg" % (crane_heading, calculated_declination))

        pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
        myfont = pygame.font.SysFont('Comic Sans MS', 20)
        bigfont = pygame.font.SysFont('Comic Sans MS', 30)
        bhydro = myfont.render('B', False, (255, 0, 0))
        chydro = myfont.render('C', False, (255, 0, 0))
        dhydro = myfont.render('D', False, (255, 0, 0))
        north  = myfont.render('180', False, (255, 255, 0))
        west  = myfont.render('270', False, (255, 255, 0))
        east  = myfont.render('90', False, (255, 255, 0))
        south  = myfont.render('0', False, (255, 255, 0))
        heading  = bigfont.render(('Bearing: %0.1f' % crane_heading), False, (255, 255, 255))
        declination  = bigfont.render(('Declination: %0.1f' % calculated_declination), False, (255, 255, 255))
       
        self.screen.fill((0,0,0))

        angle = crane_heading+180
        pos = 100,150

        arrow=pygame.Surface((5,80))
        arrow.fill((0,255,0))
        pygame.draw.rect(arrow, (0,0,0), pygame.Rect(0, 40, 5, 80))
        #pygame.draw.line(arrow, (0,0,0), (0,50), (25,25))
        arrow.set_colorkey((255,255,255))

        nar=pygame.transform.rotate(arrow,angle)
        nrect=nar.get_rect(center=pos)
        self.screen.blit(nar, nrect)
        pygame.display.flip()

        self.screen.blit(bhydro,(95,105))
        self.screen.blit(chydro,(60,155))
        self.screen.blit(dhydro,(130,155))
        self.screen.blit(north,(95,90))
        self.screen.blit(east,(143,155))
        self.screen.blit(west,(35,155))
        self.screen.blit(south,(90,195))
        self.screen.blit(heading,(10,30))
        self.screen.blit(declination,(220,30))


        pygame.draw.circle(self.screen, (255,255,0), (100, 150), 5, 0)
        pygame.draw.circle(self.screen, (255,255,255), (100, 125), 5, 0)
        pygame.draw.circle(self.screen, (255,255,255), (78, 163), 5, 0)
        pygame.draw.circle(self.screen, (255,255,255), (122, 163), 5, 0)       

        angle2 = calculated_declination-90
        pos2 = 220,90

        arrow2=pygame.Surface((5,200))
        arrow2.fill((0,255,0))
        pygame.draw.rect(arrow2, (0,0,0), pygame.Rect(0, 100, 5, 200))
        #pygame.draw.line(arrow, (0,0,0), (0,50), (25,25))
        arrow2.set_colorkey((255,255,255))

        nar2=pygame.transform.rotate(arrow2,angle2)
        nrect2=nar2.get_rect(center=pos2)
        self.screen.blit(nar2, nrect2)
        pygame.display.flip()        

        pygame.draw.line(self.screen,(255,255,255),(220,90),(220,220),2)  
        pygame.draw.line(self.screen,(255,255,255),(220,220),(340,220),2)  

        '''ping_service = rospy.ServiceProxy('hydrophones/ping', Ping)
    
        ping = ping_service()

        channels = ping.channels
        samples = float(ping.samples/channels)
        #print samples
        sample_rate = ping.sample_rate
        #print sample_rate
        adc_bit = ping.adc_bit
        data = ping.data        

        print adc_bit'''

        return Ping_receivedResponse()

    def location_service(self, data):

        #can change x1, x2, x3, y2, y3

        #MIL T-shape layout
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-100.0,  0,     0]
        #hydro3_xyz = [0,  -100.0, 0]      

        # Equilateral layout (actual)
        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [100.0,   0,     0]
        #hydro2_xyz = [-50,  86.6,     0]
        #hydro3_xyz = [-50,  -86.6, 0]

        #experimental layout
        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [173.2,   0,     0]
        hydro2_xyz = [86.6,  -150,     0]
        hydro3_xyz = [86.6,  -50, -100]

        return Hydrophone_locations_serviceResponse(hydro0_xyz, hydro1_xyz, hydro2_xyz ,hydro3_xyz)

    def __init__(self):

        rospy.init_node('sonar_handler')

        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        done = False   

        #self.pingpub = rospy.Publisher('/hydrophones/ping', , queue_size=1)
        rospy.Service('hydrophones/hydrophone_position', Hydrophone_locations_service, self.location_service)
        rospy.Service('hydrophones/ready', Ping_received, self.received)

        rate = rospy.Rate(10)

        try:
            rospy.loginfo('waiting for hydrophones/ping service')
            rospy.wait_for_service('hydrophones/ping', timeout = 2)
            rospy.loginfo('detected hydrophones/ping service')
        except rospy.ROSException:
            rospy.logwarn("The hydrophones/ping service never showed up!")
            rospy.signal_shutdown("ROSPy Shutdown")
        
        while not rospy.is_shutdown():

            for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                            done = True
            
            pygame.display.flip()
            rate.sleep()

def main():
    rospy.init_node('sonar_handler', anonymous=False)

    plotter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()