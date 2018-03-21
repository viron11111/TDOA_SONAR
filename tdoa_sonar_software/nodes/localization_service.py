#!/usr/bin/env python
import rospy
import numpy as np
import math

from std_msgs.msg import Header, Float32
from tdoa_sonar_software.msg import *
from tdoa_sonar_software.srv import * #Localization_query

class solver():

    def location_response(self, data):
        self.bearing = data.cardinal_bearing
        stamp1 = data.stamp1
        stamp2 = data.stamp2
        stamp3 = data.stamp3

        #rospy.logwarn("printed from location response")

        (x_pos, y_pos, z_pos) = self.calc_vals(stamp1, stamp2, stamp3)

        crane_heading = np.arctan2(x_pos, y_pos) - np.pi/2       

        crane_horizontal_distance = np.sqrt(x_pos**2 + y_pos**2)
        
        if crane_horizontal_distance != 0:
            calculated_declination = np.arctan(z_pos/crane_horizontal_distance)
        else:
            calculated_declination = 0.0

        heading = math.degrees(crane_heading)
        declination = math.degrees(calculated_declination)

        if heading < 0:
            heading = heading + 360
        elif heading > 360:
            heading = heading - 360

        return Localization_queryResponse(x_pos, y_pos, z_pos, heading, declination)
       
    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return(x, y)        

    def crane_calc(self,del1i,del2i,del3i,hydro1,hydro2,hydro3):

        #print "del1: %0.10f del2: %0.10f del3: %0.10f" % (del1i, del2i, del3i)        
       
        del1 = del1i
        del2 = del2i
        del3 = del3i 

        x1 = hydro1[0] #+ 0.75 #np.random.uniform(-1, 1)  #in mm
        x2 = hydro2[0] #- 0.75 #np.random.uniform(-1, 1)
        y2 = hydro2[1] #+ 0.75 #np.random.uniform(-1, 1)
        x3 = hydro3[0] #- 0.75 #np.random.uniform(-1, 1)
        y3 = hydro3[1] #+ 0.75 #np.random.uniform(-1, 1)
        z3 = hydro3[2]            


        print "********"

        P1 = [0,0,0]
        P2 = [0,0,0]        

        left = x1/del1 if del1 != 0 else 0
        right = x2/del2 if del2 != 0 else 0

        #print "left = %f B1 = %f right = %f" % (left, B1, right)

        A1 = left - right    # eqn (12)
        B1 = -y2/del2 if del2 != 0 else 0

        #print "A1: %f" % A1
        
        zero = (x2*x2 + y2*y2-del2*del2)/(2.0*del2) if del2 != 0 else 0
        catch = (x1*x1-del1*del1)/(2.0*del1) if del1 != 0 else 0

        D1 = zero - catch

        #print "D1: %f" % D1
        
        Q1 = -z3/del3 if del3 != 0 else 0

        Q2aleft = (A1*y3)/(B1*del3) if (B1*del3) !=0 else 0
        Q2amiddle = x3/del3 if del3 != 0 else 0
        Q2aright = x1/del1 if del1 != 0 else 0

        Q2a = Q2aleft - Q2amiddle + Q2aright

        Q2bleft = (del1*del1-x1*x1)/(2.0*del1) if (2.0*del1) != 0 else 0
        Q2bmiddle = (D1*y3)/(B1*del3) if (B1*del3) != 0 else 0
        Q2bright = (del3*del3 - x3*x3 - y3*y3 - z3*z3)/(2.0*del3) if (2.0*del3) != 0 else 0

        Q2b = Q2bleft + Q2bmiddle - Q2bright

        R1aleft = (A1*A1)/(B1*B1) if (B1*B1) != 0 else 0
        R1aright = (x1*x1)/(del1*del1) if (del1*del1) != 0 else 0

        R1a = R1aleft + 1 - R1aright

        R1bleft = (x1*x1*x1)/(del1*del1) if (del1*del1) != 0 else 0
        R1bright = (2.0*A1*D1)/(B1*B1) if (B1*B1) != 0 else 0

        R1b = R1bleft - x1 + R1bright

        R1cleft = (D1*D1)/(B1*B1) if (B1*B1) != 0 else 0
        R1cright = (x1*x1*x1*x1)/(4.0*del1*del1) if (4.0*del1*del1) != 0 else 0

        R1c = (x1*x1)/2.0 - (del1*del1)/4.0 + R1cleft - R1cright

        AA = Q1*Q1*R1a + Q2a*Q2a
        BB = Q1*Q1*R1b + 2.0*Q2a*Q2b
        CC = Q1*Q1*R1c + Q2b*Q2b

        discr = BB*BB - 4.0*AA*CC ;

        phi = math.radians(360-self.bearing)
        rho = 500.0


        if (discr < 0):
            #rospy.logerr("no real solution was found; set garbage values for P1 and P2")

            print discr

            (x,y) = self.pol2cart(rho,phi)

            P1[0] = P1[1] = P1[2] = 0.0
            P2[0] = P2[1] = P2[2] = 0.0
            z=0 #10000  #value to make sure no solution is apparent        

        else:

            P1[0] = (-BB+math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0
            P2[0] = (-BB-math.sqrt(discr))/(2.0*AA) if (2.0*AA) != 0 else 0

            # get corresponding value for y coordinate  ;  eqn (11)
            P1[1] = -(A1*P1[0]+D1)/B1 if B1 != 0 else 0
            P2[1] = -(A1*P2[0]+D1)/B1 if B1 != 0 else 0

            # get correspoinding value for z coordinate ;  eqn (16)
            P1[2] = -(Q2a*P1[0]+Q2b)/Q1 if Q1 != 0 else 0
            P2[2] = -(Q2a*P2[0]+Q2b)/Q1 if Q1 != 0 else 0

            d0_1 = -P1[0] * (x1 / del1) + (x1*x1 - del1*del1) / (2.0*del1)        
            d0_2 = -P2[0] * (x1 / del1) + (x1*x1 - del1*del1) / (2.0*del1)

            rospy.loginfo("x1: %f" % (P1[0]))
            rospy.loginfo("y1: %f" % (P1[1]))
            rospy.loginfo("z1: %f" % (P1[2]))
            rospy.loginfo("**")
            rospy.loginfo("x2: %f" % (P2[0]))
            rospy.loginfo("y2: %f" % (P2[1]))
            rospy.loginfo("z2: %f" % (P2[2]))

            dellist = [del1, del2, del3]

            check_d0 = math.sqrt(P1[0]*P1[0]+P1[1]*P1[1]+P1[2]*P1[2])
            check_d1 = math.sqrt((P1[0]-x1)*(P1[0]-x1)+P1[1]*P1[1]+P1[2]*P1[2])
            check_d2 = math.sqrt((P1[0]-x2)*(P1[0]-x2)+(P1[1]-y2)*(P1[1]-y2)+P1[2]*P1[2])
            check_d3 = math.sqrt((P1[0]-x3)*(P1[0]-x3)+(P1[1]-y3)*(P1[1]-y3)+(P1[2]-z3)*(P1[2]-z3))

            measured1_list = [check_d1 - check_d0, check_d2 - check_d0, check_d3 - check_d0]

            check_d0 = math.sqrt(P2[0]*P2[0]+P2[1]*P2[1]+P2[2]*P2[2])
            check_d1 = math.sqrt((P2[0]-x1)*(P2[0]-x1)+P2[1]*P2[1]+P2[2]*P2[2])
            check_d2 = math.sqrt((P2[0]-x2)*(P2[0]-x2)+(P2[1]-y2)*(P2[1]-y2)+P2[2]*P2[2])
            check_d3 = math.sqrt((P2[0]-x3)*(P2[0]-x3)+(P2[1]-y3)*(P2[1]-y3)+(P2[2]-z3)*(P2[2]-z3))

            measured2_list = [check_d1 - check_d0, check_d2 - check_d0, check_d3 - check_d0]

            measured1_list = [int(i) for i in measured1_list]
            measured2_list = [int(i) for i in measured2_list]
            dellist = [int(i) for i in dellist]

            print "dellist:        ", dellist
            print "measured1_list: ", measured1_list
            print "measured2_list: ", measured2_list

            x=0
            y=0
            z=0           

            p1_heading = np.arctan2(P1[0],P1[1]) - np.pi/2
            p1_heading = math.degrees(p1_heading)
            p2_heading = np.arctan2(P2[0], P2[1]) - np.pi/2
            p2_heading = math.degrees(p2_heading)

            if p1_heading < 0:
                p1_heading = 360 + p1_heading
            if p2_heading < 0:
                p2_heading = 360 + p2_heading                

            rospy.loginfo("p1_heading: %0.2f" % p1_heading)
            rospy.loginfo("p2_heading: %0.2f" % p2_heading)

            inv_bearing = self.bearing + 180   
            if inv_bearing >= 360:
                inv_bearing = inv_bearing - 360         
            #inv_p1heading = p1_heading + 180
            #inv_p2heading = p2_heading + 180


            #rospy.logwarn("do_1: %0.2f" % d0_1)
            #rospy.logwarn("do_2: %0.2f" % d0_2)

            #*********************************************

            #corrections using dels lookup table.
            #rospy.logwarn(self.psolution)
            #map(abs, myList)

            angle_tolerance = 89

            if map(abs,measured1_list) == map(abs,dellist) or map(abs,measured2_list) == map(abs,dellist):
                sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])

                if measured1_list == measured2_list and measured1_list == dellist:
                    #rospy.logwarn("P1 == P2 == dellist")
                    sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                    sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])
                    print "sum1: ", sum1
                    print "sum2: ", sum2      
                    solution = []
                    if sum1 > sum2:
                        solution = P1
                        heading = p1_heading
                        self.psolution = 1
                        print "P1"
                    else:
                        solution = P2
                        heading = p2_heading
                        self.psolution = 2
                        print "P2"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360                         

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing)

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")   

                elif sum1 > sum2:
                    #rospy.logwarn("measured1_list")
                    solution = P1
                    heading = p1_heading
                    self.psolution = 1    
                    print "P1"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360                      

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing)    

                    print "bearing: %0.2f" % self.bearing
                    print "inv_bearing: %0.2f" % inv_bearing

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")

                elif sum2 > sum1:
                    #rospy.logwarn("measured2_list")
                    solution = P2
                    heading = p2_heading
                    self.psolution = 2  
                    print "P2"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360                      

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing) 

                    print "bearing: %0.2f" % self.bearing
                    print "inv_bearing: %0.2f" % inv_bearing

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff                          

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")                    

                elif (measured1_list == measured2_list) and measured1_list != dellist:
                    #rospy.logwarn("not equal")
                    sum1 = abs(P1[0]) + abs(P1[1]) + abs(P1[2])
                    sum2 = abs(P2[0]) + abs(P2[1]) + abs(P2[2])
                    print "sum1: ", sum1
                    print "sum2: ", sum2      
                    solution = []
                    if sum1 > sum2:
                        solution = P1
                        heading = p1_heading
                        self.psolution = 1
                        print "P1"
                    else:
                        solution = P2
                        heading = p2_heading
                        self.psolution = 2
                        print "P2"

                    if self.bearing > 360-angle_tolerance and heading < 0 + angle_tolerance:
                        self.bearing = self.bearing - 360
                    elif heading > 360-angle_tolerance and self.bearing < 0 + angle_tolerance:
                        self.bearing = self.bearing + 360 

                    angle_diff = abs(heading-self.bearing)
                    inv_angle_diff = abs(heading - inv_bearing)

                    print "bearing: %0.2f" % self.bearing
                    print "inv_bearing: %0.2f" % inv_bearing

                    print "angle_diff: %0.2f" % angle_diff
                    print "inv_angle_diff: %0.2f" % inv_angle_diff

                    if angle_diff < inv_angle_diff and angle_diff < angle_tolerance:
                        x = solution[0]
                        y = solution[1]
                        z = solution[2]
                        rospy.loginfo("vector standard")
                    elif angle_diff > inv_angle_diff and inv_angle_diff < angle_tolerance:
                        x = -solution[0]
                        y = -solution[1]
                        z = -solution[2]
                        rospy.loginfo("vector inverted")
                    else:
                        (x,y) = self.pol2cart(rho,phi)
                        rospy.logerr("defaulting to cardinal")                     

                else:
                    (x,y) = self.pol2cart(rho,phi)
                    rospy.logerr("defaulting to cardinal")     
            else:
                (x,y) = self.pol2cart(rho,phi)
                rospy.logerr("defaulting to cardinal")   

        return (x,y,z,P1,P2)     

    def calc_vals(self, stamp1, stamp2, stamp3):

        c = 1.484 # speed of sound in 20 C water per uSec
        #c = 0.343 #speed of sound in air 

        #self.bearing = self.cardinal(data.calculated_time_stamps[1],data.calculated_time_stamps[2],data.calculated_time_stamps[3])        

        P1 = [0,0,0]
        P2 = [0,0,0]

        #print "stamp1: %f stamp2: %f stamp3: %f" % (stamp1, stamp2, stamp3)

        del1i = stamp1*c #mm/uSec
        del2i = stamp2*c #mm/uSec
        del3i = stamp3*c #mm/uSec

        #print "*******************POSITION 1*******************"
        #print "del1: %0.10f del2: %0.10f del3: %0.10f" % (del1i, del2i, del3i)         

        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [173.2,   0,     0]
        #hydro2_xyz = [86.6,  150.0,     0]
        #hydro3_xyz = [86.6,  50.0, -87.55]      

        #hydro1: [170.75, 0, 0] hydro2: [83.6, 147.65, 0] hydro3: [86.75, 42.4, -87.55] 

        hydro0_xyz = [0,      0,     0]
        hydro1_xyz = [166.2,   0,     0]        #[170.75, 0, 0]        [166,   0,     0] 
        hydro2_xyz = [80.9,  141.7,     0]      #[81.6, 147.65,0]      [80.9,  141,     0] 
        hydro3_xyz = [76.4,  42.3, -87.55]   #[86.75, 42.4, -87.55] [76.4,  42.3, -87.55]       

        #hydro0_xyz = [0,      0,     0]
        #hydro1_xyz = [170.75, 0, 0]        #[166,   0,     0] 
        #hydro2_xyz = [81.6, 147.65,0]      #[80.9,  141,     0] 
        #hydro3_xyz = [86.75, 42.4, -87.55] #[76.4,  42.3, -87.55]            

        (x1,y1,z1,P11,P21) = self.crane_calc(del1i,del2i,del3i,hydro1_xyz,hydro2_xyz,hydro3_xyz)    

        x = x1 #-173.2-173.2 #(x1+x2)/2
        y = y1 #(y1+y2)/2
        z = z1 #(z1+z2)/2       

        (x,y,z) = (x1,y1,z1)         

        return (x, y, z)        

    def __init__(self):
        rospy.init_node('locating_service')
        #rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.calc_vals)

        self.location_serv = rospy.Service('/hydrophones/location_query', Localization_query, self.location_response)

        self.ref_hydro = 0
        self.psolution = 0

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()

def main():
    
    solver()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              