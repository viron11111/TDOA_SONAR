#!/usr/bin/env python
import rospy
import numpy as np
import operator
import sys
import math

from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Vector3, Vector3Stamped
from tdoa_sonar_software.msg import *

from tdoa_sonar_software.srv import *

class solver():

    def cardinal(self, data):

        stamps = [data.calculated_time_stamps[0],data.calculated_time_stamps[1],data.calculated_time_stamps[2],data.calculated_time_stamps[3]]

        del1 = data.calculated_time_stamps[1]
        del2 = data.calculated_time_stamps[2]
        del3 = data.calculated_time_stamps[3]

        del0 = 0.0
        self.bearing = 0.0
        tolerance = 18
        self.psolution = 0

        #print "del1-2: %f del1-3: %f del2-3: %f" % (abs(del1-del2), abs(del1-del3), abs(del2-del3))

        dels = {"del0": del0, "del1": del1, "del2": del2, "del3": del3}
        sorted_dels = sorted(dels.items(), key=operator.itemgetter(1))
        sorted_dels = (sorted_dels[0][0],sorted_dels[1][0],sorted_dels[2][0],sorted_dels[3][0])  
        self.sorted_dels = sorted_dels   
        #print self.sorted_dels
        if sorted_dels == ('del2', 'del3', 'del0', 'del1'): #double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 60.0
            else:
                self.bearing = 75.0  
            self.psolution = 1
        elif sorted_dels == ('del3', 'del2', 'del0', 'del1'): #new
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 60.0
            else:
                self.bearing = 75.0  
            self.psolution = 1            
        elif sorted_dels == ('del2', 'del3', 'del1', 'del0'): #double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del1-del3) < tolerance:
                self.bearing = 120
            else:
                self.bearing = 105     
            self.psolution = 1  #changed changed back
        elif sorted_dels == ('del3', 'del2', 'del1', 'del0'): #new
            if abs(del1-del2) < tolerance:
                self.bearing = 150         
            elif abs(del1-del3) < tolerance:
                self.bearing = 120
            else:
                self.bearing = 135
            self.psolution = 1
        elif sorted_dels == ('del2', 'del1', 'del3', 'del0'): #double checked
            if abs(del1-del2) < tolerance:
                self.bearing = 150         
            elif abs(del1-del3) < tolerance:
                self.bearing = 120
            else:
                self.bearing = 135
            self.psolution = 1
        elif sorted_dels == ('del1', 'del2', 'del3', 'del0'): #double checked
            if abs(del3-del2) < tolerance:
                self.bearing = 180
            elif abs(del1-del2) < tolerance:
                self.bearing = 150
            else:                
                self.bearing = 165.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del3', 'del2', 'del0'): #switching between 1 and 2
            if abs(del2-del3) < tolerance:
                self.bearing = 180
            elif abs(del2-del0) < tolerance:
                self.bearing = 210
            else:                            
                self.bearing = 195
            self.psolution = 1
        elif sorted_dels == ('del3', 'del1', 'del2', 'del0'): #new
            if abs(del2-del3) < tolerance:
                self.bearing = 180
            elif abs(del2-del0) < tolerance:
                self.bearing = 210
            else:                            
                self.bearing = 195
            self.psolution = 1            
            #print "here2" 
        elif sorted_dels == ('del1', 'del3', 'del0', 'del2'): #double checked
            if abs(del2) < tolerance:
                self.bearing = 210
            elif abs(del3-del0)<tolerance:
                self.bearing = 240
            else:
                self.bearing = 225
            self.psolution = 2
        elif sorted_dels == ('del3', 'del1', 'del0', 'del2'): #new
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 240.0
            else:
                self.bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del0', 'del3', 'del2'): #double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 240.0
            else:
                self.bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del0', 'del1', 'del3', 'del2'):#double checked
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del3-del1) < tolerance:
                self.bearing = 300.0
            else:
                self.bearing = 285.0
            self.psolution = 2
            #print "here4" 
        elif sorted_dels == ('del0', 'del3', 'del1', 'del2'):#double checked
            if abs(del1-del2) < tolerance:
                self.bearing = 330
            elif abs(del1-del3) < tolerance:
                self.bearing = 300
            else:                            
                self.bearing = 315 
            self.psolution = 1
        elif sorted_dels == ('del3', 'del0', 'del1', 'del2'):#new
            if abs(del1-del2) < tolerance:
                self.bearing = 330
            elif abs(del1-del3) < tolerance:
                self.bearing = 300
            else:                            
                self.bearing = 315 
            self.psolution = 1            
        elif sorted_dels == ('del0', 'del3', 'del2', 'del1'):#double checked 
            if abs(del1-del2) < tolerance:
                self.bearing = 330.0
            elif abs(del2-del3) < tolerance:
                self.bearing = 0.0
            else:                            
                self.bearing = 345.0
            self.psolution = 2
        elif sorted_dels == ('del0', 'del2', 'del1', 'del3'):#new bottom
            if abs(del1-del2) < tolerance:
                self.bearing = 330.0
            elif abs(del2-del3) < tolerance:
                self.bearing = 0.0
            else:                            
                self.bearing = 345.0
            self.psolution = 2            
        elif sorted_dels == ('del0', 'del1', 'del2', 'del3'):#new bottom
            if abs(del1-del2) < tolerance:
                self.bearing = 330
            elif abs(del1-del3) < tolerance:
                self.bearing = 300
            else:                            
                self.bearing = 315 
            self.psolution = 1 
        elif sorted_dels == ('del1', 'del0', 'del2', 'del3'):#new bottom
            if abs(del1-del0) < tolerance:
                self.bearing = 270.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 240.0
            else:
                self.bearing = 255.0
            self.psolution = 1
        elif sorted_dels == ('del1', 'del2', 'del0', 'del3'):#new bottom
            if abs(del2-del3) < tolerance:
                self.bearing = 180
            elif abs(del2-del0) < tolerance:
                self.bearing = 210
            else:                            
                self.bearing = 195
            self.psolution = 1    
        elif sorted_dels == ('del2', 'del1', 'del0', 'del3'):#new bottom 
            if abs(del3-del2) < tolerance:
                self.bearing = 180
            elif abs(del1-del2) < tolerance:
                self.bearing = 150
            else:                
                self.bearing = 165.0
            self.psolution = 1
        elif sorted_dels == ('del2', 'del0', 'del1', 'del3'):#new bottom             
            if abs(del1-del0) < tolerance:
                self.bearing = 90.0
            elif abs(del0-del3) < tolerance:
                self.bearing = 60.0
            else:
                self.bearing = 75.0  
            self.psolution = 1  

            
        elif sorted_dels == ('del0', 'del2', 'del3', 'del1'):#double checked             
            if abs(del2-del3) < tolerance:            
                self.bearing = 0.0
            elif abs(del2-del0) < tolerance:
                self.bearing = 30.0
            else:
                self.bearing = 15.0  
            self.psolution = 2  
        elif sorted_dels == ('del2', 'del0', 'del3', 'del1'):#double checked 
            if abs(del0-del3) < tolerance:
                self.bearing = 60.0
            if abs(del0-del2) < tolerance:
                self.bearing = 30.0
            else:
                self.bearing = 45.0
            self.psolution = 2
            #print "here6" 
        elif sorted_dels == ('del3', 'del0', 'del2', 'del1'): #new
            if abs(del2-del3) < tolerance:            
                self.bearing = 0.0
            elif abs(del2-del0) < tolerance:
                self.bearing = 30.0
            else:
                self.bearing = 15.0  
            self.psolution = 2           

        else:
            rospy.logerr("CARDINAL failed to find solution!")
            os.system("rosnode kill crane_method_service")
            self.bearing = -1

        if sorted_dels[3] == 'del0':
            self.ref_hydro = 0
        elif sorted_dels[3] == 'del1':
            self.ref_hydro = 1
        elif sorted_dels[3] == 'del2':
            self.ref_hydro = 2

        #self.cardinal_pub = rospy.Publisher('hydrophones/cardinal', Float32, queue_size = 1)
        #self.cardinal_pub.publish(Float32(self.bearing))

        if del1 > 115:
            del1 = del1 - 33
        elif del1 < -115:
            del1 = del1 + 33

        if del2 > 115:
            del2 = del2 - 33
        elif del2 < -115:
            del2 = del2 + 33

        if del3 > 86:
            del3 = del3 - 33


        #print "del1: %f del2: %f del3: %f" % (del1, del2, del3)

        localization = rospy.ServiceProxy('/hydrophones/location_query', Localization_query)
        crane_ret = localization(self.bearing, del1, del2, del3)

        print "cardinal: %f heading: %f declination: %f" % (self.bearing, crane_ret.heading, crane_ret.declination)

        if self.bearing < 90 and crane_ret.heading > 270:
            crane_ret.heading = crane_ret.heading - 360
        elif self.bearing > 270 and crane_ret.heading < 90:
            crane_ret.heading = crane_ret.heading + 360

        x = float(crane_ret.x_pos)
        y = float(crane_ret.y_pos)
        z = float(crane_ret.z_pos)

        holder = [x,y,z]

        #print "x: %f y: %f z: %f" % (x, y, z)

        self.ping_direction_pub.publish(Vector3Stamped(
            header=Header(stamp=rospy.Time.now(),
                          frame_id='pinger_direction'),
            vector=Vector3(x,y,z)))

        #if self.bearing - crane_ret.heading > 15.0:
            #crane_ret = localization(self.bearing, del1, del2, del3)
            #print"del1: %f del2: %f del3: %f" % (del1, del2, del3)
            #print "bearing discrepency: %f" % (self.bearing - crane_ret.heading)
            #print "cardinal: %f heading: %f declination: %f" % (self.bearing, crane_ret.heading, crane_ret.declination)
        print "*********************"
            





    def __init__(self):
        rospy.init_node('cardinal')
        #rospy.Subscriber('/hydrophones/actual_time_stamps', Actual_time_stamps, self.calc_vals) #self.actu_vals)
        rospy.Subscriber('/hydrophones/calculated_time_stamps', Calculated_time_stamps, self.cardinal)
        #rospy.Subscriber('hydrophones/hydrophone_locations', Hydrophone_locations, self.hydrophone_locations)

        self.cardinal_pub = rospy.Publisher('hydrophones/cardinal', Float32, queue_size = 1)
        self.ping_direction_pub = rospy.Publisher('hydrophones/ping_direction', Vector3Stamped, queue_size = 1)


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