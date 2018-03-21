#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Header
from advantech_pci1714.msg import *


class condition():     

    def change_sensitivity(self, data):
        sens_rate = data.sensitivity
        if sens_rate == -1:
            self.break_val -= 0.01
            if self.break_val < self.min_break_val:
                self.break_val = self.min_break_val
        elif sens_rate == 1:
            self.break_val += 0.02
            if self.break_val > self.max_break_val:
                self.break_val = self.max_break_val  
        #print self.break_val        

    def refine_data(self, channels, samples, sample_rate, data):  
        self.signal = []        

        #Seperate list into individual channels
        for i in range(channels):
            self.signal.append([])
            self.signal[i] = data[i::4]

        self.break_num = 0

        #determine average noise offset and apply offset
        avg_offset = [0]*channels
        for b in range(channels):
            #use first 100 samples
            l = self.signal[b][:200]
            avg_offset[b] = sum(l) / float(len(l))
            self.signal[b] = [x-avg_offset[b] for x in self.signal[b]]

        #find the first signal
        #looks for first signal to go above self.break_val value

        #print self.break_val

        self.break_num = [0]*channels
        #print self.break_val
        for b in range(0,4):
            for i in range(samples/4):
                if self.signal[b][i] >= self.break_val:
                    print self.signal[b][i]
                    self.break_num[b] = i
                    break

        print self.break_num
        self.earliest_break_num = min(self.break_num)

        #rejection statement for samples triggered in middle of transmission (no start point)
        #print earliest_break_num

        overall_max_value = max(data)
        print "max val: %f" % overall_max_value
        print "earliest_break_num: %i" % self.earliest_break_num

        if self.earliest_break_num > 50 and overall_max_value >= 0.25:

            #Buffering holder (zeros) based on the time of 1 period at 25 kHz
            #num_samples_save = int((6.0/25000.0)*sample_rate)
            num_samples_save = int((2.0/25000.0)*sample_rate)
            zeros = [0]*(num_samples_save/4)

            #eliminate all information before first signal by adding zeros in front of signal
            #appy to other 3 signals
            for b in range(channels):
                self.signal[b] = self.signal[b][self.earliest_break_num-num_samples_save::]#::]
                #self.signal[b] = np.append(zeros,self.signal[b])
                
            '''for b in range(channels):
                #print "break_num: ",break_num[b]
                #print "earliest_break_num ", earliest_break_num
                for i in range(len(self.signal[b])):
                    if i < self.break_num[b]-self.earliest_break_num: 
                        self.signal[b][i] = 0
                    else: 
                        break     '''           
                    

            '''for b in range(channels):
                for i in range(len(self.signal[b])):
                    if i < break_num[b]-num_samples_save: self.signal[b][i] = 0
                    else: break'''

            #holder for new signal length (still contains samples following initial signal)
            final_length = len(self.signal[0])

            lastest_signal = 0
            current_signal = 0

            #using same variable as above
            #Buffering holder for keeping X periods of actual signal at 25 kHz
            num_samples_save = int((2.0/25000.0)*sample_rate)
            

            error = 0

            #*********************************************************************
            #****************NORMALIZATION****************************************
            #*********************************************************************
            abs_signal = [[],[],[],[]]
            signal_average = [0]*channels

            estimated_signal_start = 0 #300 

            #signal_holder
  
            for i in range(channels):
                abs_signal[i] = map(abs, self.signal[i][estimated_signal_start:])
                signal_average[i] = np.mean(abs_signal[i])

            if max(signal_average)/min(signal_average) > 6:
                print "Weak signal detected"
                error = 1

            total_average = np.average(signal_average)
            
            #print signal_average

            max_signal_average = max(signal_average)

            for i in range(channels):
                self.signal[i] = [z * (max_signal_average/signal_average[i]) for z in self.signal[i]]

            #*********************************************************************
            #*********************************************************************
            #*********************************************************************          

            old = self.signal[0][0]
            sample_list = []

            max_cut_length = [0]*channels

            #test_var = [[100,50],[101,25]]
            #print test_var[0][1]
            print "***************************** "

            time_cut_positions = [0]*channels

            for b in range(channels):
                sample_list = []
                list_dif = []
                positive_list = []
                negative_list = []

                #print "----channel %i-------" % b

                #find zero crossings and save them into sample list
                for i in range(final_length):
                    new = self.signal[b][i]
                    if (old < 0 and new > 0) or (old > 0 and new < 0):
                        sample_list = np.append(sample_list,i)
                        if old < 0:
                            positive_list = np.append(positive_list,i) #placeholder for n-shape start of signal

                    old = new


                list_dif = [0]*len(sample_list)

                for i in range(len(sample_list)-1):
                    list_dif[i] = sample_list[i+1] - sample_list[i] 

                #if b == 0:
                #    print list_dif

                              

                number_of_crossings = 4#4

                space_counter = 0
                space_holder = 0
                positive_voltage_thresh = 0.15 #0.2

                sign = 0

                if b == 0:
                    positive_voltage_thresh -= 0.05

                if b == 1:
                    positive_voltage_thresh -= 0.05

                if b == 2:
                    positive_voltage_thresh -= 0.05

                if b == 3:
                    positive_voltage_thresh -= 0.1

                negative_voltage_thresh = -positive_voltage_thresh

                #if b == 0:
                #    print list_dif

                for i in range(len(list_dif)):


                    if list_dif[i] >= 26 and list_dif[i] <= 71:
                        max_value = max(self.signal[b][int(sample_list[i]):int(sample_list[i+1])])
                        min_value = min(self.signal[b][int(sample_list[i]):int(sample_list[i+1])])

                        #print "max_value: %f" % (max_value)
                        #print negative_voltage_thresh
                        if max_value > positive_voltage_thresh or min_value < negative_voltage_thresh:
                            if space_counter == 0:
                                if min_value < -positive_voltage_thresh:
                                    sign = 1
                                space_holder = i
                            space_counter += 1
                            #print "space_holder: %i" % (space_holder)
                            #print "sample_list: %i" % sample_list[space_holder]
                            #print space_counter
                        elif max_value >= 0 or min_value <= 0:
                            '''if b == 0:
                                lister = [0]*4
                                for y in range(4):
                                    lister[y] = self.signal[b][int(sample_list[i+y-4])]
                                print "lister: %s" % lister
                                print "counter zeroed due to max_value"'''
                            space_counter = 0
                    else:
                        '''if b == 0:
                            lister = [0]*4
                            for y in range(4):
                                lister[y] = list_dif[i+y-4]
                            print "lister: %s" % lister
                            print "counter zeroed due to list_dif"'''
                        space_counter = 0

                    if space_counter >= 6:
                        if sign == 1:
                            number_of_crossings += 1
                        #print "space_counter max reached. "
                        self.signal[b] = self.signal[b][:int(sample_list[space_holder+number_of_crossings])]
                        time_cut_positions[b] = int(sample_list[space_holder+number_of_crossings])
                        space_counter = 0
                        break

                        #print "max: %f min: %f" % (max_value, min_value)



                    '''if i < len(list_dif)-5:
                        





                        average = (list_dif[i]+list_dif[i+1]+list_dif[i+2]+list_dif[i+3]+list_dif[i+4]+list_dif[i+5]+list_dif[i+6]+list_dif[i+7]+list_dif[i+8]+list_dif[i+9])/10
                        #average = (list_dif[i]+list_dif[i+1]+list_dif[i+2])/3

                        peaks = [0]*2
                        g = 0

                        for j in range(0,4,2):
                            ###print j                            
                            pos_peak_val = max(self.signal[b][int(sample_list[(i)+j]):int(sample_list[(i)+j+1])])
                            peaks[g] = pos_peak_val
                            #neg_peak_val = min(self.signal[b][int(sample_list[i]):int(sample_list[i+1])])
                            g += 1

                        #if b == 3:
                            #print peaks
                            #print np.average(peaks)
                        trigger_val = 1

                        if average > 31.0 and average < 36.0: #and np.average(peaks) > trigger_val*((max_signal_average/(signal_average[b]*2))):#*2): #(pos_peak_val > 0.07 or neg_peak_val < -0.07):
                        
                            #print np.average(peaks)    
                            #start_of_signal = i
                            start_position = sample_list[i]
                            nearest_positive_arch = min(positive_list, key=lambda x:abs(x-start_position))

                            for i in range(len(positive_list)):
                                if nearest_positive_arch == positive_list[i]:
                                    cut_point = positive_list[i+2]
                                    time_cut_positions[b] = positive_list[i]

                            #print "start_position: %i nearest_positive_arch: %i" % (start_position, nearest_positive_arch)
                            #print "nearest_positive_arch: %i cut_point: %i " % (nearest_positive_arch, cut_point)
                            self.signal[b] = self.signal[b][:int(cut_point)]

                            if cut_point > 1200:
                                error = 1

                            break

                    else:
                        rospy.logerr("Bad signal on channel %i" % b)'''

            #print time_cut_positions
            time_cut_diffs = [0]*4
            for i in range(len(time_cut_positions)):
                #print "time1: %f time2: %f" % (time_cut_positions[i], time_cut_positions[i+1])
                time_cut_diffs[i] = time_cut_positions[0] - time_cut_positions[i]
                time_cut_diffs[i] = time_cut_diffs[i]/2.0

            print "time_cut_diffs: %s" % time_cut_diffs

            #self.calc_stamps_pub = rospy.Publisher('/hydrophones/calculated_time_stamps', Calculated_time_stamps, queue_size = 1)
            #self.calc_stamps_pub.publish(Calculated_time_stamps(
            #    header=Header(stamp=rospy.Time.now(),
            #                  frame_id='phase_shift'),
            #    calculated_time_stamps=time_cut_diffs))

            lengths = [len(self.signal[0]),len(self.signal[1]),len(self.signal[2]),len(self.signal[3])]
            #print lengths

            max_samples = max(lengths)
            #max_samples = max(max_cut_length)

            #for i in range(channels):
            #    self.signal[i] = self.signal[i][:max_samples]

            #print "max_number of samples: %i" % max_samples

            possible_time_stamps = [0]*channels

            for i in range(channels):
                possible_time_stamps[i] = (lengths[0] - lengths[i])/2.0
            #print possible_time_stamps

            for i in range(channels):
                difference = max_samples - lengths[i]
                zeros = [0]*(difference+50)
                self.signal[i] = np.append(self.signal[i], zeros)


            
            #*********************************************************************
            #****************NORMALIZATION****************************************
            #*********************************************************************
            '''abs_signal = [[],[],[],[]]
            signal_average = [0]*channels
  
            for i in range(channels):
                abs_signal[i] = map(abs, self.signal[i])
                signal_average[i] = np.mean(abs_signal[i])

            #print signal_average
            max_signal_average = max(signal_average)

            for i in range(channels):
                self.signal[i] = [z * (max_signal_average/signal_average[i]) for z in self.signal[i]]'''

            #*********************************************************************
            #*********************************************************************
            #*********************************************************************                  
            

            lengths = [len(self.signal[0]),len(self.signal[1]),len(self.signal[2]),len(self.signal[3])]
            #print lengths

            max_samples = max(lengths)

            possible_time_stamps = [0]*channels     

            condition_data = []

            for i in range(len(self.signal[0])):
                condition_data = np.append(condition_data,self.signal[0][i])
                condition_data = np.append(condition_data,self.signal[1][i])
                condition_data = np.append(condition_data,self.signal[2][i])
                condition_data = np.append(condition_data,self.signal[3][i])

            if error != 1:
                self.simulate_pub.publish(Pingdata(
                    header=Header(stamp=rospy.Time.now(),
                                  frame_id='signal_conditioner'),
                    channels=channels,
                    samples=len(self.signal[0])*4,
                    data=condition_data,
                    adc_bit = 12,
                    sample_rate=sample_rate))               

 


    def condition_data(self, msg):
        channels = msg.channels
        samples  = msg.samples
        sample_rate = msg.sample_rate
        adc_bit = msg.adc_bit
        data = msg.data      

        self.refine_data(channels, samples, sample_rate, data)

        error = 0

        #check to make sure signal sensitivity is not too low
        for i in range(channels):
            if len(self.signal[i]) < 1100 and error !=1:
                if max(self.signal[i]) < 8.0*self.break_val:
                    self.break_val += 0.01

                    #print self.break_val

                    if self.break_val >= self.max_break_val:
                        self.break_val = self.max_break_val   
                        i = 3                                                    
                        self.counter = 10

                    self.counter += 1

                    #if self.counter < 10:                        
                     #   self.refine_data(channels, samples, sample_rate, data)

            else:
                #rospy.logwarn("Bad signal on channel %i" % i)
                error = 1
                break

        if self.counter >= 9:
            #rospy.logerr("counter filled")
            self.break_val = 0.1
            error = 1
            self.counter = 0            

        if error != 1:

            #combine four signals back into one array
            condition_data = []

            for i in range(len(self.signal[0])):
                condition_data = np.append(condition_data,self.signal[0][i])
                condition_data = np.append(condition_data,self.signal[1][i])
                condition_data = np.append(condition_data,self.signal[2][i])
                condition_data = np.append(condition_data,self.signal[3][i])

            '''self.simulate_pub.publish(Pingdata(
                header=Header(stamp=rospy.Time.now(),
                              frame_id='signal_conditioner'),
                channels=channels,
                samples=len(self.signal[0])*4,
                data=condition_data,
                adc_bit = 12,
                sample_rate=sample_rate))   '''  

        else:
            hydro = '*you should not see this*'
            #rospy.logwarn("Missed beginning of signal, triggered late")
            #ospy.logwarn("Interference or weak channel")
            for i in range(4):
                if self.break_num[i] == self.earliest_break_num:
                    if i == 0:
                        hydro = 'A'
                    elif i == 1:
                        hydro = 'B'
                    elif i == 2:
                        hydro = 'C'
                    elif i == 3:
                        hydro = 'D'
                    #rospy.logerr("Problem with hydrophone %c.  break_num = %s." % (hydro, self.break_num))


    def __init__(self):
        rospy.init_node('signal_conditioner')

        #rospy.Subscriber('hydrophones/pingmsg', Pingdata, self.condition_data) # for simulation and bags
        rospy.Subscriber('hydrophones/pingraw', Pingdata, self.condition_data)
        #rospy.Subscriber('hydrophones/sensitivity', Sensitivity, self.change_sensitivity)

        self.simulate_pub = rospy.Publisher('hydrophones/pingconditioned', Pingdata, queue_size = 1)
        #self.slope_pub = rospy.Publisher('hydrophones/slope', Slope, queue_size = 1)
        #self.neg_slope_pub = rospy.Publisher('hydrophones/negative_slope', Negative_slope, queue_size=1)
        #self.plot_pub = rospy.Publisher('/hydrophones/plot', Plot, queue_size=1)

        self.break_val = 0.05 #0.15 #voltage in which threshold is triggered
        self.min_break_val = -self.break_val

        self.max_break_val = 0.25
        self.min_break_val = 0.02
        self.counter = 0

        self.previous_noise_floor_position = [400,400,400,400]

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()

def main():
    
    condition()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print "Shutting down"
        pass
    


if __name__ == '__main__':
    main()              

    '''

        zz = (x1/del1) if del1 != 0 else 0
        z = (x3/del3) if del3 != 0 else 0

        A2 = zz - z   # eqn (14)

        #print "A2: %f" % A2

        B2 = -y3/del3 if del3 != 0 else 0

        #print "B2: %f" % B2

        holder = (x3*x3 + y3*y3-del3*del3)/(2.0*del3) if del3 != 0 else 0
        holder2 = (x1*x1-del1*del1)/(2*del1) if del1 != 0 else 0

        D2 = holder - holder2 

        #print "D2: %f" % D2

        x =  (B1*D2-B2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0  # eqn (15)
        y = -(A1*D2-A2*D1)/(A1*B2-A2*B1) if (A1*B2-A2*B1) != 0 else 0

        myx = x 
        myy = y        

        T1 = -4*del1*del1
        T2 =  4*(x1*x1-del1*del1)*myx*myx + 4*x1*(del1*del1-x1*x1)*myx + del1*del1*del1*del1 -2*del1*del1*x1*x1 -4*del1*del1*myy*myy + x1*x1*x1*x1

        zsquared = -T2/T1 if T1 != 0 else 0

        z = -math.sqrt(abs(zsquared))'''
