#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "../libraries/compatibility.h"
#include "../libraries/bdaqctrl.h"
#include <fstream>
#include <sys/time.h>

#include <unistd.h>

#include "ros/ros.h"

#include "std_msgs/String.h"
#include <ros/package.h>

#include <sstream>
#include <typeinfo>

#include "advantech_pci1714_driver/Ping_received.h"
#include "advantech_pci1714_driver/Ping.h"
#include "advantech_pci1714_driver/Pingdata.h"

using namespace Automation::BDaq;
using namespace std;

//-----------------------------------------------------------------------------------
// Configure the following three parameters before running the sample
#define      deviceDescription  L"PCI-1714UL,BID#15"
int32        startChannel = 0;
const int32  channelCount = 4;
const int32  intervalCount = 64; 

double        samplingFrequency = 2000000;  //in Hz
//float         sampleCount_dec = samplingFrequency*0.00330188679;
const int32  sampleCount =  4000;//int32(sampleCount_dec)*channelCount; //2048   // for each channel, to decide the capacity of buffer in kernel.

#define      SECTION_BUFFERE_SIZE   intervalCount*channelCount
#define		 USER_BUFFER_SIZE    sampleCount*channelCount
double       Data[USER_BUFFER_SIZE];

// Set trigger parameters
TriggerAction triggerAction = DelayToStop;
ActiveSignal  triggerEdge = RisingEdge;

double        triggerLevel = 0.06; //0.06 determined experimentally from oscope;
//double        triggerLevel = 1.5;  //1.5V to overcome pool pump
int           triggerDelayCount = sampleCount/2.0;//1.25;

BufferedAiCtrl * bfdAiCtrl = AdxBufferedAiCtrlCreate();

// set which trigger be used for this demo, trigger0(0) or trigger1(1).
int           triggerUsed = 0;

inline void waitAnyKey()
{
	do{SLEEP(1);} while(!kbhit());
} 

class StoppedHandler : public BfdAiEventListener
{
public:
	BufferedAiCtrl * bufferedAiCtrl; //= (BufferedAiCtrl*)sender;

	virtual void BDAQCALL BfdAiEvent(void * sender, BfdAiEventArgs * args)
	{
		//auto now = high_resolution_clock::now();
		//double diff = duration_cast<microseconds>(now.time_since_epoch()-start.time_since_epoch()).count();;
		
		bufferedAiCtrl = (BufferedAiCtrl*)sender;
		printf("Buffered AI data ready !\n");

		int32 channelCountMax = bufferedAiCtrl->getFeatures()->getChannelCountMax();
		int32 chanStart = bufferedAiCtrl->getScanChannel()->getChannelStart();
		int32 chanCount = bufferedAiCtrl->getScanChannel()->getChannelCount();
		bufferedAiCtrl->GetData(args->Count, Data);

		int delayCount = 0;
      if (triggerUsed == 0 && bufferedAiCtrl->getFeatures()->getTriggerSupported())
      {
         delayCount = bufferedAiCtrl->getTrigger()->getDelayCount();
      
		int triggerPointIndex = args->Count/chanCount - delayCount;
		std::cout << "triggerPointIndex: " << triggerPointIndex << std::endl;
		printf("The data count each channel:%d,  trigger point each channel: %d\n\n",args->Count/chanCount,triggerPointIndex );

		ofstream chan1data;
		//auto str = std::to_string(diff); 
		//auto samplingrate = std::to_string(samplingFrequency); 
		//string data_file1 = "Data/" + samplingrate + "_" + ".csv";

		//chan1data.open (data_file1);
		//chan1data << "Channel 1" << "\t" << "Channel 2" << endl;

		for(int32 i = 0; i < chanCount; ++i)
		{
			for(int z = 0; z <= sampleCount ; z+=channelCount){
				//chan1data << Data[z] << "\t";
				//chan1data << Data[z+1] << endl;
				//chan1data << Data[z+2] << "\t";
				//chan1data << Data[z+3] << endl; //"%10.6f \n" % Data[z];
			}
			//chan1data.close();
		}
		printf("completed sample\n");	

		ros::NodeHandle n;

		ros::Publisher pingpub = n.advertise<advantech_pci1714_driver::Pingdata>("/hydrophones/pingraw",1000);
		advantech_pci1714_driver::Pingdata msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/world";
		msg.channels = channelCount;
		msg.samples = USER_BUFFER_SIZE;
		msg.sample_rate = samplingFrequency;
		msg.adc_bit = 12;
		std::vector<double> v(Data, Data + sizeof Data / sizeof Data[0]);
		msg.data = v;

		pingpub.publish(msg);
		//cout << v << endl;
		//printf("%i",msg.adc_bit);
		//ROS_INFO("%f",msg.data;
			

		ros::ServiceClient client = n.serviceClient<advantech_pci1714_driver::Ping_received>("/hydrophones/ready");
		advantech_pci1714_driver::Ping_received srv;
		srv.request;
		client.call(srv);	

	  } 
	  //delete bufferedAiCtrl;
	  stop_trigger();
	  printf("SLEEPING\n");
	  usleep(500000);  //Add delay to ensure we don't pick up same signal twice
	  printf("READY\n");
	  set_trigger();
	  return;
	}

	void stop_trigger()
	{
		ErrorCode ret = Success;
		bufferedAiCtrl->Stop();  		
	}

	void set_trigger()
	{
		ErrorCode ret = Success;
		
		do{
			ret = bfdAiCtrl->Prepare();
			CHK_RESULT(ret);

			printf("SETTRIGGER\n");

			ret = bfdAiCtrl->RunOnce();
			CHK_RESULT(ret);

		}while(false);
		return;
	}	
};

bool ping_publish(advantech_pci1714_driver::Ping::Request &req,
				  advantech_pci1714_driver::Ping::Response &res)
{
	res.channels = channelCount;
	res.samples = USER_BUFFER_SIZE;
	res.sample_rate = samplingFrequency;
	res.adc_bit = 12;
	std::vector<double> v(Data, Data + sizeof Data / sizeof Data[0]);
	res.data = v;

	return true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "advantechDriver_node");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("/hydrophones/ping", ping_publish);
	ros::Publisher pingpub = n.advertise<advantech_pci1714_driver::Pingdata>("/hydrophones/pingraw",1);
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


	/*
	if(client.call(srv))
	{
		ROS_INFO("Client called ping_received");
	}
	else
	{	
		ROS_ERROR("Unable to contact /hydrophones/ready");
		return 1;
	}*/

	//ros::ServiceServer service = n.advertiseService("/hydrophones/ping_received", ping_received);

	ErrorCode ret = Success;

	StoppedHandler onStopped;

	bfdAiCtrl->addStoppedListener(onStopped);

	do
	{
		DeviceInformation devInfo(deviceDescription);
		ret = bfdAiCtrl->setSelectedDevice(devInfo);
		CHK_RESULT(ret);
		ret = bfdAiCtrl->setStreaming(false);// specify the running mode: one-buffered.
		CHK_RESULT(ret);

		ScanChannel* scanChannel = bfdAiCtrl->getScanChannel();
		ret = scanChannel->setChannelStart(startChannel);
		CHK_RESULT(ret);
		ret = scanChannel->setChannelCount(channelCount);
		CHK_RESULT(ret);
		ret = scanChannel->setSamples(sampleCount);
		CHK_RESULT(ret);

		ret = scanChannel->setIntervalCount(intervalCount);
		CHK_RESULT(ret);

		//double setBurnoutRetValue(10);
		//bfdAiCtrl -> setValueRange(5);
		
		// Step 4.1: Set sampling frequency
		ConvertClock * convertClock = bfdAiCtrl->getConvertClock();		
		ret = convertClock->setRate(samplingFrequency);
		CHK_RESULT(ret);

		//CountingPulseByDevTime

		//Step 5: Trigger parameters setting
		Trigger* trigger = bfdAiCtrl->getTrigger();
		if (trigger != NULL)
		{
			ret = trigger->setAction(triggerAction);
			CHK_RESULT(ret);
			ICollection<SignalDrop>*  srcs = bfdAiCtrl->getFeatures()->getTriggerSources();
			ret = trigger->setSource(srcs->getItem(1));
			CHK_RESULT(ret);
			ret = trigger->setDelayCount(triggerDelayCount) ;
			CHK_RESULT(ret);
			ret = trigger->setEdge(triggerEdge);
			CHK_RESULT(ret);
			ret = trigger->setLevel(triggerLevel);
			CHK_RESULT(ret);
		}

		// Step 6: prepare the buffered AI. 
		ret = bfdAiCtrl->Prepare();
		CHK_RESULT(ret);

		printf("Asynchronous finite acquisition is in progress.\n");
		printf("Please wait... any key to quit !\n\n");

		//ret = bfdAiCtrl->Start();
		ret = bfdAiCtrl->RunOnce();
		CHK_RESULT(ret);

		ros::Rate loop_rate(10);
	    while (ros::ok())
	    {

		    ros::spinOnce();

		    loop_rate.sleep();

		}

		// step 9: stop the operation if it is running.
		ret = bfdAiCtrl->Stop(); 
		CHK_RESULT(ret);
		cout << "STOPPED PROGRAM" << endl;

		// Step 10: close device, release any allocated resource before quit.
		bfdAiCtrl->Dispose();
	}
	while(false);		

	// If something wrong in this execution, print the error code on screen for tracking.
	if(BioFailed(ret))
	{
		printf("Some error occurred. And the last error code is Ox%X.\n", ret);
		waitAnyKey();// wait any key to quit!
	}

  return 0;
}



/*



  ros::init(argc, argv, "advantechDriver");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  */

/*
using namespace std::chrono;
using namespace Automation::BDaq;
using namespace std;
//-----------------------------------------------------------------------------------
// Configure the following three parameters before running the sample
#define      deviceDescription  L"PCI-1714UL,BID#15"
int32        startChannel = 1;
const int32  channelCount = 2;
const int32  intervalCount = 64; 

double        samplingFrequency = 5000000;  //in Hz
//float         sampleCount_dec = samplingFrequency*0.00330188679;
const int32  sampleCount =  40000;//int32(sampleCount_dec)*channelCount; //2048   // for each channel, to decide the capacity of buffer in kernel.

#define       SECTION_BUFFERE_SIZE   intervalCount*channelCount
#define		 USER_BUFFER_SIZE    sampleCount*channelCount
double       Data[USER_BUFFER_SIZE];



// Set trigger parameters
TriggerAction triggerAction = DelayToStop;
ActiveSignal  triggerEdge = FallingEdge;

double        triggerLevel = -0.1;


int           triggerDelayCount = sampleCount/1.25;

auto start = high_resolution_clock::now();



BufferedAiCtrl * bfdAiCtrl = AdxBufferedAiCtrlCreate();

// set which trigger be used for this demo, trigger0(0) or trigger1(1).
int           triggerUsed = 0;

inline void waitAnyKey()
{
	do{SLEEP(1);} while(!kbhit());
} 

// This class is used to deal with 'DataReady' Event, we should overwrite the virtual function BfdAiEvent.
//class Channel1 : public BfdAiEventListener
class StoppedHandler : public BfdAiEventListener
{
public:
	BufferedAiCtrl * bufferedAiCtrl; //= (BufferedAiCtrl*)sender;

	virtual void BDAQCALL BfdAiEvent(void * sender, BfdAiEventArgs * args)
	{
		auto now = high_resolution_clock::now();
		double diff = duration_cast<microseconds>(now.time_since_epoch()-start.time_since_epoch()).count();;
		
		bufferedAiCtrl = (BufferedAiCtrl*)sender;
		printf("Buffered AI data ready !\n");

		int32 channelCountMax = bufferedAiCtrl->getFeatures()->getChannelCountMax();
		int32 chanStart = bufferedAiCtrl->getScanChannel()->getChannelStart();
		int32 chanCount = bufferedAiCtrl->getScanChannel()->getChannelCount();
		bufferedAiCtrl->GetData(args->Count, Data);

		int delayCount = 0;
      if (triggerUsed == 0 && bufferedAiCtrl->getFeatures()->getTriggerSupported())
      {
         delayCount = bufferedAiCtrl->getTrigger()->getDelayCount();
      
		int triggerPointIndex = args->Count/chanCount - delayCount;
		std::cout << "triggerPointIndex: " << triggerPointIndex << std::endl;
		printf("The data count each channel:%d,  trigger point each channel: %d\n\n",args->Count/chanCount,triggerPointIndex );

		ofstream chan1data;
		auto str = std::to_string(diff); 
		auto samplingrate = std::to_string(samplingFrequency); 
		string data_file1 = "Data/" + samplingrate + "_" + str + ".csv";

		chan1data.open (data_file1);
		chan1data << "Channel 1" << "\t" << "Channel 2" << endl;

		for(int32 i = 0; i < chanCount; ++i)
		{
			for(int z = 0; z <= sampleCount ; z+=channelCount){
				chan1data << Data[z] << "\t";
				chan1data << Data[z+1] << endl;
				//chan1data << Data[z+2] << "\t";
				//chan1data << Data[z+3] << endl; //"%10.6f \n" % Data[z];
			}
			chan1data.close();
		}
		printf("completed sample\n");
	  } 
	  //delete bufferedAiCtrl;
	  stop_trigger();
	  set_trigger();
	  return;
	}

	void stop_trigger()
	{
		ErrorCode ret = Success;
		bufferedAiCtrl->Stop();  
		//bufferedAiCtrl->Cleanup();
			//CHK_RESULT(ret);
			
		
	}

	void set_trigger()
	{
		ErrorCode ret = Success;
		

		//StoppedHandler onStopped;
		//bfdAiCtrl->addStoppedListener(onStopped);	

		do{
			ret = bfdAiCtrl->Prepare();
			CHK_RESULT(ret);

			// Step 7: start Asynchronous Buffered AI, 'Asynchronous' means the method returns immediately
			// after the acquisition has been started. The StoppedHandler's 'BfdAiEvent' method will be called
			// after the acquisition is completed.
			printf("SETTRIGGER\n");
			//printf("Please wait... any key to quit !\n\n");

			//ret = bfdAiCtrl->Start();
			ret = bfdAiCtrl->RunOnce();
			CHK_RESULT(ret);

		}while(false);
		return;
	}	
};

int main(int argc, char* argv[])
{

	cout << "sampleCount: " << sampleCount << endl;

	ErrorCode ret = Success;

	// Step 1: Create a 'BufferedAiCtrl' for buffered AI function.
	//BufferedAiCtrl * bfdAiCtrl = AdxBufferedAiCtrlCreate();
	//bufferedAiCtrl->setSelectedDevice(devinfo)

	// Step 2: Set the notification event Handler by which we can known the state of operation effectively.
	//DataReadyHandler onDataReady;
	//OverrunHandler       onOverrun;
	//CacheOverflowHandler onCacheOverflow;
	StoppedHandler onStopped;
	//bfdAiCtrl->addDataReadyListener(onDataReady);
	//bfdAiCtrl->addOverrunListener(onOverrun);
	//bfdAiCtrl->addCacheOverflowListener(onCacheOverflow);
	bfdAiCtrl->addStoppedListener(onStopped);

	DeviceInformation devInfo(deviceDescription);
	ret = bfdAiCtrl->setSelectedDevice(devInfo);
	//CHK_RESULT(ret);
	ret = bfdAiCtrl->setStreaming(false);// specify the running mode: one-buffered.
	//CHK_RESULT(ret);

	do
	{
		// Step 3: Select a device by device number or device description and specify the access mode.
		// in this example we use AccessWriteWithReset(default) mode so that we can 
		// fully control the device, including configuring, sampling, etc.


		// Step 4: Set necessary parameters for Buffered AI operation, 
		// Note: some of operation of this step is optional(you can do these settings via "Device Configuration" dialog).
		ScanChannel* scanChannel = bfdAiCtrl->getScanChannel();
		ret = scanChannel->setChannelStart(startChannel);
		CHK_RESULT(ret);
		ret = scanChannel->setChannelCount(channelCount);
		CHK_RESULT(ret);
		ret = scanChannel->setSamples(sampleCount);
		CHK_RESULT(ret);

		ret = scanChannel->setIntervalCount(intervalCount);
		CHK_RESULT(ret);

		
		// Step 4.1: Set sampling frequency
		ConvertClock * convertClock = bfdAiCtrl->getConvertClock();		
		ret = convertClock->setRate(samplingFrequency);
		CHK_RESULT(ret);

		//CountingPulseByDevTime

		//Step 5: Trigger parameters setting
		Trigger* trigger = bfdAiCtrl->getTrigger();
		if (trigger != NULL)
		{
			ret = trigger->setAction(triggerAction);
			CHK_RESULT(ret);
			ICollection<SignalDrop>*  srcs = bfdAiCtrl->getFeatures()->getTriggerSources();
			ret = trigger->setSource(srcs->getItem(1));
			CHK_RESULT(ret);
			ret = trigger->setDelayCount(triggerDelayCount) ;
			CHK_RESULT(ret);
			ret = trigger->setEdge(triggerEdge);
			CHK_RESULT(ret);
			ret = trigger->setLevel(triggerLevel);
			CHK_RESULT(ret);
		}

		// Step 6: prepare the buffered AI. 
		ret = bfdAiCtrl->Prepare();
		CHK_RESULT(ret);

		// Step 7: start Asynchronous Buffered AI, 'Asynchronous' means the method returns immediately
		// after the acquisition has been started. The StoppedHandler's 'BfdAiEvent' method will be called
		// after the acquisition is completed.
		printf("Asynchronous finite acquisition is in progress.\n");
		printf("Please wait... any key to quit !\n\n");

		//ret = bfdAiCtrl->Start();
		ret = bfdAiCtrl->RunOnce();
		CHK_RESULT(ret);

		// Step 8: Do anything you are interesting while the device is acquiring data.
		do
		{
		
			cout << "do loop loop loop" << endl;	
			SLEEP(1);
		}while(!kbhit());

		// step 9: stop the operation if it is running.
		ret = bfdAiCtrl->Stop(); 
		CHK_RESULT(ret);
		cout << "STOPPED PROGRAM" << endl;
	}
	while(false);

	// Step 10: close device, release any allocated resource before quit.
	bfdAiCtrl->Dispose();

	// If something wrong in this execution, print the error code on screen for tracking.
	if(BioFailed(ret))
	{
		printf("Some error occurred. And the last error code is Ox%X.\n", ret);
		waitAnyKey();// wait any key to quit!
	}

	return 0;   
}

*/
