// subpub.cpp inside
#include <stdlib.h>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "autominy_msgs/SpeedCommand.h"
#include "autominy_msgs/NormalizedSteeringCommand.h"

int angle1=10;
float length1 = 0.8;
bool signal1=false;
bool signal2=false;
//***************************************************************************
void cb(const sensor_msgs::LaserScan::ConstPtr & msg)
{

printf("r=%f\n",msg->ranges[45]);
for(uint16_t i=0;i<angle1;++i)
{
if((msg->ranges[i] )< length1)     {if(msg->ranges[45] > msg->ranges[270])signal1 = true; else signal2 =true; return;}
if((msg->ranges[359-i] )< length1) {if(msg->ranges[45] > msg->ranges[270])signal1 = true; else signal2 =true; return;}
} // end for loop



signal1=false;
signal2=false;
}
//***************************************************************************


int main(int argc, char **argv)
{

  ros::init(argc, argv, "subpub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/sensors/rplidar/scan", 1, cb);

ros::Publisher pub1 = n.advertise<autominy_msgs::NormalizedSteeringCommand>("/actuators/steering_normalized", 1000);
ros::Publisher pub2 = n.advertise<autominy_msgs::SpeedCommand>("/actuators/speed",1000);
  
 ros::Rate loop_rate(10);
 

 while(ros::ok() ){
autominy_msgs::NormalizedSteeringCommand msg1;
autominy_msgs::SpeedCommand msg2;

if(signal1)msg1.value =  1.0;
else {if(signal2)msg1.value = -1.0; else msg1.value = 0.0;}
msg2.value = 0.3;

pub1.publish(msg1);
//printf("send msg1 with value: %f \n",msg1.value);

pub2.publish(msg2);
//printf("send msg2 with value: %f \n",msg2.value);


ros::spinOnce();
loop_rate.sleep();
if(signal1)printf("1\n");else if(signal2) printf("2\n");
 }


  return 0;

}
