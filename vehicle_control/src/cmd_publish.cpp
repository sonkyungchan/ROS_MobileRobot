#include "ros/ros.h"
#include "vehicle_control/commendMsg.h"
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"cmd_publisher");
  ros::NodeHandle nh;

  ros::Publisher cmd_pub=nh.advertise<vehicle_control::commendMsg>("/ns1/cmd_msg",100); //100 que size//

  ros::Rate loop_rate(100); // Setting 50 Hz //

/*
  int N=1000;

  double xd[N];
  double yd[N];
  double phid[N];


  for(int i=0;i<N;i++)
  {
      xd[i] = 1 * cos( (double) 2.0*3.141592/N*i);
		///(1+sin( (double) 2.0*3.141592/N*i)*sin( (double) 2.0*3.141592/N*i));
      yd[i] = 1 * sin( (double) 2.0*3.141592/N*i); //*cos( (double) 2.0*3.141592/N*i)
		///(1+sin( (double) 2.0*3.141592/N*i)*sin( (double) 2.0*3.141592/N*i));
      phid[i] = 1.5 * sin( (double) 2.0*3.141592/N*i);

  }
  int j=0;

*/
  while(ros::ok())
  {

/*
    vehicle_control::commendMsg cmd_msg;

    cmd_msg.xd = xd[j];
    cmd_msg.yd = yd[j];
    cmd_msg.phid = phid[j];


    cmd_pub.publish(cmd_msg);


    ROS_INFO("Pub Msg : %lf %lf %lf",cmd_msg.xd,cmd_msg.yd,cmd_msg.phid);
*/
    loop_rate.sleep();
/*

    j++;

    if(j==N)
    {

        j=0;
    }

*/
  }

  return 0;

}

