#include "ros/ros.h"
#include "vehicle_control/motorsMsg.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

/* Declare global variables to get the subscribed values */

/* p : position
 * q : quaternion
 * v : linear velocity
 * w : angular velocity
*/

#define PI 3.1415926536

double p[3];
double q[4];
double v[3];
double w[3];

double w_mot[4];

void ClbkOdom(const nav_msgs::Odometry::ConstPtr& odometry){
    p[0] = odometry -> pose.pose.position.x;
    p[1] = odometry -> pose.pose.position.y;
    p[2] = odometry -> pose.pose.position.z;

    q[0] = odometry -> pose.pose.orientation.x;
    q[1] = odometry -> pose.pose.orientation.y;
    q[2] = odometry -> pose.pose.orientation.z;
    q[3] = odometry -> pose.pose.orientation.w;

    v[0] = odometry -> twist.twist.linear.x;
    v[1] = odometry -> twist.twist.linear.y;
    v[2] = odometry -> twist.twist.linear.z;

    w[0] = odometry -> twist.twist.angular.x;
    w[1] = odometry -> twist.twist.angular.y;
    w[2] = odometry -> twist.twist.angular.z;

}

void ClbkMotion(const vehicle_control::motorsMsg::ConstPtr& motor_vel){
    w_mot[0] = motor_vel -> omega1;
    w_mot[1] = motor_vel -> omega2;
    w_mot[2] = motor_vel -> omega3;
    w_mot[3] = motor_vel -> omega4;
}


/* White Gaussian noise generator to reflect slip effect */
double AWGN_generator()
{
    double temp1;
    double temp2;
    double result;
    int p;

    p=1;

    while( p > 0 )
    {
        temp2 = ( rand() /((double) RAND_MAX));


        if( temp2 == 0){
            p = 1;
        }
        else
        {
            p = -1;
        }

    temp1 = cos( (-2.0 * (double)PI )* rand() /((double)RAND_MAX));
    result = sqrt(-2.0 * log(temp2)) * temp1;
    }
    return result;
}


int main(int argc,char **argv){
    ros::init(argc,argv,"odom_noise_publisher");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_1",100);

    ros::Subscriber odom_sub = nh.subscribe("/odom",100,ClbkOdom);
    ros::Subscriber motor_sub = nh.subscribe("/input",100,ClbkMotion);

    ros::Rate loop_rate(50);





    /* Get Previous Time*/
    double prev_t = 0;
    double dt = 0;
    double curr_t;

    double wheel_diameter = 0.152;
    double wheel_rad = wheel_diameter / 2.0;


    /* Declare array to get previous and recent motor velocity */
    double omega1[2] = {0,0};
    double omega2[2] = {0,0};
    double omega3[2] = {0,0};
    double omega4[2] = {0,0};

    /* Slip - White Gaussian noise for each wheel */
    double del1=0;
    double del2=0;
    double del3=0;
    double del4=0;


    /* velocity reflecting slip effect {x,y,phi}*/
    double vel_act[3] = {0,0,0};


    while(ros::ok())
    {
        nav_msgs::Odometry odom_data;


        /* Current time which is needed to get time duration */
        curr_t=ros::Time::now().toSec();
        dt = curr_t - prev_t;
        prev_t = curr_t;

        del1 = AWGN_generator();
        del2 = AWGN_generator();
        del3 = AWGN_generator();
        del4 = AWGN_generator();



        ROS_INFO("del1 : %lf, del2 : %lf, del3 : %lf, del4 : %lf ",del1,del2,del3,del4);

        vel_act[0] = wheel_rad / 4.0 * (w_mot[0] + w_mot[1]
                + w_mot[2] + w_mot[3]
                + abs(del1) + abs(del2)
                + abs(del3) + abs(del4));



        /* Noise is applied to odom data */
        odom_data.header.frame_id = "odom_1";

        odom_data.pose.pose.position.x = p[0];
        odom_data.pose.pose.position.y = p[1];
        odom_data.pose.pose.position.z = p[2];

        odom_data.pose.pose.orientation.x = q[0];
        odom_data.pose.pose.orientation.y = q[1];
        odom_data.pose.pose.orientation.z = q[2];
        odom_data.pose.pose.orientation.w = q[3];

        odom_data.twist.twist.linear.x = v[0];
        odom_data.twist.twist.linear.y = v[1];
        odom_data.twist.twist.linear.z = v[2];

        odom_data.twist.twist.angular.x = w[0];
        odom_data.twist.twist.angular.y = w[1];
        odom_data.twist.twist.angular.z = w[2];

        odom_pub.publish(odom_data);


        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

