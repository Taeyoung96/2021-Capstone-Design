#include "ros/ros.h"
#include "omni_bot_msg/omnibot.h"
#include "sensor_msgs/Joy.h"

#define distance 0.318

float V_x = 0;
float V_y = 0;
float W = 0;

float vm_0 = 0,vm_1 = 0, vm_2 = 0;


void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //ROS_INFO("x axes [0] : [%f]  y axes [0] : [%f]", msg->axes[0],msg->axes[1]);
  V_x = msg->axes[1];
  V_y = msg->axes[0];
  W   = msg->axes[2];

  if(msg->axes[3]==-1)
  {
    V_x = 0.2;
  }
  if(msg->axes[4]==-1)
  {
    V_x = -0.2;
  }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "joycallback");
    ros::NodeHandle nh;

    ros::Subscriber sub2  = nh.subscribe("joy", 1000, joyCallback);
    ros::Publisher  pub   = nh.advertise<omni_bot_msg::omnibot>("CCR",1000);

    omni_bot_msg::omnibot msg;

    ros::Rate rate(50);
    while (ros::ok()){


        vm_0 = -0.866*V_x+0.5*V_y+distance*W;
        vm_1 = -V_y+distance*W;
        vm_2 = 0.866*V_x+0.5*V_y+distance*W;

        msg.Radian_0 = vm_0 * 10;
        msg.Radian_1 = vm_1 * 10;
        msg.Radian_2 = vm_2 * 10;

        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
}