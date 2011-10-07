/*
 * H2TF_test.cpp
 *
 *  Created on: Oct 6, 2011
 *      Author: yury
 */

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "LinearMath/btMatrix3x3.h"
#include <sstream>
std::vector<double> XYZYRPtoHMatrix(double x, double y,double z,double yaw,double roll, double pitch)
{
  btMatrix3x3 rotMatrix;
  rotMatrix.setEulerYPR(btScalar(yaw), btScalar(pitch), btScalar(roll));
  std::vector<double> temp;
  temp.resize(16);
  for(int j=0;j<3;j++)
    for(int i=0;i<3;i++)
      temp[i+j*4]=rotMatrix[i][j];
  temp[3]=x;
  temp[7]=y;
  temp[11]=z;
  temp[15]=1;
  return temp;

 }
std::ostream& operator << (std::ostream & out,const std::vector<double>& data){
  for(int i=0;i<data.size();i++)
    {
    if (i%4==0) out<<"\n";
      out<<data[i]<<",\t";

    }
  return out;
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "H2TF_test");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("sample_H_flat_matrix", 1);
  while (ros::ok())
   {
  double x;
  double y;
  double z;
  double yaw;
  double roll;
  double pitch;
  std::cout<<"Please provide x,  y, z, yaw, roll,  pitch"<<std::endl;
  std::cin>> x>>  y>> z>> yaw>> roll>> pitch;
  std::cout<<"You gave x,  y, z, yaw, roll,  pitch"<<std::endl;
  std::cout<<"\t"<< x<<"\t"<<"\t"<< y<<"\t"<< z<<"\t"<< yaw<<"\t"<< roll<<"\t"<< pitch<<std::endl;
  std::cout<<XYZYRPtoHMatrix(x, y, z, yaw, roll,  pitch)<<std::endl;
  std_msgs::Float64MultiArray msg;
  msg.data= XYZYRPtoHMatrix(x, y, z, yaw, roll,  pitch);
  for(int i=0;i<10;i++)
    {
    ros::spinOnce();
    chatter_pub.publish(msg);
    sleep(1);
    }
 }
//    /**
//     * This is a message object. You stuff it with data, and then publish it.
//     */
//    std_msgs::String msg;
//
//    std::stringstream ss;
//    ss << "hello world " << count;
//    msg.data = ss.str();
//
//    ROS_INFO("%s", msg.data.c_str());
//
//    /**
//     * The publish() function is how you send messages. The parameter
//     * is the message object. The type of this object must agree with the type
//     * given as a template parameter to the advertise<>() call, as was done
//     * in the constructor above.
//     */
//    chatter_pub.publish(msg);
//
//    ros::spinOnce();
//
//    loop_rate.sleep();
//    ++count;
//  }


  return 0;
}
