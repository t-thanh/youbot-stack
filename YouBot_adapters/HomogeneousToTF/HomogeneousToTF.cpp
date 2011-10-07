#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>

std::string topic_name;


void toTransformMatrix(const std::vector<double> & tf, tf::Transform& trans) {
        trans.setOrigin(tf::Vector3(tf[3], tf[7], tf[11]));
        btMatrix3x3 rotMatrix(tf[0], tf[1], tf[2],
                        tf[4], tf[5], tf[6],
                        tf[8], tf[9], tf[10]);
        tf::Quaternion quat;
        rotMatrix.getRotation(quat);
        trans.setRotation(quat);

}

void adapterCallback(std_msgs::Float64MultiArrayConstPtr msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  toTransformMatrix(msg->data,transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", topic_name));

}



int main(int argc, char** argv){
  ros::init(argc, argv, "HomogeneousToTF");
  if (argc != 2){ROS_ERROR("need topic name as argument"); return -1;};
  topic_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(topic_name, 10, &adapterCallback);

  ros::spin();
  return 0;
};
