#include "tf_publish.h"

//TODO: Check all values, verify physically
//Multisense
#define MULTISENSE_X 0.10
#define MULTISENSE_Y -0.10
#define MULTISENSE_Z 0.60

// // GPS_link
#define GPS_X 0.00
#define GPS_Y 0.00
#define GPS_Z 2.0

// //IMU
#define IMU_X 0.00
#define IMU_Y 0.00
#define IMU_Z 0.80

//sick  
#define SICK_X 0.5
#define SICK_Y 0.0
#define SICK_Z 0.15

#define PI 3.14159

geometry_msgs::QuaternionStamped quat;
geometry_msgs::PointStamped position;
nav_msgs::Odometry odom;

int got_data=0;

void publish_tf()
{
  tf::Transform transform;

  static tf::TransformBroadcaster br2;
  transform.setOrigin( tf::Vector3(SICK_X, SICK_Y, SICK_Z) );
  tf::Matrix3x3 mat1(0,1,0,-1,0,0,0,0,1);
  tf::Quaternion q2;
  mat1.getRotation(q2);
  transform.setRotation(q2);
  br2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sick_intermediate"));

  static tf::TransformBroadcaster br7;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  tf::Matrix3x3 mat6(1,0,0,0,0,1,0,-1,0);
  mat6.getRotation(q2);
  transform.setRotation(q2);
  br7.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "sick_intermediate", "camera_tof"));


  static tf::TransformBroadcaster br8;
  transform.setOrigin( tf::Vector3(MULTISENSE_X, MULTISENSE_Y, MULTISENSE_Z) );
  tf::Matrix3x3 mat7(0,1,0,-1,0,0,0,0,1);
  mat7.getRotation(q2);
  transform.setRotation(q2);
  br8.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "multisense_intermediate"));

  static tf::TransformBroadcaster br3;
  transform.setOrigin( tf::Vector3(0,0 ,0) );
  tf::Matrix3x3 mat2(1,0,0,0,0,1,0,-1,0);
  mat2.getRotation(q2);
  transform.setRotation(q2);
  br3.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "multisense_intermediate", "multisense/left_camera_optical_frame"));

  static tf::TransformBroadcaster br4;
  transform.setOrigin( tf::Vector3(GPS_X, GPS_Y, GPS_Z) );
  tf::Matrix3x3 mat3(1,0,0,0,1,0,0,0,1);
  mat3.getRotation(q2);
  transform.setRotation(q2);
  br4.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "gps_link"));


  static tf::TransformBroadcaster br5;
  transform.setOrigin( tf::Vector3(IMU_X, IMU_Y, IMU_Z) );
  tf::Matrix3x3 mat4(1,0,0,0,1,0,0,0,1);
  mat4.getRotation(q2);
  transform.setRotation(q2);
  br5.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", " mti_imu_link"));

  static tf::TransformBroadcaster br;
  transform.setOrigin( tf::Vector3(position.point.x,position.point.y,position.point.z) );
  tf::Quaternion q(quat.quaternion.x,quat.quaternion.y,quat.quaternion.z,quat.quaternion.w);
  transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "NWU_frame", "base_link"));

  static tf::TransformBroadcaster br6;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Matrix3x3 mat5(-1,0,0,0,-1,0,0,0,1);
  // tf::Matrix3x3 mat5(1,0,0,0,1,0,0,0,1);
  mat5.getRotation(q2);
  transform.setRotation(q2);
  br5.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_BRU", "base_link"));
}

void odom_callback(const nav_msgs::Odometry &msg)
{
  got_data=1;
  odom=msg;

  position.point.x= odom.pose.pose.position.x;
  position.point.y=odom.pose.pose.position.y;
  position.point.z=odom.pose.pose.position.z;

  quat.quaternion.x=odom.pose.pose.orientation.x;
  quat.quaternion.y=odom.pose.pose.orientation.y;
  quat.quaternion.z=odom.pose.pose.orientation.z;
  quat.quaternion.w=odom.pose.pose.orientation.w;

  tf::Quaternion q(quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w);
  tf::Matrix3x3 m(q);
  double rollImu, pitchImu, yawImu; // tilt angles
  m.getRPY(rollImu, pitchImu, yawImu);
  // ROS_INFO_STREAM("Roll:"<<rollImu*180.0/PI<<"Pitch:"<<pitchImu*180.0/PI<<"Yaw:"<<yawImu*180.0/PI);
}

int main(int argc, char** argv)
{
  int count=0;
  ros::init(argc, argv, "publish_tf");
  
  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("/wheel_encoder/odom", 10, &odom_callback);

  ros::Rate loop_rate(20);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();

    if(got_data==1)
    {
      publish_tf();
    }   

  }
  return 0;
}
