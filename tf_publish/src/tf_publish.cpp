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
nav_msgs::Odometry gps;

// Initialize Imu publisher, which will hold the heading information calculated in the callBack
ros::Publisher imu_pub;

int got_data=0;
int got_gps=0;

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
  br4.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "GPS_link"));


  static tf::TransformBroadcaster br5;
  transform.setOrigin( tf::Vector3(IMU_X, IMU_Y, IMU_Z) );
  tf::Matrix3x3 mat4(1,0,0,0,1,0,0,0,1);
  mat4.getRotation(q2);
  transform.setRotation(q2);
  br5.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", " mti_imu_link"));

  // static tf::TransformBroadcaster br;
  // transform.setOrigin( tf::Vector3(position.point.x,position.point.y,position.point.z) );
  // tf::Quaternion q(quat.quaternion.x,quat.quaternion.y,quat.quaternion.z,quat.quaternion.w);
  // transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "NWU_frame", "base_link"));

  // static tf::TransformBroadcaster br6;
  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Matrix3x3 mat5(-1,0,0,0,-1,0,0,0,1);
  // mat5.getRotation(q2);
  // transform.setRotation(q2);
  // br5.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_BRU", "base_link"));
  
  // static tf::TransformBroadcaster br9;
  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Matrix3x3 mat8(1,0,0,0,1,0,0,0,1);
  // mat8.getRotation(q2);
  // transform.setRotation(q2);
  // br9.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom/coarse_gps", "wheel_encoder/odom"));

  // static tf::TransformBroadcaster br10;
  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Matrix3x3 mat9(1,0,0,0,1,0,0,0,1);
  // mat9.getRotation(q2);
  // transform.setRotation(q2);
  // br10.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "/odom/coarse_gps"));

  // static tf::TransformBroadcaster br11;
  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Matrix3x3 mat10(1,0,0,0,1,0,0,0,1);
  // mat10.getRotation(q2);
  // transform.setRotation(q2);
  // br11.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "/wheel_encoder/odom"));
}

// void odom_callback(const nav_msgs::Odometry &msg)
// {
//   got_data=1;
//   odom=msg;

//   tf::TransformBroadcaster odom_broadcaster;

//   //first, we'll publish the transform over tf
//   geometry_msgs::TransformStamped odom_trans;
//   odom_trans.header.stamp = ros::Time::now();;
//   odom_trans.header.frame_id = "wheel_encoder/odom";
//   odom_trans.child_frame_id = "base_link";

//   odom_trans.transform.translation.x = odom.pose.pose.position.x;
//   odom_trans.transform.translation.y = odom.pose.pose.position.y;
//   odom_trans.transform.translation.z = odom.pose.pose.position.z;
//   odom_trans.transform.rotation = odom.pose.pose.orientation;

//   // send the transform
//   odom_broadcaster.sendTransform(odom_trans);
// }

void gps_callback(const nav_msgs::Odometry &msg)
{
  got_gps=1;
  gps=msg;

  position.point.x=gps.pose.pose.position.x;
  position.point.y=gps.pose.pose.position.y;
  position.point.z=gps.pose.pose.position.z;

  quat.quaternion.x=gps.pose.pose.orientation.x;
  quat.quaternion.y=gps.pose.pose.orientation.y;
  quat.quaternion.z=gps.pose.pose.orientation.z;
  quat.quaternion.w=gps.pose.pose.orientation.w;


  // tf::Quaternion q(quat.quaternion.x, quat.quaternion.y, quat.quaternion.z, quat.quaternion.w);
  // tf::Matrix3x3 m(q);
  // double rollImu, pitchImu, yawImu; // tilt angles
  // m.getRPY(rollImu, pitchImu, yawImu);
  // ROS_ERROR_STREAM("Roll:"<<rollImu*180.0/PI<<"Pitch:"<<pitchImu*180.0/PI<<"Yaw:"<<yawImu*180.0/PI);

}

void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  // Get roll and pitch from xsens IMU
	double quat_x = imu_msg->orientation.x;
  double quat_y = imu_msg->orientation.y;
  double quat_z = imu_msg->orientation.z;
  double quat_w = imu_msg->orientation.w;
  tf::Quaternion q(quat_x, quat_y, quat_z, quat_w);
  tf::Matrix3x3 m(q);
  double rollImu, pitchImu, yawImu; // tilt angles
  m.getRPY(rollImu, pitchImu, yawImu);

	// Yaw is being calculated right now in 
	// yawImu = yawImu + 3.14159;

  // TODO HACK change this hardcoded number 65 deg from east is heading where we started during the NSH test
  yawImu = yawImu + 1.48353;//1.13446;

  // ROS_ERROR_STREAM("Roll:"<<rollImu*180.0/PI<<"Pitch:"<<pitchImu*180.0/PI<<"Yaw:"<<yawImu*180.0/PI);

  // Populate and publish message
	sensor_msgs::Imu new_imu_msg;

	new_imu_msg.header.frame_id = "mti_imu_link";
	new_imu_msg.header.stamp = ros::Time::now();

	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(rollImu, pitchImu, yawImu);

	new_imu_msg.orientation = pose_quat;

	new_imu_msg.orientation_covariance[0] = 1e-6;
	new_imu_msg.orientation_covariance[4] = 1e-6;
	new_imu_msg.orientation_covariance[8] = 1e-6;

  // new_imu_msg.orientation_covariance=imu_msg->orientation_covariance;
	new_imu_msg.angular_velocity=imu_msg->angular_velocity;
	new_imu_msg.angular_velocity_covariance=imu_msg->angular_velocity_covariance;
	new_imu_msg.linear_acceleration=imu_msg->linear_acceleration;
	new_imu_msg.linear_acceleration_covariance=imu_msg->linear_acceleration_covariance;

	imu_pub.publish(new_imu_msg);
}

// void filtered_odom_cb(const nav_msgs::Odometry &msg)
// {
//   tf::TransformBroadcaster odom_broadcaster;

//   //first, we'll publish the transform over tf
//   geometry_msgs::TransformStamped odom_trans;
//   odom_trans.header.stamp = ros::Time::now();;
//   odom_trans.header.frame_id = "odom/coarse_gps";
//   odom_trans.child_frame_id = "base_link";

//   odom_trans.transform.translation.x = msg.pose.pose.position.x;
//   odom_trans.transform.translation.y = msg.pose.pose.position.y;
//   odom_trans.transform.translation.z = msg.pose.pose.position.z;
//   odom_trans.transform.rotation = msg.pose.pose.orientation;

//   // send the transform
//   odom_broadcaster.sendTransform(odom_trans);
// }

int main(int argc, char** argv)
{
  int count=0;
  ros::init(argc, argv, "publish_tf");
  
  ros::NodeHandle n;

  imu_pub = n.advertise<sensor_msgs::Imu>("imu_yaw_corrected", 1, false);

  ros::Subscriber imu_sub = n.subscribe("/mti/sensor/imu", 10, &imu_callback);
  // ros::Subscriber gps_sub = n.subscribe("/odometry/coarse_gps", 10, &gps_callback);
  // ros::Subscriber filtered_sub = n.subscribe("/odometry/coarse_gps", 10, &filtered_odom_cb);

  ros::Rate loop_rate(20);
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
      publish_tf();
  }
  return 0;
}
