#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
ros::Publisher rc1_speed_pub;
ros::Publisher rc2_speed_pub;
ros::Publisher rc3_speed_pub;
ros::Subscriber tf_sub;
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry>
    MySyncPolicy;
Eigen::Quaterniond q_rc1;
Eigen::Quaterniond q_rc2;
Eigen::Quaterniond q_rc3;
Eigen::Vector3d t_rc1;
Eigen::Vector3d t_rc2;
Eigen::Vector3d t_rc3;
bool initialized = false;
Eigen::Vector3d rc1_prev_pt;
Eigen::Vector3d rc2_prev_pt;
Eigen::Vector3d rc3_prev_pt;
double frequency = 5; // Hz

void tf_callback(const tf::tfMessageConstPtr tf_msg) {
  for (int i = 0; i < tf_msg->transforms.size(); i++) {
    if (tf_msg->transforms[i].header.frame_id == "global" &&
        tf_msg->transforms[i].child_frame_id == "cmu_rc1_map") {
      q_rc1.x() = tf_msg->transforms[i].transform.rotation.x;
      q_rc1.y() = tf_msg->transforms[i].transform.rotation.y;
      q_rc1.z() = tf_msg->transforms[i].transform.rotation.z;
      q_rc1.w() = tf_msg->transforms[i].transform.rotation.w;
      t_rc1(0) = tf_msg->transforms[i].transform.translation.x;
      t_rc1(1) = tf_msg->transforms[i].transform.translation.y;
      t_rc1(2) = tf_msg->transforms[i].transform.translation.z;
    }
    if (tf_msg->transforms[i].header.frame_id == "global" &&
        tf_msg->transforms[i].child_frame_id == "cmu_rc2_map") {
      q_rc2.x() = tf_msg->transforms[i].transform.rotation.x;
      q_rc2.y() = tf_msg->transforms[i].transform.rotation.y;
      q_rc2.z() = tf_msg->transforms[i].transform.rotation.z;
      q_rc2.w() = tf_msg->transforms[i].transform.rotation.w;
      t_rc2(0) = tf_msg->transforms[i].transform.translation.x;
      t_rc2(1) = tf_msg->transforms[i].transform.translation.y;
      t_rc2(2) = tf_msg->transforms[i].transform.translation.z;
    }
    if (tf_msg->transforms[i].header.frame_id == "global" &&
        tf_msg->transforms[i].child_frame_id == "cmu_rc3_map") {
      q_rc3.x() = tf_msg->transforms[i].transform.rotation.x;
      q_rc3.y() = tf_msg->transforms[i].transform.rotation.y;
      q_rc3.z() = tf_msg->transforms[i].transform.rotation.z;
      q_rc3.w() = tf_msg->transforms[i].transform.rotation.w;
      t_rc3(0) = tf_msg->transforms[i].transform.translation.x;
      t_rc3(1) = tf_msg->transforms[i].transform.translation.y;
      t_rc3(2) = tf_msg->transforms[i].transform.translation.z;
    }
  }
  initialized = true;
}

void transform_rc1_to_darpa(const Eigen::Vector3d &pt,
                            Eigen::Vector3d &transformed, ros::Time stamp) {

  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translate(t_rc1);
  transformation.rotate(q_rc1);
  transformed = (transformation)*pt;
}

void transform_rc2_to_darpa(const Eigen::Vector3d &pt,
                            Eigen::Vector3d &transformed) {

  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translate(t_rc2);
  transformation.rotate(q_rc2);
  transformed = (transformation)*pt;
}

void transform_rc3_to_darpa(const Eigen::Vector3d &pt,
                            Eigen::Vector3d &transformed) {

  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translate(t_rc3);
  transformation.rotate(q_rc3);
  transformed = (transformation)*pt;
}

void callback(const nav_msgs::OdometryConstPtr rc1_odom,
              const nav_msgs::OdometryConstPtr rc2_odom,
              const nav_msgs::OdometryConstPtr rc3_odom) {
  if (!initialized) {
    return;
  }
  Eigen::Vector3d rc1_darpa_pt;
  transform_rc1_to_darpa(Eigen::Vector3d(rc1_odom->pose.pose.position.x,
                                         rc1_odom->pose.pose.position.y,
                                         rc1_odom->pose.pose.position.z),
                         rc1_darpa_pt, rc1_odom->header.stamp);
  Eigen::Vector3d rc2_darpa_pt;
  transform_rc2_to_darpa(Eigen::Vector3d(rc2_odom->pose.pose.position.x,
                                         rc2_odom->pose.pose.position.y,
                                         rc2_odom->pose.pose.position.z),
                         rc2_darpa_pt);
  Eigen::Vector3d rc3_darpa_pt;
  transform_rc3_to_darpa(Eigen::Vector3d(rc3_odom->pose.pose.position.x,
                                         rc3_odom->pose.pose.position.y,
                                         rc3_odom->pose.pose.position.z),
                         rc3_darpa_pt);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "global";
  marker.header.stamp = rc1_odom->header.stamp;
  marker.ns = "lines_between_rc1_and_rc2";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.25;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.points.resize(2);
  marker.points[0].x = rc1_darpa_pt(0);
  marker.points[0].y = rc1_darpa_pt(1);
  marker.points[0].z = rc1_darpa_pt(2);
  marker.points[1].x = rc2_darpa_pt(0);
  marker.points[1].y = rc2_darpa_pt(1);
  marker.points[1].z = rc2_darpa_pt(2);

  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "global";
  marker2.header.stamp = rc2_odom->header.stamp;
  marker2.ns = "lines_between_rc2_and_rc3";
  marker2.id = 1;
  marker2.type = visualization_msgs::Marker::LINE_LIST;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = 0.25;
  marker2.color.b = 1.0;
  marker2.color.a = 1.0;
  marker2.points.resize(2);
  marker2.points[0].x = rc2_darpa_pt(0);
  marker2.points[0].y = rc2_darpa_pt(1);
  marker2.points[0].z = rc2_darpa_pt(2);
  marker2.points[1].x = rc3_darpa_pt(0);
  marker2.points[1].y = rc3_darpa_pt(1);
  marker2.points[1].z = rc3_darpa_pt(2);

  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "global";
  marker3.header.stamp = rc3_odom->header.stamp;
  marker3.ns = "lines_between_rc3_and_rc1";
  marker3.id = 2;
  marker3.type = visualization_msgs::Marker::LINE_LIST;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.pose.orientation.w = 1.0;
  marker3.scale.x = 0.25;
  marker3.color.g = 1.0;
  marker3.color.a = 1.0;
  marker3.points.resize(2);
  marker3.points[0].x = rc3_darpa_pt(0);
  marker3.points[0].y = rc3_darpa_pt(1);
  marker3.points[0].z = rc3_darpa_pt(2);
  marker3.points[1].x = rc1_darpa_pt(0);
  marker3.points[1].y = rc1_darpa_pt(1);
  marker3.points[1].z = rc1_darpa_pt(2);

  marker_pub.publish(marker);
  marker_pub.publish(marker2);
  marker_pub.publish(marker3);

  // publish speeds
  std_msgs::Float32 rc1_speed;
  rc1_speed.data = (rc1_darpa_pt - rc1_prev_pt).norm() / (1 / frequency);
  rc1_speed_pub.publish(rc1_speed);
  std_msgs::Float32 rc2_speed;
  rc2_speed.data = (rc2_darpa_pt - rc2_prev_pt).norm() / (1 / frequency);
  rc2_speed_pub.publish(rc2_speed);
  std_msgs::Float32 rc3_speed;
  rc3_speed.data = (rc3_darpa_pt - rc3_prev_pt).norm() / (1 / frequency);
  rc3_speed_pub.publish(rc3_speed);
  rc1_prev_pt(0) = rc1_darpa_pt(0);
  rc1_prev_pt(1) = rc1_darpa_pt(1);
  rc1_prev_pt(2) = rc1_darpa_pt(2);
  rc2_prev_pt(0) = rc2_darpa_pt(0);
  rc2_prev_pt(1) = rc2_darpa_pt(1);
  rc2_prev_pt(2) = rc2_darpa_pt(2);
  rc3_prev_pt(0) = rc3_darpa_pt(0);
  rc3_prev_pt(1) = rc3_darpa_pt(1);
  rc3_prev_pt(2) = rc3_darpa_pt(2);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle nh;
  marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  rc1_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc1/speed_custom", 10);
  rc2_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc2/speed_custom", 10);
  rc3_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc3/speed_custom", 10);
  tf_sub = nh.subscribe("/tf", 10, tf_callback);
  message_filters::Subscriber<nav_msgs::Odometry> rc1_odom_sub(
      nh, "/cmu_rc1/aft_mapped_to_init_imu", 10);
  message_filters::Subscriber<nav_msgs::Odometry> rc2_odom_sub(
      nh, "/cmu_rc2/aft_mapped_to_init_imu", 10);
  message_filters::Subscriber<nav_msgs::Odometry> rc3_odom_sub(
      nh, "/cmu_rc3/aft_mapped_to_init_imu", 10);

  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), rc1_odom_sub, rc2_odom_sub, rc3_odom_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();
  return -1;
}