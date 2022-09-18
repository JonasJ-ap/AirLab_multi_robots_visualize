#include "super_odometry_msgs/OptimizationStats.h"
#include <jsk_rviz_plugins/OverlayText.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
ros::Publisher rc1_speed_pub;
ros::Publisher rc2_speed_pub;
ros::Publisher rc3_speed_pub;
ros::Subscriber tf_sub;
ros::Publisher rc1_status_pub;
ros::Publisher rc2_status_pub;
ros::Publisher rc3_status_pub;
ros::Publisher rc1_status_color_pub;
ros::Publisher rc2_status_color_pub;
ros::Publisher rc3_status_color_pub;
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, nav_msgs::Odometry, nav_msgs::Odometry,
    super_odometry_msgs::OptimizationStats,
    super_odometry_msgs::OptimizationStats,
    super_odometry_msgs::OptimizationStats>
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
int marker_id = 3;    // 0 - 2 reserves for lines between robots
double small_offset = 2;
double threshold = 0.15;
double line_width_custom = 0.15;

void publish_uncertainty(
    Eigen::Vector3d &car_pt, Eigen::Quaterniond &car_q,
    const super_odometry_msgs::OptimizationStats::ConstPtr &msg, int car_id) {
  bool bad_enough = false;
  jsk_rviz_plugins::OverlayText status_text;
  status_text.fg_color.r = 1.0;
  status_text.fg_color.g = 1.0;
  status_text.fg_color.b = 1.0;
  status_text.fg_color.a = 1.0;
  status_text.bg_color.a = 0.0;
  status_text.font = "DejaVu Sans Mono";
  status_text.text = "normal";
  visualization_msgs::Marker marker;
  marker.header.frame_id = "global";
  marker.header.stamp = msg->header.stamp;
  marker.ns = "uncertainty";
  marker.id = marker_id;
  marker_id++;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = car_pt(0);
  marker.pose.position.y = car_pt(1);
  marker.pose.position.z = car_pt(2);
  marker.pose.orientation.x = car_q.x();
  marker.pose.orientation.y = car_q.y();
  marker.pose.orientation.z = car_q.z();
  marker.pose.orientation.w = car_q.w();
  if (msg->uncertainty_x < msg->uncertainty_y &&
      msg->uncertainty_x < msg->uncertainty_z) {
    marker.scale.x = (1 - msg->uncertainty_x) + small_offset;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 0.5;
    marker.color.r = 242.0 / 255;
    marker.color.g = 75.0 / 255;
    marker.color.b = 231.0 / 255;
    if (msg->uncertainty_x <= threshold) {
      bad_enough = true;
      status_text.text = "X-degraded";
      status_text.fg_color.g = 0.0;
      status_text.fg_color.b = 0.0;
    }
  } else if (msg->uncertainty_y < msg->uncertainty_x &&
             msg->uncertainty_y < msg->uncertainty_z) {
    marker.scale.x = 1;
    marker.scale.y = (1 - msg->uncertainty_y) + small_offset;
    marker.scale.z = 1;
    marker.color.a = 0.5;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    if (msg->uncertainty_y <= threshold) {
      bad_enough = true;
      status_text.text = "Y-degraded";
      status_text.fg_color.g = 0.0;
      status_text.fg_color.b = 0.0;
    }
  } else if (msg->uncertainty_z < msg->uncertainty_x &&
             msg->uncertainty_z < msg->uncertainty_y) {
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = (1 - msg->uncertainty_z) + small_offset;
    marker.color.a = 0.5;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    if (msg->uncertainty_z <= threshold) {
      bad_enough = true;
      status_text.text = "Z-degraded";
      status_text.fg_color.g = 0.0;
      status_text.fg_color.b = 0.0;
    }
  } else {
    marker.scale.x = (1 - msg->uncertainty_x) + small_offset;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
  }
  marker.lifetime = ros::Duration(0.2);
  if (bad_enough) {
    marker_pub.publish(marker);
  }
  std_msgs::String fixed_text;

  switch (car_id) {
  case 1:
    fixed_text.data = "rc1 status:";
    rc1_status_pub.publish(fixed_text);
    rc1_status_color_pub.publish(status_text);
    break;
  case 2:
    fixed_text.data = "rc2 status:";
    rc2_status_pub.publish(fixed_text);
    rc2_status_color_pub.publish(status_text);
    break;
  case 3:
    fixed_text.data = "rc3 status:";
    rc3_status_pub.publish(fixed_text);
    rc3_status_color_pub.publish(status_text);
    break;
  }
}

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
                            const Eigen::Quaterniond &q,
                            Eigen::Vector3d &transformed,
                            Eigen::Quaterniond &q_transformed) {

  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translate(t_rc1);
  transformation.rotate(q_rc1);
  transformed = (transformation)*pt;
  q_transformed = q_rc1 * q;
}

void transform_rc2_to_darpa(const Eigen::Vector3d &pt,
                            const Eigen::Quaterniond &q,
                            Eigen::Vector3d &transformed,
                            Eigen::Quaterniond &q_transformed) {

  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translate(t_rc2);
  transformation.rotate(q_rc2);
  transformed = (transformation)*pt;
  q_transformed = q_rc2 * q;
}

void transform_rc3_to_darpa(const Eigen::Vector3d &pt,
                            const Eigen::Quaterniond &q,
                            Eigen::Vector3d &transformed,
                            Eigen::Quaterniond &q_transformed) {

  Eigen::Affine3d transformation = Eigen::Affine3d::Identity();
  transformation.translate(t_rc3);
  transformation.rotate(q_rc3);
  transformed = (transformation)*pt;
  q_transformed = q_rc3 * q;
}

void callback(const nav_msgs::OdometryConstPtr rc1_odom,
              const nav_msgs::OdometryConstPtr rc2_odom,
              const nav_msgs::OdometryConstPtr rc3_odom,
              const super_odometry_msgs::OptimizationStats::ConstPtr &msg_rc1,
              const super_odometry_msgs::OptimizationStats::ConstPtr &msg_rc2,
              const super_odometry_msgs::OptimizationStats::ConstPtr &msg_rc3) {
  if (!initialized) {
    return;
  }
  Eigen::Vector3d rc1_darpa_pt;
  Eigen::Quaterniond rc1_darpa_q;
  transform_rc1_to_darpa(Eigen::Vector3d(rc1_odom->pose.pose.position.x,
                                         rc1_odom->pose.pose.position.y,
                                         rc1_odom->pose.pose.position.z),
                         Eigen::Quaterniond(rc1_odom->pose.pose.orientation.w,
                                            rc1_odom->pose.pose.orientation.x,
                                            rc1_odom->pose.pose.orientation.y,
                                            rc1_odom->pose.pose.orientation.z),
                         rc1_darpa_pt, rc1_darpa_q);
  Eigen::Vector3d rc2_darpa_pt;
  Eigen::Quaterniond rc2_darpa_q;
  transform_rc2_to_darpa(Eigen::Vector3d(rc2_odom->pose.pose.position.x,
                                         rc2_odom->pose.pose.position.y,
                                         rc2_odom->pose.pose.position.z),
                         Eigen::Quaterniond(rc2_odom->pose.pose.orientation.w,
                                            rc2_odom->pose.pose.orientation.x,
                                            rc2_odom->pose.pose.orientation.y,
                                            rc2_odom->pose.pose.orientation.z),
                         rc2_darpa_pt, rc2_darpa_q);
  Eigen::Vector3d rc3_darpa_pt;
  Eigen::Quaterniond rc3_darpa_q;
  transform_rc3_to_darpa(Eigen::Vector3d(rc3_odom->pose.pose.position.x,
                                         rc3_odom->pose.pose.position.y,
                                         rc3_odom->pose.pose.position.z),
                         Eigen::Quaterniond(rc3_odom->pose.pose.orientation.w,
                                            rc3_odom->pose.pose.orientation.x,
                                            rc3_odom->pose.pose.orientation.y,
                                            rc3_odom->pose.pose.orientation.z),
                         rc3_darpa_pt, rc3_darpa_q);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "global";
  marker.header.stamp = rc1_odom->header.stamp;
  marker.ns = "lines_between_rc1_and_rc2";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width_custom;
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
  marker2.scale.x = line_width_custom;
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
  marker3.scale.x = line_width_custom;
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

  // publish uncertainty
  publish_uncertainty(rc1_darpa_pt, rc1_darpa_q, msg_rc1, 1);
  publish_uncertainty(rc2_darpa_pt, rc2_darpa_q, msg_rc2, 2);
  publish_uncertainty(rc3_darpa_pt, rc3_darpa_q, msg_rc3, 3);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle nh;
  marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  rc1_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc1/speed_custom", 10);
  rc2_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc2/speed_custom", 10);
  rc3_speed_pub = nh.advertise<std_msgs::Float32>("/cmu_rc3/speed_custom", 10);
  rc1_status_pub = nh.advertise<std_msgs::String>("/cmu_rc1/fixed_status", 10);
  rc2_status_pub = nh.advertise<std_msgs::String>("/cmu_rc2/fixed_status", 10);
  rc3_status_pub = nh.advertise<std_msgs::String>("/cmu_rc3/fixed_status", 10);
  rc1_status_color_pub = nh.advertise<jsk_rviz_plugins::OverlayText>(
      "/cmu_rc1/fixed_status_color", 10);
  rc2_status_color_pub = nh.advertise<jsk_rviz_plugins::OverlayText>(
      "/cmu_rc2/fixed_status_color", 10);
  rc3_status_color_pub = nh.advertise<jsk_rviz_plugins::OverlayText>(
      "/cmu_rc3/fixed_status_color", 10);
  tf_sub = nh.subscribe("/tf", 10, tf_callback);
  message_filters::Subscriber<nav_msgs::Odometry> rc1_odom_sub(
      nh, "/cmu_rc1/aft_mapped_to_init_imu", 10);
  message_filters::Subscriber<nav_msgs::Odometry> rc2_odom_sub(
      nh, "/cmu_rc2/aft_mapped_to_init_imu", 10);
  message_filters::Subscriber<nav_msgs::Odometry> rc3_odom_sub(
      nh, "/cmu_rc3/aft_mapped_to_init_imu", 10);
  message_filters::Subscriber<super_odometry_msgs::OptimizationStats>
      rc1_stats_sub(nh, "/cmu_rc1/super_odometry_stats", 10);
  message_filters::Subscriber<super_odometry_msgs::OptimizationStats>
      rc2_stats_sub(nh, "/cmu_rc2/super_odometry_stats", 10);
  message_filters::Subscriber<super_odometry_msgs::OptimizationStats>
      rc3_stats_sub(nh, "/cmu_rc3/super_odometry_stats", 10);

  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), rc1_odom_sub, rc2_odom_sub, rc3_odom_sub, rc1_stats_sub,
      rc2_stats_sub, rc3_stats_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

  ros::spin();
  return -1;
}