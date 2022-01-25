#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H

#include <boost/thread/mutex.hpp>
#include <message_filters/subscriber.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace TimeStampFilter
{
typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;
/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
class TimeStampFilterNodelet : public nodelet::Nodelet
{
public:
  TimeStampFilterNodelet();

private:
  virtual void onInit();

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                 tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  void connectCb();

  void disconnectCb();

  ros::NodeHandle nh_, private_nh_;
  ros::Publisher pub_pc_;
  boost::mutex connect_mutex_;
  ros::Time stamp_last_;

  boost::shared_ptr<tf2_ros::Buffer> tf2_;
  boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pc_;
  boost::shared_ptr<MessageFilter> message_filter_;

  // ROS Parameters
  unsigned int input_queue_size_;
  std::string target_frame_, prefix_, front_axis_;
  double tolerance_;
  bool use_inf_;
  double inf_epsilon_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H
