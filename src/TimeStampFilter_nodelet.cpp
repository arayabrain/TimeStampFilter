#include <limits>
#include <pluginlib/class_list_macros.h>
#include <TimeStampFilter/TimeStampFilter.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <queue>

namespace TimeStampFilter
{
TimeStampFilterNodelet::TimeStampFilterNodelet()
{
  stamp_last_ = ros::Time(0);
}

void TimeStampFilterNodelet::onInit()
{
  NODELET_INFO("Initializing nodelet.");
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");

  private_nh_.param<std::string>("prefix", prefix_, "");
  int concurrency_level = private_nh_.param("concurrency_level", concurrency_level);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }
  sub_pc_.subscribe(nh_, "cloud_in", input_queue_size_);

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty())
  {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_pc_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(boost::bind(&TimeStampFilterNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(boost::bind(&TimeStampFilterNodelet::failureCb, this, _1, _2));
  }
  else  // otherwise setup direct subscription
  {
    sub_pc_.registerCallback(boost::bind(&TimeStampFilterNodelet::cloudCb, this, _1));
  }

  ros::SubscriberStatusCallback status = boost::bind(&TimeStampFilterNodelet::connectCb, this);

  pub_pc_ = nh_.advertise<sensor_msgs::PointCloud2>(prefix_ + "cloud", 5, status, status);
}

void TimeStampFilterNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_pc_.getNumSubscribers() > 0 && sub_pc_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to pointcloud, starting subscriber to pointcloud");
  }
}

void TimeStampFilterNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_pc_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to pointcloud, shutting down subscriber to pointcloud");
    sub_pc_.unsubscribe();
  }
}

void TimeStampFilterNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

void TimeStampFilterNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  sensor_msgs::PointCloud2ConstPtr cloud_out = cloud_msg;
  if(stamp_last_ >= cloud_msg->header.stamp) return;

  pub_pc_.publish(cloud_out);
  stamp_last_ = cloud_msg->header.stamp;
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(TimeStampFilter::TimeStampFilterNodelet, nodelet::Nodelet)
