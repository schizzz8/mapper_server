#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "mapper.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace mapper_server {

  class PointCloudPublisherTrigger: public Mapper::Trigger{
  public:
    PointCloudPublisherTrigger(Mapper* mapper,
              int event, int priority,
              ros::NodeHandle & nh,
              tf::TransformBroadcaster* broadcaster_ = 0,
              const std::string& prefix_ = "/tracker",
              const std::string& odom_frame_id = "/odom") ;

    inline const std::string& prefix() const { return _prefix; }
    inline void setPrefix(const std::string& prefix_)  { _prefix = prefix_; }
    virtual void action(Mapper::TriggerEvent e);
    inline int skipRate() const {return _skip_rate;}
    inline void setSkipRate(int sr) { _skip_rate = sr; }
  protected:
    int _count;
    int _skip_rate;
    std::string _odom_frame_id;
    std::string _prefix;
    ros::Publisher _reference_cloud_publisher;
    tf::TransformBroadcaster* _broadcaster;
  };

}
