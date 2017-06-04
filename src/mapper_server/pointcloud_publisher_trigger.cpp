#include "pointcloud_publisher_trigger.h"
#include <srrg_core_map_ros/map_msgs_ros.h>
#include <srrg_core_ros/StampedCloudMsg.h>

namespace mapper_server {

using namespace srrg_core_ros;
using namespace srrg_core_map_ros;

PointCloudPublisherTrigger::PointCloudPublisherTrigger(Mapper* mapper,
                                                       int event, int priority,
                                                       ros::NodeHandle & nh,
                                                       tf::TransformBroadcaster* broadcaster_,
                                                       const std::string& prefix_,
                                                       const std::string& odom_frame_id) :
    Mapper::Trigger(mapper, event, priority){
    _broadcaster = broadcaster_;
    _prefix = prefix_;
    _odom_frame_id = odom_frame_id;
    _reference_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(_prefix+"/reference_cloud", 10);
    _count = 0;
    _skip_rate = 2;
}

void PointCloudPublisherTrigger::action(Mapper::TriggerEvent )  {
    if (! mapper())
        return;
    _count ++;

    std_msgs::Header header;
    header.frame_id = "/base_link";
    header.seq = mapper()->lastSeq();
    header.stamp = ros::Time(mapper()->lastTimestamp());
    if (_reference_cloud_publisher.getNumSubscribers() && mapper()->referenceGood()) {
        sensor_msgs::PointCloud2 msg;
        msg.header = header;
        msg.height = 1;
        msg.width = mapper()->referenceModel().size();
        msg.is_dense = false;

        sensor_msgs::PointCloud2Modifier pcd_modifier(msg);
        pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for(size_t i = 0; i < mapper()->referenceModel().size(); ++i,++iter_x,++iter_y,++iter_z){
            const Eigen::Vector3f& point = mapper()->referenceModel().at(i).point();

            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();
        }

        _reference_cloud_publisher.publish(msg);
    }
}

}
