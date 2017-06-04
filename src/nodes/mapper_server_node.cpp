#include <srrg_system_utils/system_utils.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <iostream>
#include <srrg_txt_io/static_transform_tree.h>
#include <srrg_txt_io/message_timestamp_synchronizer.h>
#include <srrg_nicp/depth_utils.h>
#include <srrg_nicp/nn_aligner.h>
#include <srrg_nicp/projective_aligner.h>
#include "mapper_server/mapper.h"
#include "mapper_server/pointcloud_publisher_trigger.h"
#include "mapper_server/local_mapper_trigger.h"
#include "mapper_server/call_mapper_trigger.h"

#include <image_transport/image_transport.h>

#include <srrg_ros_wrappers/image_message_listener.h>


using namespace std;
using namespace Eigen;
using namespace srrg_boss;
using namespace srrg_core;
using namespace srrg_core_ros;
using namespace mapper_server;

tf::TransformListener * listener = 0;
std::string base_link_frame_id = "";
std::string odom_frame_id = "/odom";
Mapper* mapper = 0;
std::vector<MessageTimestampSynchronizer> synchronizers;

const char* banner[] = {
    "srrg_mapper3d_node: online mapper working as ros node",
    "usage:",
    " srrg_mapper3d_node [options]",
    " where: ",
    "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
    "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
    "  -t:            [string] specifies which image topic to use, if unset will use all",
    "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",  "  -rgbt:         [string] specifies which rgb image topics to use. same as above. The number of -rgbt should match the order and the number of -t.",
    "  -base_link_frame_id [string]: if specified listens for odom, and tracks the pose of the base_link specified",
    "  -odom_frame_id [string]: odometry frame, default /odom",
    "  -o:            [string] output filename where to write the model, default \"\"",
    "once the gui has started you can dump the current cloud by pressing W ",
    0
};

void saveCloud(const std::string& prefix, int& num ){
    if (!mapper->referenceGood())
        return;
    if (!prefix.length())
        return;

    char buf[1024];
    sprintf(buf, "%s-%05d.cloud", prefix.c_str(), num);
    ofstream os(buf);
    mapper->referenceModel().write(os);
    cerr << "Saving cloud in file " << buf << endl;
    num++;
}

int main(int argc, char **argv) {
    std::vector<std::string> depth_topics;
    std::vector<std::string> rgb_topics;
    std::list<ImageMessageListener*> camera_listeners;

    std::string output_filename="";

    float min_distance = 0;
    int c = 1;
    float max_distance = 3;
    float tbb = 5;
    float obb = 1;
    float clipping_distance = 0;
    int cloud_count = 0;

    while (c<argc){
        if (! strcmp(argv[c], "-h")){
            printBanner(banner);
            return 0;
        }
        else if (! strcmp(argv[c], "-max_distance")){
            c++;
            max_distance = atof(argv[c]);
        }
        else if (! strcmp(argv[c], "-min_distance")){
            c++;
            min_distance = atof(argv[c]);
        } else if (! strcmp(argv[c], "-base_link_frame_id")){
            c++;
            base_link_frame_id = argv[c];
        } else if (! strcmp(argv[c], "-odom_frame_id")){
            c++;
            odom_frame_id = argv[c];
        }
        else if (! strcmp(argv[c], "-t")){
            c++;
            depth_topics.push_back(argv[c]);
        }
        else if (! strcmp(argv[c], "-rgbt")){
            c++;
            rgb_topics.push_back(argv[c]);
        }
        else if (! strcmp(argv[c], "-o")){
            c++;
            output_filename = argv[c];
        }
        c++;
    }

    cerr << "resizing synchronizers" << endl;
    synchronizers.resize(depth_topics.size());
    if (rgb_topics.size()>0){
        if (rgb_topics.size()!=depth_topics.size()){
            cerr << "fatal error the number of RGB topics should be the same as the -t topics" << endl;
            return 0;
        }
        for (size_t i=0; i<depth_topics.size(); i++){
            std::vector<string> depth_plus_rgb_topic;
            depth_plus_rgb_topic.push_back(depth_topics[i]);
            depth_plus_rgb_topic.push_back(rgb_topics[i]);
            synchronizers[i].setTopics(depth_plus_rgb_topic);
        }
    } else {
        for (size_t i=0; i<depth_topics.size(); i++){
            std::vector<string> depth_topic;
            depth_topic.push_back(depth_topics[i]);
            synchronizers[i].setTopics(depth_topic);
        }
    }

    ros::init(argc, argv, "mapper_server");
    if (base_link_frame_id.length()>0){
        cerr << "making listener" << endl;
        listener = new tf::TransformListener(ros::Duration(60.0));
    }
    ros::NodeHandle nh;
    image_transport::ImageTransport itr(nh);

    cerr << "constructing mapper ... ";
    mapper = new Mapper(ros::this_node::getName());
    if (! mapper) {
        cerr << "unknown mapper type, aborting" << endl;
        return 0;
    }

    mapper->setMaxDistance(max_distance);
    cerr << " Done" << endl;
    cerr << "ALL IN PLACE" << endl;

//    Serializer* ser = 0;
//    if(output_filename != "") {
//        ser  = new Serializer();
//        ser->setFilePath(output_filename);
//        ser->setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
//    }

//    LocalMapperTrigger* local_mapper_maker = new LocalMapperTrigger(mapper,
//                                                      Mapper::TRACK_GOOD|
//                                                      Mapper::TRACK_BROKEN|
//                                                      Mapper::REFERENCE_FRAME_RESET|
//                                                      Mapper::TRACKING_DONE,
//                                                      1, ser);

//    local_mapper_maker->setTrajectoryMaxTranslation(tbb);
//    local_mapper_maker->setTrajectoryMaxOrientation(obb);
//    local_mapper_maker->setClippingDistance(clipping_distance);

    SensorMessageSorter sorter;
    sorter.setTimeWindow(0.);

    CallMapperTrigger* caller = new CallMapperTrigger(&sorter, 0, mapper, &synchronizers);
    for (std::vector<std::string>::iterator it = depth_topics.begin(); it!=depth_topics.end(); it++) {
        std::string topic = *it;
        ImageMessageListener* camera_listener =
                new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
        camera_listener->setDequeLength(1);
        camera_listener->subscribe(topic,depth_topics.size());
        cerr << "subscribing for topic: " << topic << endl;
        camera_listeners.push_back(camera_listener);
    }

    for (std::vector<std::string>::iterator it = rgb_topics.begin(); it!=rgb_topics.end(); it++) {
        std::string topic = *it;
        ImageMessageListener* camera_listener =
                new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
        camera_listener->setDequeLength(1);
        camera_listener->setVerbose(true);
        camera_listener->subscribe(topic,depth_topics.size());
        cerr << "subscribing to topic: " << topic << endl;
        camera_listeners.push_back(camera_listener);
    }

    tf::TransformBroadcaster* broadcaster = new tf::TransformBroadcaster;

    PointCloudPublisherTrigger* cloud_publisher = new PointCloudPublisherTrigger(mapper,
                                                                                 Mapper::PROCESSING_DONE,
                                                                                 100, nh, broadcaster);
    ros::spin();

    saveCloud(output_filename,cloud_count);

}

