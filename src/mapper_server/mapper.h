#pragma once

#include <srrg_boss/serializer.h>

#include "sparse_grid.h"

#include <srrg_image_utils/depth_utils.h>
#include <srrg_types/base_camera_info.h>
#include "srrg_core_map_2/map_node_list.h"
#include "srrg_core_map_2/local_map.h"
#include "srrg_core_map_2/map_node_relation.h"
#include <srrg_types/camera_info_manager.h>
#include <srrg_nicp/pinhole_projector.h>

#include <fstream>

//#include "srrg_types/cloud_3d.h"
//#include "srrg_types/traversability_map.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <actionlib/server/simple_action_server.h>
#include "mapper_server/MapperAction.h"

namespace mapper_server {

class Mapper {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class Trigger;

    enum TriggerEvent  {
        NEW_FRAME_CREATED = 0x1,  // when making a new cloud of an image
        NEW_CAMERA_ADDED = 0x2,  // when making a new camera for the first time
        ALIGNMENT_DONE = 0x4,     // called after alignment
        TRACK_GOOD = 0x8,         // if the alignment succeeds
        TRACK_BROKEN = 0x10,       // if the alignment fails
        TRACKING_DONE = 0x20,      // after all has been done
        PROCESSING_DONE = 0x40,   // after processing, before the end of ptocessframe
        REFERENCE_FRAME_RESET = 0x80, // when the very first frame is created
        ITERATION_DONE=0x100,      // after each iteration;
    };

    //! class that defines a trigger, invoked when a specific event happens
    //! it registers himself on the tracker, responds to an eventl
    //! triggers for the same event are invoked according to their oriority (lower number, higher priority)
    class Trigger{
    public:
        Trigger(Mapper* mapper, int event, int priorory);
        virtual ~Trigger();
        virtual void action(TriggerEvent event) = 0;
        inline Mapper* mapper() { return _mapper;}
        inline const int event() const {return _event;}
        inline const int priority() const {return _priority;}
    protected:
        Mapper* _mapper;
        int _event;
        int _priority;
    };

    //! ctor if a = 0, it creates its own instance of Projective Aligner with a Pinhole projector;
    //! if you pass your aligner it will use that one
    //! this way you can construct a tracker based on different alignment policies
    Mapper(std::string name, srrg_boss::Serializer* ser=0, srrg_nicp::BaseProjector*p=0);

    //! processes a frame, froma raw depth image, updates the transform and adds the new colud to the local map
    //! @param depth : uint16_t depth image
    //! @param K: camera matrix
    //! @param depthSCale: the depth conversion to pass from pixel units to meters (for kinect 1e-3)
    //! @param seq: the seq field in the header of the ros Image message
    //! @param sensorOffset: the position of the camera on the base.
    //! The pose offset is computed by the system in the reference frame of the base. This allows to handle multiple cams
    //! @param topic: the topic of the image
    virtual void processFrame(const srrg_core::RawDepthImage& depth,
                              const srrg_core::RGBImage& rgb,
                              const Eigen::Matrix3f& K,
                              float depthScale,
                              int seq,
                              double timestamp,
                              const std::string& topic = "/camera/depth/image_raw",
                              const std::string& frame_id = "/camera/depth/frame_id",
                              const Eigen::Isometry3f& sensor_offset = Eigen::Isometry3f::Identity(),
                              const Eigen::Isometry3f& odom_guess = Eigen::Isometry3f::Identity());

    inline srrg_boss::Serializer* serializer() const {return _serializer;}
    inline void setSerializer(srrg_boss::Serializer* ser) {_serializer = ser;}

    //! access to the internal projector object
    srrg_nicp::BaseProjector& projector() { return *_projector;}

    //! clears the internal state of the tracker
    //! starts a new local map using the current frame as initial frame
    //! keeps the old global transform unaltered
    void clearStatus();

    //! returns the current global transform estimate (depth-based odometry)
    inline const Eigen::Isometry3f globalT() const {return _global_transform;}

    //! sets the global T. When doing it, be sure that the reference cloud is transformed
    //! so that the origin is in the globalT.
    inline void setGlobalT(const Eigen::Isometry3f global_t)  { _global_transform = global_t;}

    //! returns the current local transform estimate (depth-based odometry)
    inline const Eigen::Isometry3f odomDelta() const {return _odom_delta;}

    //! sets the local T.
    inline void setOdomDelta(const Eigen::Isometry3f odom_delta_)  { _odom_delta = odom_delta_;}

    //! returns the current array of camera infos
    //! a camera is created the first time an image message with an unseen topic is
    //! passed to processFrame()
    //! this object will contain a list of all cameras
    inline srrg_core::CameraInfoManager& cameras() { return _cameras; }

    //! access to the current cloud (the last one passed to the tracker)
    inline srrg_core::Cloud3D& currentModel() { return _current;}
    //! access to the reference cloud (the local map cloud)
    inline srrg_core::Cloud3D& referenceModel() { return _reference;}
    //! sets the reference model
    inline void setReferenceModel(const srrg_core::Cloud3D& reference_model) { _reference = reference_model; }

    //! controls the integration of a new cloud
    //! if false, the robot pose is only tracked but not augmented
    inline void enableMerging (bool m) {_merging_enabled = m; }
    inline bool mergingEnabled () const { return _merging_enabled; }

    //! points in a track that are closed than this threshold are claimed to be inliers
    inline float mergingDistance() const {return _merging_distance;}
    inline void  setMergingDistance(float t) { _merging_distance = t;}

    inline double startTime() const {return _start_time;}
    inline double currentTime() const {return _current_time;}
    inline double makeCloudTime() const {return _make_cloud_time;}
    inline double alignmentTime() const {return _alignment_time;}
    inline double validateTime() const {return _validate_time;}
    inline double mergeTime() const {return _merge_time;}
    inline double tailTime() const {return _tail_time;}

    inline int lastSeq() const {return _last_seq;}
    inline srrg_core::RawDepthImage& lastRawDepth() {return _last_raw_depth;}
    inline float lastDepthScale() const {return _last_depth_scale;}
    inline srrg_core::BaseCameraInfo* lastCamera() {return _last_camera;}
    inline const std::string& lastTopic() const {return _last_topic;}
    inline const Eigen::Isometry3f& lastInitialGuess() const {return _last_initial_guess;}
    inline const Eigen::Isometry3f& lastCameraOffset() const {return _last_camera_offset;}
    inline float lastOutliersRatio() const {return _last_outliers_ratio;}
    inline double lastTimestamp() const { return _last_timestamp;}

    void setMaxDistance(float max_distance);
    void setMinDistance(float min_distance);

    inline bool referenceGood() const {return _reference_good;}

    srrg_core_map_2::Pose3DMapNodeList* nodes() {return _nodes;}
    srrg_core_map_2::Pose3DPose3DMapNodeRelationSet* relations() { return _relations;}
    void setNodes(srrg_core_map_2::Pose3DMapNodeList* nodes_) {_nodes = nodes_;}

    // min translation between two trajectory nodes
    inline float trajectoryMinTranslation()  const { return _trajectory_min_translation; }
    inline void setTrajectoryMinTranslation(float v)  { _trajectory_min_translation = v; }

    // min orientation between two trajectory nodes
    inline float trajectoryMinOrientation()  const { return _trajectory_min_orientation; }
    inline void setTrajectoryMinOrientation(float v)  { _trajectory_min_orientation = v; }


    void goalCB();
    void preemptCB();

protected:
    ros::NodeHandle _nh;
    actionlib::SimpleActionServer<mapper_server::MapperAction> _as;
    std::string _action_name;

    mapper_server::MapperActionFeedback _feedback;
    mapper_server::MapperResult _result;

    typedef std::map<int, Trigger*> PriorityTriggerMap;
    typedef std::map<TriggerEvent, PriorityTriggerMap> EventTriggerMap;

    srrg_core::CameraInfoManager _cameras;
    srrg_nicp::BaseProjector* _projector;

    srrg_core::Cloud3D _reference;
    srrg_core::Cloud3D _current;
    srrg_core::Cloud3D _temp_current;

    float _merging_gain;
    Eigen::Isometry3f _global_transform;
    Eigen::Isometry3f _odom_delta;
    int _total_points;
    bool _merging_enabled;
    float _merging_distance;

    srrg_boss::Serializer* _serializer;
    srrg_core_map_2::Pose3DMapNodeList* _nodes;
    srrg_core_map_2::Pose3DPose3DMapNodeRelationSet* _relations;
    float _trajectory_min_translation;
    float _trajectory_min_orientation;
    srrg_core_map_2::Pose3DMapNodeList* _local_maps;
    srrg_core_map_2::Pose3DPose3DMapNodeRelationSet* _local_maps_relations;

    float _resolution;
    float _distance_threshold;
    float _connectivity_threshold;

    double _start_time;
    double _current_time;
    double _make_cloud_time;
    double _alignment_time;
    double _validate_time;
    double _merge_time;
    double _tail_time;

    float _last_outliers_ratio;
    srrg_core::RawDepthImage _last_raw_depth;
    srrg_core::RGBImage _last_raw_rgb;

    int _last_seq;
    float _last_depth_scale;
    srrg_core::BaseCameraInfo* _last_camera;
    Eigen::Isometry3f _last_odom;
    double _last_timestamp;

    srrg_core::IndexImage _cur_indices, _ref_indices;
    srrg_core::FloatImage _cur_buffer, _ref_buffer;
    void reprojectBoth(int rows, int cols, float scale);
    void computeTrackBroken();
    std::string _last_topic;
    Eigen::Isometry3f _last_initial_guess;
    Eigen::Isometry3f _last_camera_offset;
    bool _information_matrix_reset;

    std::ofstream tracker_debug_stream;
    bool _reference_good;

    void callTriggers(TriggerEvent event);
    EventTriggerMap _triggers;

    srrg_core_map_2::Pose3DMapNode* makeNode();
    srrg_core_map_2::Pose3DPose3DMapNodeRelation* makeNodesRelation(srrg_core_map_2::Pose3DMapNode* new_node,
                                                                    srrg_core_map_2::Pose3DMapNode* previous_node);
    srrg_core_map_2::LocalMap3D* makeLocalMap();
    void saveLocalMap(srrg_core_map_2::LocalMap3D& lmap);
    void saveCameras(srrg_core::CameraInfoManager& manager);

    inline bool same(srrg_core_map_2::LocalMap3D* lmap1, srrg_core_map_2::LocalMap3D* lmap2){
        if(lmap1==lmap2){
            std::cerr << "Same local maps (" <<  lmap1 << " = " << lmap2 << ")" << std::endl;
        }
        return (lmap1 == lmap2) ? true : false;
    }

    inline bool closeEnough(srrg_core_map_2::LocalMap3D* lmap1, srrg_core_map_2::LocalMap3D* lmap2){
        if((lmap1->estimate().translation() - lmap2->estimate().translation()).norm()>_distance_threshold){
            std::cerr << "Local maps not close enough (" << (lmap1->estimate().translation() - lmap2->estimate().translation()).norm()
                      << " > " << _distance_threshold << ")" << std::endl;
        }

        return ((lmap1->estimate().translation() - lmap2->estimate().translation()).norm() <= _distance_threshold) ? true : false;
    }

    inline bool alreadyConnected(srrg_core_map_2::LocalMap3D* lmap1, srrg_core_map_2::LocalMap3D* lmap2){
        if(_local_maps_relations->empty())
            return false;

        for(srrg_core_map_2::Pose3DPose3DMapNodeRelationSet::iterator kt = _local_maps_relations->begin();
            kt != _local_maps_relations->end(); kt++)
            if((*kt)->from() == lmap1 && (*kt)->to() == lmap2 ||
                    (*kt)->from() == lmap2 && (*kt)->to() == lmap1) {
                std::cerr << "Local Maps already connected!" << std::endl;
                return true;
            }

        return false;
    }

    bool addEdge(srrg_core_map_2::LocalMap3D* lmap1, srrg_core_map_2::LocalMap3D* lmap2);
};

}
