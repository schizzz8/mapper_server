#include <srrg_system_utils/system_utils.h>
#include <srrg_types/pinhole_camera_info.h>
#include "mapper.h"
#include <tr1/memory>
#include <typeinfo>
#include <stdexcept>
#include <srrg_nicp/nn_aligner.h>
#include "surface_extractor.h"


#define BIG_NUMBER_OF_POINTS 10000000

namespace mapper_server {

using namespace std;
using namespace srrg_core;
using namespace srrg_core_map_2;
using namespace srrg_nicp;

Mapper::Trigger::Trigger(Mapper *mapper, int e, int p) {
    _mapper = mapper;
    _event = e;
    _priority = p;
    cerr << "Trigger ctor, Events: " << e << endl;
    for (Mapper::EventTriggerMap::iterator it =_mapper->_triggers.begin();
         it!=_mapper->_triggers.end(); it++){
        if ( (e & it->first) ) {
            Mapper::PriorityTriggerMap::iterator pit = it->second.find(_priority);
            if (pit != it->second.end())
                throw std::runtime_error("error, trigger with the same priority already exists");
            it->second.insert(make_pair(_priority, this));
            cerr << "  adding trigger:" << this << " for event: " << it->first << " priority: " << p;
        }
    }
}

Mapper::Trigger::~Trigger() {
    for (Mapper::EventTriggerMap::iterator it =_mapper->_triggers.begin();
         it!=_mapper->_triggers.end(); it++){
        if (_event & it->first ) {
            Mapper::PriorityTriggerMap::iterator pit = it->second.find(_priority);
            if (pit == it->second.end())
                throw std::runtime_error("error, deleting a non existing trigger");
            it->second.erase(pit);
            cerr << "destroying trigger" << endl;
        }
    }
}

Mapper::Mapper (string name, srrg_boss::Serializer *ser, BaseProjector* p):_as(_nh, name, false){

    _action_name = "mapping";
    _as.registerGoalCallback(boost::bind(&Mapper::goalCB, this));
    _as.registerPreemptCallback(boost::bind(&Mapper::preemptCB,this));
    _as.start();

    _start_time = 0;
    _current_time = 0;
    _make_cloud_time = 0;
    _alignment_time = 0;
    _validate_time = 0;
    _merge_time = 0;
    _tail_time = 0;

    _serializer = ser;

    _distance_threshold = 20;
    _resolution = 0.01;
    _connectivity_threshold = 0.01;

    _trajectory_min_translation = 0.05;
    _trajectory_min_orientation = 0.1;
    _local_maps = 0;
    _local_maps_relations = 0;
    _nodes = new Pose3DMapNodeList;
    _relations = new Pose3DBinaryNodeRelationSet;
    _local_maps = new Pose3DMapNodeList;
    _local_maps_relations = new Pose3DBinaryNodeRelationSet;

    _reference;
    _current;
    _reference_good=false;
    _merging_gain = .5;
    _total_points = 0;

    _global_transform.setIdentity();
    _merging_enabled = true;
    _merging_distance = 0.1;

    _last_odom.setIdentity();
    _last_camera_offset.setIdentity();
    _last_initial_guess.setIdentity();
    _last_topic="";
    if (!p)
        _projector=new PinholeProjector();
    else
        _projector=p;
    _reference.reserve(BIG_NUMBER_OF_POINTS);
    _current.reserve(BIG_NUMBER_OF_POINTS);
    _temp_current.reserve(BIG_NUMBER_OF_POINTS);
    tracker_debug_stream.open("/home/giorgio/tracker_debug.txt");

    // populate the trigger map
    _triggers.insert(make_pair(NEW_FRAME_CREATED, PriorityTriggerMap()));
    _triggers.insert(make_pair(NEW_CAMERA_ADDED, PriorityTriggerMap()));
    _triggers.insert(make_pair(ALIGNMENT_DONE, PriorityTriggerMap()));
    _triggers.insert(make_pair(TRACK_GOOD, PriorityTriggerMap()));
    _triggers.insert(make_pair(TRACK_BROKEN, PriorityTriggerMap()));
    _triggers.insert(make_pair(TRACKING_DONE, PriorityTriggerMap()));
    _triggers.insert(make_pair(PROCESSING_DONE, PriorityTriggerMap()));
    _triggers.insert(make_pair(REFERENCE_FRAME_RESET, PriorityTriggerMap()));
}

void Mapper::goalCB(){
    ROS_INFO ("Received goal: %s", _as.acceptNewGoal()->trigger.c_str());

}

void Mapper::preemptCB(){
    ROS_INFO("%s: Preempted", _action_name.c_str());
    // set the action state to preempted
    _as.setPreempted();


    LocalMap3D* lmap = 0;
    lmap  = makeLocalMap();

    if (_serializer) {
        saveCameras(_cameras);
        saveLocalMap(*lmap);
    }

    std::tr1::shared_ptr<Pose3DMapNode> current_map_ptr(lmap);

    if (_local_maps) {
        cerr << "adding local map to list!" << endl;
        _local_maps->push_back(current_map_ptr);
    }

    if(!_local_maps->empty()){
        cerr << endl << "Checking connectivity for " << _local_maps->size() << " local maps!" << endl;
        for(Pose3DMapNodeList::iterator it = _local_maps->begin(); it != _local_maps->end(); it++){
            LocalMap3D* lmap2 = dynamic_cast<LocalMap3D*> (it->get());
            if(lmap2){
                cerr << "From: " << lmap << " To: " << lmap2 << endl;

                if(!same(lmap,lmap2) &&
                        closeEnough(lmap,lmap2) &&
                        !alreadyConnected(lmap,lmap2) &&
                        addEdge(lmap,lmap2)) {

                    cerr << "Adding edge!" << endl;
                    cerr << "Lmap1: " << lmap << " class name: " << lmap->className();
                    cerr << " Lmap2: " << lmap2 << " class name: " << lmap2->className() << endl;
                    std::tr1::shared_ptr<Pose3DBinaryNodeRelation>
                            rel (new Pose3DBinaryNodeRelation(lmap->estimate().inverse()*lmap2->estimate()));
                    rel->setFrom(lmap);
                    rel->setTo(lmap2);
                    cerr << "From: " << rel->from();
                    cerr << " To: " << rel->to() << endl;

                    if (_serializer){
                        _serializer->writeObject(*rel);
                        cerr << "writing local map relation to disk" << endl;
                    }

                    if (_local_maps_relations){
                        _local_maps_relations->insert(rel);
                        cerr << "adding local map relation to list!" << endl;
                    }

                } else
                    cerr << "Local maps are not connected!" << endl;
            }
        }
    }

    _nodes->clear();
    _relations->clear();

    clearStatus();

}

void Mapper::reprojectBoth(int rows, int cols, float scale){
    _projector->pushState();
    _projector->setImageSize(scale*rows, scale*cols);
    _projector->scaleCamera(scale);
    _projector->project(_cur_buffer, _cur_indices, Eigen::Isometry3f::Identity(), _current);
    _projector->project(_ref_buffer, _ref_indices, Eigen::Isometry3f::Identity(), _reference);
    _projector->popState();
}

void Mapper::processFrame(const RawDepthImage& depth,
                          const RGBImage& rgb,
                          const Eigen::Matrix3f& K,
                          float depth_scale,
                          int seq,
                          double timestamp,
                          const std::string& topic,
                          const std::string& frame_id,
                          const Eigen::Isometry3f& sensor_offset_,
                          const Eigen::Isometry3f& odom_guess){

    if(!_as.isActive())
        return;

    _last_seq = seq;
    _start_time  = getTime();
    _last_depth_scale = depth_scale;
    _last_topic = topic;
    _last_initial_guess = odom_guess;
    _last_timestamp = timestamp;

    Eigen::Matrix3f myK = K;
    _last_raw_rgb=RGBImage();

    _last_raw_depth = depth;
    if (rgb.rows && rgb.cols)
        _last_raw_rgb = rgb;


    BaseCameraInfo* cam = _cameras.getCamera(topic);
    bool new_camera = false;
    if (! cam) {
        cam=new PinholeCameraInfo(topic, frame_id, myK, sensor_offset_, depth_scale);
        _cameras.addCamera(cam);
        new_camera = true;
    }
    _last_camera = cam;
    _last_camera_offset = cam->offset();
    if (new_camera)
        callTriggers(NEW_CAMERA_ADDED);

    std::tr1::shared_ptr<BaseCameraInfo> internal_camera(cam->scale(1));

    if (depth.cols == 0 && depth.rows == 0)
        return;

    _odom_delta = _last_odom.inverse()*odom_guess;
    Eigen::AngleAxisf aa(_odom_delta.linear());
    bool make_new_node = _odom_delta.translation().norm()>_trajectory_min_translation
            || fabs(aa.angle())>_trajectory_min_orientation;
    if (! make_new_node)
        return;

    Pose3DMapNode* new_node=makeNode();
    Pose3DMapNode* previous_node =  (_nodes && _nodes->size()) ? _nodes->rbegin()->get() : 0;
    _nodes->addElement(new_node);

    Pose3DBinaryNodeRelation* rel=makeNodesRelation(new_node, previous_node);
    if (rel) {
        _relations->insert(std::tr1::shared_ptr<Pose3DBinaryNodeRelation>(rel));
    }

    _current.clear();

    _make_cloud_time = getTime();

    // build the cloud from the image
    _projector->setImageSize(_last_raw_depth.rows, _last_raw_depth.cols);
    _projector->setCameraInfo(internal_camera.get());
    _projector->setRawDepthScale(_last_depth_scale);
    _projector->unproject(_current, _last_raw_depth, _last_raw_rgb);
    _temp_current=_current;

    _current_time = getTime();

    _make_cloud_time = getTime() - _make_cloud_time;
    callTriggers(NEW_FRAME_CREATED);

    // if it is not the first cloud, do the work
    if (_reference_good ) {

        if (_projector->supportsCameraInfo(internal_camera.get())) {
            _projector->setCameraInfo(internal_camera.get());
        } else {
            cerr << "warning, unsupported camera info in tracker" << endl;
        }

        _global_transform = _global_transform * _odom_delta;

        _current.transformInPlace(_odom_delta);

        if (_merging_enabled) {
            _merge_time = getTime();
            reprojectBoth(_last_raw_depth.rows, _last_raw_depth.cols, _merging_gain);
            merge(_ref_buffer, _ref_indices, _reference,
                  _cur_buffer, _cur_indices, _current,
                  _merging_distance);
            _merge_time = getTime() - _merge_time;
        } else {
            _reference.add(_current);
        }
        _tail_time = getTime();
        _reference.transformInPlace(_odom_delta.inverse());
        _tail_time = getTime() - _tail_time;
    }

    _current=_temp_current;
    callTriggers(TRACKING_DONE);

    if (! _reference_good ){
        _reference = _temp_current;
        _reference_good=true;
        _current_time = getTime();
        callTriggers(REFERENCE_FRAME_RESET);
    }

    _current_time = getTime();
    _last_odom = odom_guess;

    callTriggers(PROCESSING_DONE);

}

void Mapper::clearStatus() {
    _reference=_current;
    _reference_good=false;
    _total_points = 0;
}

void Mapper::setMaxDistance(float max_distance) {
    if (_projector)
        _projector->setMaxDistance(max_distance);
}

void Mapper::setMinDistance(float min_distance) {
    if (_projector)
        _projector->setMinDistance(min_distance);
}

void Mapper::callTriggers(TriggerEvent event){
    Mapper::EventTriggerMap::iterator it = _triggers.find(event);
    if (it == _triggers.end()) {
        throw std::runtime_error("error, unsupported event in mapper");
    }

    for (PriorityTriggerMap::iterator pit = it->second.begin(); pit!= it->second.end(); pit++){
        pit->second->action(event);
    }
}

Pose3DMapNode *Mapper::makeNode(){
    Pose3DMapNode* new_node = 0;
    Pose3DMapNode* previous_node = 0;
    if (_nodes->size())
        previous_node = _nodes->rbegin()->get();

    new_node = new Pose3DMapNode(_global_transform);
    return new_node;
}

Pose3DBinaryNodeRelation *Mapper::makeNodesRelation(Pose3DMapNode *new_node, Pose3DMapNode *previous_node){
    if (! _relations || ! previous_node)
        return 0;
    Pose3DBinaryNodeRelation* rel = new Pose3DBinaryNodeRelation(previous_node->estimate().inverse()*new_node->estimate());
    rel->setFrom(previous_node);
    rel->setTo(new_node);
    
    return rel;
}

LocalMap3D *Mapper::makeLocalMap(){
    if (! _nodes)
        return 0;
    if (! _reference_good || ! _reference.size())
        return 0;

    Eigen::Isometry3f origin = _nodes->middlePose();
    Eigen::Isometry3f invT = origin.inverse();
    LocalMap3D* local_map = new LocalMap3D(origin);

    for(Pose3DMapNodeList::iterator it = _nodes->begin(); it != _nodes->end(); ++it){
        Pose3DMapNode* node = it->get();
        node->parents().insert(local_map);
        node->setEstimate(invT*node->estimate());
        local_map->nodes().insert(node);
    }
    for(Pose3DBinaryNodeRelationSet::iterator it = _relations->begin(); it != _relations->end(); ++it){
        Pose3DBinaryNodeRelation* rel = it->get();
        rel->setOwner(local_map);
        local_map->relations().insert(rel);
    }

    local_map->setCloud(new Cloud3D(_reference));
    local_map->cloud()->transformInPlace(invT*_global_transform);

    TraversabilityMap* traversability = new TraversabilityMap;
    traversability->setResolution(_resolution);
    traversability->compute(local_map->cloud());

    local_map->setTraversabilityMap(traversability);

    return local_map;
}

void Mapper::saveLocalMap(LocalMap3D &lmap){

    for (MapNodePtrSet::iterator it = lmap.nodes().begin();
         it!=lmap.nodes().end(); ++it){
        Pose3DMapNode* n = dynamic_cast<Pose3DMapNode*> (*it);
        _serializer->writeObject(*n);
    }

    for (BinaryNodeRelationPtrSet::iterator it = lmap.relations().begin();
         it!=lmap.relations().end(); ++it){
        Pose3DBinaryNodeRelation* r = dynamic_cast<Pose3DBinaryNodeRelation*> (*it);
        _serializer->writeObject(*r);
    }
    // write the local map
    _serializer->writeObject(lmap);
}

void Mapper::saveCameras(CameraInfoManager &manager){
    for(size_t i = 0; i< manager.cameras().size(); i++) {
        BaseCameraInfo* cam = manager.cameras()[i];
        _serializer->writeObject(*cam);
    }
}

//bool Mapper::addEdge(LocalMap3D* lmap1, LocalMap3D* lmap2){

//    cerr << "Sparse grid based connectivity check!" << endl;

//    Cloud3D* cloud = new Cloud3D;
//    Cloud3D transformed_cloud;
//    lmap1->cloud()->transform(transformed_cloud,lmap1->estimate());
//    cloud->add(transformed_cloud);
//    int size1 = cloud->size();
//    transformed_cloud.clear();
//    lmap2->cloud()->transform(transformed_cloud,lmap2->estimate());
//    cloud->add(transformed_cloud);

//    Eigen::Vector3f lower,higher;
//    cloud->computeBoundingBox(lower,higher);
//    _resolution=0.05;
//    Eigen::Vector3i dimensions = ((higher-lower)/_resolution).cast<int> ();
//    SparseGrid grid (_resolution,lower,dimensions,size1);

//    cerr << "Input cloud has " << cloud->size() << " points." << endl;
//    cerr << "Resolution: " << grid.resolution() << endl;
//    cerr << "Origin: " << grid.origin().transpose() << endl;
//    cerr << "Dimensions: " << grid.dimensions() << endl;
//    grid.insertCloud(*cloud);
//    grid.extractCloud();
//    grid.extractSurface();

//    return grid.checkConnectivity(_connectivity_threshold);

//    return true;
//}

bool Mapper::addEdge(LocalMap3D* lmap1, LocalMap3D* lmap2){
    int count = 0;
    SurfaceExtractor extractor;
    extractor.setResolution(_resolution);
    Cloud3D* cloud = new Cloud3D;
    Cloud3D transformed_cloud;
    lmap1->cloud()->transform(transformed_cloud,lmap1->estimate());
    cloud->add(transformed_cloud);
    int size1 = cloud->size();
    transformed_cloud.clear();
    lmap2->cloud()->transform(transformed_cloud,lmap2->estimate());
    cloud->add(transformed_cloud);
    extractor.compute(cloud);
    const IntImage indices = extractor.indices();
    const UnsignedCharImage classified = extractor.classified();
    for (int r=0; r<indices.rows; r++)
        for (int c=0; c<indices.cols; c++) {
            int idx1 = indices.at<int>(r,c);
            if (idx1<0)
                continue;
            if(classified.at<unsigned char>(r,c) != 0)
                continue;
            int rmin = r-1<0?0:r-1;
            int rmax = r+1>indices.rows-1?indices.rows-1:r+1;
            int cmin = c-1<0?0:c-1;
            int cmax = c+1>indices.cols-1?indices.cols-1:c+1;
            for (int rr =rmin; rr<=rmax; rr++)
                for (int cc =cmin; cc<=cmax; cc++)
                    if (rr!=r || cc!=c) {
                        int idx2 = indices.at<int>(rr,cc);
                        if (idx2<0)
                            continue;
                        if(classified.at<unsigned char>(rr,cc) != 0)
                            continue;
                        if((idx1 < size1)^(idx2 < size1)){
                            count++;
                            return true;
                        }
                    }
        }

    return false;
}

}
