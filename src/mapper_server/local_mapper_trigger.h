#pragma once

#include <srrg_core_map/local_map.h>
#include <srrg_core_map/binary_node_relation.h>
#include <srrg_boss/serializer.h>
#include "mapper.h"


namespace mapper_server {


class LocalMapperTrigger: public Mapper::Trigger {
public:
    LocalMapperTrigger(Mapper* mapper,
                int events =
            Mapper::TRACK_GOOD|
            Mapper::TRACK_BROKEN|
            Mapper::REFERENCE_FRAME_RESET|
            Mapper::TRACKING_DONE|
            Mapper::NEW_CAMERA_ADDED,
                int priorory = 10,
                srrg_boss::Serializer* ser=0);

    virtual void action(Mapper::TriggerEvent e);

    srrg_core_map::MapNodeList* nodes() {return _nodes;}
    srrg_core_map::BinaryNodeRelationSet* relations() { return _relations;}
    void setNodes(srrg_core_map::MapNodeList* nodes_) {_nodes = nodes_;}


    inline srrg_boss::Serializer* serializer() const {return _serializer;}
    inline void setSerializer(srrg_boss::Serializer* ser) {_serializer = ser;}

    inline srrg_core_map::MapNodeList* localMaps() { return _local_maps; }
    inline void setLocalMaps(srrg_core_map::MapNodeList* local_maps) { _local_maps = local_maps; }

    inline srrg_core_map::BinaryNodeRelationSet* localMapsRelations() { return _local_maps_relations; }
    inline void setLocalMapsRelations(srrg_core_map::BinaryNodeRelationSet* local_maps_relations) { _local_maps_relations = local_maps_relations; }


    // min translation between two trajectory nodes
    inline float trajectoryMinTranslation()  const { return _trajectory_min_translation; }
    inline void setTrajectoryMinTranslation(float v)  { _trajectory_min_translation = v; }

    // min orientation between two trajectory nodes
    inline float trajectoryMinOrientation()  const { return _trajectory_min_orientation; }
    inline void setTrajectoryMinOrientation(float v)  { _trajectory_min_orientation = v; }

    // max size of the bounding box of the trajectory translation, after which a new local map is created
    inline float trajectoryMaxTranslation()  const { return _trajectory_max_translation; }
    inline void setTrajectoryMaxTranslation(float v)  { _trajectory_max_translation = v; }

    // max size of the bounding box of the trajectory orientation, after which a new local map is created
    inline float trajectoryMaxOrientation()  const { return _trajectory_max_orientation; }
    inline void setTrajectoryMaxOrientation(float v)  { _trajectory_max_orientation = v; }

    // callbacks
    virtual void onNewNodeCreated(srrg_core_map::MapNode* node, srrg_core_map::BinaryNodeRelation* rel=0);
    virtual void onCameraInfoCreated(srrg_core_map::BaseCameraInfo* camera_info);
    virtual void onLocalMapCreated(srrg_core_map::LocalMap* lmap);
    virtual void onRelationCreated(srrg_core_map::BinaryNodeRelation* rel);


    inline float clippingDistance() const {return _clipping_distance;}
    void setClippingDistance(float clipping_distance_) {_clipping_distance=clipping_distance_;}
    bool isTrajectoryBoundReached();


protected:
    srrg_core_map::MapNode* makeNode();
    srrg_core_map::BinaryNodeRelation* makeNodesRelation(srrg_core_map::MapNode* new_node,
                                                         srrg_core_map::MapNode* previous_node);
    srrg_core_map::LocalMap* makeLocalMap();
    void saveLocalMap(srrg_core_map::LocalMap& lmap);
    void saveCameras(srrg_nicp::CameraInfoManager& manager);

    std::tr1::shared_ptr<srrg_core_map::LocalMap> _last_local_map, _previous_local_map;
    std::tr1::shared_ptr<srrg_core_map::BinaryNodeRelation> _last_relation;
    bool _enable;
    srrg_boss::Serializer* _serializer;
    float _trajectory_max_translation;
    float _trajectory_max_orientation;
    float _clipping_distance;
    Eigen::Isometry3f _last_global_pose;
    srrg_core_map::MapNodeList* _nodes;
    srrg_core_map::BinaryNodeRelationSet* _relations;
    float _trajectory_min_translation;
    float _trajectory_min_orientation;

    srrg_core_map::MapNodeList* _local_maps;
    srrg_core_map::BinaryNodeRelationSet* _local_maps_relations;

};

}

