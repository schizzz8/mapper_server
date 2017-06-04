#include <iostream>
#include <srrg_core_map/image_map_node.h>
#include <srrg_core_map/multi_image_map_node.h>
#include "local_mapper_trigger.h"

namespace mapper_server {

using namespace std;
using namespace srrg_boss;
using namespace srrg_core;
using namespace srrg_core_map;
using namespace srrg_nicp;

void LocalMapperTrigger::onCameraInfoCreated(BaseCameraInfo* ){}

void LocalMapperTrigger::onNewNodeCreated(MapNode*, BinaryNodeRelation*){}

void LocalMapperTrigger::onRelationCreated(BinaryNodeRelation* ){}

LocalMapperTrigger::LocalMapperTrigger(Mapper* mapper,
                                       int events,
                                       int priorory,
                                       Serializer* ser) :
    Mapper::Trigger(mapper, events,  priorory) {
    cerr<< " name: LocalMapper "<< endl;
    _trajectory_min_translation = 0.05;
    _trajectory_min_orientation = 0.1;
    _trajectory_max_translation = 0.25;
    _trajectory_max_orientation = 1;
    _last_global_pose = _mapper->globalT();
    _serializer = ser;
    _local_maps = 0;
    _local_maps_relations = 0;
    _nodes = new MapNodeList;
    _relations = new BinaryNodeRelationSet;
}

MapNode* LocalMapperTrigger::makeNode(){
    MapNode* new_node = 0;
    MapNode* previous_node = 0;
    if (_nodes->size())
        previous_node = _nodes->rbegin()->get();

    new_node = new ImageMapNode(_mapper->globalT(), _mapper->lastCamera(), _mapper->lastTopic(), _mapper->lastSeq());
    return new_node;
}

BinaryNodeRelation* LocalMapperTrigger::makeNodesRelation(MapNode* new_node, MapNode* previous_node) {
    if (! _relations || ! previous_node)
        return 0;
    BinaryNodeRelation* rel = new BinaryNodeRelation;
    rel->setFrom(previous_node);
    rel->setTo(new_node);
    rel->setTransform(previous_node->transform().inverse()*new_node->transform());
    return rel;
}


void LocalMapperTrigger::action(Mapper::TriggerEvent e){
    if (e&Mapper::NEW_CAMERA_ADDED){
        onCameraInfoCreated(mapper()->lastCamera());
        return;
    }
    if (e&Mapper::TRACKING_DONE) {
        Eigen::Isometry3f delta = _last_global_pose.inverse()*_mapper->globalT();
        Eigen::AngleAxisf aa(delta.linear());
        bool make_new_node = delta.translation().norm()>_trajectory_min_translation
                || fabs(aa.angle())>_trajectory_min_orientation;

        if (! make_new_node)
            return;

        MapNode* new_node=makeNode();
        MapNode* previous_node =  (_nodes && _nodes->size()) ? _nodes->rbegin()->get() : 0;
        _last_global_pose=_mapper->globalT();
        _nodes->addElement(new_node);

        BinaryNodeRelation* rel=makeNodesRelation(new_node, previous_node);
        if (rel) {
            _relations->insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
        }
        onNewNodeCreated(new_node, rel);

        LocalMap* lmap = 0;
        // check if the trajectory bounds exceeded
        bool bound_exceded = isTrajectoryBoundReached();

        if (! bound_exceded)
            return;

        lmap  = makeLocalMap();
        //std::cerr << "Size before calling onLocalMapCreated(lmap): "
        // 		<< lmap->nodes().size() << std::endl;
        // assert(lmap->nodes().size() > 0 && "Size before calling onLocalMapCreated(lmap) is zero");
        onLocalMapCreated(lmap);
        if (_serializer) {
            saveCameras(_mapper->cameras());
            saveLocalMap(*lmap);
        }

        if (_last_local_map) {
            Eigen::Isometry3f dt = _last_local_map->transform().inverse()*lmap->transform();
            Matrix6f info = Matrix6f::Identity();
            info *=1e-2;

            std::tr1::shared_ptr<BinaryNodeRelation>
                    rel (new BinaryNodeRelation(_last_local_map.get(), lmap, dt, info) );
            onRelationCreated(rel.get());
            if (_serializer)
                _serializer->writeObject(*rel);

            _last_relation = rel;
            if (_relations) {
                _relations->insert(rel);
                if (_local_maps_relations) {
                    _local_maps_relations->insert(rel);
                }
            }
        }

        std::tr1::shared_ptr<LocalMap> current_map_ptr(lmap);

        if (_local_maps) {
            _local_maps->push_back(current_map_ptr);
        }
        _previous_local_map = _last_local_map;
        _last_local_map = current_map_ptr;
        _nodes->clear();
        _relations->clear();
        if(_mapper->referenceGood() && _clipping_distance>0)
            _mapper->referenceModel().clip(_clipping_distance);
        else {
            _mapper->clearStatus();
        }
    }

}

LocalMap* LocalMapperTrigger::makeLocalMap() {
    if (! _nodes)
        return 0;
    if (! _mapper->referenceGood() || ! _mapper->referenceModel().size())
        return 0;

    Eigen::Isometry3f origin = _nodes->middlePose();
    Eigen::Isometry3f invT = origin.inverse();
    LocalMap * local_map = new LocalMap(origin);
    local_map->nodes()=*_nodes;
    local_map->relations()=*_relations;
    for (MapNodeList::iterator it = _nodes->begin();
         it!= _nodes->end(); it++) {
        MapNode* node = it->get();
        node->parents().insert(local_map);
        node->setTransform(invT*node->transform());
    }
    for (BinaryNodeRelationSet::iterator it= _relations->begin(); it!=_relations->end(); it++) {
        BinaryNodeRelation* rel = it->get();
        rel->setParent(local_map);
    }
    local_map->setCloud(new Cloud(_mapper->referenceModel()));
    local_map->cloud()->transformInPlace(invT*_mapper->globalT());
    return local_map;
}

void LocalMapperTrigger::saveLocalMap(LocalMap& lmap){
    for (MapNodeList::iterator tt = lmap.nodes().begin();
         tt!=lmap.nodes().end(); tt++){
        MapNode* n = tt->get();
        _serializer->writeObject(*n);
    }

    for (BinaryNodeRelationSet::iterator it = lmap.relations().begin();
         it!=lmap.relations().end(); it++){
        BinaryNodeRelation* r = it->get();
        _serializer->writeObject(*r);
    }

    // write the local map
    _serializer->writeObject(lmap);

}

void LocalMapperTrigger::onLocalMapCreated(LocalMap* lmap){
    cerr << "Local Map created" << endl;
}

void LocalMapperTrigger::saveCameras(CameraInfoManager& manager) {
    for(size_t i = 0; i< manager.cameras().size(); i++) {
        BaseCameraInfo* cam = manager.cameras()[i];
        _serializer->writeObject(*cam);
    }
}

bool LocalMapperTrigger::isTrajectoryBoundReached() {
    if (!_nodes || _nodes->size()<2)
        return false;
    Eigen::Vector3f tbb = _nodes->upperTranslation() - _nodes->lowerTranslation();
    if (tbb.norm()>_trajectory_max_translation)
        return true;

    Eigen::Vector3f obb = _nodes->upperOrientation() - _nodes->lowerOrientation();
    if (obb.norm()>_trajectory_max_orientation)
        return true;
    return false;
}

}
