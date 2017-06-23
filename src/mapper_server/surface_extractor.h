#pragma once
#include "srrg_types/cloud.h"
//#include "srrg_distance_map/distance_map.h"


namespace mapper_server {

class SurfaceExtractor {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SurfaceExtractor(float resolution_ = 0.05f);

    void compute(srrg_core::Cloud* cloud_, const Eigen::Isometry3f& iso=Eigen::Isometry3f::Identity());

    inline void setResolution(float resolution_) { _resolution = resolution_; }
    
    inline float resolution() const { return _resolution; }

    inline const srrg_core::IntImage& indices() const {return _indices;}

    inline const srrg_core::FloatImage& elevations() const {return _elevations; }
    inline const srrg_core::FloatImage& obstacles() const {return _obstacles; }

    inline const Eigen::Vector3f& upper() const { return _upper; }
    inline const Eigen::Vector3f& lower() const { return _lower; }
    inline const Eigen::Vector3f& bottom() const { return _bottom; }
    inline const Eigen::Vector3i& size() const { return _size;}

    inline srrg_core::UnsignedCharImage& classified() {return _classified; }
    inline const srrg_core::UnsignedCharImage& classified() const {return _classified; }

protected:
    srrg_core::IntImage _indices;
    srrg_core::FloatImage _elevations;
    srrg_core::FloatImage _obstacles;
    srrg_core::Cloud _transformed_cloud;
    float _resolution;
    Eigen::Vector3f _upper;
    Eigen::Vector3f _lower;
    Eigen::Vector3f _bottom;
    Eigen::Vector3i _size;
    
    srrg_core::UnsignedCharImage _classified;
};

}
