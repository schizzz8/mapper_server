#pragma once

#include <iostream>
#include <map>
#include <unordered_map>

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <unordered_map>
#include <Eigen/Core>

#include <srrg_types/cloud.h>


namespace mapper_server {

typedef std::map<Eigen::Vector3f,int> PointIndexMap;

class Cell {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Cell(const Eigen::Vector3i& idx=Eigen::Vector3i::Zero()):
        _idx(idx){
        _ground = false;
        _first_cloud = false;
        _second_cloud = false;
    }

    inline bool operator < (const Cell& c) const {
        for (int i=0; i<3; i++){
            if (_idx[i]<c._idx[i])
                return true;
            if (_idx[i]>c._idx[i])
                return false;
        }
        return false;
    }

    inline bool operator == (const Cell& c) const {
        for (int i=0; i<3; i++)
            if(_idx[i] != c._idx[i])
                return false;
        return true;
    }

    inline void setCenter(Eigen::Vector3f origin, float resolution) {
        _center = origin + _idx.cast<float>()*resolution + Eigen::Vector3f(resolution/2,resolution/2,resolution/2);
    }

    inline const bool ground() const {return _ground;}
    inline void setGround(const bool& ground_){_ground = ground_;}
    inline const bool overlap() const {return _first_cloud & _second_cloud;}

    Eigen::Vector3i _idx;
    Eigen::Vector3f _center;
    PointIndexMap _points;
    bool _ground;
    bool _first_cloud,_second_cloud;

};

template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
typedef std::unordered_map<Eigen::Vector3i,Cell*,matrix_hash<Eigen::Vector3i> > Vector3iCellPtrMap;

class SparseGrid : public Vector3iCellPtrMap {
public:
    SparseGrid (float resolution_ = 0.05,
                Eigen::Vector3f origin_ = Eigen::Vector3f::Zero(),
                Eigen::Vector3i dimensions_ = Eigen::Vector3i::Zero(),
                int half_ = std::numeric_limits<int>::max())
        :_resolution(resolution_),
          _origin(origin_),
          _dimensions(dimensions_),
          _half(half_){
        _inverse_resolution = 1./_resolution;
        _num_cells = _dimensions.x()*_dimensions.y()*_dimensions.z();
    }

    void insertCloud(const srrg_core::Cloud& cloud);

    srrg_core::Cloud extractCloud();

    srrg_core::UnsignedCharImage extractSurface();

    bool checkConnectivity(float connectivity_threshold);

    inline float resolution(){ return _resolution;}
    inline const Eigen::Vector3i dimensions(){ return _dimensions;}
    inline const Eigen::Vector3f origin(){ return _origin;}
    inline int numCells(){ return _num_cells;}

    inline const Eigen::Vector3i toGrid(const Eigen::Vector3f& point) const {
        return ((point - _origin)*_inverse_resolution).cast<int>();
    }
    inline const Eigen::Vector3f toWorld(const Eigen::Vector3i& cell) const{
        return (_origin + cell.cast<float>() *_resolution);
    }

protected:
    float _resolution;
    float _inverse_resolution;
    Eigen::Vector3f _origin;
    Eigen::Vector3i _dimensions;
    int _num_cells;
    int _half;
    srrg_core::Cloud _cloud;
    srrg_core::UnsignedCharImage _classified;

    inline const bool hasCell(const Eigen::Vector3i& idx){
        Vector3iCellPtrMap::iterator it = find(idx);
        return (it != end()) ? true : false;
    }
};


}
