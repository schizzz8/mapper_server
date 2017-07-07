#include "sparse_grid.h"

namespace std {
template<>
bool std::less<Eigen::Vector3f>::operator ()(const Eigen::Vector3f& a,const Eigen::Vector3f& b) const {
    for(size_t i=0;i<3;++i) {
        if(a[i]<b[i]) return true;
        if(a[i]>b[i]) return false;
    }
    return false;
}
}

namespace mapper_server {

using namespace srrg_core;
using namespace std;

void SparseGrid::insertCloud(const Cloud3D &cloud){
    for(size_t ii=0; ii < cloud.size(); ii++) {

        Eigen::Vector3i idx = toGrid(cloud.at(ii).point());

        if(hasCell(idx)) {
            at(idx)->_points.insert(std::pair<Eigen::Vector3f,int> (cloud.at(ii).point(),ii));

            if(ii < _half)
                at(idx)->_first_cloud = true;
            else
                at(idx)->_second_cloud = true;
        } else {
            Cell* cell = new Cell(idx);
            cell->setCenter(_origin,_resolution);
            cell->_points.insert(std::pair<Eigen::Vector3f,int> (cloud.at(ii).point(),ii));
            Vector3iCellPtrMap::iterator it = begin();
            insert(it,std::pair<Eigen::Vector3i,Cell*>(idx,cell));
        }
    }

    cerr << "Sparse grid has " << size() << " cells" << endl;

}

Cloud3D SparseGrid::extractCloud(){
    for(Vector3iCellPtrMap::iterator it = begin();
        it != end();
        it++){

        const PointIndexMap& points = it->second->_points;
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

        for(PointIndexMap::const_iterator jt = points.begin();
            jt != points.end();
            jt++){
            const Eigen::Vector3f& point = jt->first;
            centroid += point;
        }

        centroid /= points.size();
        _cloud.push_back(RichPoint3D(centroid,Eigen::Vector3f::Zero(),0,Eigen::Vector3f (1,0,0)));
    }

    cerr << "Downsampled cloud has " << _cloud.size() << " points" << endl;

    return _cloud;
}

UnsignedCharImage SparseGrid::extractSurface(){
    IntImage indices;
    FloatImage elevations;
    //UnsignedCharImage classified;
    Eigen::Vector3f bottom;

    indices.release();
    elevations.release();

    bottom = _origin;
    bottom.z() = 0;

    int cols = _dimensions.x();
    int rows = _dimensions.y();

    indices.create(rows,cols);
    elevations.create(rows,cols);
    _classified.create(rows,cols);

    indices = -1;
    elevations = 5;
    _classified = 127;

    float robot_climb_step=0.1;
    float robot_height=0.5;

    for (int i = 0; i < _cloud.size(); i++){
        const RichPoint3D& point = _cloud.at(i);
        float z = point.point().z();
        Eigen::Vector3f projected_point = (point.point() - bottom)*_inverse_resolution;
        int row = projected_point.y();
        int col = projected_point.x();
        if(row>=rows || row<0)
            continue;
        if(col>=cols || col<0)
            continue;
        float& height = elevations.at<float> (row,col);
        int& idx = indices.at<int> (row,col);
        if(z<height){
            height = z;
            idx = i;
        }
    }

    for (int i = 0; i < _cloud.size(); i++){
        const RichPoint3D& point = _cloud.at(i);
        float z = point.point().z();
        Eigen::Vector3f projected_point = (point.point() - bottom)*_inverse_resolution;
        int row = projected_point.y();
        int col = projected_point.x();
        if(row>=rows || row<0)
            continue;
        if(col>=cols || col<0)
            continue;
        float& height = elevations.at<float> (row,col);
        int& idx = indices.at<int> (row,col);
        float min_obstacle_height = height+robot_climb_step;
        float max_obstacle_height = height+robot_height;
        if (z < min_obstacle_height)
            continue;
        if (z > max_obstacle_height)
            continue;
        idx = -2;
        height = z;
    }

    for (int r=0; r<rows; r++)
        for (int c=0; c<cols; c++) {
            int idx = indices.at<int>(r,c);
            if (idx==-1)
                continue;
            if (idx<-1){
                _classified.at<unsigned char>(r,c)=255;
                continue;
            }
            _classified.at<unsigned char>(r,c)=0;
        }

    for (int r=1; r<rows-1; r++)
        for (int c=1; c<cols-1; c++) {
            unsigned char & cell=_classified.at<unsigned char>(r,c);
            if (cell!=255)
                continue;
            bool one_big=false;
            for (int rr=-1; rr<=1; rr++)
                for (int cc=-1; cc<=1; cc++) {
                    if (rr==0 && cc==0)
                        continue;
                    one_big |= _classified.at<unsigned char>(r+rr,c+cc)==255;
                }
            if (! one_big) {
                cell=0;
            }
        }

    for (int r=0; r<rows; r++)
        for (int c=0; c<cols; c++) {
            int idx = indices.at<int>(r,c);
            if (idx<0)
                continue;
            unsigned char & cell=_classified.at<unsigned char>(r,c);
            if(cell==0){
                const RichPoint3D& point = _cloud.at(idx);
                Eigen::Vector3i index = toGrid(point.point());
                at(index)->setGround(true);
            }
        }

    return _classified;
}

bool SparseGrid::checkConnectivity(float connectivity_threshold){
    float ground_count=0;
    float overlap_count=0;
    for(Vector3iCellPtrMap::iterator it = begin();
        it != end();
        it++){
        Cell* cell = it->second;
        if(cell->ground() == true){
            ground_count++;
            if(cell->overlap() == true)
                overlap_count++;
        }
    }
    cerr << "Overlap: " << overlap_count << endl;
    cerr << "Ground: " << ground_count << endl;
    cerr << "Connection percentage: " << overlap_count/ground_count << endl;
    return (overlap_count/ground_count > connectivity_threshold) ? true : false;
}

}
