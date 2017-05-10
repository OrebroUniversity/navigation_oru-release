#ifndef CITITRUCK_FORKS_SDF_H
#define CITITRUCK_FORKS_SDF_H

#include "sdf.h"

namespace sdf {

//! Define a Euro pallet - warning, this will also do allocation
std::vector<sdf::Primitive*>
defineCitiTruckForks() {

    std::vector<sdf::Primitive*> geometry;
    Eigen::Vector4d geometryParam;

//DEFINING THE SDF OF THE CITITRUCK FORKS - USED FOR CALIBRATION
//The origin of the frame cooresponds to the origin of the vehicle.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

    // Blocks -> 260 x 135 x 85 (the biggest part of the forks where the wheels are mounted.
    // Offset in x -> 22.5:
    double xo = 0.0225;

    // Left, right:
    double yo_right = 0.215;
    double yo_left = -0.215;
        
    geometryParam << 0.260, 0.135, 0.085;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    T.translation() = Eigen::Vector3d(xo, yo_left, 0.085/2.);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
    
    T.translation() = Eigen::Vector3d(xo,yo_right, 0.085/2.);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    return geometry;
}

} // namespace

#endif
