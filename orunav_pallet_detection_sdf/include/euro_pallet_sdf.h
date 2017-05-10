#ifndef EURO_PALLET_SDF_H
#define EURO_PALLET_SDF_H

#include "sdf.h"

namespace sdf {

//! Define a Euro pallet - warning, this will also do allocation
std::vector<sdf::Primitive*>
defineEuroPallet() {

    std::vector<sdf::Primitive*> geometry;
    Eigen::Vector4d geometryParam;

//DEFINING THE SDF OF AN EU PALLET
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
    double xo = 0;//-0.6;
    double yo = -0.4;


    // Boards 100x1200x22 (there are 4 of them)
    geometryParam << 1.200, 0.100, 0.022, 0.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.0+0.05, 0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); //the last flag flips the sign of the box
    
    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.700+0.05, 0.0+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.1875+0.05, 0.122+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.5125+0.05, 0.122+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 


    // Boards 145x1200x22 (there are 4 of them)
    geometryParam << 1.200, 0.145, 0.022, 0.0;

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.3275+0.0725, 0.0+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.0+0.0725, 0.122+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.3275+0.0725, 0.122+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.655+0.0725, 0.122+0.011);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    // Boards 800x145x22 (there are 3 of them)
    geometryParam << 0.145, 0.800, 0.022, 0.0;

    T.translation() = Eigen::Vector3d(xo+-0.6+0.0725, yo+0+.4, 0.111);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
    T.translation() = Eigen::Vector3d(xo+0.6-0.0725, yo+0+.4, 0.111);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0+.4, 0.111);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    

    // Blocks 145x145x78 (there are 3 of them)
    geometryParam << 0.145, 0.145, 0.078, 0.0;

    T.translation() = Eigen::Vector3d(xo+-.600+0.0725, yo+0.40, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+.600-0.0725, yo+0.40, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0, yo+0.40, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    // Blocks 100x145x78 (there are 6 of them)
    geometryParam << 0.145, 0.100, 0.078, 0.0;

    T.translation() = Eigen::Vector3d(xo-.600+0.0725, yo+0.05, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+.600-0.0725, yo+0.05, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+ 0, yo+0.05, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo-.600+0.0725, yo+.80-0.05, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+.600-0.0725, yo+.80-0.05, 0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0, yo+.80-0.05,  0.078/2+0.022);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));


// OLD --------------------------------------------------------------------------------------------


    /* // Boards 100x1200x22 (there are 4 of them) */
    /* geometryParam << 0.100, 1.200, 0.022, 0.0; */
    /* Eigen::Affine3d T = Eigen::Affine3d::Identity(); */

    /* T.translation() = Eigen::Vector3d(0.0+0.05, 0.0, 0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); //the last flag flips the sign of the box */
    
    /* T.translation() = Eigen::Vector3d(0.700+0.05, 0.0, 0.0+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */
    
    /* T.translation() = Eigen::Vector3d(0.1875+0.05, 0.0, 0.122+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0.5125+0.05, 0.0, 0.122+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */


    /* // Boards 145x1200x22 (there are 4 of them) */
    /* geometryParam << 0.145, 1.200, 0.022, 0.0; */

    /* T.translation() = Eigen::Vector3d(0.3275+0.0725, 0.0, 0.0+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */
    
    /* T.translation() = Eigen::Vector3d(0.0+0.0725, 0.0, 0.122+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */
    
    /* T.translation() = Eigen::Vector3d(0.3275+0.0725, 0.0, 0.122+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0.655+0.0725, 0.0, 0.122+0.011); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* // Boards 800x145x22 (there are 3 of them) */
    /* geometryParam << 0.800, 0.145, 0.022, 0.0; */

    /* T.translation() = Eigen::Vector3d(0+.4, -0.6+0.0725, 0.111); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */
    
    /* T.translation() = Eigen::Vector3d(0+.4, +0.6-0.0725, 0.111); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0+.4, 0.0, 0.111); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */
    

    /* // Blocks 145x145x78 (there are 3 of them) */
    /* geometryParam << 0.145, 0.145, 0.078, 0.0; */

    /* T.translation() = Eigen::Vector3d(0.40, -.600+0.0725, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0.40, +.600-0.0725, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0.40,  0, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* // Blocks 100x145x78 (there are 6 of them) */
    /* geometryParam << 0.100, 0.145, 0.078, 0.0; */

    /* T.translation() = Eigen::Vector3d(0.05, -.600+0.0725, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0.05, +.600-0.0725, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(0.05,  0, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(.80-0.05, -.600+0.0725, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(.80-0.05, +.600-0.0725, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */

    /* T.translation() = Eigen::Vector3d(.80-0.05,  0, 0.078/2+0.022); */
    /* geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));  */
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
    return geometry;
}

} // namespace

#endif
