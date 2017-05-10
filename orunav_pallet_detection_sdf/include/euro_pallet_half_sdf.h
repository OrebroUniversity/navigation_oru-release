#ifndef EURO_PALLET_HALF_SDF_H
#define EURO_PALLET_HALF_SDF_H

#include "sdf.h"

namespace sdf {

//! Define a Euro pallet halfsize - warning, this will also do allocation
std::vector<sdf::Primitive*>
defineEuroPalletHalf() {

    std::vector<sdf::Primitive*> geometry;
    Eigen::Vector4d geometryParam;

    // Translation starts from the top z coordinate of the geometry.
    // The KUKA booth pallet
    // Big block on top
    geometryParam << 0.6, 0.8, 0.04, 0.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    
    T.translation() = Eigen::Vector3d(0., 0., 0.18);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
    
    // Four smaller blocks
    geometryParam << 0.14, 0.096, 0.08, 0.0;
    T.translation() = Eigen::Vector3d(-0.3+0.14/2., -0.4+0.096/2., 0.12);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(-0.3+0.14/2., 0.4-0.096/2., 0.12);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(0.3-0.14/2., -0.4+0.096/2., 0.12);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(0.3-0.14/2., 0.4-0.096/2., 0.12);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
    
    // Two bigger blocks
    geometryParam << 0.14, 0.14, 0.08, 0.0;
    T.translation() = Eigen::Vector3d(-0.3+0.14/2., 0.0, 0.12);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(0.3-0.14/2., 0.0, 0.12);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
    
    // Two smaller long on the ground
    geometryParam << 0.6, 0.096, 0.04;
    T.translation() = Eigen::Vector3d(0., -0.4+0.096/2., 0.06);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(0., 0.4-0.096/2., 0.06);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    // One big long on the ground
    geometryParam << 0.6, 0.14, 0.04;
    T.translation() = Eigen::Vector3d(0., 0., 0.06);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
    

//DEFINING THE SDF OF AN EU PALLET
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
    double xo = 0;//-0.3;
    double yo = -0.4;

    double zo = 0.06;
#if 0
    // Use a big block
    geometryParam << 0.6, 0.8, 0.18, 0.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    T.translation() = Eigen::Vector3d(0., 0., 0.18);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); //the last flag flips the sign of the box


    // Use a big board on the top.
    geometryParam << 0.6, 0.8, 0.02, 0.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.40, zo + 0.120);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); //the last flag flips the sign of the box
     
    // Boards 600x100x20 (there are 3 of them)
    geometryParam << 0.600, 0.08, 0.02, 0.0;

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.05, 0.010);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.40, 0.010);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    T.translation() = Eigen::Vector3d(xo+0.0, yo+0.75, 0.010);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

    // Blocks 80x80x100 (there are 9 of them)
    geometryParam << 0.08, 0.08, 0.100, 0.0;

    T.translation() = Eigen::Vector3d(xo-0.25, yo+0.05, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0.00, yo+0.05, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0.25, yo+0.05, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo-0.25, yo+0.40, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0.00, yo+0.40, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0.25, yo+0.40, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo-0.25, yo+0.75, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0.00, yo+0.75, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));

    T.translation() = Eigen::Vector3d(xo+0.25, yo+0.75, 0.07);
    geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false));
#endif

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
