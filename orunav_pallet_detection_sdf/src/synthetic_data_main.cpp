#include "sdf.h"
#include <vector>
#include <random>
#include <vtkImageData.h>
#include <vtkFloatArray.h>
#include <vtkXMLImageDataWriter.h>
// #include <vtkXMLImageDataReader.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

int main( int argc, char* argv[] )
{
    Eigen::Affine3d Tcam = Eigen::Affine3d::Identity();
    Tcam.translation() = Eigen::Vector3d(-0.3,-1.5,0.2);
    Tcam.linear() = (Eigen::AngleAxisd(-M_PI/16, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                  Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()) ).toRotationMatrix(); //rotation about X and about Y
    

    Eigen::Vector4f cam_params(525.f, 525.f, 320.f, 240.f);

    std::vector<sdf::Primitive*> geometry;
    Eigen::Vector4d geometryParam;

// //DEFINING THE SDF OF AN EU PALLET
// //-----------------------------------------------------------------------------
// //-----------------------------------------------------------------------------
//     // Boards 100x1200x22 (there are 4 of them)
//     geometryParam << 0.100, 1.200, 0.022, 0.0;
//     Eigen::Affine3d T = Eigen::Affine3d::Identity();

//     T.translation() = Eigen::Vector3d(0.0+0.05, 0.0, 0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); //the last flag flips the sign of the box
    
//     T.translation() = Eigen::Vector3d(0.700+0.05, 0.0, 0.0+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
//     T.translation() = Eigen::Vector3d(0.1875+0.05, 0.0, 0.122+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0.5125+0.05, 0.0, 0.122+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 


//     // Boards 145x1200x22 (there are 4 of them)
//     geometryParam << 0.145, 1.200, 0.022, 0.0;

//     T.translation() = Eigen::Vector3d(0.3275+0.0725, 0.0, 0.0+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
//     T.translation() = Eigen::Vector3d(0.0+0.0725, 0.0, 0.122+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
//     T.translation() = Eigen::Vector3d(0.3275+0.0725, 0.0, 0.122+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0.655+0.0725, 0.0, 0.122+0.011);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     // Boards 800x145x22 (there are 3 of them)
//     geometryParam << 0.800, 0.145, 0.022, 0.0;

//     T.translation() = Eigen::Vector3d(0+.4, -0.6+0.0725, 0.111);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    
//     T.translation() = Eigen::Vector3d(0+.4, +0.6-0.0725, 0.111);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0+.4, 0.0, 0.111);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
    

//     // Blocks 145x145x78 (there are 3 of them)
//     geometryParam << 0.145, 0.145, 0.078, 0.0;

//     T.translation() = Eigen::Vector3d(0.40, -.600+0.0725, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0.40, +.600-0.0725, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0.40,  0, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     // Blocks 100x145x78 (there are 6 of them)
//     geometryParam << 0.100, 0.145, 0.078, 0.0;

//     T.translation() = Eigen::Vector3d(0.05, -.600+0.0725, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0.05, +.600-0.0725, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(0.05,  0, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(.80-0.05, -.600+0.0725, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(.80-0.05, +.600-0.0725, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 

//     T.translation() = Eigen::Vector3d(.80-0.05,  0, 0.078/2+0.022);
//     geometry.push_back(new sdf::Box( sdf::bool_union, T, geometryParam , false)); 
// //-----------------------------------------------------------------------------
// //-----------------------------------------------------------------------------

    // Modify the orientation of this. x-> along the main direction of the pallet...

    // Boards 100x1200x22 (there are 4 of them)
    geometryParam << 1.200, 0.100, 0.022, 0.0;
    Eigen::Affine3d T = Eigen::Affine3d::Identity();

    double xo = 0;//-0.6;
    double yo = -0.4;

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



    // The SDF is given in the camera coordinate system where x -> width, y -> height and z -> depth

    // Henrik : using values from the ASUS camera... 
    Eigen::Vector4f cam_params_asus(570.342f, 570.342f, 314.5f, 235.5f);

        sdf::Render(Tcam,        //cam pose
                    480,         //height
                    640,         //width 
                    cam_params_asus,  //camera parameters
                    100,         //max number of steps
                    5,           //max ray lentgh (meters)
                    0.001,       //error tolerance 
                    geometry     //geometry vector
                    );
          cv::waitKey(200);

//Sample and save the SDF as a 3D image

      vtkSmartPointer<vtkImageData> tsdf_volume = vtkSmartPointer<vtkImageData>::New();

//number of samples
      int X=200;
      int Y=300;
      int Z=100;

//sampling period      
      float cellsize = 0.005;

      tsdf_volume->SetDimensions(X,Y,Z);
      tsdf_volume->SetOrigin( 0,0,0);
      tsdf_volume->SetSpacing(cellsize,cellsize,cellsize);
      
      vtkSmartPointer<vtkFloatArray> distance = vtkSmartPointer<vtkFloatArray>::New();
      
      int numCells = X*Y*Z;
      distance->SetNumberOfTuples(numCells);

      int offset_y, offset_z;
      for (int idx_z = 0; idx_z < Z; ++idx_z)
      {
        offset_z = idx_z*X*Y;
        for (int idx_y = 0; idx_y < Y; ++idx_y)
        {
          offset_y = idx_y*X;
          for (int idx_x = 0; idx_x < X; ++idx_x)
          {
            Eigen::Vector3d location(idx_x, idx_y, idx_z);
            location = location*cellsize - Eigen::Vector3d((X*cellsize-0.8)/2, 0.6+(Y*cellsize-1.2)/2, (Z*cellsize-0.144)/2) ;      
            int offset = idx_x + offset_y + offset_z;

            //uncomment this if you want a truncated SDF instead (trung_neg and trung_plus have to be defined)
            // float signed_dist = std::min(std::max(trunc_neg,(sdf::SDF(location, geometry)-tolerance)),trunc_plus);
            float signed_dist = sdf::SDF(location, geometry);

            distance->SetValue(offset, signed_dist );
            
          }
        }
      }

        tsdf_volume->GetPointData()->AddArray(distance);
        distance->SetName("Distance");


        vtkSmartPointer<vtkXMLImageDataWriter> writer =
        vtkSmartPointer<vtkXMLImageDataWriter>::New();
        writer->SetFileName("EU_pallet.vti");

        writer->SetInput(tsdf_volume);
        writer->Write();
  

   return 0;
}
