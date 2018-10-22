/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of the Willow Garage, Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>
#include "rviz/frame_manager.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "myviz.h"
// BEGIN_TUTORIAL
// Constructor for MyViz. This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{
  // Construct and lay out labels and slider controls.
  QLabel* thickness_label = new QLabel( "Line Thickness" );
  QSlider* thickness_slider = new QSlider( Qt::Horizontal );
  thickness_slider->setMinimum( 1 );
  thickness_slider->setMaximum( 100 );
  QLabel* cell_size_label = new QLabel( "Cell Size" );
  QSlider* cell_size_slider = new QSlider( Qt::Horizontal );
  cell_size_slider->setMinimum( 1 );
  cell_size_slider->setMaximum( 100 );
  QLabel* aspect_ratio_label = new QLabel( "Aspect Ratio" );
  QSlider* aspect_ratio_slider = new QSlider( Qt::Horizontal );
  aspect_ratio_slider->setMinimum( 1 );
  aspect_ratio_slider->setMaximum( 400 );
  QGridLayout* controls_layout = new QGridLayout();
  controls_layout->addWidget( thickness_label, 0, 0 );
  controls_layout->addWidget( thickness_slider, 0, 1 );
  controls_layout->addWidget( cell_size_label, 1, 0 );
  controls_layout->addWidget( cell_size_slider, 1, 1 );
  controls_layout->addWidget( aspect_ratio_label, 2, 0 );
  controls_layout->addWidget( aspect_ratio_slider, 2, 1 );
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addLayout( controls_layout );
  main_layout->addWidget( render_panel_ );
  
  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );
  // Make signal/slot connections.
  connect( thickness_slider, SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
  connect( cell_size_slider, SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));
  connect( aspect_ratio_slider, SIGNAL( valueChanged( int )), this, SLOT( setAspectRatio( int )));
  // Next we initialize the main RViz classes.
  //
  // The VisualizationManager is the container for Display objects,
  // holds the main Ogre scene, holds the ViewController, etc. It is
  // very central and we will probably need one in every usage of
  // librviz.
  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->setFixedFrame( "/world" ); //robot1/base_footprint
  std::cout << "getFixedFrame() : " << manager_->getFixedFrame().toStdString() << std::endl;
  manager_->startUpdate();


  // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( grid_ != NULL );
  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Billboards" );
  grid_->subProp( "Color" )->setValue( Qt::yellow );
  marker_ = manager_->createDisplay( "rviz/Marker", "marker", true );
  marker_->setFixedFrame("robot1/base_footprint");

  // Initialize the slider values.
  thickness_slider->setValue( 25 );
  cell_size_slider->setValue( 10 );

  // Test code how to change the aspect ratio etc. below...
  
  // QSizePolicy qsp(QSizePolicy::Preferred,QSizePolicy::Preferred);
  // qsp.setHeightForWidth(true);
  // setSizePolicy(qsp);
  //resize(width(), height()/2.);
  Ogre::Camera* cam = render_panel_->getCamera();
  
  std::cout << "cam : " << cam << std::endl;
  std::cout << "getAspectRatio() : " << cam->getAspectRatio() << std::endl;
  cam->setAspectRatio(1.);
  std::cout << "getAspectRatio() : " << cam->getAspectRatio() << std::endl;
}
// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}
// This function is a Qt slot connected to a QSlider's valueChanged()
// signal. It sets the line thickness of the grid by changing the
// grid's "Line Width" property.
void MyViz::setThickness( int thickness_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
  }
}
// This function is a Qt slot connected to a QSlider's valueChanged()
// signal. It sets the cell size of the grid by changing the grid's
// "Cell Size" Property.
void MyViz::setCellSize( int cell_size_percent )
{
  if( grid_ != NULL )
  {
    grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
  }
  // QPixmap pixmap(this->size()); 
  // render(&pixmap);
  // pixmap.save("tmp.png");
}

void MyViz::setAspectRatio( int aspect_ratio_percent )
{
  Ogre::Camera* cam = render_panel_->getCamera();
  if (cam != NULL)
  {
    cam->setAspectRatio( aspect_ratio_percent / 100.0f );
    std::cout << "getAspectRatio() : " << cam->getAspectRatio() << std::endl;

    Ogre::Matrix4 p = cam->getProjectionMatrix();

    std::cout << "p : " << p << std::endl;

    Ogre::Matrix4 wp;
    cam->getWorldTransforms(&wp);
    
    std::cout << "wp : " << wp << std::endl;

    Ogre::Vector3 pos = cam->getPosition();
    std::cout << "pos : " << pos << std::endl;
    //    pos[0] += 0.01;
    cam->move(pos);
  }

  rviz::FrameManager* fm = manager_->getFrameManager();
  if (fm != NULL) 
  {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    fm->getTransform("/robot1/base_footprint", ros::Time::now(), position, orientation);
    std::cout << "position : " << position << std::endl;
    std::cout << "orientation : " << orientation << std::endl;
  }
  rviz::ViewManager* vm = manager_->getViewManager();
  if (vm != NULL) 

  {
    rviz::ViewController* vc = vm->getCurrent();
    if (vc != NULL) 
    {
      std::cout << "vc : " << vc << std::endl;
      Ogre::Camera* cam2 = vc->getCamera();
      if (cam2 != NULL) 
      {
        std::cout << "cam2 : " << cam2 << std::endl;
        cam2->setPosition(10, 10, 10);
      }
      //      vc->lookAt(0,0,0);
    }
  }

}
