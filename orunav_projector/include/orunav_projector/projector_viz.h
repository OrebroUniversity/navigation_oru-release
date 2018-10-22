#pragma once

#include <ndt_visualisation/ndt_viz.h>
#include <orunav_projector/projector_utils.h>


typedef struct NDTVizGlutPoint { GLfloat p[3]; } NDTVizGlutPoint;

class NDTVizGlutPolygon : public NDTVizGlutObject {
public:
  
  void setPolygon(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts, double height = 0.) {
    pts_.clear();
    for (unsigned int i = 0; i < pts.size(); i++) {
      NDTVizGlutPoint p;
      p.p[0] = pts[i](0);
      p.p[1] = pts[i](1);
      p.p[2] = height;
      pts_.push_back(p);
    }
  }

  void draw() {
    glDisable(GL_LIGHTING);

    glBegin(GL_POLYGON);
    glColor4f(m_color.R,m_color.G,m_color.B,m_color.A);
    for (unsigned int i = 0; i < pts_.size(); i++) {
      glVertex3fv(pts_[i].p);
    }
    glEnd();

    glEnable(GL_LIGHTING);

  }

    void setColor4(float R, float G, float B, float A) {
    m_color.R = R; m_color.G = G; m_color.B = B; m_color.A = A;
  }

private:
  std::vector<NDTVizGlutPoint> pts_;
  NDTVizGlutColor4f m_color;
};

class NDTVizGlutTexturePolygon : public NDTVizGlutObject {
public:

  void loadTexture(const std::string &fileName) {
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
    // The following two lines enable semi transparent
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
    int width, height;
    bool has_alpha;
    bool success = loadPngImage(fileName, width, height, has_alpha, &texture_image_);
    if (!success) {
      std::cout << "Unable to load png file : " << fileName << std::endl;
      return;
    }
    std::cout << "Image loaded " << width << " " << height << " alpha " << has_alpha << std::endl;
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, has_alpha ? 4 : 3, width,
                 height, 0, has_alpha ? GL_RGBA : GL_RGB, GL_UNSIGNED_BYTE,
                 texture_image_);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glEnable(GL_TEXTURE_2D);
    glShadeModel(GL_FLAT);
  }

  void setPolygon(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts) {
    pts_.clear();
    for (unsigned int i = 0; i < pts.size(); i++) {
      NDTVizGlutPoint p;
      p.p[0] = pts[i](0);
      p.p[1] = pts[i](1);
      p.p[2] = 0.;
      pts_.push_back(p);
    }
    updateTexCoord(pts, tex_coord_);
  }

  void draw() {
    
    glBegin(GL_POLYGON);
    
    for (unsigned int i = 0; i < pts_.size(); i++) {
      glTexCoord2f(tex_coord_[i](0),tex_coord_[i](1));
      // std::cout << "pts_[i].p : " << pts_[i].p[0] << " " << pts_[i].p[1] << " " << pts_[i].p[2] << std::endl;
      // std::cout << "tex_coord_[i] : " << tex_coord_[i] << std::endl;
      glVertex3fv(pts_[i].p);
    }
    glEnd();

  }
private:
  std::vector<NDTVizGlutPoint> pts_;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > tex_coord_;
  GLubyte *texture_image_;
};

class ProjectorViz : public NDTViz {

public:

  NDTVizGlutSetOfLines gl_breakregion;
  NDTVizGlutSetOfLines gl_slowdownregion;
  NDTVizGlutPointCloudColor gl_lasercloud;
  NDTVizGlutSetOfLines gl_lasercloud_lines;
  NDTVizGlutPolygon gl_poly_breakregion;
  NDTVizGlutPolygon gl_poly_slowdownregion;
  NDTVizGlutPolygon test_region;
  NDTVizGlutTexturePolygon gl_texturepoly_slowdownregion;
  ProjectorViz() 
  {

  }

  void setSlowdownRegionTexture(std::string &fileName) {
    gl_texturepoly_slowdownregion.loadTexture(fileName);
  }

  void addEBrakeRegion(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts, double R=1.0, double G=1.0, double B=1.0) 
  {
    // gl_breakregion.clear();
    // if (pts.size() < 2)
    //   return;
    // for (unsigned int i = 0; i < pts.size()-1; i++) {
    //   unsigned int j = i+1;
    //   gl_breakregion.appendLine(pts[i](0),pts[i](1),0.,
    //                             pts[j](0),pts[j](1),0.);
    // }
    // gl_breakregion.setColor(R,G,B);
    // gl_breakregion.setThickness(5.);
    // win3D->addObject(&gl_breakregion);

    gl_poly_breakregion.setPolygon(pts);
    gl_poly_breakregion.setColor4(R, G, B, 0.5);
    win3D->addObject(&gl_poly_breakregion);
  }

  void addSlowdownRegion(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts, double R=1.0, double G=1.0, double B=1.0)
  {
    // gl_slowdownregion.clear();
    // if (pts.size() < 2)
    //   return;
    // for (unsigned int i = 0; i < pts.size()-1; i++) {
    //   unsigned int j = i+1;
    //   gl_slowdownregion.appendLine(pts[i](0),pts[i](1),0.,
    //                             pts[j](0),pts[j](1),0.);
    // }
    // gl_slowdownregion.setColor(0.9, 0.9, 0.9);
    // gl_slowdownregion.setThickness(5.);
    // win3D->addObject(&gl_slowdownregion);

    gl_poly_slowdownregion.setPolygon(pts, -0.0001);
    gl_poly_slowdownregion.setColor4(0.,1.,0.,1.);
    win3D->addObject(&gl_poly_slowdownregion);
    
    //gl_texturepoly_slowdownregion.setPolygon(pts);
    //win3D->addObject(&gl_texturepoly_slowdownregion);
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // RAVI -- new function for making a color filled arrow

    void addColorArrow(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts, double R=1.0, double G=1.0, double B=1.0)
    {
      test_region.setPolygon(pts, -0.0001);
      test_region.setColor4(R,G,B,1.);
      win3D->addObject(&test_region);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  void addLaserCloud(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts, double R=1.0, double G=1.0, double B=1.0)
  {
#if 0
    gl_lasercloud.clear();
    if (pts.size() < 2)
      return;
    for (unsigned int i = 0; i < pts.size()-1; i++) {
      unsigned int j = i+1;
      gl_lasercloud.push_back(pts[i](0),pts[i](1),0.,
                              R, G, B);
    }
    gl_lasercloud.setPointSize(1);
    win3D->addObject(&gl_lasercloud);
#endif
    gl_lasercloud_lines.clear();
    if (pts.size() < 2)
      return;
    for (unsigned int i = 0; i < pts.size()-1; i++) {
      unsigned int j = i+1;
      gl_lasercloud_lines.appendLine(pts[i](0),pts[i](1),0.,
                                pts[j](0),pts[j](1),0.);
    }
    gl_lasercloud_lines.setColor(R,G,B);
    gl_lasercloud_lines.setThickness(5.);
    win3D->addObject(&gl_lasercloud_lines);
  }


  void addAllowedRegion(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts)
  {

  }

  void addZebraStripes(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &pts)
  {

  }


};
