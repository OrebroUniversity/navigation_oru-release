// simpletexture.cpp
// by Glenn G. Chappell
// November 2003
//
// For CS 381
// Simple Example of Texture Mapping

#include <iostream>
using std::cerr;
using std::endl;
#include <stdlib.h>
//using std::exit;
#include <GL/freeglut.h>
//#include <GL/glut.h> // GLUT stuff - includes OpenGL headers

#include <SOIL/SOIL.h> // For loading textures...


// Global variables
// Window/viewport
const int startwinsize = 400; // Starting window width & height, in pixels
const double neardist = 0.5;  // Near & far clippng planes
const double fardist = 10.;

// Keyboard
const int ESCKEY = 27;        // ASCII value of escape character

// For texture
const int img_width = 32;
const int img_height = 32;
GLubyte the_image[img_height][img_width][3];
   // The image
   // 3rd subscript 0 = R, 1 = G, 2 = B
GLuint texname;  // Name for this texture
GLuint loaded_texture;

// display
// The GLUT display function
void display()
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   glEnable(GL_TEXTURE_2D);  // Turn on texture.
      // Turn it off if you draw something without a texture.

   // Draw square
   glPushMatrix();  // We are in model/view mode

      glTranslated(0., 0., -3.);      

      glBegin(GL_POLYGON);
         glTexCoord2d(1.0, 1.0); glVertex3d( 1.0, 1.0, 0.0);
         glTexCoord2d(0.0, 1.0); glVertex3d(-1.0,-1.0, 0.0);
         glTexCoord2d(0.0, 0.0); glVertex3d(-1.0,-1.0, 0.0);
         glTexCoord2d(1.0, 0.0); glVertex3d( 1.0,-1.0, 0.0);
      glEnd();

   glPopMatrix();

   glutSwapBuffers();
}


// reshape
// The GLUT reshape function
void reshape(int w, int h)
{
    glViewport(0, 0, w, h);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(60, double(w)/h, neardist, fardist);

   glMatrixMode(GL_MODELVIEW);  // Always go back to modelview mode
}


// keyboard
// The GLUT keyboard function
void keyboard(unsigned char key, int x, int y)
{
   switch (key)
   {
   case ESCKEY:  // ESC: Quit
      exit(0);
      break;
   }
}


// idle
// The GLUT idle function
void idle()
{
   // Print OpenGL errors, if there are any (for debugging)
   if (GLenum err = glGetError())
   {
      cerr << "OpenGL ERROR: " << gluErrorString(err) << endl;
   }

}


// makeimage
// Make image in the_image
// (Code taken without change from drawimage.cpp)
void makeimage()
{
   for (int i=0; i<img_width; ++i)
   {
      double x = double(i)/(img_width-1);
      for (int j=0; j<img_height; ++j)
      {
         double y = double(j)/(img_height-1);

         the_image[j][i][0] = int(x*255)*15 % 256;
         the_image[j][i][1] = int(y*255)*15 % 256;
         the_image[j][i][2] = 0.75*255;
      }
   }
}


// init
// Initializes GL states
// Called by main
void init()
{
   glClearColor(1.0, 1.0, 1.0, 0.0);
   glEnable(GL_DEPTH_TEST);

   // Texture set-up

   // Basic image set-up
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   makeimage();

   // Generate a texture name and bind it to GL_TEXTURE_2D.
   // Then we no longer use texname, since GL_TEXTURE_2D is bound to it.
   // NOTE: binding a texture name is actually NOT necessary when we
   //  use only one texture, but it does not hurt.
   glGenTextures(1, &texname);
   glBindTexture(GL_TEXTURE_2D, texname);

   // Set various texture parameters
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);  // Needs OpenGL 1.1

   // Create the actual texture
   glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img_width, img_height,
                0, GL_RGB, GL_UNSIGNED_BYTE, the_image);
}

void init2()
{
   glClearColor(1.0, 1.0, 1.0, 0.0);
   glEnable(GL_DEPTH_TEST);

   // Texture set-up

   // Basic image set-up
   glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
   
   texname = SOIL_load_OGL_texture
     (
      "image.png",
      SOIL_LOAD_AUTO,
      SOIL_CREATE_NEW_ID,
      SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT
      );

   
   /* check for an error during the load process */
   if( 0 == texname )
   {
     std::cerr <<  "SOIL loading error: " << SOIL_last_result();
   }

   // Generate a texture name and bind it to GL_TEXTURE_2D.
   // Then we no longer use texname, since GL_TEXTURE_2D is bound to it.
   // NOTE: binding a texture name is actually NOT necessary when we
   //  use only one texture, but it does not hurt.
   glGenTextures(1, &texname);
   glBindTexture(GL_TEXTURE_2D, texname);

   // Set various texture parameters
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
   glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);  // Needs OpenGL 1.1
}


int main(int argc, char ** argv)
{
   // Initialize OpenGL/GLUT
   glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

   // Make a window
   glutInitWindowSize(startwinsize, startwinsize);
   glutInitWindowPosition(50, 50);
   glutCreateWindow("CS 381 - Simple Texture Demo");

   // Initialize GL states & register callbacks
   init2();
   glutDisplayFunc(display);
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutIdleFunc(idle);

   // Do something
   glutMainLoop();

   return 0;
}
