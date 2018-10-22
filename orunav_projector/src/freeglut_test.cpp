#define FREEGLUT_STATIC // defined so you can link to freeglut_static.lib when compiling
#define GLEW_STATIC     // defined so you can link to glew's static .lib when compiling

//#include <GL/glew.h>     // has to be included before gl.h, or any header that includes gl.h
#include <GL/freeglut.h>

void draw()
{
    // code for rendering here
    glutSwapBuffers();   // swapping image buffer for double buffering
    glutPostRedisplay(); // redrawing. Omit this line if you don't want constant redraw
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA); // enabling double buffering and RGBA
    glutInitWindowSize(600, 600);
    glutCreateWindow("OpenGL"); // creating the window
    glutFullScreen();           // making the window full screen
    glutDisplayFunc(draw);      // draw is your function for redrawing the screen

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    
    glutMainLoop();

    return 0;
}
