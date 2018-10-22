
#include <GL/freeglut.h>
#include <stdlib.h>
#include <stdio.h>
#include <png.h>
#include <iostream>
 
GLubyte *textureImage;
float rotateX = 0;
float rotateY = 0;
 
int mouseX;
int mouseY;
 
bool loadPngImage(char *name, int &outWidth, int &outHeight, bool &outHasAlpha, GLubyte **outData) {
    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    FILE *fp;
 
    if ((fp = fopen(name, "rb")) == NULL)
        return false;
 
    /* Create and initialize the png_struct
     * with the desired error handler
     * functions.  If you want to use the
     * default stderr and longjump method,
     * you can supply NULL for the last
     * three parameters.  We also supply the
     * the compiler header file version, so
     * that we know if the application
     * was compiled with a compatible version
     * of the library.  REQUIRED
     */
    png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING,
                                     NULL, NULL, NULL);
 
    if (png_ptr == NULL) {
        fclose(fp);
        return false;
    }
 
    /* Allocate/initialize the memory
     * for image information.  REQUIRED. */
    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        fclose(fp);
        png_destroy_read_struct(&png_ptr, NULL, NULL);
        return false;
    }
 
    /* Set error handling if you are
     * using the setjmp/longjmp method
     * (this is the normal method of
     * doing things with libpng).
     * REQUIRED unless you  set up
     * your own error handlers in
     * the png_create_read_struct()
     * earlier.
     */
    if (setjmp(png_jmpbuf(png_ptr))) {
        /* Free all of the memory associated
         * with the png_ptr and info_ptr */
        png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
        fclose(fp);
        /* If we get here, we had a
         * problem reading the file */
        return false;
    }
 
    /* Set up the output control if
     * you are using standard C streams */
    png_init_io(png_ptr, fp);
 
    /* If we have already
     * read some of the signature */
    png_set_sig_bytes(png_ptr, sig_read);
 
    /*
     * If you have enough memory to read
     * in the entire image at once, and
     * you need to specify only
     * transforms that can be controlled
     * with one of the PNG_TRANSFORM_*
     * bits (this presently excludes
     * dithering, filling, setting
     * background, and doing gamma
     * adjustment), then you can read the
     * entire image (including pixels)
     * into the info structure with this
     * call
     *
     * PNG_TRANSFORM_STRIP_16 |
     * PNG_TRANSFORM_PACKING  forces 8 bit
     * PNG_TRANSFORM_EXPAND forces to
     *  expand a palette into RGB
     */
    png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_STRIP_16 | PNG_TRANSFORM_PACKING | PNG_TRANSFORM_EXPAND, NULL);
 
    png_uint_32 width, height;
    int bit_depth;
    png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type,
                 &interlace_type, NULL, NULL);
    outWidth = width;
    outHeight = height;
 
    std::cout << "bit depth : " << bit_depth << std::endl;
    std::cout << "color type: " << color_type << std::endl;

    unsigned int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
    std::cout << "row_bytes : " << row_bytes << std::endl;
    *outData = (unsigned char*) malloc(row_bytes * outHeight);
 
    png_bytepp row_pointers = png_get_rows(png_ptr, info_ptr);
 
    for (int i = 0; i < outHeight; i++) {
        // note that png is ordered top to
        // bottom, but OpenGL expect it bottom to top
        // so the order or swapped
      memcpy(*outData+(row_bytes * (outHeight-1-i)), row_pointers[i], row_bytes);
    }
 
    /* Clean up after the read,
     * and free any memory allocated */
    png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
 
    /* Close the file */
    fclose(fp);
 
    /* That's it */
    return true;
}
 
void init(void) {
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glEnable(GL_DEPTH_TEST);
    // The following two lines enable semi transparent
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
    int width, height;
    bool hasAlpha;
    char filename[] = "image.png";
    bool success = loadPngImage(filename, width, height, hasAlpha, &textureImage);
    hasAlpha = true; // This is not currently set in the above function(!).
    hasAlpha = false;
    if (!success) {
        std::cout << "Unable to load png file" << std::endl;
        return;
    }
    std::cout << "Image loaded " << width << " " << height << " alpha " << hasAlpha << std::endl;
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, hasAlpha ? 4 : 3, width,
                 height, 0, hasAlpha ? GL_RGBA : GL_RGB, GL_UNSIGNED_BYTE,
                 textureImage);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glEnable(GL_TEXTURE_2D);
    glShadeModel(GL_FLAT);
}
 
void display(void) {
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -3.6);
    glRotatef(rotateX, 0,1,0);
    glRotatef(rotateY, 1,0,0);
 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBegin(GL_POLYGON/*QUADS*/);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);

    glTexCoord2f(0.0, 1.0);
    glVertex3f(0.0, 1.0, 0.0);

    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0, 1.0, 0.0);

    glTexCoord2f(0.4, 0.0);
    glVertex3f(0.4, 0.0, 0.0);
 
    glEnd();
    glutSwapBuffers();
}
 
void myReshape(int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, 1.0 * (GLfloat) w / (GLfloat) h, 1.0, 30.0);
    glMatrixMode(GL_MODELVIEW);
}
 
void mousePassive(int x, int y){
    mouseX = x;
    mouseY = y;
}
 
void mouseMotion(int x, int y){
    const float SPEED = 2;
 
    rotateX += (mouseX-x)/SPEED;
    rotateY += (mouseY-y)/SPEED;
    mousePassive(x, y);
    glutPostRedisplay();
}
 
int
main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH);
    glutCreateWindow("PNG texture");
    glutMotionFunc(mouseMotion);
    glutPassiveMotionFunc(mousePassive);
    init();
    glutReshapeFunc(myReshape);
    glutDisplayFunc(display);
    std::cout << "Use mouse drag to rotate." << std::endl;
    glutMainLoop();
    return 0;
}
