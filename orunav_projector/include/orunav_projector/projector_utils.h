#include <GL/freeglut.h>
#include <stdlib.h>
#include <stdio.h>
#include <png.h>
#include <iostream>


bool loadPngImage(const std::string &name, int &outWidth, int &outHeight, bool &outHasAlpha, GLubyte **outData) {
  outHasAlpha = false; // TODO - this needs to be set(!).

    png_structp png_ptr;
    png_infop info_ptr;
    unsigned int sig_read = 0;
    int color_type, interlace_type;
    FILE *fp;
 
    if ((fp = fopen(name.c_str(), "rb")) == NULL)
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
 
    unsigned int row_bytes = png_get_rowbytes(png_ptr, info_ptr);
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


// TODO - this should have the point2d interface instead...
void getMinMaxPoint2dValue(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > const &pts, Eigen::Vector2d &minPoint2d, Eigen::Vector2d &maxPoint2d) {
  minPoint2d(0) = std::numeric_limits<double>::max();
  minPoint2d(1) = std::numeric_limits<double>::max();

  maxPoint2d(0) = std::numeric_limits<double>::min();
  maxPoint2d(1) = std::numeric_limits<double>::min();

  for (size_t i = 0; i < pts.size(); i++) {
    if (minPoint2d(0) > pts[i](0))
      minPoint2d(0) = pts[i](0);

    if (minPoint2d(1) > pts[i](1))
      minPoint2d(1) = pts[i](1);

    if (maxPoint2d(0) < pts[i](0))
      maxPoint2d(0) = pts[i](0);

    if (maxPoint2d(1) > pts[i](1))
      maxPoint2d(1) = pts[i](1);
  }
}



void updateTexCoord(std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > const &pts, std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &texCoords) {

  texCoords.resize(pts.size());

  // Quick hack - need to get the textural coords, for a texture this is between 0..1 in both x and y direction. Here we simply assume that the size of the zebra is 10x10 meter and we place the texCoords to make the zebra stay with the same orientation on the floor.
  
  // We'll assume that the pattern is repeated every 1 meter (thats std::floor)
  Eigen::Vector2d min_point, max_point;
  getMinMaxPoint2dValue(pts, min_point, max_point);

  int offset_meter_x = std::floor(min_point(0));
  int offset_meter_y = std::floor(min_point(1));
  
  for (size_t i = 0; i < pts.size(); i++) {
    texCoords[i](0) = (pts[i](0)-offset_meter_x)*0.1; // 0.1 is the 0..10 meter over 0..1 texture.
    texCoords[i](1) = (pts[i](1)-offset_meter_y)*0.1; // 0.1 is the 0..10 meter over 0..1 texture.
  }
}
