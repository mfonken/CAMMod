#ifndef _CENTROID_H    /* Guard against multiple inclusion */
#define _CENTROID_H

#include <stdint.h>

#define MAX_BLOBS       6
#define MAX_GAP         15
    
#define NULL_G          0xef
#define NULL_C          0xef

#define CENTROID_HEAD   0xee

typedef struct _blob_t
{
    float      X;
    float      Y;
  
    int    mass;
    int    height;
    int    w_last;
    int    x_last;
    int    y_last;
} blob_t;

typedef struct _centroids_t
{
    int     numBlobs;
    blob_t      blobs[MAX_BLOBS];
} centroids_t;

centroids_t centroids;

int    CENTROIDS_WIDTH;
int    CENTROIDS_HEIGHT;
int    CENTROIDS_INTERVAL;
int    CENTROIDS_THRESH;
int    getBlobId(float x, float y, int n_c, int *num_blobs);
void   getCentroids( uint8_t image_line[], int line_number );
void   initCentroids( int width, int height, int interval, int thresh );
void   resetBlobs( void );

#endif /* _EXAMPLE_FILE_NAME_H */
