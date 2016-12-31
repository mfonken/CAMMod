#ifndef _CENTROID_H    /* Guard against multiple inclusion */
#define _CENTROID_H

#include <stdint.h>

#define MAX_BLOBS       10
#define MAX_GAP         3
    
typedef struct _blob_t
{
    double      X;
    double      Y;

    uint16_t    height;
    uint16_t    w_last;
    uint16_t    x_last;
    uint16_t    y_last;
} blob_t;

typedef struct _centroids_t
{
    uint8_t     numBlobs;
    blob_t      blobs[MAX_BLOBS];
} centroids_t;

centroids_t centroids;

uint16_t    CENTROIDS_WIDTH;
uint16_t    CENTROIDS_HEIGHT;
uint8_t     CENTROIDS_INTERVAL;
    
uint8_t getBlobId(uint16_t x, uint16_t y, uint16_t n_c, uint8_t * num_blobs );
void    getCentroids( uint8_t *image_line, uint16_t line_number );
void    initCentroids( uint16_t width, uint16_t height, uint8_t interval );


#endif /* _EXAMPLE_FILE_NAME_H */
