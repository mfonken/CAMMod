#ifndef _CENTROID_H    /* Guard against multiple inclusion */
#define _CENTROID_H

#include <stdint.h>
#include <stdbool.h>
#include "app.h"

#define NULL_G          0xef
#define NULL_C          0xef

#define CENTROID_HEAD   0xee

#define MAX_CENTROIDS   100
#define MAP_SIZE        MAX_CENTROIDS
#define MAX_SEGMENTS    4000
#define MAX_GAP         10
#define MIN_MASS        50

typedef struct
{
    float        X;                      /**< X center value */
    float        Y;                      /**< Y center value */
    uint16_t     M;
    uint16_t     n;
} centroid_t;

typedef struct
{
    uint16_t  i;
    uint16_t  l;
    float     x;
    float     w;
} segment_t;

typedef struct
{
    uint16_t    p; // point
    bool        s; // single
    bool        a; // active
    uint16_t    y; // last y
    float       x; // last x
    uint16_t    w; // last width
} map_t;

extern centroid_t  centroids[];

uint16_t    CENTROIDS_WIDTH;
uint16_t    CENTROIDS_HEIGHT;
uint16_t    CENTROIDS_INTERVAL;
uint16_t    CENTROIDS_THRESH;
uint16_t    processCentroids( void );
uint16_t    getSegmentId( uint16_t y, float x, uint16_t w );
void        getCentroids( uint8_t image_line[], uint16_t line_number );
void        initCentroids( uint16_t width, uint16_t height, uint16_t interval, uint16_t thresh );
void        resetBlobs( void );

#endif /* _EXAMPLE_FILE_NAME_H */
