#ifndef _CENTROID_H    /* Guard against multiple inclusion */
#define _CENTROID_H

#include <stdint.h>

#define NULL_G          0xef
#define NULL_C          0xef

#define CENTROID_HEAD   0xee

#define MAX_CENTROIDS   100
#define MAP_SIZE        MAX_CENTROIDS * 2
#define MAX_SEGMENTS    400
#define MAX_GAP         1

typedef struct
{
    double  X;                      /**< X center value */
    double  Y;                      /**< Y center value */
    int     M;
    int     n;
} centroid_t;

typedef struct
{
    int     i;
    int     l;
    double  x;
    double  w;
    int     m;
} segment_t;

int map_index = 0;
int segment_index = 0;

typedef struct
{
    int     p; // point
    bool    a; // active
    int     y; // last y
    double  x; // last x
    int     w; // last width
} map_t;

map_t       map[MAP_SIZE];
segment_t   segments[MAX_SEGMENTS];
centroid_t  centroids[MAX_CENTROIDS];  /**< Global array of detected object */

int     CENTROIDS_WIDTH;
int     CENTROIDS_HEIGHT;
int     CENTROIDS_INTERVAL;
int     CENTROIDS_THRESH;
int     processCentroids( void );
int     getSegmentId( int y, double x, int w );
void    getCentroids( uint8_t image_line[], int line_number );
void    initCentroids( int width, int height, int interval, int thresh );
void    resetBlobs( void );

#endif /* _EXAMPLE_FILE_NAME_H */
