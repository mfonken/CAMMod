#include "centroid.h"

map_t       map[MAP_SIZE];
segment_t   segments[MAX_SEGMENTS];
centroid_t  centroids[MAX_CENTROIDS];  /**< Global array of detected object */

uint16_t    map_index = 0;
uint16_t    segment_index = 0;
uint16_t    num_centroids = 0;

inline float cma( float new_val, float avg, uint16_t num )
{
    return avg + ( new_val - avg ) / num; // num is pre-incremented
}

uint16_t processCentroids( void )
{
    uint16_t j = 0, i = 0;
    for( ; j < segment_index; j++ )
    {
        for( ; i < map_index; i++ )
        {
            if( segments[j].i == i )
            {
                uint16_t p = map[i].p;
                centroids[p].M += segments[j].w;
                centroids[p].n++;
                centroids[p].X = cma( segments[j].x, centroids[p].X, centroids[p].n);
                centroids[p].Y = cma( segments[j].l, centroids[p].Y, centroids[p].n);
            }
        }
    }
    for( i = 0; i < num_centroids; i++)
    {
        if( centroids[i].M == 0)
        {
            while( j == 0 )
            {
                j = centroids[--num_centroids].M;
                if( num_centroids == 0 ) return 0;
            }
            centroids[i] = centroids[num_centroids];
        }
    }
    return num_centroids;
}

uint16_t getSegmentId( uint16_t y, float x, uint16_t w )
{
    uint16_t id = NULL_C, i;
    for(i = 0; i < map_index; i++)
    {
        if( map[i].a ) // Ensure map is still active
        {
            if( ( y - map[i].y ) > MAX_GAP)
            {
                map[i].a = false;
            }
            else
            {
                /* Get current average and previous row width */
                float    x_l = map[i].x;
                uint16_t w_l = map[i].w;
                float  w_c_2 =   w / 2;
                float  w_l_2 = w_l / 2;  
                /* Check overlap of lower bound of centroid and upper (with gap tolerance) of new
                 and of upper bound of centroid and lower (with gap tolerance) of new */
                if( ( ( x + w_c_2 + MAX_GAP ) >= ( x_l - w_l_2 ) ) &&
                    ( ( x - w_c_2 - MAX_GAP ) <= ( x_l + w_l_2 ) ) )
                {
                    /* If object is unclaimed, claim it, otherwise combine it */
                    if( id == NULL_C )
                    {
                        id = i;
                    }
                    else
                    {
                        if(map[i].s)
                        {
                            num_centroids--;
                            map[i].s  = false;
                            map[id].s = false;
                            map[i]    = map[id];
                        }
                    }
                }
            }
        }
    }
    /* Valid if object was claimed, null if not */
    return id;
}

void getCentroids( uint8_t image_line[], uint16_t line_number )
{
    uint16_t gap = NULL_G, temp_id, num_adj = 0, x = 1;                           // Global variables
    float a_x_last = 0;                                 // Global last X and Y averages
    while( x < CENTROIDS_WIDTH )                        // Traverse all columns
    {
        uint8_t a = image_line[x];
        if( a > CENTROIDS_THRESH )          // Check if pixel is on
        {
            gap = 0;
            num_adj++; 
            a_x_last = cma( x, a_x_last, num_adj );               // Average adjacent pixels
                                             // Increment adjacent pixels
        }
        else if( gap != MAX_GAP )                                           // Otherwise, if gap counter is counting (i.e. there was a recent pixel
        {
            gap++;
            if( gap == MAX_GAP )
            {
                temp_id = getSegmentId( line_number, a_x_last, num_adj );
                if( temp_id == NULL_C )             // If no blob return
                {
                    temp_id = map_index++;
                    map[temp_id].p = temp_id;
                    map[temp_id].a = true;
                    map[temp_id].s = true;
                    num_centroids++;
                }
                map[temp_id].x = a_x_last;
                map[temp_id].w = num_adj;
                map[temp_id].y = line_number;
                segments[segment_index].i = map[temp_id].p;
                segments[segment_index].l = line_number;
                segments[segment_index].x = a_x_last;
                segments[segment_index].w = num_adj;
                segment_index++;
                num_adj  = 0;                                            // Reset number of adjacent pixels
                a_x_last = 0;                                           // Reset adjacent pixel average
                gap      = NULL_G;
            }
        }
        x += CENTROIDS_INTERVAL;
    }
}

void initCentroids( uint16_t width, uint16_t height, uint16_t interval, uint16_t thresh )
{  
    CENTROIDS_WIDTH     = width;
    CENTROIDS_HEIGHT    = height;
    CENTROIDS_INTERVAL  = interval;
    CENTROIDS_THRESH    = thresh;
}

void resetBlobs( void )
{
    num_centroids = 0;
    segment_index = 0;
    map_index     = 0;
    uint16_t i    = sizeof( centroids );
    uint8_t * p   = ( uint8_t * )&centroids;
    while (i-- > 0) *p++ = 0;
}
