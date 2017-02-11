#include "centroid.h"

inline void cma( float new_val, float *avg, int num )
{
    *avg += ( new_val - *avg ) / num); // num is pre-incremented
}

int processCentroids( void )
{
    for( int j = 0; j < segment_index; j++ )
    {
        segment_t s = segments[j];
        for( int i = 0; i < map_index; i++ )
        {
            if( s.i == i )
            {
                int p = map[i].p;
                centroids[p].M += s.w;
                centroids[p].n++;
                cma( s.x, &centroids[p].X, centroids[p].n);
                /* Quick Y estimate */
                cma( s.l, &centroids[p].Y, centroids[p].n);
            }
        }
    }
    return map_index;
}

int getSegmentId( int y, double x, int w )
{
    /* Null id (no adjacents) */
    int id = -1, i;
    w /= 2;
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
                double x_l = map[i].x;
                double w_l = map[i].w;
                /* Check overlap of lower bound of centroid and upper (with gap tolerance) of new
                 and of upper bound of centroid and lower (with gap tolerance) of new */
                if( ( ( x + w + MAX_GAP ) >= ( x_l - w_l ) ) &&
                   ( ( x - w - MAX_GAP ) <= ( x_l + w_l ) ) )
                {
                    /* If object is unclaimed, claim it, otherwise combine it */
                    if( id == -1 )
                    {
                        id = i;
                    }
                    else
                    {
                        // NOTE: Merged maps will re-merge every time (more efficient than checking)
                        map[i] = map[id];
                    }
                }
            }
        }
    }
    /* Valid if object was claimed, null if not */
    return id;
}

void getCentroids( uint8_t image_line[], int line_number )
{
    unsigned int gap = NULL_G, temp_id;
    unsigned int num_adj = 0;                           // Global variables
    float a_x_last = 0;                                 // Global last X and Y averages
    float x = 0, line_number;
    while( x < CENTROIDS_WIDTH )                        // Traverse all columns
    {
        if( image_line[x] > CENTROIDS_THRESH )          // Check if pixel is on
        {
            gap = 0;                                    // Reset gap counter
            cma( x, &a_x_last, num_adj + 1);               // Average adjacent pixels
            num_adj++;                                  // Increment adjacent pixels
        }
        else                                            // Otherwise, if gap counter is counting (i.e. there was a recent pixel
        {
            switch(gap)
            {
                default:
                    gap++;
                case MAX_GAP:
                    temp_id = getSegmentId( line_number, a_x_last, num_adj );
                    if( temp_id == NULL_C )             // If no blob return
                    {
                        temp_id = map_index++;
                        map[temp_id].p = temp_id;
                        map[temp_id].a = true;
                    }
                    map[temp_id].x = a_x_last;
                    map[temp_id].w = num_adj / 2;
                    map[temp_id].y = line_number;
                    segments[segment_index].i = map[temp_id].p;
                    segments[segment_index].l = line_number;
                    segments[segment_index].x = a_x_last;
                    segments[segment_index].w = num_adj;
                    segment_index++;
                    num_adj = 0;                                            // Reset number of adjacent pixels
                    a_x_last = 0;                                           // Reset adjacent pixel average
                    gap = -1;
                case NULL_G:
            }
        }
        x += CENTROIDS_INTERVAL;
    }
}

void initCentroids( int width, int height, int interval, int thresh )
{
    CENTROIDS_WIDTH     = width;
    CENTROIDS_HEIGHT    = height;
    CENTROIDS_INTERVAL  = interval;
    CENTROIDS_THRESH    = thresh;
    centroids.numBlobs  = 0;
}

void resetBlobs( void )
{
    unsigned int i = sizeof( centroids );
    register unsigned char * = (unsigned char*)&centroids;
    while (i-- > 0) *p++ = 0;
}
