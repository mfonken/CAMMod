#include "centroid.h"

inline void cma_d( float new_val, float *avg, int num )
{
    *avg += ( new_val - *avg ) / num); // num is pre-incremented
}

inline void cma( int new_val, float *avg, int num )
{
    *avg += ( float(new_val) - *avg ) / num;
}

int getBlobId( float x, float y, int n_c )
{
    n_c >>= 2;
    unsigned int x_n_c_p = x + n_c;
    unsigned int x_n_c_n = x - n_c;
    unsigned int id = NULL_C;                                         // NULL_ id (no adjacents)
    unsigned int i = 0, e;
    float n_l, avg;
    for( ; i < centroids.numBlobs; i++ )
    {
        if( ( y - centroids.blobs[i].y_last ) <= MAX_GAP )      // Ensure blob hasn't expired
        {
            n_l = centroids.blobs[i].w_last >> 2;               // Last row width
            if( ( ( x_n_c_p + MAX_GAP)  >= ( centroids.blobs[i].X - n_l ) ) &&   // Check overlap of lower bound of blob and upper (with gap tolerance) of new
                ( ( x_n_c_n - MAX_GAP ) <= ( centroids.blobs[i].X + n_l ) ) )    // and of upper bound of blob and lower (with gap tolerance) of new
            {
                if( id == NULL_C )                              // If new blob is not claimed
                {
                    id = i;                                     // Claim it under current id
                }
                else
                {
                    avg;
                    avg = ( centroids.blobs[id].X + centroids.blobs[i].X ) >> 1;
                    centroids.blobs[id].X = avg;
                    avg = ( centroids.blobs[id].Y + centroids.blobs[i].Y ) >> 1;
                    centroids.blobs[id].Y = avg;
                    centroids.blobs[id].mass += centroids.blobs[i].mass;
                    
                    e = --centroids.numBlobs;
                    centroids.blobs[i] = centroids.blobs[e];
                    centroids.blobs[e].w_last = 0;
                    centroids.blobs[e].height = 0;
                    centroids.blobs[e].mass = 0;
                }
            }
        }
    }
    return id;                                          // Return id: Valid if claimed, NULL_ if not
}

void getCentroids( uint8_t image_line[], int line_number )
{
    unsigned int gap = NULL_G, temp_id;
    unsigned int num_adj = 0;                           // Global variables
    float a_x_last = 0;                                 // Global last X and Y averages
    unsigned int x = 0;
    while( x < CENTROIDS_WIDTH )                        // Traverse all columns
    {
        if( image_line[x] > CENTROIDS_THRESH )          // Check if pixel is on
        {
            gap = 0;                                    // Reset gap counter
            cma( x, &a_x_last, num_adj );               // Average adjacent pixels
            num_adj++;                                  // Increment adjacent pixels
        }
        else                                            // Otherwise, if gap counter is counting (i.e. there was a recent pixel
        {
            switch(gap)
            {
                default:
                    gap++;
                case MAX_GAP:
                    temp_id = getBlobId( a_x_last, line_number, num_adj );      // Get a blob to add to by coordinates and adjacent pixel width
                    if( temp_id == NULL_C )             // If no blob return
                    {
                        temp_id = centroids.numBlobs++; // Otherwise make a new id for the blob and increment the id counter
                    }
                    if( temp_id != NULL_C )
                    {
                        centroids.blobs[temp_id].height++;
                        cma_d(  a_x_last, &centroids.blobs[temp_id].X, centroids.blobs[temp_id].height ); // Cumulate the new pixels into the blob's X average
                        cma( line_number, &centroids.blobs[temp_id].Y, centroids.blobs[temp_id].height ); // Cumulate the new row into the blob's Y average
                        centroids.blobs[temp_id].mass  += num_adj;
                        centroids.blobs[temp_id].w_last = num_adj;              // Update blob with: New last row width
                        centroids.blobs[temp_id].x_last = a_x_last;             // last row average
                        centroids.blobs[temp_id].y_last = line_number;          // last row
                        
                        num_adj = 0;                                            // Reset number of adjacent pixels
                        a_x_last = 0;                                           // Reset adjacent pixel average
                        gap = NULL_G;                                           // Reset the gap to NULL_
                    }
                    break;
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
