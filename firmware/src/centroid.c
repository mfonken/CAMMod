#include "centroid.h"

static void cma( double new_val, double *avg, uint16_t num )
{
    *avg += ( new_val - *avg ) / ( num + 1 );
}
uint8_t getBlobId(uint16_t x, uint16_t y, uint16_t n_c, uint8_t num_blobs)
{
    uint8_t id = -1;                                // Null id (no adjacents)
    uint8_t i, j;
    for( i = 0; i < *num_blobs; i++ )
    {
        double x_l = centroids.blobs[i].X;                   // Current average
        double n_l = centroids.blobs[i].w_last;              // Last row width
        if( ( y - centroids.blobs[i].y_last ) <= MAX_GAP )    // Ensure blob hasn't expired
        {
            if( ( ( x + n_c/2 + MAX_GAP) >= ( x_l - n_l/2) ) && // Check overlap of lower bound of blob and upper (with gap tolerance) of new
                ( ( x - n_c/2 ) <= ( x_l + n_l/2 + MAX_GAP) ) ) // and of upper bound of blob and lower (with gap tolerance) of new
            {
                if( id == -1 )                    // If new blob is not claimed
                {
                    id = i;                     // Claim it under current id
                }
                else                            // Otherwise combine
                {
                    cma( centroids.blobs[i].X, &centroids.blobs[id].X, 2 );
                    cma( centroids.blobs[i].Y, &centroids.blobs[id].Y, 2 );
                    ( *num_blobs )--;             // Decrement blob counter
                    for( j = i; j < *num_blobs; j++ )
                    {
                        centroids.blobs[i] = centroids.blobs[i+1];  // and pack blobs down
                    }
                    centroids.blobs[*num_blobs].w_last = 0; // Recent remains (muahahah...)
                    centroids.blobs[*num_blobs].height = 0;
                }
            }
        }
    }
    return id;                                  // Return id: Valid if claimed, null if not
}

void getCentroids( uint8_t *image_line, uint16_t line_number, uint8_t skip )
{
    uint8_t gap = -1, num_adj = 0, temp_id = 0;   // Global variables
    double a_x_last = 0;                  // Global last X and Y averages
    centroids.numBlobs = 0;
    uint16_ x;
    for( x = 0; x < CENTROIDS_WIDTH; x += skip )            // Traverse all columns
    {
        if(image_line[x])                             // Check if pixel is on
        { 
            gap = 0;                                // Reset gap counter
            cma( x, &a_x_last, num_adj );               // Average adjacent pixels
            num_adj++;                              // Increment adjacent pixels
        }
        else if( gap != -1 )                          // Otherwise, if gap counter is counting (i.e. there was a recent pixel
        {
            gap++;                                  // Increment the gap counter
            if( gap == MAX_GAP                 ||   // If max gap reached
                  x == ( CENTROIDS_HEIGHT - 1 ) )   // or end of row
            {                                               // Include last pixel into a blob
                temp_id = getBlobId( a_x_last, line_number, num_adj, &centroids.numBlobs );   // Get a blob to add to by coordinates and adjacent pixel width
                if( temp_id == -1 )                   // If no blob return
                {
                    if( centroids.numBlobs == MAX_BLOBS )           // Check if max blobs have already been filled
                    {
                        return;                   // If so, end and output
                    }
                    temp_id = centroids.numBlobs++;               // Otherwise make a new id for the blob and increment the id counter
                }
                cma( a_x_last, &centroids.blobs[temp_id].X, centroids.blobs[temp_id].w_last ); // Cumulate the new pixels into the blob's X average
                cma( line_number, &centroids.blobs[temp_id].Y, centroids.blobs[temp_id].height ); // Cumulate the new row into the blob's Y average
                centroids.blobs[temp_id].w_last = num_adj;    // Update blob with: New last row width
                centroids.blobs[temp_id].x_last = a_x_last;   // last row average
                centroids.blobs[temp_id].height++;            // height
                centroids.blobs[temp_id].y_last = line_number;// last row

                num_adj = 0;                        // Reset number of adjacent pixels
                a_x_last = 0;                       // Reset adjacent pixel average
                gap = -1;                           // Reset the gap to null

            }
        }
    }
}

void initCentroids( uint16_t width, uint16_t height )
{
    CENTROIDS_WIDTH  = width;
    CENTROIDS_HEIGHT = height;
    centroids.numBlobs = 0;
}
