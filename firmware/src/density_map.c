#include "density_map.h"
#define MAX_PEAKS 100

#define MIN_DENSITY 1500

#define xkl 20
#define ykl 20
#define xcl (  DENSITY_MAP_WIDTH + xkl - 1 )
#define ycl ( DENSITY_MAP_HEIGHT + ykl - 1 )

double xk[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
double yk[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
double xi[DENSITY_MAP_WIDTH];
double yi[DENSITY_MAP_HEIGHT];
double xc[xcl];
double yc[ycl];

int xpn = 0;
int ypn = 0;

int xp[MAX_PEAKS], yp[MAX_PEAKS];
density_peak_t density_peaks[MAX_PEAKS];

void convolve(const double Signal[], size_t SignalLen,
              const double Kernel[], size_t KernelLen,
              double Result[])
{
    size_t n;
    int i = 0;
    for (n = 0; n < SignalLen + KernelLen - 1; n++)
    {
        size_t kmin, kmax, k;
        
        Result[n] = 0;
        
        kmin = (n >= KernelLen - 1) ? n - (KernelLen - 1) : 0;
        kmax = (n < SignalLen - 1) ? n : SignalLen - 1;
        
        
        for (k = kmin; k <= kmax; k++)
        {
            i++;
            Result[n] += Signal[k] * Kernel[n - k];
        }
    }
}

void calculatePeaks( void )
{
    int peak = 1;
    double prev, diff;
    prev = xc[0];
    uint16_t i, j;
    for (j = 1; j < xcl; j++ )
    {
        diff = xc[j] - prev;
        if ( peak )
        {
            if ( diff < 0 )
            {
                peak = 0;
                xp[xpn++] = j;
            }
            prev = xc[j];
            if (xpn == MAX_PEAKS) break;
        }
        else
        {
            if ( diff > 0 ) peak = 1;
        }
    }
    
    prev = yc[0];
    for (i = 1; i < ycl; i++ )
    {
        diff = yc[i] - prev;
        if ( peak )
        {
            if ( diff < 0 )
            {
                peak = 0;
                yp[ypn++] = i;
            }
            prev = yc[i];
            if (ypn == MAX_PEAKS) break;
        }
        else
        {
            if ( diff > 0 ) peak = 1;
        }
    }
}
  
void processDensityMaps( void )
{
    convolve(xi,  DENSITY_MAP_WIDTH, xk, xkl, xc);
    convolve(yi, DENSITY_MAP_HEIGHT, yk, ykl, yc);
    calculatePeaks();
    int index = 0, i, j;
    for(i = 0; i < ypn; i++ )
    {
        int y = (int)((float)yp[i] * (float)DENSITY_MAP_HEIGHT/(float)ycl);
        for(j = 0; j < xpn; j++ )
        {
            int x = (int)((float)xp[j] * (float)DENSITY_MAP_WIDTH/(float)xcl);

            if(( xc[x] + yc[y] ) > MIN_DENSITY )
            {
                /* Peak height */
                density_peaks[index].X = x;
                density_peaks[index].Y = y;
                density_peaks[index].X = xc[x] + yc[y];
                index++;
            }
        }
    }
}

void generateDensityMaps( uint8_t image_line[], uint16_t y )
{
    uint16_t x = 0;
    while( x < DENSITY_MAP_WIDTH )                        // Traverse all columns
    {
        uint8_t a = image_line[x];
        if( a > DENSITY_MAP_THRESH )          // Check if pixel is on
        {
            xi[x] += 1;
            yi[y] += 1;
        }
        x += DENSITY_MAP_INTERVAL;
    }
}

void initDensityMaps( uint16_t width, uint16_t height, uint16_t interval, uint16_t thresh )
{  
//    DENSITY_MAP_WIDTH     = width;
//    DENSITY_MAP_HEIGHT    = height;
    DENSITY_MAP_INTERVAL  = interval;
    DENSITY_MAP_THRESH    = thresh;
}