#include "app.h"

#define    DENSITY_MAP_WIDTH  APP_FRAME_WIDTH_RGGB
#define    DENSITY_MAP_HEIGHT APP_CAMERA_HEIGHT
uint16_t    DENSITY_MAP_INTERVAL;
uint16_t    DENSITY_MAP_THRESH;

typedef struct
{
    uint16_t     X;                      /**< X center value */
    uint16_t     Y;                      /**< Y center value */
    uint16_t     Z;
} density_peak_t;

extern density_peak_t density_peaks[];

void convolve(const double Signal[], size_t SignalLen,
              const double Kernel[], size_t KernelLen,
              double Result[]);
void calculatePeaks( void );
void getCentroids( uint8_t image_line[], uint16_t line_number );
void initDensityMaps( uint16_t width, uint16_t height, uint16_t interval, uint16_t thresh );