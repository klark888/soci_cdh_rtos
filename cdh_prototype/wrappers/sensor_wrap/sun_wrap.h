#include "arm_math.h"

typedef struct _Sun {
  double angles[3];           /* measured alpha and beta angles and sun detection */
  uint8_t error;              /* used to identify errors - see list below */
  uint8_t isValid;            /* used to identify if data is usable - 0 is unusable, 1 is usable */
} sun_t;

extern sun_t Sun1;                /* sun sensor 1*/

void getSunAngles( sun_t * Sun );
