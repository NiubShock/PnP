#include <xc.h>
#include "adc.h"
#include "motors.h"
#include "conf_bits.h"
#include "pwm.h"
#include "usart.h"

#define _XTAL_FREQ 4000000

#define ALL_OK                  0       //no error related
#define FEED_LINE_NOT_DEFINED   1       //not existing line selected
#define END_Z_AXIS              2       //reached the end of the Z-axis without touching anything
#define RESET_ERROR             3       //error while resetting the position (timeout)
#define POINT_ERROR             4       //error while moving to a point (timeout)
#define BOUNDARY_ERROR          5       //second point outside the border
#define POINTZ_ERROR            6       //error while moving to a point on z-axis (timeout)
#define FATAL_ERROR             7       //fatal error. All off and wait the reset
#define NO_CMD                  8       //not recognized command

#define PICK_AND_PLACE          0
#define NEW_PICK                1
#define FATAL_CMD               2

#define NEW_FEEDER              0xFF


#define TIME_REF_LIFT           100
#define TIME_REF_RESET          100
#define TIME_REF_POINT          100



char executeData(void);