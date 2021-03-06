#include <xc.h>
#include "adc.h"
#include "motors.h"
#include "conf_bits.h"
#include "timer.h"
#include "interrupt.h"
#include "usart.h"

#define _XTAL_FREQ 4000000

#define START                   0       //start of the PNP sequence
#define MOVEPICK                1       //move to the pick position
#define PICKOBJ                 2       //pick the object
#define OBJPICKD                3       //object picked
#define LIFTARM                 4       //lifting the arm
#define MOVEPLACE               5       //move to the place position
#define ROTATE                  6       //rotating the object
#define PLACEOBJ                7       //place the object on the PCB
#define OBJRLSD                 8       //object relased
#define INITRESET               9       //intial position reset
#define READY                   10      //ready to work

#define ALL_OK                  0       //no error related
#define FEED_LINE_NOT_DEFINED   1       //not existing line selected
#define END_Z_AXIS              2       //reached the end of the Z-axis without touching anything
#define RESET_ERROR             3       //error while resetting the position (timeout)
#define POINT_ERROR             4       //error while moving to a point (timeout)
#define BOUNDARY_ERROR          5       //second point outside the border
#define POINTZ_ERROR            6       //error while moving to a point on z-axis (timeout)
#define FATAL_ERROR             7       //fatal error. All off and wait the reset
#define NO_CMD                  8       //not recognized command

#define PICK_AND_PLACE          1
#define NEW_PICK                2
#define FATAL_CMD               3

#define NEW_FEEDER              0xFF


#define TIME_REF_LIFT           200
#define TIME_REF_RESET          200
#define TIME_REF_POINT          200

#define nSIMULATION             1



char executeData(void);