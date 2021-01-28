/*
 * File:   adc.c
 * Author: alle9
 *
 * Created on December 15, 2020, 9:58 AM
 */


#include <xc.h>
#include "adc.h"

static volatile unsigned char touch_rel = 0;
static volatile unsigned char therm_rel = 0;

/*
 * Description: Function used to initialize all the parameter of the ADC
 */
void initADC(void){
    ADCON0bits.ADCS = 0x01;         //Fosc/16
    ADCON1bits.ADCS2 = 0x01;        //Fosc/16
    
    ADCON0bits.CHS = 0x00;          //AVDD reference
    ADCON0bits.GO_DONE = 0x00;      //idle
    ADCON0bits.ADON = 0;            //off
    
    ADCON1bits.ADFM = 0x00;         //Left just.
    ADCON1bits.PCFG = 0x0E;         //All digital except AN0 and refer as Vdd/Vss
    
    PIE1bits.ADIE = 1;              //interrupt active
    IPR1bits.ADIP = 1;              //High priority
}

/*
 * Decription: Function used to start the ADC
 */
void startADC(void){
    ADCON0bits.ADON = 1;
}

/*
 * Decription: Function used to stop the ADC
 */
void stopADC(void){
    ADCON0bits.ADON = 0;
}

/*
 * Description: Function used to return the pressure state
 */
unsigned int returnTouch(void){
    return touch_rel;
}

/*
 * Description: Function used to return the pressure state
 */
unsigned int returnTherm(void){
    return therm_rel;
}

/*
 * Description: Function used to clear the pressure state
 */
void resetTouch(void){
    touch_rel = 0;
}

/*
 * Description: Function used to clear the pressure state
 */
void resetTherm(void){
    therm_rel = 0;
}

/*
 * Description: Function used to set the pressure state
 */
void setThermRel(void){
    therm_rel = 1;
}

/*
 * Description: Function used to set the pressure state
 */
void setTouchRel(void){
    touch_rel = 1;
}