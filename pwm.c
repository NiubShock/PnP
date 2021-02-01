/*
 * File:   pwm.c
 * Author: alle9
 *
 * Created on 11 dicembre 2020, 10.07
 */


#include <xc.h>

static volatile unsigned char pwmStep = 0;
static volatile unsigned char stepToggle = 0;
static unsigned int pwmPeriod = 0;


/*
 * Description: Initialization function for the timer TMR0 -> used to check the
 *              time of the operations
 * Note: The TMR0 is still off after this function has been called
 */
void tim0Init(void){
    T0CONbits.TMR0ON = 0;       //start off
    T0CONbits.T08BIT = 0;       //16 bit
    T0CONbits.T0CS = 0;         //internal instruct
    T0CONbits.PSA = 1;          //prescaler on
    T0CONbits.T0PS = 0x07;      //1:256 prescaler
}

/*
 * Description: Initialization function for the timer TMR2 -> used to generate 
 *              the pwm used for the steps generation
 * Note: The TMR2 is still off after this function has been called
 */
void tim2Init(unsigned int _pwmPeriod){
    T2CONbits.TMR2ON = 0;       //TIM2 off
    T2CONbits.T2CKPS = 0x03;    //x16 prescaler
    
    //fix the period related to the pwm generation
    pwmPeriod = _pwmPeriod;
}

/*
 * Description: This function return the value of the stepToggle. It can be one
 *              only if a step has just been made. Then after 1 read is automatically
 *              resetted.
 */
unsigned int stepMade(void){
    unsigned char temp = stepToggle;
    
    stepToggle = 0;
    
    return temp;
}

/*
 * Description: Return the total number of steps made (count both H and L)
 */
unsigned int stepCounter(void){
    return pwmStep;
}

/*
 * Description: Reset the number of steps made
 */
void resetStep(void){
    pwmStep = 0;
}

/*
 * Description: Increase by 1 the number of steps made
 */
void increaseStep(void){
    pwmStep++;
}

/*
 * Description: Function used to set the variable that identify the step has been 
 *              made
 */
void toggleStep(void){
    stepToggle = 1;
}

/*
 * Description: Return the period of the pwm cycle
 * Note: Period fixed by the tim2 Initialization
 */
unsigned int retPeriod(void){
    return pwmPeriod;
}
