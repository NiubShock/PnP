/*
 * File:   interrupt.c
 * Author: alle9
 *
 * Created on December 15, 2020, 10:37 AM
 */


#include <xc.h>
#include "main.h"


static unsigned int ADC_res = 0;
static const unsigned int touch_pressure = 0x200;   //pressure required for touch relevation
static const unsigned int therm_pressure = 0x400;   //pressure required for thermal paste

void __interrupt() isr(){
    static unsigned int single_cycle = 0;   //variable used to store the single cycle of the pwm
    static unsigned char tm0Count = 0;      //variable to count the overflow of the timer0
    static unsigned char completeStep = 0;  //variable used to help count only complete steps
    
    //TIM0 interrupt routine
    if(INTCONbits.T0IF){
        INTCONbits.T0IF = 0;    //clear the flag
        
        //increase the value of the timer0 overflow
        tm0Count++;
        //verify if the limit has been surpassed
        if(tm0Count >= whatsTM0Limit()){
            T0CONbits.TMR0ON = 0;           //turn off the timer
            tm0Count = 0;                   //reset the counter
            writeTM0();                     //toggle the error
        }
    }
    
    //TIM2 interrupt routine
    if(PIR1bits.TMR2IF){
        PIR1bits.TMR2IF = 0;    //clear the flag
        single_cycle++;         //each overflow represent a time unit
        
        //verify if we surpass the number of cycle required for 1 step (high or low)
        if(single_cycle > retPeriod()){
            completeStep++;
            
            //A complete step is verified when the cunter is not 0 but is a multiple of 2
            if(completeStep != 0 && !(completeStep % 2)){
                //reset of the help variable. Reason to avoid the value of 0
                completeStep = 0;
                //increase the number of steps made -> used to count the position
                //used in function just to verify if the total steps are enough to move in position
                increaseStep();                  
            }
            
            //toggle one variable to indicate the steps made
            //used to fix the time duration if the H and L step
            toggleStep();
        }
    }
    
    //ADC interrupt routine
    if(PIR1bits.ADIF){
        PIR1bits.ADIF = 0;      //reset the flag
        
        //get the result
        ADC_res = ADRESL;
        ADC_res |= (ADRESH << 8);
        
        //verify if surpass the threshold
        if(ADC_res > therm_pressure){
            //set both the variable -> the code will use just one
            setTouchRel();
            setThermRel();
        }else if(ADC_res > touch_pressure){
            //set only one variable (Lower threshold)
            setTouchRel();
        }
    }
    
    //USART Interrupt
    if(PIR1bits.RC1IF){
        PIR1bits.RC1IF = 0;
        
        //store the data and leave the register for new data
        unsigned char temp = RCREG1;
        //send the data so that can be stored
        storeData(temp);
    }
}