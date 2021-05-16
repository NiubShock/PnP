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

static unsigned int single_cycle = 0;   //variable used to store the single cycle of the pwm
static unsigned char tm0Count = 0;      //variable to count the overflow of the timer0
static unsigned char completeStep = 0;  //variable used to help count only complete steps


void resetTM0_Temp(){
    tm0Count = 0;
}

void resetTM2_Temp(){
    single_cycle = 0;
    completeStep = 0;
}

/*
 * Description: This function is used to turn on all the interrupt required 
 *              for the application
 */
void interruptInit(void){
    //turn on the interrupt if not already on
    if(!INTCONbits.GIE){
        INTCONbits.GIE = 1;
    }
    if(!INTCONbits.PEIE){
        INTCONbits.PEIE = 1;
    }
    
    
    RCONbits.IPEN = 1;              //turn on the priority for interrupt
    INTCONbits.T0IE = 1;            //turn on the interrupt for the timer0
    PIE1bits.TMR1IE = 1;            //turn on the interrupt for the timer1
    PIE1bits.TMR2IE = 1;            //turn on the interrput on timer 2
    PIE1bits.RC1IE = 1;             //turn on the interrupt for the usart RX
}

void __interrupt() isr(){ 
    
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
    
    //TIM1 interrupt routine
    if(PIR1bits.TMR1IF){
        PIR1bits.TMR1IF = 0;    //clear the flag
        if(fatalError()){
            abortAll();
        }
    }
    
    //TIM2 interrupt routine
    if(PIR1bits.TMR2IF){
        PIR1bits.TMR2IF = 0;    //clear the flag
        single_cycle++;         //each overflow represent a time unit
        
        //verify if we surpass the number of cycle required for 1 step (high or low)
        if(single_cycle > retPeriod()){
            single_cycle = 0;
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
