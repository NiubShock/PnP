/*
 * File:   motors.c
 * Author: alle9
 *
 * Created on December 11, 2020, 6:40 AM
 */

#include "main.h"

static unsigned int tm0Error = 0;
static unsigned char tm0Limit = 0;

//TODO: [ ] ADD eeprom read
static unsigned int maxX = 1000;
static unsigned int maxY = 1000;

/*
 * Description: Function to write inside the tm0error. A value different from
 *              0 will trigger the error relevation
 */
void writeTM0(){
    tm0Error = 1;
}

/*
 * Description: Function used to clear the tm0error
 */
void clearTM0(){
    tm0Error = ALL_OK;
}

/*
 * Description: Function used to read the value of the limit of the timer0error
 */
unsigned char whatsTM0Limit(){
    return(tm0Limit);
}

/*
 * Description: Function used to initialize all the pin required by the motors
 */
void initPinMotors(void){
    //set all the pins as output
    ADCON1bits.PCFG = 0x06; //remove the analog function -> rewritten in ADC
    TRISA &= 0b11010001;    //Set as output RA1-RA2-RA3-RA5
    
    TRISB &= 0b00000010;    //Set all as output except RB1
    TRISC &= 0b11000000;    //Don't touch RC6-RC7
    TRISD &= 0b11000000;    //Set all as output except RD6-RD7
    
    //input pin used are for endstroke
    TRISEbits.TRISE0 = 1;   //axis x
    TRISEbits.TRISE1 = 1;   //axis y
    TRISEbits.TRISE2 = 1;   //axis z
    
    //input pins for error
    TRISDbits.TRISD7 = 1;   //this pin is used to verify the end of the z-axis
                            //used to report eventual error
}

/*
 * Desciption: Set the required step to the desired motor
 * Input: Step need to follow the indication present in the datasheet of the 
 *        driver used
 *        Motor is the defined in the header file  
 */
void setStep(unsigned char step, unsigned char motor){
    switch(motor){
        case MOTOR1:
            //this get the LSB of the step
            LATAbits.LATA5 = (step & 0x01);
            //this get the MSB of the step
            LATBbits.LATB0 = (step & 0x02);
            break;
        case MOTOR2:
            LATBbits.LATB6 = (step & 0x01);
            LATBbits.LATB7 = (step & 0x02);
            break;
        case MOTOR3:
            LATCbits.LATC4 = (step & 0x01);
            LATCbits.LATC5 = (step & 0x02);
            break;
        default:
            break;
    }
}

/*
 * Description: Function used to enable the single motor as specified in the
 *              driver datasheet
 * Input: Motor is defined in the header file and enable can be or 1 or 0. 
 *        1 enable the motor and 0 disable it
 */
void enableMotor(unsigned char enable, unsigned char motor){
    switch(motor){
        case MOTOR1:
            LATAbits.LATA3 = enable;
            break;
        case MOTOR2:
            LATBbits.LATB5 = enable;
            break;
        case MOTOR3:
            LATCbits.LATC3 = enable;
            break;
        default:
            break;
    }
}

/*
 * Description: Function used to set the rotation direction of the single motor 
 *              as specified in the driver datasheet
 * Input: Motor is defined in the header file and direction can be or 1 or 0
 */
void setDirection(unsigned char direction, unsigned char motor){
    switch(motor){
        case MOTOR1:
            LATAbits.LATA1 = direction;
            break;
        case MOTOR2:
            LATBbits.LATB3 = direction;
            break;
        case MOTOR3:
            LATCbits.LATC1 = direction;
            break;
        default:
            break;
    }
}

/*
 * Description: Function used to set the decay for the single motor 
 *              as specified in the driver datasheet
 * Input: Motor is defined in the header file and decay can be or 1 or 0
 */
void setDecay(unsigned char decay, unsigned char motor){
    switch(motor){
        case MOTOR1:
            LATBbits.LATB2 = decay;
            break;
        case MOTOR2:
            LATCbits.LATC0 = decay;
            break;
        case MOTOR3:
            LATDbits.LATD0 = decay;
            break;
        default:
            break;
    }
}

/*
 * Description: Function used to set the motor in the initial position
 *              This function uses the TMR0 to check if there are some errors
 *              due to motors' issue (check if to reach the end-stroke the 
 *              time required is too much)
 */
char resetPosition(){
    //set up the motors
    setStep(MOTOR1, FULLSTEP);
    setStep(MOTOR2, FULLSTEP);
    setStep(MOTOR3, FULLSTEP);
    
    setDirection(BACKWARD, MOTOR1);
    setDirection(BACKWARD, MOTOR2);
    setDirection(BACKWARD, MOTOR3);
    
    setDecay(1, MOTOR1);
    setDecay(1, MOTOR2);
    setDecay(1, MOTOR3);
    
    enableMotor(TURNON, MOTOR1);
    enableMotor(TURNON, MOTOR2);
    enableMotor(TURNON, MOTOR3);
    
    //these variables are used to keep moving even if one direction already 
    //reached to end-stroke
    unsigned char keepMovingX = 1;
    unsigned char keepMovingY = 1;
    unsigned char keepMovingZ = 1;
    
    //turn on the TMR2 -> this timer is used to generate the pwm
    T2CONbits.TMR2ON = 1;
    
    tm0Error = ALL_OK;              //set all_ok to start
    tm0Limit = TIME_REF_RESET;      //set the limit time required to reach the reset condition
    T0CONbits.TMR0ON = 1;           //turn on the timer
    
    while(keepMovingX || keepMovingY || keepMovingZ){
        
        //verify if the limit time has been reached
        if(tm0Error){
            //stop the motors
            LATAbits.LATA2 = 0;
            LATBbits.LATB4 = 0;
            LATCbits.LATC2 = 0;
            //save the error
            tm0Error = RESET_ERROR;
            break;
        }
        
        //check if one of the motors has reached the end-stroke
        //if not keep moving
        
        //Verify the X axis
        if(PORTEbits.RE0 || !keepMovingX){
            //change this variable to stop the movement
            keepMovingX = 0;
        }else{
            //Keep moving (H part of the step)
            LATAbits.LATA2 = 1;
        }
        
        //Verify the Y axis
        if(PORTEbits.RE1 || !keepMovingY){
            //change this variable to stop the movement
            keepMovingY = 0;
        }else{
            //Keep moving (H part of the step)
            LATBbits.LATB4 = 1;
        }
        
        //Verify the Z axis
        if(PORTEbits.RE2 || !keepMovingZ){
            //change this variable to stop the movement
            keepMovingZ = 0;
        }else{
            //Keep moving (H part of the step)
            LATCbits.LATC2 = 1;
        }
        
        //wait for the step to be made
        while(!stepMade());
        
        //send the L part of the step
        LATAbits.LATA2 = 0;
        LATBbits.LATB4 = 0;
        LATCbits.LATC2 = 0;
        
        //wait for the step to be made
        while(!stepMade());
    }
    
    //turn off both the timers
    T0CONbits.TMR0ON = 0;
    T2CONbits.TMR2ON = 0;
    //reset the value saved in the register -> keep the ideal starting condition
    TMR0 = 0;
    TMR2 = 0;
    
    //reset the step counter variable
    resetStep();
    
    //return the error
    return(tm0Error);
}

/*
 * Description: Function used to move from one point to another x1,y1 represent
 *              the first point, x2,y2 is the second point.
 *              It uses TMR0 to control that the time required don't exceed
 *              the time allowed
 */
char moveToPoint(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2){
    
    unsigned char MOT1Direction, MOT2Direction;
    
    //verify if the endpoint is within the limit
    if(x2 > maxX || y2 > maxY){
        return(BOUNDARY_ERROR);
    }
    
    //set the correct direction for the X axis
    if((x2 - x1) > 0){
        setDirection(FORWARD, MOTOR1);
        MOT1Direction = FORWARD;
    }else{
        setDirection(BACKWARD, MOTOR1);
        MOT1Direction = BACKWARD;
    }
    //set the correct direction for the X axis
    if((y2 - y1) > 0){
        setDirection(FORWARD, MOTOR2);
        MOT2Direction = FORWARD;
    }else{
        setDirection(BACKWARD, MOTOR2);
        MOT2Direction = BACKWARD;
    }
    
    unsigned char keepMovingX = 1;
    unsigned char keepMovingY = 1;
    
    //start the timer 0
    T2CONbits.TMR2ON = 1;
    
    tm0Error = ALL_OK;              //set the all_ok
    tm0Limit = TIME_REF_POINT;      //set the limit
    T0CONbits.TMR0ON = 1;           //turn on the timer used for the interrupt
    
    while(keepMovingX || keepMovingY){
        
        //verify if the time exceeded the expectation
        if(tm0Error){
            //stop the motors
            LATAbits.LATA2 = 0;
            LATBbits.LATB4 = 0;
            tm0Error = POINT_ERROR;
            break;
        }
        
        
        //check if one of the motors has reached the end-stroke
        //if not keep moving
        
        //Verify the X axis
        if(MOT1Direction == FORWARD){
            if(x1 + stepCounter() == x2){
                //change this variable to stop the movement
                keepMovingX = 0;
            }else{
                //Keep moving (H part of the step)
                LATAbits.LATA2 = 1;
            }
        }else if(MOT1Direction == BACKWARD){
            if(x1 - stepCounter() == x2){
                //change this variable to stop the movement
                keepMovingX = 0;
            }else{
                //Keep moving (H part of the step)
                LATAbits.LATA2 = 1;
            }
        }
        
        if(MOT2Direction == FORWARD){
            //Verify the Y axis
            if(y1 + stepCounter() == y2){
                //change this variable to stop the movement
                keepMovingY = 0;
            }else{
                //Keep moving (H part of the step)
                LATBbits.LATB4 = 1;
            }
        }else if(MOT2Direction == BACKWARD){
            //Verify the Y axis
            if(y1 + stepCounter() == y2){
                //change this variable to stop the movement
                keepMovingY = 0;
            }else{
                //Keep moving (H part of the step)
                LATBbits.LATB4 = 1;
            }
        }
        
        //wait for the step to be made
        while(!stepMade());
        
        //send the L part of the step
        LATAbits.LATA2 = 0;
        LATBbits.LATB4 = 0;
        
        //wait for the step to be made
        while(!stepMade());
    }
    
    //turn off both the timer
    T0CONbits.TMR0ON = 0;
    T2CONbits.TMR2ON = 0;
    //reset the counter register
    TMR0 = 0;
    TMR2 = 0;
    
    //reset the step
    resetStep();
    
    return(tm0Error);
}

/*
 * Description: Function used to get in touch with the object that need to be
 *              picked up. It moves until the touch sensor doesn't return something
 *              or the endstroke is verified (Error). 
 *              It uses the TMR0 to verify the time required
 */
char touchObject(){
    
    //move down
    setDirection(FORWARD, MOTOR3);
    
    T2CONbits.TMR2ON = 1;
    
    tm0Error = ALL_OK;              //set all ok
    tm0Limit = TIME_REF_LIFT;       //set the limit
    T0CONbits.TMR0ON = 1;           //turn on the timer used for the interrupt
    
    //start the ADC to relevate the pressure
    startADC();
    
    //Stop if touch the object or if reached endstroke (Error)
    while(!returnTouch() || PORTDbits.RD7){
        
        //verify if there are errors due to the time
        if(tm0Error){
            //stop the motors
            LATCbits.LATC2 = 0;
            tm0Error = POINTZ_ERROR;
            break;
        }
        
        //step H
        LATCbits.LATC2 = 1;
        
        //wait for the step to be made
        while(!stepMade());
        
        //send the L part of the step
        LATCbits.LATC2 = 0;
        
        //wait for the step to be made
        while(!stepMade());
    }
    
    //stop the adc
    stopADC();
    
    //reset the variable that indicate if the object has been touched
    resetTouch();
    
    //turn off both the timer
    T0CONbits.TMR0ON = 0;
    T2CONbits.TMR2ON = 0;
    //reset the counter registers
    TMR0 = 0;
    TMR2 = 0;
    
    //reset the step
    resetStep();
    
    //if the endstroke is touched than fix the correct error
    if(PORTDbits.RD7){
        tm0Error = END_Z_AXIS;
    }
    
    return(tm0Error);
}

/*
 * Description: Function used to get in touch with the thermal paste. This function
 *              is different form the previous one because generally to touch the
 *              thermal paste there are more pressure requirement. Because the 
 *              memory of the uC is enough 2 separate functions allow more clearness.
 *              It moves until the touch sensor doesn't return something
 *              or the endstroke is verified (Error). 
 *              It uses the TMR0 to verify the time required
 */
char touchTherm(){
    
    //move down
    setDirection(FORWARD, MOTOR3);
    
    T2CONbits.TMR2ON = 1;
    
    tm0Error = ALL_OK;              //set all ok
    tm0Limit = TIME_REF_LIFT;       //set the limit
    T0CONbits.TMR0ON = 1;           //turn on the timer used for the interrupt
    
    //start the adc
    startADC();
    
    //stop if we get in touch with the paste or endstroke
    while(!returnTherm() || PORTDbits.RD7){
        
        //verify if there are problems with the time
        if(tm0Error){
            //stop the motors
            LATCbits.LATC2 = 0;
            tm0Error = POINTZ_ERROR;
            break;
        }
        
        //step H
        LATCbits.LATC2 = 1;
        
        //wait for the step to be made
        while(!stepMade());
        
        //send the L part of the step
        LATCbits.LATC2 = 0;
        
        //wait for the step to be made
        while(!stepMade());
    }
    
    //stop the adc
    stopADC();
    
    //reset the variable that determine the touch of the thermal paste
    resetTherm();
    
    //stop both the timer
    T0CONbits.TMR0ON = 0;
    T2CONbits.TMR2ON = 0;
    //reset the counter registers
    TMR0 = 0;
    TMR2 = 0;
    
    //reset the step
    resetStep();
    
    //set the correct error for endstrokes
    if(PORTDbits.RD7){
        tm0Error = END_Z_AXIS;
    }
    
    return(tm0Error);
}

/*
 * Description: Function used to lift the arm. Called after the 
 *              touch obj or thermal paste. 
 */
char liftArm(){
    
    //move up
    setDirection(BACKWARD, MOTOR3);
    
    T2CONbits.TMR2ON = 1;
    
    tm0Error = ALL_OK;              //set all ok
    tm0Limit = TIME_REF_LIFT;       //set the limit
    T0CONbits.TMR0ON = 1;           //turn on the timer used for the interrupt
    
    
    //Wait to reach the starting postion
    while(!PORTEbits.RE2){
        
        //verify if there are problems related to the time
        if(tm0Error){
            //stop the motors
            LATCbits.LATC2 = 0;
            tm0Error = POINTZ_ERROR;
            break;
        }
        
        
        //Keep moving (H part of the step)
        LATCbits.LATC2 = 1;
        
        //wait for the step to be made
        while(!stepMade());
        
        //send the L part of the step
        LATCbits.LATC2 = 0;
        
        //wait for the step to be made
        while(!stepMade());
    }
    
    //turn off both the timers
    T0CONbits.TMR0ON = 0;
    T2CONbits.TMR2ON = 0;
    //reset both the counter registers
    TMR0 = 0;
    TMR2 = 0;
    
    //reset the step
    resetStep();
    
    return(tm0Error);
}

/*
 * Description: Function used to rotate the object by a defined angle
 * Input: Angle of the rotation, expressed in degrees
 */
void rotateObj(unsigned char rotAngle){
    static const float stepAngle = 0.08789;                         //dimension of a single step in the motor
    static char rotSequence[] = {0b1001, 0b0011, 0b0110, 0b1100};   //sequence to use in order to rotate as desired
    unsigned int i;
    int totStep = rotAngle/stepAngle;                               //total step required
    
    //proceed using the sequence defined above
    for(i = 0; i < totStep; i++){
        //Reset the pins that control the rot motor
        LATD &= 0xC3;
        //Set these pins (shift of 2 bit since start at RD2 not RD0)
        LATD |= rotSequence[i%4] << 2;
        
        //TODO: [ ]change?
        //Delay so that the rotation can be done
        while(!stepMade());
    }
}

/*
 * Description: Function used to enable the breezer
 */
void pickObject(){
    LATDbits.LATD1 = 1;
    
    //TODO: [ ]change?
    //Delay so that the breezer can grab the object
    while(!stepMade());
}

/*
 * Description: Function used to release the breezer?
 */
void releaseObj(){
    LATDbits.LATD1 = 0;
    
    //TODO: [ ]change?
    //Delay so that the breezer can release the object
    while(!stepMade());
}