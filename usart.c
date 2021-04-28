/*
 * File:   usart.c
 * Author: alle9
 *
 * Created on December 17, 2020, 9:14 AM
 */


#include <xc.h>
#include "main.h"
#include "string.h"

static unsigned char errString_Feed[] = "Error, feed line not defined\n";
static unsigned char errString_ZEND[] = "Error, no object relevated along Z Axis\n";
static unsigned char errString_Reset[] = "Error, time exceeded to reset position\n";
static unsigned char errString_Point[] = "Error, time exceeded to reach the point\n";
static unsigned char errString_Bound[] = "Error, point outside boundaries\n";
static unsigned char errString_PointZ[] = "Error, time exceeded to reach the end of the Z Axis\n";
static unsigned char errString_Fatal[] = "Fatal Error, please reset the device!\n";
static unsigned char errString_Command[] = "Command not recognized\n";
static unsigned char dataCounter = 0;       //cunter of complete sequence of data
static unsigned char _fatalError = 0;
static unsigned char _newSequence = 0;

//1st byte is the feeder line
//2nd and 3rd byte is the final position
//4th byte is the rotation
static t_sequence dataSequence[5];
static t_newSequence newSequenceData;

/*
 * Description: Function used to return the data received. Return just the 
 *              pointer that store the data
 */
t_sequence* getData(){
    return(&dataSequence[0]);
}

/*
 * Description: Function used to return the new data received. Return just the 
 *              pointer that store the data
 */
t_newSequence* getNewSequence(){
    return(&newSequenceData);
}

/*
 * Description: Function used to reduce by 1 the number of data stored (the counter)
 */
void reduceSeq(){
    dataCounter--;
}

/*
 * Description: Function used to understand how many data have been saved
 */
unsigned char readSeq(){
    return dataCounter;
}

unsigned char fatalError(){
    return _fatalError;
}

unsigned char newSequence(){
    return _newSequence;
}

/*
 * Description: Used to reset the newSeq. variable. Automatically set if the
 *              command is obtained
 */
void resetNewSequence(){
    _newSequence = 0;
}

/*
 * Description: Function called to move all the data on the left by 1
 *              If called the data in the position 0 have been read already
 */
void shiftData(){
    char i;
    
    for(i = 0; i < 4; i++){
        dataSequence[i] = dataSequence[i+1];
    }
}

/*
 * Description: Function used to intialize the uart
 */
void usartInit(void){
    
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 1;
    
    TXSTAbits.TX9 = 0;     //8 bit
    TXSTAbits.TXEN = 1;    //enable
    TXSTAbits.SYNC = 0;    //asynch
    TXSTAbits.BRGH = 0;    //low speed (9600)
    
    RCSTAbits.SPEN = 1;    //Serial port enabled (pin control)
    RCSTAbits.RX9 = 0;     //receive 8bit
    RCSTAbits.CREN = 0;    //disable rx
    RCSTAbits.ADDEN = 0;   //no address detection
    
    //load the baudrate (Fosc = 40MHZ, +0.16% error) 64 total
    SPBRG = 25;
}

void printError(unsigned char errCode){
    
    switch(errCode){
        case FEED_LINE_NOT_DEFINED:
            uartTx(&errString_Feed[0], sizeof(errString_Feed));
            break;
        case END_Z_AXIS:
            uartTx(&errString_ZEND[0], sizeof(errString_ZEND));
            break;
        case RESET_ERROR:
            uartTx(&errString_Reset[0], sizeof(errString_Reset));
            break;
        case POINT_ERROR:
            uartTx(&errString_Point[0], sizeof(errString_Point));
            break;
        case BOUNDARY_ERROR:
            uartTx(&errString_Bound[0], sizeof(errString_Bound));
            break;
        case POINTZ_ERROR:
            uartTx(&errString_PointZ[0], sizeof(errString_PointZ));
            break; 
        case FATAL_ERROR:
            uartTx(&errString_Fatal[0], sizeof(errString_Fatal));
            break;
        case NO_CMD:
            uartTx(&errString_Command[0], sizeof(errString_Command));
            break;
                 
        default:
            break;
    }
}

/*
 * Description: Function used to send data through the serial communication.
 *              Used for error report
 */
void uartTx(unsigned char *ptr, unsigned char length)
{    
    //proceed for all the lenght of the message
    for(char i = 1; i < length; i++){
        //store the data in the register
        TXREG = *ptr;
        //wait for the comple transmission
        while(!TXSTAbits.TRMT);
        //advance in the pointer
        ptr++;
    }
}

/*
 * Description: Function called to store the data
 */
void storeData(unsigned char data){
    
    static unsigned char counter = 0;       //counter for each single sequence
    static unsigned char mexLength = 5;
    static unsigned char command;
    static unsigned char receivedMex[9];
    
    
    //save the data inside the array
    receivedMex[counter] = data;    
    //increase the counter so the next data is gonna be next in the array
    counter++;
    
    //correct the length of the message
    switch(receivedMex[0]){
        case PICK_AND_PLACE:
            mexLength = 5;
            break;
        case NEW_PICK:
            mexLength = 9;
            break;
        case FATAL_CMD:
            mexLength = 1;
            break;
        default:
            break;

    }
    
    //all the message has been received
    if(counter >= mexLength){
        counter = 0;                    //reset the counter
        
        //check what is the command and save the data
        switch(receivedMex[0]){
            case PICK_AND_PLACE:
                dataSequence[dataCounter].feederLine = receivedMex[1];
                dataSequence[dataCounter].posX = receivedMex[2] * 5;
                dataSequence[dataCounter].posY = receivedMex[3] * 5;
                dataSequence[dataCounter].rotation = receivedMex[4];

                dataCounter++;
                break;
            case NEW_PICK:                
                //save the data
                newSequenceData.L = receivedMex[1];
                newSequenceData.W = receivedMex[2];
                newSequenceData.init_posX = receivedMex[3] * 5;
                newSequenceData.init_posY = receivedMex[4] * 5;
                newSequenceData.init_rot = receivedMex[5];
                newSequenceData.end_posX = receivedMex[6] * 5;
                newSequenceData.end_posY = receivedMex[7] * 5;
                newSequenceData.end_rot = receivedMex[8];
                
                _newSequence = 1;
                break;
            case FATAL_CMD:
                //variable used to notify the issues
                _fatalError = 1;
                break;
            default:
                //report the error of not recognized command
                printError(NO_CMD);
                break;
                
        }
    }
}