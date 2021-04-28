#include <xc.h>
#include "main.h"

void serial_tx_char(unsigned char val);
char executeData();

static unsigned char posVector[3] = {0, 0, 0};
static int rotAngle = 0;

static const unsigned char feeder1Pos[2] = {30, 50};
static const unsigned char feeder2Pos[2] = {30, 100};
static const unsigned char feeder3Pos[2] = {30, 150};

static const unsigned char maxFeedX = 60;
static const unsigned char maxFeedY= 200;

static unsigned char newFeeder[2] = {0, 0};

void main(void) {
    
    unsigned char errCode = ALL_OK;
    
    interruptInit();
    initPinMotors();
    initADC();
    usartInit();
    tim0Init();
    tim1Init();
    tim2Init(10);
    
    //reset the initial position
    errCode = resetPosition();
    if(errCode != ALL_OK){
        printError(errCode);
        while(1);
    }
     
    //enable the serial communication
    RCSTA1bits.CREN = 1;    //enable rx
    
    while(1){
        if(newSequence()){
            t_newSequence* newData = getNewSequence();
            
            newFeeder[0] = newData ->init_posX;
            newFeeder[1] = newData ->init_posY;
            
            
            //disable the reception here
            RCSTAbits.CREN = 0;
            
            
            //TODO:Verify that part
            //verify the boundaries and store only if everything is ok
            if(newData ->end_posX > maxFeedX || newData ->end_posY > maxFeedY){
                errCode = BOUNDARY_ERROR;
            }else{
                storeData(newData ->end_posX);
                storeData(newData ->end_posY);
            }
            
            //store the remaining data if no errors occurred
            if(errCode == ALL_OK){
                //start the storing of the data
                storeData(PICK_AND_PLACE);
                storeData(NEW_FEEDER);
                storeData(newData ->end_rot - newData ->init_rot);
            }else{
                printError(errCode); 
            }
            
            //turn on the reception again
            RCSTAbits.CREN = 1;
            
            //reset the variable for new pick and place sequence
            resetNewSequence();            
        }
        if(readSeq()){
            //check if there are any data available
            
            //call the function that will execute the data
            errCode = executeData(); 
            //print the possible error
            printError(errCode);  
            //eliminate the eventual error before proceed
            //with the next instruction
            clearTM0();                     
                                            
            
            //reduce by one the counter -> 1 data used
            reduceSeq();
            shiftData();
        }
    }
    return;
}

/*
 * Description: During the normal operation the data are gonna be received 
 *              through the serial communication. This function will handle 
 *              these data and activate the required function
 * Return: Return one char that is used to describe any kind of error that may
 *         be encountered during the operation
 */
char executeData(){
    t_sequence *data = getData();    //pointer used to store the data
    
    char errCode = ALL_OK;              //start considering everything ok
    
    //the arm can be considered as already lifted here
    //the 1st byte received is the feeder line
    switch(data->feederLine){
        case 0:
            //move to the feeder position
            errCode = moveToPoint(posVector[0], posVector[1], feeder1Pos[0], feeder1Pos[1]);
            //change the actual position -> used as reference for future movement
            posVector[0] = feeder1Pos[0];
            posVector[1] = feeder1Pos[1];
            break;
        case 1:
            errCode = moveToPoint(posVector[0], posVector[1], feeder2Pos[0], feeder2Pos[1]);
            posVector[0] = feeder2Pos[0];
            posVector[1] = feeder2Pos[1];
            break;
        case 2:
            errCode = moveToPoint(posVector[0], posVector[1], feeder3Pos[0], feeder3Pos[1]);
            posVector[0] = feeder3Pos[0];
            posVector[1] = feeder3Pos[1];
            break;
        
        //use this code to define a new pick and place position
        case NEW_FEEDER:
            errCode = moveToPoint(posVector[0], posVector[1], newFeeder[0], newFeeder[1]);
            posVector[0] = newFeeder[0];
            posVector[1] = newFeeder[1];
            break;
        default:
            //in this case the feed line has not been defined
            //fix the error to display that on the serial comunication
            errCode = FEED_LINE_NOT_DEFINED;
            break;
    }
    
    //proceed only if no error is present
    if(!errCode){
        
        //get in touch with the object
        errCode = touchObject();
        if(errCode != ALL_OK){
            return(errCode);
        }
        //pick the object
        pickObject();

        //lift the arm
        errCode = liftArm();
        if(errCode != ALL_OK){
            return(errCode);
        }

        //move to the desired position -> byte number 2-3
        errCode = moveToPoint(posVector[0], posVector[1], data->posX, data->posY);
        if(errCode != ALL_OK){
            return(errCode);
        }
        posVector[0] = data->posX;
        posVector[1] = data->posY;

        //Rotate the object -> byte number 4
        rotAngle = data->rotation;
        //if the angle is smaller than 0 then rotate by 360 degrees
        if(rotAngle < 0){
            rotAngle += 360;
        }
        rotateObj(rotAngle);
        
        //touch the thermal paste
        errCode = touchTherm();
        if(errCode != ALL_OK){
            return(errCode);
        }
        //release the object
        releaseObj();
        
        //lift the arm once again -> restore the initial condition
        errCode = liftArm();
        if(errCode != ALL_OK){
            return(errCode);
        }
    }
    
    return(errCode);
}