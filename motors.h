/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */


#define MOTOR1      0
#define MOTOR2      1
#define MOTOR3      2

//same struct as in the datasheet
#define FULLSTEP    0
#define HALFSTEP    1
#define QUARTSTEP   2
#define SIXTHSTEP   3

//from datasheet enable is on when low
#define TURNON      0
#define TURNOFF     1

//this depends on the mounting side
#define FORWARD     1
#define BACKWARD    0

#define ZCONTACT    100


void writeTM0(void);
unsigned char whatsTM0Limit(void);
void clearTM0(void);

//Interface function


void setStep(unsigned char step, unsigned char motor);
void enableMotor(unsigned char enable, unsigned char motor);
void setDirection(unsigned char direction, unsigned char motor);
void setDecay(unsigned char decay, unsigned char motor);


//global function

/*
 * Description: Function used to initialize the pin as required from the
 *              schematic
 */
void initPinMotors(void);
char resetPosition(void);   //use this function to reset the step motors' positions
char moveToPoint(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);

char touchObject(void);
char touchTherm(void);
char liftArm(void);
void rotateObj(unsigned char rotAngle);
void pickObject(void);
void releaseObj(void);

void abortAll(void);