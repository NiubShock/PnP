typedef struct{
    unsigned char feederLine;
    unsigned char posX;
    unsigned char posY;
    unsigned char rotation;
}t_sequence;

typedef struct{
    unsigned char L;
    unsigned char W;
    unsigned char init_posX;
    unsigned char init_posY;
    unsigned char init_rot;
    unsigned char end_posX;
    unsigned char end_posY;
    unsigned char end_rot; 
}t_newSequence;

void usartInit(void);
void storeData(unsigned char data);
t_sequence* getData(void);
t_newSequence* getNewSequence(void);
void uartTx(unsigned char *ptr, unsigned char length);
void printError(unsigned char errCode);
void printStatus(unsigned char status);

unsigned char newSequence(void);
void resetNewSequence(void);
unsigned char readSeq(void);
unsigned char fatalError(void);
void reduceSeq(void);
void increaseSeq(void);
void shiftData(void);