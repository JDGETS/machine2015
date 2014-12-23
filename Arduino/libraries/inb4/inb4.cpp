




//#include "inb4.h"







/*
    VITESSE     -1...0...1  (Backward-Stopped-Forward)
    DIRECTION   -1...0...1  (Left-Straight-Right)
 */

/*

/// INB4 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ///

const uint8_t inb4::PIN_OUT_STATES[] = {53, 51, 49, 47, 45, 43, 35, 33, 31, 29, 27, 25};
const uint8_t inb4::PIN_OUT_LAST_INDEX = 12;

const unsigned short PULL_UP_PIN = 38;

void inb4::setup()


void inb4::loop()

void Level1()
{
    const unsigned long timeout = 20;

    digitalWrite(inb4::PIN_OUT_STATES[0], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[0], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[1], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[1], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[2], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[2], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[3], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[3], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[4], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[4], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[5], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[5], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[6], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[6], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[7], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[7], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[8], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[8], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[9], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[9], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[10], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[10], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[11], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[11], LOW);


    digitalWrite(inb4::PIN_OUT_STATES[12], HIGH);
    delay(timeout);
    digitalWrite(inb4::PIN_OUT_STATES[12], LOW);
};


*/


/// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< INB4 ///

/// MOTORS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ///

//const unsigned short Motors::LEFT_MOTOR_ID = -1us;





/// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< MOTORS ///
/*

#include <SoftwareSerial.h>

#define MOTOR_RIGHT   0
#define MOTOR_LEFT    1

#define MOTOR_LEFT_DIRA_PIN 5
//#define MOTOR_LEFT_DIRB_PIN 5
#define MOTOR_LEFT_PWM_PIN  12
#define MOTOR_RIGHT_DIRA_PIN  6
//#define MOTOR_RIGHT_DIRB_PIN  7
#define MOTOR_RIGHT_PWM_PIN   11

#define XBEE_RX_PIN         10
#define XBEE_TX_PIN         9
#define XBEE_COMM_BAUDRATE  57600


SoftwareSerial XBeeComm(XBEE_RX_PIN, XBEE_TX_PIN);

typedef struct Motor
{
  int PwmVal;
  int CmdVitess;
  int CmdDirection;
}motor_t;

motor_t MotorRight;
motor_t MotorLeft;


ISR(TIMER1_OVF_vect)
{
  OCR1A = MotorLeft.PwmVal;
  OCR1B = MotorRight.PwmVal;
}



void setup()
{
    XBeeComm.begin(XBEE_COMM_BAUDRATE);
    delayMicroseconds(100);
    XBeeComm.println("\bCHAR DE FEU SERIAL ACTIVATED");
    MotorInit();
    XBeeComm.println("CHAR DE FEU PWM ACTIVATED");
}

char DataIn;
int TempSpeed,TempDirection,MotorSpeed;


void loop()
{
    if(XbeeCommGetChar(&DataIn))
    {
        if(DataIn == 'w')//Fwd
        {
            TempSpeed +=1;
        }
        if(DataIn == 's')//Rev
        {
            TempSpeed -=1;
        }
        if(DataIn == 'a')//Left
        {
            TempDirection -=1;
        }
        if(DataIn == 'd')//Right
        {
            TempDirection +=1;
        }
        if(DataIn == 'q')//Stop
        {
            MotorLeft.PwmVal  = map(0, 0, 100, 0, 1599);
            MotorRight.PwmVal = map(0, 0, 100, 0, 1599);
        }
        if(DataIn == 'f')
        {
            MotorSetFoward(MOTOR_RIGHT);
            MotorSetFoward(MOTOR_LEFT);
            MotorLeft.PwmVal  = map(100, 0, 100, 0, 1599);
            MotorRight.PwmVal = map(100, 0, 100, 0, 1599);
        }
        if(DataIn == 'g')
        {
            MotorSetFoward(MOTOR_RIGHT);
            MotorSetReverse(MOTOR_LEFT);
            MotorLeft.PwmVal  = map(100, 0, 100, 0, 1599);
            MotorRight.PwmVal = map(100, 0, 100, 0, 1599);
        }
        XBeeComm.print(MotorLeft.PwmVal,DEC);
        XBeeComm.print("\t");
        XBeeComm.print(MotorRight.PwmVal,DEC);
        XBeeComm.print("\t");
        XBeeComm.print(TempSpeed,DEC);
        XBeeComm.print("\t");
        XBeeComm.println(TempDirection,DEC);
    }
//    //Limit check
//    if(TempSpeed > 100) TempSpeed =100;//100% of speed foward
//    if(TempSpeed < -100)TempSpeed =-100;//100% of speed backward
//    if(TempDirection > 100) TempDirection = 100;//100% turn right
//    if(TempDirection < -100)TempDirection = -100;//100% turn left
//
//    //Motor fwd/rev calculation
//    if(TempSpeed > 0) //FWD
//    {
//      MotorSpeed = TempSpeed;
//      MotorSetFoward(MOTOR_RIGHT);
//      MotorSetFoward(MOTOR_LEFT);
//    }
//    else if (TempSpeed < 0) //REV
//    {
//      MotorSpeed = TempSpeed *(-1);
//      MotorSetReverse(MOTOR_RIGHT);
//      MotorSetReverse(MOTOR_LEFT);
//    }
//    else //Stop
//    {
//      MotorSpeed = 0;
//      MotorStop(MOTOR_RIGHT);
//      MotorStop(MOTOR_LEFT);
//    }
//
    //Direction calculation
//    if(TempDirection == 100)
//    {
//
//    }
//    if(TempDirection == 0)
//    {
//
//      MotorLeft.PwmVal  = map(MotorSpeed, 0, 100, 0, 1599);
//      MotorRight.PwmVal = map(MotorSpeed, 0, 100, 0, 1599);
//    }
//    else if(TempDirection > 0)//Right turn
//    {
//      //To turn right, Left motor must go faster then right
//      MotorLeft.PwmVal = map(MotorSpeed, 0, 100, 0, 1599);
//      MotorSpeed-=TempDirection;
//      if (MotorSpeed<0) MotorSpeed=0;
//      MotorRight.PwmVal = map(MotorSpeed, 0, 100, 0, 1599);
//
//    }
//    else if(TempDirection < 0)//Left turn
//    {
//      //To turn left, Right motor must go faster then left
//      MotorRight.PwmVal = map(MotorSpeed, 0, 100, 0, 1599);
//      MotorSpeed+=TempDirection;//TempDirection is negative;
//      if (MotorSpeed<0) MotorSpeed=0;
//      MotorLeft.PwmVal = map(MotorSpeed, 0, 100, 0, 1599);
//    }
}

/*
int XbeeCommGetChar(char *DataIn)
{
    if(XBeeComm.available())
    {
        (*DataIn) = XBeeComm.read();
        return 1; //New Data
    }
    else
    {
        return 0; //No new data
    }
}


void MotorStop(int Motor)
{
    if(Motor)//Left
    {
        //digitalWrite(MOTOR_LEFT_PWM_PIN, LOW);
        //digitalWrite(MOTOR_LEFT_DIRB_PIN, LOW);
        MotorLeft.PwmVal = 0;
    }
    else//Right
    {
        //digitalWrite(MOTOR_RIGHT_PWM_PIN, LOW);
        //digitalWrite(MOTOR_RIGHT_DIRB_PIN, LOW);
        MotorRight.PwmVal = 0;
    }
}

void MotorSetFoward(int Motor)
{
    if(Motor)//Left
    {
        digitalWrite(MOTOR_LEFT_DIRA_PIN, LOW);
        //digitalWrite(MOTOR_LEFT_DIRB_PIN, LOW);
    }
    else//Right
    {
        digitalWrite(MOTOR_RIGHT_DIRA_PIN, LOW);
        //digitalWrite(MOTOR_RIGHT_DIRB_PIN, LOW);
    }
}

void MotorSetReverse(int Motor)
{
    if(Motor)//Left
    {
        digitalWrite(MOTOR_LEFT_DIRA_PIN, HIGH);
        //digitalWrite(MOTOR_LEFT_DIRB_PIN, HIGH);
    }
    else//Right
    {
        digitalWrite(MOTOR_RIGHT_DIRA_PIN, HIGH);
        //digitalWrite(MOTOR_RIGHT_DIRB_PIN, HIGH);
    }
}

void MotorInit(void)
{
    //SET PIN DIRECTION
    pinMode(MOTOR_LEFT_PWM_PIN, OUTPUT);//OC1A
    pinMode(MOTOR_RIGHT_PWM_PIN, OUTPUT);//OC1B
    pinMode(MOTOR_LEFT_DIRA_PIN, OUTPUT);//Dir1A
    //pinMode(MOTOR_LEFT_DIRB_PIN, OUTPUT);//Dir1B
    pinMode(MOTOR_RIGHT_DIRA_PIN, OUTPUT);//Dir2A
    //pinMode(MOTOR_RIGHT_DIRB_PIN, OUTPUT);//Dir2B

    //SET STOP
    MotorStop(MOTOR_RIGHT);
    MotorStop(MOTOR_LEFT);

    //Setup PWM
    //SET : Fast PWM with Timer OverFlow Interrupt
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(WGM13) | _BV(CS10);
    TIMSK1 = _BV(TOIE1);
    ICR1 = 1599; //Period value, TODO :  Add the correct formula from ATMEGA128 Datasheet
    interrupts();
}

 */



