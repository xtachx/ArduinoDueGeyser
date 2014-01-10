/*  ********************************************************************
 *  This is a library for the the ArdAMP protocol for the arduino.
 *
 *  This code is inspired by the AMP protocol for Twisted by
 *  twistedmatrix labs. Instead of the full AMP structure,
 *  this has been modified to always "answer" to variables which needs
 *  continuous monitoring, and respond to other "_ask" commands.
 *
 *  Written by Pitam Mitra for the PICO Dark Matter Search Experiment
 *  detector R&D. GNU GPLv3 License. Enjoy!
 *
 *  Rev 1: Wrote the code. (Aug 26, 2013)
 *
 *  ********************************************************************
 *
 *  I should make a section dedicating the horrible nature of Arduino's
 *  documentation somewhere here. Lets start. Firstly, if you look at the doc
 *  then, serial.write(buf, size) SAYS that this can print a buffer like an
 *  array. But guess what, it FUCKING DOES NOT. It expects (const uint8_t, size_t).
 *  One way around is to use print::write and another is to typecast the char array.
 *  Yea, sit around and do silly typecasts which shouldnt be needed to begin with.
 *
 *  Serial.write((uint8_t *)buf, size);
 *
 */

#include "StorageSpace.h"
#include "ByteBuffer.h"
#include "ArdAMP.hpp"


/* ***************************INCOMING MESSAGES ****************************** */

/*initializer for incoming messages. */
ArdAMP::ArdAMP(char* AMPcommand, float* Setpoint1, float* Setpoint2, float* Setpoint3)
{
    /* Byte 1 and 2 denote the request ID. Cast them to an int */
    AMPreq.ReqID = *(AMPcommand+1)<<8 | *(AMPcommand+2);
    /* Byte 3 and 4 represents the Ops Bytecode. Cast them to an int as well */
    AMPreq.OpsByte = *(AMPcommand+3)<<8 | *(AMPcommand+4);

    /*Bytecode 5,6 are par1 and 7-10 are par2. Cast them to an int, in union with a float */
    AMPreq.par1union.IntVal = *(AMPcommand+5)<<8 | *(AMPcommand+6);
    AMPreq.par2union.IntVal = *(AMPcommand+7)<<24 | *(AMPcommand+8)<<16 | *(AMPcommand+9)<<8 | *(AMPcommand+10) ;

    /* Pointers for Setpoints of the 3 stoodges... ahem PELTSYSes */
    SP1 = Setpoint1;
    SP2 = Setpoint2;
    SP3 = Setpoint3;
}

/* The next line isnt exactly needed, except to debug if memory is being freed correctly */
ArdAMP::~ArdAMP() {}

/* Handler function. Basically decide which action to call */
void ArdAMP::AMPHandler()
{

    /* Pay attention here. Use constructs like this: AMPreq.OpsByte == CHG_PELT_TEMP
     * and use #define directive in the header file to define things like CHG_PELT_TEMP as
     * an uint16_t. You have 16 bit of choices and if that isnt enough, buy a bigger  arduino
     */

    if (AMPreq.OpsByte == CHG_PELT_TEMP)
    {
        this->ChangePelt();
    }
    else if (AMPreq.OpsByte == 2)
    {
        Serial.print("Yada");
    }
    else if (AMPreq.OpsByte == 3)
    {
        Serial.print("Yada");
    }
    else if (AMPreq.OpsByte == 4)
    {
        Serial.print("Yada");
    }
    else
    {
        Serial.print("Bad Opcode");
    }
}

/* Action Function, change peltier temperature */

void ArdAMP::ChangePelt()
{
    /* First get, which peltier (par 1) and what temp (par2) */
    int peltSYSNum = this->AMPreq.par1union.IntVal;
    float newTemperature = this->AMPreq.par2union.FloatVal;

    /* Find out the varible for the corresponding peltier
     * and change it.
     */
    switch (peltSYSNum)
    {
    case 1:
        *SP1 = newTemperature;
        break;
    case 2:
        *SP2 = newTemperature;
        break;
    case 3:
        *SP3 = newTemperature;
        break;
    default:
        Serial.print("No such PELTSYS");
    }
}





/* ***************************OUTGOING MESSAGES ****************************** */



void ArdAMPUnattended::TemperatureSweep(TC_Container * TCData)
{

    char networkPacket[26];

    networkPacket[0] = 0xAA;
    for (int field=0; field<=11; field++){
        memcpy(&networkPacket[2*(field)+1], TCData[field].cVal, 2);
    }
    networkPacket[25]=0x0A;

    Serial.write((uint8_t *) networkPacket, 26);
}


void ArdAMPUnattended::FastPressureSweep(ByteBuffer* buffer)
{
    Serial.write(0xAC);
    while(buffer->getSize() > 0 )
        Serial.write(buffer->get());
    Serial.write(0x0A);
}


void ArdAMPUnattended::PISweep(SlowPressure * SP, CS_Container * CS, char PWM1, char PWM2, char PWM3)
{

    char networkPacket[15];

    /*ISense*/
    networkPacket[0] = 0xAB;
    for (int field=0; field<=3; field++){
        memcpy(&networkPacket[2*(field)+1], CS[field].cVal, 2);
    }

    /*Slow Pressure*/
    memcpy(&networkPacket[9], SP->cVal, 2);

    /*PWM Values*/
    networkPacket[11]= PWM1;
    networkPacket[12]= PWM2;
    networkPacket[13]= PWM3;

    /*Newline char*/
    networkPacket[14]=0x0A;

    Serial.write((uint8_t *) networkPacket, 15);
}

