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
 *  ********************************************************************/


#ifndef ARDAMP_HPP_INCLUDED
#define ARDAMP_HPP_INCLUDED

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "StorageSpace.h"
#include "ByteBuffer.h"

/* ***************************INCOMING MESSAGES ****************************** */


/*
 * Put all your defines here. Define each function as a number.
 * I am taking number 1 for change peltier temperature.
 */

#define CHG_PELT_TEMP 1

/*
 * This is an AMP Box. This contains the entire message
 * as sent by a computer. The protocol is defined as:
 *
 * FF (2 byte ID) (2 byte Ops Bytecode) (2 bytes par1) (4 bytes par2) 0a
 *
 * The ID: It is the message identifier. If the Arduino answers back, it
 * needs this to "tell" the computer what it is answering to.
 *
 * Ops Byte code: 2 Bytes identifying what action the arduino is supposed
 * to execute. Kind of like a map of 2bytes --> function to exec
 *
 * par1 and 2: parameters for the function.
 *
 * The AMP Box will store the request ID, OpsByte and an union sharing
 * int and float values of par1union and par2.
 */

struct AMPbox{
        uint16_t ReqID;
        int OpsByte;
        union{
            float FloatVal;
            int IntVal;
        } par1union, par2union;
};


/*
 * This class shares all the information and methods
 * needed to process an incoming message. this includes
 * access to pointers of other variables to modify them
 * eg SetPoints, and its own message parser as well as
 * methods to act upon the messages it gets and send an
 * error if that is the case.
 */

class ArdAMP{
    public:

        /* Constructor and Destructor are obvious */
        ArdAMP(char *, float*, float*, float*);
        ~ArdAMP(void );

        /* Handler for the incoming msg */
        void AMPHandler(void );

        /* ***HELPER FUNCTIONS ****
         * Funtions which defines the actions possible from a message
        */

        void ChangePelt(void );
        //The next fer are not implemented yet, but will be, sooner or later.
        //void CJTemperatures(void);

        /* The error handler */
        //bool AMPError(int error);

    private:
        /* 3 float pointers to point to "setpoint"s of the PWM controls */
        float* SP1;
        float* SP2;
        float* SP3;

        /* The AMPBox struct */
        struct AMPbox AMPreq;
};


/* ***************************Outgoing MESSAGES ****************************** */

class ArdAMPUnattended{

    public:
        //ArdAMPUnatttended(void);
        void TemperatureSweep(TC_Container* );
        void PISweep(SlowPressure*, CS_Container*, char, char, char );
        void FastPressureSweep(ByteBuffer* );

        //void StatusSweep(struct*);
        //private:
};



#endif // MUXDICT_HPP_INCLUDED
