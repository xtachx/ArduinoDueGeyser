/*  ********************************************************************
 *  This is a library for the analog MUX CD74HC4067SMD.
 *
 *  This code handily MUXes any channel with the routines built in.
 *
 *  Written by Pitam Mitra for the PICO Dark Matter Search Experiment
 *  detector R&D. GNU GPLv3 License. Enjoy!
 *
 *  Rev 1: Wrote the code. (Aug 11, 2013)
 *
 *  ********************************************************************
 *
 *  Look at the Arduino documentation on the pinouts.
 *
 *  We will connect the COM to a digital and an analog pin, and use
 *  it to read/write signals. The MUX is 2 way.
 *
 * *********************************************************************
 */

#include <map>

#ifndef MUXDICT_HPP_INCLUDED
#define MUXDICT_HPP_INCLUDED

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*Muxer pin selector */

#define MUX_A 24
#define MUX_B 26
#define MUX_C 28
#define MUX_D 30

#define MUX_COM_D A8
#define MUX_COM_A 11

class MUXer{
    public:
        MUXer(void);
        int WriteDigitalSignal(int channel, bool highLow);
        int ReadAnalogSignal(int channel);


    private:
        std::map <uint8_t, int> muxmap;
        int SelectChannel(uint8_t channel);
};

#endif // MUXDICT_HPP_INCLUDED
