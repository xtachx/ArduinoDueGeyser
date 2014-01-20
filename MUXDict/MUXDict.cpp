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


#include "MUXDict.hpp"
#include <map>

/*Main initializer, where we define the muxer
 *maps, and set the pinouts */

MUXer::MUXer(void)
{
    /* We generate a mapping function to map
     * the inputs to the 4 switches, represented
     * by a 4 digit binary number */

    /* Assign the maps.
     * ARM-GCC knows Bxxxx as a binary number just like
     * 0xYYYY is a hex number */
    muxmap[0] = B0000;
    muxmap[1] = B1000;
    muxmap[2] = B0100;
    muxmap[3] = B1100;
    muxmap[4] = B0010;
    muxmap[5] = B1010;
    muxmap[6] = B0110;
    muxmap[7] = B1110;
    muxmap[8] = B0001;
    muxmap[9] = B1001;
    muxmap[10] = B0101;
    muxmap[11] = B1101;
    muxmap[12] = B0011;
    muxmap[13] = B1011;
    muxmap[14] = B0111;
    muxmap[15] = B1111;

    /* initialize the COM Pins */
    pinMode(MUX_COM_D, OUTPUT);
    pinMode(MUX_COM_A, INPUT);

    /* initialize the Muxer Pins */
    pinMode(MUX_A, OUTPUT);
    pinMode(MUX_B, OUTPUT);
    pinMode(MUX_C, OUTPUT);
    pinMode(MUX_D, OUTPUT);

}

/* Function to select a channel on chip, and mux it */
int MUXer::SelectChannel(uint8_t channel)
{
    /* We have to use map::find because operator[]
     * is buggy. IF you have a case where there is no map for
     * a certain value, then operator [] adds it to the map
     * and random pins get triggered!!*/

    std::map<uint8_t, int>::iterator it;
    it = this->muxmap.find(channel);

    /* What to do in case no map is found */
    if (it == this->muxmap.end())
    {
        return 1;
    } else {
        /* Store the 4 bit map in an int and use bits to set
         * pinouts.*/
        int binMap;
        binMap = it->second;

        /* Set pinouts defined by the 4 bits */
        B0001&binMap ? digitalWrite(MUX_D, HIGH) : digitalWrite(MUX_D, LOW);
        B0010&binMap ? digitalWrite(MUX_C, HIGH) : digitalWrite(MUX_C, LOW);
        B0100&binMap ? digitalWrite(MUX_B, HIGH) : digitalWrite(MUX_B, LOW);
        B1000&binMap ? digitalWrite(MUX_A, HIGH) : digitalWrite(MUX_A, LOW);

        return 0;
    }
}

/* Function to write a digital signal to the COM pin and mux a channel */
int MUXer::WriteDigitalSignal(int channel, bool highLow)
{
    this->SelectChannel(channel);
    highLow ? digitalWrite(MUX_COM_D, HIGH) : digitalWrite(MUX_COM_D, LOW);
    return 0;
}

/*Function to read analog signal from muxer, usually to read CS pins*/
int MUXer::ReadAnalogSignal(int channel)
{
    int AnalogValue;
    /*Change pinMode to INPUT for analogRead*/
    pinMode(MUX_COM_D, INPUT);
    /*Select channel AFTER setting pinmode to input so we have high impedance*/
    this->SelectChannel(channel);
    AnalogValue = analogRead(MUX_COM_D);
    /*once done, set pinmode to output*/
    pinMode(MUX_COM_D, OUTPUT);

    return AnalogValue;
}
