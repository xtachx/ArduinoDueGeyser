/*  ********************************************************************
 *  This is a library for the Maxim IC MAX31855 Cold-Junction
 *  compensation IC..
 *
 *  Designed for the MAX31855-JASA+, but will work with the same
 *  family of ICs
 *
 *  The code in the Adafruit GIT repo emulates an SPI port but is not
 *  a hardwired SPI bus. this causes problems if you have more than
 *  one SPI device, and hence I have decided to rewrite the code in
 *  hardwired SPI. I hope, this time_t my codes are well commented
 *  and it wont be a spagetti. Trust me, I am trying!
 *
 *  Written by Pitam Mitra for the PICO Dark Matter Search Experiment
 *  detector R&D. GNU GPLv3 License. Enjoy!
 *
 *  Rev 1: Wrote the code. (July 14, 2013)
 *
 *  ********************************************************************
 *
 *  Look at the Arduino documentation on the pinouts for SPI.
 *
 *  Uno: MOSI: 11, MISO:12, SCK 13
 *  Due: MOSI: ICSP-4, MISO: ICSP-1, SCK: ICSP-3.
 *
 *  We will use our own SS pin.
 *
 * *********************************************************************
 */


#ifndef SPITEMPERATURE_HPP_INCLUDED
#define SPITEMPERATURE_HPP_INCLUDED

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class SPITC{
    public:
        SPITC(int8_t SS);

        float readInternalCJ(void);
        float readTC(void);
        uint8_t readError(uint32_t MAXdata);

    private:
        int8_t SlaveSelect;
        uint32_t SPIReadMAX(void);
};

#endif // SPITEMPERATURE_HPP_INCLUDED
