/*  ********************************************************************
 *  This is a library for the Maxim IC MAX31855 Cold-Junction
 *  compensation IC..
 *
 *  Designed for the MAX31855-JASA+, but will work with the same
 *  family of ICs
 *
 *  The code in the Adafruit GIT repo emulates an SPI port but is not
 *  a hardwired SPI bus. This causes problems if you have more than
 *  one SPI device, and hence I have decided to rewrite the code in
 *  hardwired SPI. I hope, this time my codes are well commented
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


#include "SPITemperature.hpp"
#include <SPI.h>
#include <stdint.h>



SPITC::SPITC(int8_t SS)
{
    SlaveSelect = SS;
    /* initialize the ss pin as an output */
    pinMode(SlaveSelect, OUTPUT);
    /*Begin SPI*/
    //SPI.begin();
    /*Turn SlaveSelect HIGH*/
    digitalWrite(SlaveSelect, HIGH);
}

/* Function to Read the MAX31855 chip on the SS */
uint32_t SPITC::SPIReadMAX(void)
{
    /* Need a 32 bit number to store the data */
    uint32_t result = 0;

    /* Start ChipSelect */
    digitalWrite(SlaveSelect, LOW);

    /*Read 32 bits = 4x1 byte*/
    result = result | SPI.transfer(4, 0x00, SPI_CONTINUE);
    result = result << 8;
    result = result| SPI.transfer(4, 0x00, SPI_CONTINUE);
    result = result<< 8;
    result = result | SPI.transfer(4, 0x00, SPI_CONTINUE);
    result= result<< 8;
    result= result | SPI.transfer(4, 0x00);

    /*Stop ChipSelect*/
    digitalWrite(SlaveSelect, HIGH);

    return result;
}

uint8_t SPITC::readError(uint32_t MAXdata)
{
    /* Check if any of the last 3 bits are 1 */
    //Serial.write(MAXdata&0x7);
    //Serial.println("--");
    //Serial.println(MAXdata);
    //Serial.println("--");
    return MAXdata & 0x7;
}

/*Read Temperature*/
float SPITC::readTC(void)
{
    /* Get raw data */
    register int32_t tempdata;
    register int16_t temp16bit;
    float temperature_celcius;
    tempdata = this->SPIReadMAX();

    /* check for errors */
    if (this->readError(tempdata))
        return NAN;

    /*Get rid of the first 18 bits*/
    tempdata >>= 18;

    /* Take the 13 bits, remove the sign bit */
    temp16bit = tempdata &0x3FFF;

    /* Handle sign bit */
    if (tempdata & 0x2000)
        temp16bit |= 0xC000;

    temperature_celcius = temp16bit;
    /*Apply the scaling - see doc */
    temperature_celcius *= 0.25;

    return temperature_celcius;
}

/* Read internal CJ Temperature */
float SPITC::readInternalCJ(void)
{
    /* Get raw data */
    register int32_t tempdata;
    float temperature_internal;
    tempdata = this->SPIReadMAX();

    /* Ignore bottom 4 bits */
    tempdata >>=4;

    /* Take the 11 lower bits */
    temperature_internal = tempdata & 0x7FF;
    /* Scaling */
    temperature_internal *= 0.0625;
    /* Handle sign */
    if (tempdata&0x800)
        temperature_internal *= -1.0;

    return temperature_internal;
}
