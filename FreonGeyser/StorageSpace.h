/*
  Header file to track all storage / memory
  structure definitions.
 */


#ifndef STORAGE_HPP_INCLUDED
#define STORAGE_HPP_INCLUDED

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//Thermocouples
typedef union TC_Container_U{
      short iVal;
      char cVal[2];
}TC_Container ;//TCData[12];


//Surrent Sense
typedef union CS_Container_U{
    int ADCVal;
    char cVal[2];
}CS_Container;


//pressure
typedef union SlowPressure_U{
    short iVal;
    char cVal[2];
}SlowPressure;


#endif // Storage_HPP_INCLUDED
