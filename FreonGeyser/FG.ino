/*
  Testing code for the MUX - if this even compiles
 */

#include <MUXDict.hpp>
#include <SPITemperature.hpp>
#include "PID_v1.h"
#include <SPI.h>


#include "ByteBuffer.h"
#include "ArdAMP.hpp"
#include "StorageSpace.h"

/*SET THESE BEFORE USING!!!!*/

#define PELT_PWM1 7
/* Peltier 2 and 3 are flipped in the board!!*/
#define PELT_PWM2 6
#define PELT_PWM3 5

/*The ISense channels*/
#define ISense1 12
#define ISense2 14
#define ISense3 13
#define ISense4 15

/*Define Thermocouples used for the peltier calculation*/
/* Ch 4- BL*/
#define TC_PELTSYS1 0
/* Ch 1 - ML */
#define TC_PELTSYS2 1
/* Ch 5  TL */
#define TC_PELTSYS3 5

/*Make instances of stuff we need. Need the mux library*/
MUXer MUXOne;
/*some variables for channel selection*/
int selectChannel, parsedChannel;
/*assign the SS pin to MUX_COM_D*/
int SlaveSelect = MUX_COM_D;
/*and init the SPI TC code*/
SPITC tc_one(SlaveSelect);
/*PID Stuff*/
//Define Variables we'll be connecting to
float Setpoint1, Input1, Output1;
float Setpoint2, Input2, Output2;
float Setpoint3, Input3, Output3;


/*Variables to Store all sense data */
//Temperature sense;
TC_Container TCData[12];
//Current sense
CS_Container CS[4];
//Slow Pressure
short pressurePin = 2;
SlowPressure SlowP;
ByteBuffer fastPressureBuffer;



/* Specify the links and initial tuning parameters
 * -------------------------
 * The params are like this:
 *
 * PID pidname(Input, Output, Set Point, Kp, Ki, Kd, ControllerDirection)
 * Input : Temperature in C (can be float)
 * Output: 0-255 arduino PWM value
 * Set Point : in degree C (float)
 *
 * Kp: Proportional constant
 * Ki: Integral constant
 * Kd: Derivative constant
 *
 * ControllerDirection: DIRECT or REVERSE
 * http://playground.arduino.cc//Code/PIDLibrarySetControllerDirection
 * We use REVERSE since, we ACCELERATE to reacha  lower temperature
 * we are like a refridgerator, not loke a car in that example
 * -------------------------
 */

PID pelt1(&Input1, &Output1, &Setpoint1, 2, 5, 1, REVERSE);
PID pelt2(&Input2, &Output2, &Setpoint2, 2, 5, 1, REVERSE);
PID pelt3(&Input3, &Output3, &Setpoint3, 2, 5, 1, REVERSE);

/*Virtual Input Variables - for debugging*/
int VInput;

/*Time tracking variable - memory allocation */
unsigned long currentMillis;
long previousMillis = 0;
long interval1 = 1000;
long interval2 = 1000;
long interval3 = 1000;

/*Variables required for the PC<-->Ard protocol */
bool stringComplete = false;  // whether the string is complete
char SerialIn[12];
int ByteCount = 0;

ArdAMPUnattended AGeyserInfoSweep;
long infoSweep_previousMillis = 0;
float infoSweep_interval = 1000;


/*Code that runs once, when your program begins*/
void setup()
{
    Serial.begin(115200);
    //Serial.println("TC Test Using MUXer");
    /*Default Values*/
    Setpoint1=50.0;
    Setpoint2=50.0;
    Setpoint3=50.0;
    //turn the PID on - set it to AUTOMATIC
    pelt1.SetMode(AUTOMATIC);
    //turn the PID on - for debug - MANUAL mode
    pelt2.SetMode(AUTOMATIC);
    //turn the PID o - debug MANUAL mode
    pelt3.SetMode(AUTOMATIC);

    SPI.begin(4);
    SPI.setClockDivider(4, 168);


    pinMode(PELT_PWM1, OUTPUT);
    pinMode(PELT_PWM2, OUTPUT);
    pinMode(PELT_PWM3, OUTPUT);

    fastPressureBuffer.init(1024);


}

/*Function to read a TC, by selecting a channel*/
float ReadTC_MUX(int selectChannel)
{
    /*First, activate a TC sensor IC by setting its
    chipselect to ground*/
    MUXOne.WriteDigitalSignal(selectChannel, 0);

    /* Test read cold junction temperature - DEBUG
    Serial.print("Cold Junction Temp: ");
    Serial.println(tc_one.readInternalCJ());
    */

    /*Read thermocouple temperature and check if there is an error */
    register float therm_temp = tc_one.readTC();

    /*Deselect the chip, by setting CS as HIGH*/
    MUXOne.WriteDigitalSignal(selectChannel, 1);

    return therm_temp;
}


/*This function calculates the new PWM values based on
the present temperature and the past hysterisis
and sets new PWM values

We have bool p1, p2 and p3 indicating
which PIDs are on and which ones to compute.

Remember to set the Input1,2,3 params before
running this function. The reason they are decoupled
is that people may want to just run the thermometer
without the PWM cntrols.
*/



void ParseSerial(void){

        if (SerialIn[0] == 0xFF & ByteCount==11)
        {
            ArdAMP AMPdaemon(SerialIn, &Setpoint1, &Setpoint2, &Setpoint3);
            AMPdaemon.AMPHandler();

        } else {
            Serial.write(0xEE);
            Serial.println(ByteCount);
        }

        stringComplete = false;


}

int ReadCurrentSense(void)
{
    /*Read isense chips*/
    CS[0].ADCVal = MUXOne.ReadAnalogSignal(ISense1);
    CS[1].ADCVal = MUXOne.ReadAnalogSignal(ISense2);
    CS[2].ADCVal = MUXOne.ReadAnalogSignal(ISense3);
    CS[3].ADCVal = MUXOne.ReadAnalogSignal(ISense4);

    return 0;
}

void SweepSenses(void)
{
    /*Update Temperatures*/
    for (int field=0; field<=11; field++){
        TCData[field].iVal = (short)(4.0*ReadTC_MUX(field));
    }

    /*Update Current Sense*/
    ReadCurrentSense();

    /*Update new inputs for PWM*/
    Input1 = (float)TCData[TC_PELTSYS1].iVal/4.0;
    Input2 = (float)TCData[TC_PELTSYS2].iVal/4.0;
    Input3 = (float)TCData[TC_PELTSYS3].iVal/4.0;

    /*Update slow pressure*/
    analogRead(pressurePin);
    SlowP.iVal = analogRead(pressurePin);


}

/*Funtion to populate the fast pressure buffer*/
void updateFastPressure(void){
    /*Populate pressure*/
    fastPressureBuffer.put(analogRead(pressurePin));

    if
}

void UpdatePIDControl(void)
{
    pelt1.Compute();
    pelt2.Compute();
    pelt3.Compute();

    analogWrite(PELT_PWM1, Output1);
    analogWrite(PELT_PWM2, Output2);
    analogWrite(PELT_PWM3, Output3);

    //PWMC_ConfigureClocks (2,2,VARIANT_MCK);
    //analogWrite(PELT_PWM1, 50);

}

/*This is the testing function nobody cares about */
void loop()
{
    //doPIDMagic(1,1,1);
    /* Memory for current time */
    register unsigned long currentMillisecs = millis();

    if(currentMillisecs - infoSweep_previousMillis > infoSweep_interval) {
        // save the last time informationsweep happened
        SweepSenses();
        //ReadTC_MUX(0);
        infoSweep_previousMillis = currentMillisecs;
        AGeyserInfoSweep.TemperatureSweep(TCData);
        AGeyserInfoSweep.PISweep(&SlowP, CS, (char) Output1, (char) Output2, (char) Output3);
        UpdatePIDControl();
    }


    if (stringComplete) {
        ParseSerial();
        //Serial.write(StrParseResult);
        stringComplete = false;
        //delay(1000);
    }


}//loop



/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEventRun() {

    ByteCount = Serial.readBytesUntil('\n', SerialIn, 12);

    if (ByteCount > 0) {
      stringComplete = true;
    }
}
