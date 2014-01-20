/*
  Testing code for the MUX
 */

#include "ArdAMP.hpp"

bool stringComplete = false;  // whether the string is complete

bool InParseResult;
char SerialIn[12];
int ByteCount = 0;

void setup()
{

  Serial.begin(115200);
  //Serial.println("TC Test Using new class");

}


void loop(){

    // print the string when a newline arrives:
    if (stringComplete) {

        //Serial.print("A");
        // do stuff
        if (SerialIn[0] == 0xFF & ByteCount==11)
        {
            ArdAMP AMPdaemon(SerialIn);
            AMPdaemon.AMPHandler();

        } else {
            Serial.println(ByteCount);
            //Serial.write(0xFF);
            //Serial.write(0xFF);
        }
        //Serial.write(SerialIn);
        //Serial.write(StrParseResult);
        stringComplete = false;
        //delay(1000);
    }



}



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
