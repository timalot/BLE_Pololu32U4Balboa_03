// This program shows how to make a Balboa balance on its two
// wheels and drive around while balancing.


//-----Begin Balancer Includes --------
#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;
//----- End Balancer Includes --------

// Integrated from Bluefruit code
//----- Begin Controller Includes ---
#include <string.h>
#include <Arduino.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
/*    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object hardware serial, CTS pin tied to ground.

Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

bool ignoreCommand = 0;
uint16_t GloballeftSpeed = 0, GlobalrightSpeed = 0;
int i = 0;

//----- End Controller Includes ---

void setup()

{

  // From BLE
  //------- Begin Controller Setup --------
  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the BLE module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  Serial.println("Waiting for connection");
  while (! ble.isConnected()) {
      delay(500);
      buzzer.playFrequency(100, 10, 11);
   }
   buzzer.playFrequency(700, 30, 12);
   delay(100);
   buzzer.playFrequency(900, 30, 12);
   delay(100);
   buzzer.playFrequency(1100, 30, 12);
   delay(300);

  Serial.println(F("******************************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
  
 //------- End Controller Setup --------


 // ------ Begin Balancer Setup -------
 
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  motors.allowTurbo(true);
  
 //------- End Balancer Setup -------
  
}

const char song[] PROGMEM =
  "!O6 T240"
  "l32ab-b>cl8r br b-bb-a a-r gr g-4 g4"
  "a-r gr g-gg-f er e-r d4 e-4"
  "gr msd8d8ml d-4d4"
  "l32efg-gl8r msd8d8ml d-4d4"
  "<bcd-d e-efg- ga-ab- a4 gr";

void playSong()
{
  if (!buzzer.isPlaying())
  {
    buzzer.playFromProgramSpace(song);
  }
}

void driveAround()
{
  uint16_t time = millis() % 5000;
  uint16_t leftSpeed, rightSpeed;
  if (time < 1000)
  {
    leftSpeed = -20;
    rightSpeed = -20;
  }
  else if ((time >= 1000) && (time < 2000))
  {
    leftSpeed = -20;
    rightSpeed = -5;
  }
  else if ((time >= 2000) && (time < 3000))
  {
    leftSpeed = -20;
    rightSpeed = -20;
  }
  else
  {
    leftSpeed = -5;
    rightSpeed = -25;
  }
  balanceDrive(leftSpeed, rightSpeed);
}

void standUp()
{
  motors.setSpeeds(0, 0);
  buzzer.play("!>grms>g16>g16>g2");
  ledGreen(1);
  ledRed(1);
  ledYellow(1);
  while (buzzer.isPlaying());
  for (int rampSpeed = 0; rampSpeed < MOTOR_SPEED_LIMIT; rampSpeed++){
    motors.setSpeeds(-rampSpeed, -rampSpeed);
    delay (2);
  }
  delay (100);
  motors.setSpeeds(150, 150);
  for (uint8_t i = 0; i < 20; i++)
  {
    delay(UPDATE_TIME_MS);
    balanceUpdateSensors();
    if(angle < 60000)
    {
      break;
    }
  }
  motorSpeed = 150;
  balanceResetEncoders();
}

void fallBack()
{
  motors.setSpeeds(-100,-100); 
  delay(300);
  motors.setSpeeds(0, 0);
  delay(1000);
}

void ReadCommand()
{
  /* Wait for new data to arrive from BLE */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // Buttons Fwd = 5, Rev = 6, Left = 7, Right = 8
  if ((packetbuffer[1] == 'B') && (packetbuffer[3] - '0')) {
    
    if ((packetbuffer[2] - '0') == 5){
      buzzer.play("c128");
      balanceDrive(-15, -15);  // Forward
      GloballeftSpeed = 0;
      GlobalrightSpeed = 0;
    }
    else if ((packetbuffer[2] - '0') == 6){
      buzzer.play("a128");
      balanceDrive(15, 15);   //  Reverse
      GloballeftSpeed = 0;
      GlobalrightSpeed = 0;
    }
    else if ((packetbuffer[2] - '0') == 8){
      buzzer.play("e128");
      balanceDrive(GloballeftSpeed + 4, GlobalrightSpeed - 4);   //  Pivot Right
    }
    else if ((packetbuffer[2] - '0') == 7){
      buzzer.play("f128");
      balanceDrive(GloballeftSpeed - 4, GlobalrightSpeed + 4);  // Pivot Left 
    }
    // Function Buttons 1, 2, 3, 4
    else if ((packetbuffer[2] - '0') == 4){
      GloballeftSpeed = GloballeftSpeed + 5;
      GlobalrightSpeed = GlobalrightSpeed + 5;
      buzzer.playFrequency((500 - 5 * GlobalrightSpeed), 30, 12);
    }
    else if ((packetbuffer[2] - '0') == 2){
      GloballeftSpeed = GloballeftSpeed - 5;
      GlobalrightSpeed = GlobalrightSpeed - 5;
      buzzer.playFrequency((500 + -5 * GlobalrightSpeed), 30, 12);
    }
    else if ((packetbuffer[2] - '0') == 1){
      standUp();             // Try to stand up
    }
    else if ((packetbuffer[2] - '0') == 3){
      fallBack();            // Lay down
    }
  }
  else {
    balanceDrive(GloballeftSpeed, GlobalrightSpeed);
    }
}
void loop()
{
  balanceUpdate();
  
  if (i > 4000){
    ReadCommand();
    i = 0;
  }
  else {
    i++;
  }


  if (isBalancing())
  {
    // Once you have it balancing well, uncomment these lines for
    // something fun.

    // playSong();
    // driveAround();
  }
  else
  {
    buzzer.stopPlaying();

    if (buttonA.getSingleDebouncedPress())
    {
      //standUp();
    }
  }

  // Illuminate the red LED if the last full update was too slow.
  ledRed(balanceUpdateDelayed());

  // Display feedback on the yellow and green LEDs depending on
  // the variable fallingAngleOffset.  This variable is similar
  // to the risingAngleOffset used in Balance.cpp.
  //
  // When the robot is rising toward vertical (not falling),
  // angleRate and angle have opposite signs, so this variable
  // will just be positive or negative depending on which side of
  // vertical it is on.
  //
  // When the robot is falling, the variable measures how far off
  // it is from a trajectory starting it almost perfectly
  // balanced then falling to one side or the other with the
  // motors off.
  //
  // Since this depends on ANGLE_RATE_RATIO, it is useful for
  // calibration.  If you have changed the wheels or added weight
  // to your robot, you can try checking these items, with the
  // motor power OFF (powered by USB):
  //
  // 1. Try letting the robot fall with the Balboa 32U4 PCB up.
  //    The green LED should remain lit the entire time.  If it
  //    sometimes shows yellow instead of green, reduce
  //    ANGLE_RATE_RATIO.
  //
  // 2. If it is tilted beyond vertical and given a push back to
  //    the PCB-up side again, the yellow LED should remain lit
  //    until it hits the ground.  If you see green, increase
  //    ANGLE_RATE_RATIO.
  //
  // In practice, it is hard to achieve both 1 and 2 perfectly,
  // but if you can get close, your constant will probably be
  // good enough for balancing.
  int32_t fallingAngleOffset = angleRate * ANGLE_RATE_RATIO - angle;
  if (fallingAngleOffset > 0)
  {
    ledYellow(1);
    ledGreen(0);
  }
  else
  {
    ledYellow(0);
    ledGreen(1);
  }

  
}
