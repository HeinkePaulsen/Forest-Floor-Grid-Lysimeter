/*
This code contains the calibration procedure for the Load cells and HX711. 
It needs to be done for every lysimeetr seperately.
*/

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <HX711.h>
#include <Wire.h>
#include "SparkFun_External_EEPROM.h"
#include "SDI12.h"
#include "RTClib.h"

// functions
void  INIT_PORTS();
void  SET_MUX16(byte Channel);
int   READ_ANALOG_MUX16(byte Channel);
void  I2C_SCANNER();
void  TEST_HX711();
void  TEST_SYSTEM();
void  EEPROM_TEST();
void  SDI_Task();
void  parseSdi12Cmd(String command, String* dValues);
void  formatOutputSDI(float* measurementValues, String* dValues, unsigned int maxChar);
void  Delte_ARRAY ();
void  printDirectory(File dir, int numTabs);
void  Run_HX();


//I2C Adressen
//I2C EEPROM  address 0x50  
//I2C RTC     address 0x68 

// Defs
#define MUX_S0 8
#define MUX_S1 9
#define MUX_S2 10
#define MUX_S3 3

#define ANALOG_READ_PIN A6
#define HX_CLK 19
#define HX_DAT 16
#define DAC_OUT A0
#define SDI12_PIN 6
#define SDI12_DIR 2

#define KIPP_1 0
#define KIPP_2 7
#define KIPP_3 4
#define KIPP_4 1

#define WAIT 0
#define INITIATE_CONCURRENT   1
#define INITIATE_MEASUREMENT  2
#define MAX_STRING_LENGHT 20

#define WIRE Wire

// variables
float Kipp[4]   = {0};
bool Kipp_Inv_1     = false;
bool Kipp_Inv_2     = false;
bool Kipp_Inv_3     = false;
bool Kipp_Inv_4     = false;
char sensorAddress  = 'a';
int  state          = 0;
float KIPP_Multi    = 1;   // Multiplikator für Kippwaage

RTC_DS1307 rtc;
ExternalEEPROM myMem;
HX711 scale1; HX711 scale2; HX711 scale3; HX711 scale4;
File root;

// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12 slaveSDI12(SDI12_PIN, SDI12_DIR);

volatile float f;

void setup() {

  INIT_PORTS();
  WIRE.begin();
  // 10BIT für DAC einstellen
  analogWriteResolution(10);
  // DAC auf 0 Stellen damit LED aus!
  analogWrite(DAC_OUT,0);

  Serial.begin(115200);
  while(!Serial); 

  slaveSDI12.begin();
  slaveSDI12.forceListen();  // sets SDIPIN as input to prepare for incoming message

  TEST_SYSTEM();
}


void loop() 
{
Run_HX();
SDI_Task();
}

void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void SDI_Task()
{   
  static String dValues[10];  // 10 String objects to hold the responses to aD0!-aD9! commands
  static String commandReceived = "";  // String object to hold the incoming command
  // If a byte is available, an SDI message is queued up. Read in the entire message
  // before proceding.  It may be more robust to add a single character per loop()
  // iteration to a static char buffer; however, the SDI-12 spec requires a precise
  // response time, and this method is invariant to the remaining loop() contents.
  int avail = slaveSDI12.available();
  if (avail < 0) {
    //Serial.println("clear");
    slaveSDI12.clearBuffer();
  }  // Buffer is full; clear

  else if (avail > 0) {
    for (int a = 0; a < avail; a++) {
      char charReceived = slaveSDI12.read(); 
     // Serial.println(charReceived);
      //Serial.println(charReceived);     
      // Character '!' indicates the end of an SDI-12 command; if the current
      // character is '!', stop listening and respond to the command
      if (charReceived == '!') {
        //Serial.println("Command:" + commandReceived);
        // Command string is completed; do something with it
        parseSdi12Cmd(commandReceived, dValues);
        // '!' should be the last available character anyway, but exit the "for" loop
        // just in case there are any stray characters
        slaveSDI12.clearBuffer();
        // Clear command string to reset for next command
        commandReceived = "";
        break;
      }

      else if(!isAlphaNumeric(charReceived))
      {
          charReceived = 0;
      }
      else {
        // Append command string with new character
        commandReceived += String(charReceived);        
      }
    }
  }

  // For aM! and aC! commands, parseSdi12Cmd will modify "state" to indicate that
  // a measurement should be taken
  switch (state) {
    case WAIT: break;
    case INITIATE_CONCURRENT:
      // Do whatever the sensor is supposed to do here
      // For this example, we will just create arbitrary "simulated" sensor data
      // NOTE: Your application might have a different data type (e.g. int) and
      //       number of values to report!
      formatOutputSDI(Kipp, dValues, 75);
      state = WAIT;
      slaveSDI12.forceListen();  // sets SDI-12 pin as input to prepare for incoming
                                 // message AGAIN
      break;
    case INITIATE_MEASUREMENT:
      // Do whatever the sensor is supposed to do here
      // For this example, we will just create arbitrary "simulated" sensor data
      // NOTE: Your application might have a different data type (e.g. int) and
      //       number of values to report!

      // Populate the "dValues" String array with the values in SDI-12 format
      formatOutputSDI(Kipp, dValues, 35);
      Delte_ARRAY();
      // For aM!, Send "service request" (<address><CR><LF>) when data is ready
      slaveSDI12.sendResponse(String(sensorAddress) + "\r\n");
      state = WAIT;
      slaveSDI12.forceListen();  // sets SDI-12 pin as input to prepare for incoming
                                 // message AGAIN
      break;
  }
}

void parseSdi12Cmd(String command, String* dValues) {

  /* Ingests a command from an SDI-12 master, sends the applicable response, and
   * (when applicable) sets a flag to initiate a measurement
   */

  // First char of command is always either (a) the address of the device being
  // probed OR (b) a '?' for address query.
  // Do nothing if this command is addressed to a different device
  if (command.charAt(0) != sensorAddress && command.charAt(0) != '?') { return; }

  // If execution reaches this point, the slave should respond with something in
  // the form:   <address><responseStr><Carriage Return><Line Feed>
  // The following if-switch-case block determines what to put into <responseStr>,
  // and the full response will be constructed afterward. For '?!' (address query)
  // or 'a!' (acknowledge active) commands, responseStr is blank so section is skipped
  String responseStr = "";
  if (command.length() > 1) {
    switch (command.charAt(1)) {
      case 'I':
        // Identify command
        // Slave should respond with ID message: 2-char SDI-12 version + 8-char
        // company name + 6-char sensor model + 3-char sensor version + 0-13 char S/N
        responseStr = "13WAGNERDE0000011.0001";  // Substitute proper ID String here
        break;
      case 'C':
        // Initiate concurrent measurement command
        // Slave should immediately respond with: "tttnn":
        //    3-digit (seconds until measurement is available) +
        //    2-digit (number of values that will be available)
        // Slave should also start a measurment and relinquish control of the data line
        responseStr =
          "00204";  // 4 values ready in 5 sec; Substitue sensor-specific values here
        // It is not preferred for the actual measurement to occur in this subfunction,
        // because doing to would hold the main program hostage until the measurement
        // is complete.  Instead, we'll just set a flag and handle the measurement
        // elsewhere.
        state = INITIATE_CONCURRENT;
        break;
        // NOTE: "aC1...9!" commands may be added by duplicating this case and adding
        //       additional states to the state flag
      case 'M':
        // Initiate measurement command
        // Slave should immediately respond with: "tttnn":
        //    3-digit (seconds until measurement is available) +
        //    1-digit (number of values that will be available)
        // Slave should also start a measurment but may keep control of the data line
        // until advertised time elapsed OR measurement is complete and service request
        // sent
        responseStr =
          "00204";  // 4 values ready in 2 sec; Substitue sensor-specific values here
        // It is not preferred for the actual measurement to occur in this subfunction,
        // because doing to would hold the main program hostage until the measurement is
        // complete.  Instead, we'll just set a flag and handle the measurement
        // elsewhere. It is preferred though not required that the slave send a service
        // request upon completion of the measurement.  This should be handled in the
        // main loop().
        state = INITIATE_MEASUREMENT;
        break;
        // NOTE: "aM1...9!" commands may be added by duplicating this case and adding
        //       additional states to the state flag

      case 'D':
        // Send data command
        // Slave should respond with a String of values
        // Values to be returned must be split into Strings of 35 characters or fewer
        // (75 or fewer for concurrent).  The number following "D" in the SDI-12 command
        // specifies which String to send    
        if((int)command.charAt(2) - 48 < 10)
        {
        responseStr = dValues[(int)command.charAt(2) - 48];
        //Serial.print("Send Data: ");     
        }    
        break;
      case 'A':
        // Change address command
        // Slave should respond with blank message (just the [new] address + <CR> +
        // <LF>)
        sensorAddress = command.charAt(2);
        break;
      default:
        // Mostly for debugging; send back UNKN if unexpected command received
        responseStr = "UNKN";
        break;
    }
  }

//Serial.print("Data Out:");
//Serial.println(String(sensorAddress) + responseStr);   
  // Issue the response speficied in the switch-case structure above.
  slaveSDI12.sendResponse(String(sensorAddress) + responseStr + "\r\n");
}

void formatOutputSDI(float* measurementValues, String* dValues, unsigned int maxChar) {
  /* Ingests an array of floats and produces Strings in SDI-12 output format */

  dValues[0] = "";
  int j      = 0;

  // upper limit on i should be number of elements in measurementValues
  for (int i = 0; i < 4; i++) {
    // Read float value "i" as a String with 6 deceimal digits
    // (NOTE: SDI-12 specifies max of 7 digits per value; we can only use 6
    //  decimal place precision if integer part is one digit)
    String valStr = String(measurementValues[i], 2);
    // Explictly add implied + sign if non-negative
    if (valStr.charAt(0) != '-') { valStr = '+' + valStr; }
    // Append dValues[j] if it will not exceed 35 (aM!) or 75 (aC!) characters
    if (dValues[j].length() + valStr.length() < maxChar) {
      dValues[j] += valStr;
    }
    // Start a new dValues "line" if appending would exceed 35/75 characters
    else {
      dValues[++j] = valStr;
    }
  }

  // Fill rest of dValues with blank strings
  while (j < 9) { dValues[++j] = ""; }
}

void Delte_ARRAY ()
{
  for (int i = 0; i < 4; i++)
  {
    Kipp[i] = 0;
  }  
}

void TEST_SYSTEM()
{
  TEST_HX711();
}

void Run_HX()
{
  Serial.println("********** HX711 Calibration Start **********");

  SET_MUX16(0);
  scale1.set_scale();
  delay(5);
  Serial.print("Channel 1: ");
  scale1.tare();
  Serial.println("Tare done...");
  Serial.print("Place a known weight on scale 1...");
  delay(5000);
  f = scale1.get_units(10);
  Serial.println(f);
  delay(5000);
  

  SET_MUX16(1);
  scale2.set_scale();
  delay(5);
  Serial.print("Channel 2: ");
  scale2.tare();
  Serial.println("Tare done...");
  Serial.print("Place a known weight on scale 2...");
  delay(5000);
  f = scale2.get_units(10);
  Serial.println(f);
  delay(5000);

  SET_MUX16(2);
  scale3.set_scale();
  delay(5);
  Serial.print("Channel 3: ");
  scale3.tare();
  Serial.println("Tare done...");
  Serial.print("Place a known weight on scale 3...");
  delay(5000);
  f = scale3.get_units(10);
  Serial.println(f);
  delay(5000);
  
  SET_MUX16(3);
  scale4.set_scale();
  delay(5);
  Serial.print("Channel 4: ");
  scale4.tare();
  Serial.println("Tare done...");
  Serial.print("Place a known weight on scale 4...");
  delay(5000);
  f = scale4.get_units(10);
  Serial.println(f);
  delay(5000);
  Serial.println("********** HX711 Calibration END **********");
}


void TEST_HX711()
{
  Serial.println("********** HX711 Test Setup **********");

  scale1.begin(HX_DAT, HX_CLK);
  scale2.begin(HX_DAT, HX_CLK);
  scale3.begin(HX_DAT, HX_CLK);
  scale4.begin(HX_DAT, HX_CLK);

  SET_MUX16(0);
  delay(5);
  Serial.print("Channel 1: ");
  f = scale1.get_units(5);
  Serial.println(f,1);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  Serial.print("Channel 2: ");
  f = scale2.get_units(5);
  Serial.println(f,1);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  Serial.print("Channel 3: ");
  f = scale3.get_units(5);
  Serial.println(f,1);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  Serial.print("Channel 4: ");
  f = scale4.get_units(5);
  Serial.println(f,1);
  delay(5);
  Serial.println("********** HX711 Test END **********");
}

void INIT_PORTS()
{
  pinMode(HX_DAT, INPUT);
  pinMode(ANALOG_READ_PIN, INPUT);

  pinMode(MUX_S0 , OUTPUT);
  pinMode(MUX_S1 , OUTPUT);
  pinMode(MUX_S2 , OUTPUT);
  pinMode(MUX_S3 , OUTPUT);
  SET_MUX16(0);
  
  pinMode(HX_CLK , OUTPUT);
  pinMode(DAC_OUT , OUTPUT);
  pinMode(SDI12_PIN , OUTPUT);
  pinMode(SDI12_DIR , OUTPUT);
}


void SET_MUX16(byte Channel)
{
if(Channel == 0)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 1)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 2)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 3)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 4)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 5)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 6)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 7)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, LOW);
}
else if(Channel == 8)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 9)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 10)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 11)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 12)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 13)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 14)
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, HIGH);
}
else if(Channel == 15)
{
  digitalWrite(MUX_S0, HIGH);
  digitalWrite(MUX_S1, HIGH);
  digitalWrite(MUX_S2, HIGH);
  digitalWrite(MUX_S3, HIGH);
}
else 
{
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
}
}

int READ_ANALOG_MUX16(byte Channel)
{
    SET_MUX16(Channel);
    delay(2);
    int tmp = analogRead(ANALOG_READ_PIN);
    SET_MUX16(0);
    return tmp;
}
