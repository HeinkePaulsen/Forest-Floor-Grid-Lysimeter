/*
Das ist V2 serial, 
hier werden alle Lesungen (COLLECT_DATA()) auf die serielle Schnittstelle ausgegeben
Das Messintervall liegt bei 15 Sekunden
getestet am 28.12. - läuft
aktuell EC, Photo und Temp ausgeblendet
Lysi 7 LC ausgetauscht und neu kalibriert am 5.7.24
*/

#define SERIAL_NUMBER 8                      // Hier definieren Sie die Seriennummer OHNE NULLEN!!!
const long interval = 10000;                 // und hier das Messintervall

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <HX711.h>
#include <Wire.h>
#include "SparkFun_External_EEPROM.h"
#include "SDI12.h"
#include "RTClib.h"

// Funktionen
void  INIT_PORTS();
void  SET_MUX16(byte Channel);
int   READ_ANALOG_MUX16(byte Channel);
void  I2C_SCANNER();
void  parseSdi12Cmd(String command, String* dValues);
void  formatOutputSDI(float* measurementValues, String* dValues, unsigned int maxChar);
void  Delte_ARRAY ();
void  printDirectory(File dir, int numTabs);
void  TEST_SYSTEM();
void  TEST_ANALOG();
void  EEPROM_TEST();
void  SDI_Task();
void  TEST_RTC();
void  SETUP_HX711();
void  TEST_SD_CARD();
void  COLLECT_DATA();

int Read_Photo1();
int Read_Photo2();
int Read_Photo3();
int Read_Photo4();
int Read_Photo5();
int Read_Photo6();
int Read_Photo7();
int Read_Photo8();

int Read_EC1();
int Read_EC2();
int Read_EC3();
int Read_EC4();

float Read_Temp1();
float Read_Temp2();
float Read_Temp3();
float Read_Temp4();

// ISR für Kippwaagen
void ISR_KIPP_1();
void ISR_KIPP_2();
void ISR_KIPP_3();
void ISR_KIPP_4();

// I2C Adressen
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
#define DAC_OUT A0      // das sind die LEDs auf 0 == aus
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

// Variablen
unsigned long previousMillis = 0; 


float Kipp[4]   = {0};
bool Kipp_Inv_1     = false;
bool Kipp_Inv_2     = false;
bool Kipp_Inv_3     = false;
bool Kipp_Inv_4     = false;
char sensorAddress  = 'a';
int  state          = 0;
float KIPP_Multi    = 1;   // Multiplikator für Kippwaage

HX711 scale1; HX711 scale2; HX711 scale3; HX711 scale4;
volatile float fa; volatile float fb; volatile float fc; volatile float fd;

File dataFile;
char filename[] = "yyyymmdd.csv";

ExternalEEPROM myMem;
RTC_DS1307 rtc;

// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
   DateTime now = rtc.now();
// return date using FAT_DATE macro to format fields
   *date = FAT_DATE(now.year(), now.month(), now.day());
// return time using FAT_TIME macro to format fields
   *time = FAT_TIME(now.hour(), now.minute(), now.second());
   }

// Create object by which to communicate with the SDI-12 bus on SDIPIN
SDI12 slaveSDI12(SDI12_PIN, SDI12_DIR);


// ***************************************************************
// hier finden sich setup und loop
// ***************************************************************


void setup() {
   delay(5000);                           // kurz warten um Verschlucken bei Stromansatz zu verhindern
   INIT_PORTS();
   WIRE.begin();
   analogWriteResolution(10);             // 10BIT für DAC einstellen 
   analogWrite(DAC_OUT,0);                // DAC auf 0 Stellen damit LED aus!

   Serial.begin(115200);
   //while(!Serial);                      // ausschalten damit Platine auch ohne Computeranschluss startet

   slaveSDI12.begin();
   slaveSDI12.forceListen();              // sets SDIPIN as input to prepare for incoming message

   TEST_SYSTEM();
   DateTime now = rtc.now();
 
     Serial.println("Read all values and print on serial");
     Serial.println("Date  Time  Scale_1  Scale_2  Scale_3 Scale_4 Total TB_1  FD_1  EC_1  Temp_1  TB_2  FD_2  EC_2  Temp_2 TB_3  FD_3  EC_3  Temp_3 TB_4  FD_4  EC_4 Temp_4");   //an aufgenommene Daten anpassen
   }


void loop() {
   unsigned long currentMillis = millis(); 
   if (currentMillis - previousMillis >= interval){
       previousMillis = currentMillis;
    COLLECT_DATA();
     }
  //SDI_Task();
  }

// ***************************************************************
// hier beginnen die einzelnen Funktionen
// ***************************************************************

void TEST_SYSTEM(){
    TEST_RTC();
    TEST_SD_CARD();
    I2C_SCANNER();
    SETUP_HX711();
    TEST_ANALOG();
    }

void TEST_SD_CARD(){
    Serial.println("*********** Test SD Card Start ****************");
    Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_SS_PIN)) {
    Serial.println("initialization failed!");
    while (1);}
    Serial.println("initialization done.");

  dataFile = SD.open("/");
  printDirectory(dataFile, 0);
  Serial.println("done!");
  Serial.println("*********** Test SD Card End ****************");
  }

void printDirectory(File dir, int numTabs) {
    while (true) {
    File entry =  dir.openNextFile();
   if (! entry) {
      // no more files
      break;}
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');}
    Serial.print(entry.name());
   if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC); }
    entry.close();}
    }

void SDI_Task(){   
  static String dValues[10];  // 10 String objects to hold the responses to aD0!-aD9! commands
  static String commandReceived = "";  // String object to hold the incoming command
      // If a byte is available, an SDI message is queued up. Read in the entire message
      // before proceding.  It may be more robust to add a single character per loop()
      // iteration to a static char buffer; however, the SDI-12 spec requires a precise
      // response time, and this method is invariant to the remaining loop() contents.
  int avail = slaveSDI12.available();
  if (avail < 0) {
      //Serial.println("clear");
    slaveSDI12.clearBuffer();}  // Buffer is full; clear

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
        break; }
  else if(!isAlphaNumeric(charReceived)){
          charReceived = 0;}
        else {
        // Append command string with new character
       commandReceived += String(charReceived);        
      } }
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
      break; }
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
  for (int i = 0; i < 4; i++) {
  Kipp[i] = 0;
  }  
}

void EEPROM_TEST()
{
  Serial.println("*********** Test EEPROM Start ****************");

if (myMem.begin() == false)
  {
    Serial.println("No memory detected. Freezing.");
    while (1)
      ;
  }
  Serial.println("Memory detected!");

  Serial.print("Mem size in bytes: ");
  Serial.println(myMem.length());

  //Yes you can read and write bytes, but you shouldn't!
  byte myValue1 = 200;
  myMem.write(0, myValue1); //(location, data)

  byte myRead1 = myMem.read(0);
  Serial.print("I read: ");
  Serial.println(myRead1);

  //You should use gets and puts. This will automatically and correctly arrange
  //the bytes for larger variable types.
  int myValue2 = -366;
  myMem.put(10, myValue2); //(location, data)
  int myRead2;
  myMem.get(10, myRead2); //location to read, thing to put data into
  Serial.print("I read: ");
  Serial.println(myRead2);

  float myValue3 = -7.35;
  myMem.put(20, myValue3); //(location, data)
  float myRead3;
  myMem.get(20, myRead3); //location to read, thing to put data into
  Serial.print("I read: ");
  Serial.println(myRead3);

  String myString = "Hi, I am just a simple test string";
  unsigned long nextEEPROMLocation = myMem.putString(30, myString);
  String myRead4 = "";
  myMem.getString(30, myRead4);
  Serial.print("I read: ");
  Serial.println(myRead4);
  Serial.print("Next available EEPROM location: ");
  Serial.println(nextEEPROMLocation);  

  Serial.println("*********** Test EEPROM END ****************");
}

void TEST_RTC(){
Serial.println("*********** Test RTC Start ****************");
    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
   if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
  }
   // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  delay(3000);
  DateTime time = rtc.now();
 
  Serial.println(String("DateTime::TIMESTAMP_FULL:\t")+time.timestamp(DateTime::TIMESTAMP_FULL));  //Full Timestamp
  Serial.println("*********** Test RTC End ****************");
}

void I2C_SCANNER()
{
  Serial.println("********** I2C Scanner Start **********");
  
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
        Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan

  Serial.println("********** I2C Scanner End **********");
}


void SETUP_HX711()            //Kalibrierung je nach Seriennummer wird hier die spezifische Kalibrierung durchgeführt
{
  Serial.println("********** HX711 Setup Start **********");

  scale1.begin(HX_DAT, HX_CLK);
  scale2.begin(HX_DAT, HX_CLK);
  scale3.begin(HX_DAT, HX_CLK);
  scale4.begin(HX_DAT, HX_CLK);


#if SERIAL_NUMBER == 1                  // balu ist die erste Chinal Platine die ich getestet habe
    SET_MUX16(0);
  delay(5);
  scale1.set_scale(290.36);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(286.39);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(283.18);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(289.87);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);
 
#elif SERIAL_NUMBER == W3                // W3 ist die grüne Platine die im Feld steht V1 (Stand 15.1.)
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(295.787);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(289.434);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(298.379);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(293.201);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 2                // Platine 002 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(278.20);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(301.44);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(296.17);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(285.87);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);
   
#elif SERIAL_NUMBER == 3                // Platine 003 kalibriert am 29.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(292.80);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(280.70);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(294.88);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(285.36);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 4                // Platine 004 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(286.48);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(319.63);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(288.41);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(286.77);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);
   
#elif SERIAL_NUMBER == 5                // Platine 005 kalibriert am 29.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(300.93);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(295.65);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(300.92);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(292.86);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);
     
#elif SERIAL_NUMBER == 6              // Platine 006 kalibriert am 29.01.2024  
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(295.56);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(295.72);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(293.88);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(288.86);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 7                // Platine 007 kalibriert am 29.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(284.83);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(306.45);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(281.18);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(295.92);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 8                // Platine 00 kalibriert am 29.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(296.19);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(289.74);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(297.93);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(286.33);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);


#elif SERIAL_NUMBER == 9                // Platine 009 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(281.97);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(285.65);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(298.78);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(282.32);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 10                // Platine 010 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(296.63);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(288.13);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(293.83);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(299.76);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);
   
#elif SERIAL_NUMBER == 12                // Platine 012 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(293.27);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(289.16);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(289.25);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(294.21);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 13                // Platine 013 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(294.21);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(325.84);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(291.68);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(290.27);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

#elif SERIAL_NUMBER == 14                // Platine 014 kalibriert am 30.01.2024
  SET_MUX16(0);
  delay(5);
  scale1.set_scale(292.18);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale1.tare();
  Serial.print("Channel 1: ");
  fa = scale1.get_units(5);
  Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(295.44);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(292.18);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(289.26);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);

   #else                               // hier wird einfach eine Standardkalibrierung verwendet (immer 290)
      SET_MUX16(0);
     delay(5);
     scale1.set_scale(290);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
     scale1.tare();
     Serial.print("Channel 1: ");
     fa = scale1.get_units(5);
     Serial.println(fa,0);
  delay(5);
  
  SET_MUX16(1);
  delay(5);
  scale2.set_scale(290);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale2.tare();
  Serial.print("Channel 2: ");
  fb = scale2.get_units(5);
  Serial.println(fb,0);
  delay(5);
  
  SET_MUX16(2);
  delay(5);
  scale3.set_scale(290);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale3.tare();
  Serial.print("Channel 3: ");
  fc = scale3.get_units(5);
  Serial.println(fc,0);
  delay(5);
  
  SET_MUX16(3);
  delay(5);
  scale4.set_scale(290);              //hier muss der Wert aus der Kalibrierung eingesetzt werden
  scale4.tare();
  Serial.print("Channel 4: ");
  fd = scale4.get_units(5);
  Serial.println(fd,0);
  delay(5);
  #endif 
  
  Serial.println("********** HX711 Setup END **********");
}


void COLLECT_DATA()
{
    SET_MUX16(0); fa = scale1.get_units(10);
    SET_MUX16(1); fb = scale2.get_units(10);  
    SET_MUX16(2); fc = scale3.get_units(10); 
    SET_MUX16(3); fd = scale4.get_units(10);

    DateTime time = rtc.now();
    Serial.print(time.timestamp(DateTime::TIMESTAMP_DATE));
    Serial.print("  ");
    Serial.print(time.timestamp(DateTime::TIMESTAMP_TIME));          //hier wird das Datum geschrieben
    Serial.print("  ");
    Serial.print(fa, 0); Serial.print("  ");
    Serial.print(fb, 0); Serial.print("  ");
    Serial.print(fc, 0); Serial.print("  ");
    Serial.print(fd, 0); Serial.print("  ");
    Serial.print(fa+fb+fc+fd,0); Serial.print("  ");              //hier werden die Gewichte geschrieben
    
    Serial.print(Kipp[0],0);                                        // count TB_1
    Serial.print("  ");
    Kipp[0] = 0; 
    //Serial.print(Read_Photo1());                                    // read FD_1
    //Serial.print("  ");
    //Serial.print(Read_EC1());                                       // read EC_1
    //Serial.print("  ");
    //Serial.print(Read_Temp1(),0);                                   // read temp_1
    //Serial.print("  ");
  
    Serial.print(Kipp[1],0);                                        // count TB_2
    Serial.print("  ");
    Kipp[1] = 0;
    //Serial.print(Read_Photo2());                                    // read FD_2
    //Serial.print("  ");
    //Serial.print(Read_EC2());                                       // read EC_2
    //Serial.print("  ");
    //Serial.print(Read_Temp2(),0);                                   // read temp_2
    //Serial.print("  ");

    Serial.print(Kipp[2],0);                                        // count TB_3
    Serial.print("  ");
    Kipp[2] = 0;
    //Serial.print(Read_Photo3());                                    // read FD_3
    //Serial.print("  ");
    //Serial.print(Read_EC3());                                       // read EC_3
    //Serial.print("  ");
    //Serial.print(Read_Temp3(),0);                                   // read temp_3
    //Serial.print("  ");

    Serial.print(Kipp[3],0);                                        // count TB_4
    Serial.println("  ");
    Kipp[3] = 0;
    //Serial.print(Read_Photo4());                                    // read FD_4
    //Serial.print("  ");
    //Serial.print(Read_EC4());                                       // read EC_4
    //Serial.print("  ");
    //Serial.println(Read_Temp4(),0);                                 // read temp_4
}

void INIT_PORTS()
{
  pinMode(KIPP_1, INPUT);
  pinMode(KIPP_2, INPUT);
  pinMode(KIPP_3, INPUT);
  pinMode(KIPP_4, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(KIPP_1), ISR_KIPP_1, FALLING );
  attachInterrupt(digitalPinToInterrupt(KIPP_2), ISR_KIPP_2, FALLING );
  attachInterrupt(digitalPinToInterrupt(KIPP_3), ISR_KIPP_3, FALLING );
  attachInterrupt(digitalPinToInterrupt(KIPP_4), ISR_KIPP_4, FALLING );

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

void TEST_ANALOG(){
  Serial.println("********** Analog Test Start **********");
  for (size_t i = 0; i < 16; i++)
  {
    Serial.print("Channel ");
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(READ_ANALOG_MUX16(i));
  }
  Serial.println("********** Analog Test End **********");
}



//************** ISRs für die Kippwaagen **************
void ISR_KIPP_1()
{
  Kipp[0]++;
  Kipp_Inv_1 = true;
}
void ISR_KIPP_2()
{
  Kipp[1]++;
  Kipp_Inv_2 = true;
}
void ISR_KIPP_3()
{
  Kipp[2]++;
  Kipp_Inv_3 = true;
}
void ISR_KIPP_4()
{
  Kipp[3]++;;
  Kipp_Inv_4 = true;
}
//************** Multiplexer Definitionen **************
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

//************** das sind die Funktionen um alle Analog Ports auszulesen **************  
int Read_EC1()
{
  SET_MUX16(1);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  return tmp; 
}

int Read_EC2()
{
  SET_MUX16(2);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  return tmp; 
}

int Read_EC3()
{
  SET_MUX16(3);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  return tmp; 
}

int Read_EC4()
{
  SET_MUX16(4);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  return tmp; 
}


int Read_Photo1()
{
  analogReadResolution(12);
  SET_MUX16(5);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo2()
{
  analogReadResolution(12);
  SET_MUX16(6);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo3()
{
  analogReadResolution(12);
  SET_MUX16(7);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo4()
{
  analogReadResolution(12);
  SET_MUX16(8);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo5()
{
  analogReadResolution(12);
  SET_MUX16(0);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo6()
{
  analogReadResolution(12);
  SET_MUX16(13);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo7()
{
  analogReadResolution(12);
  SET_MUX16(14);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}

int Read_Photo8()
{
  analogReadResolution(12);
  SET_MUX16(15);
  delay(1);
  int tmp = analogRead(ANALOG_READ_PIN);
  analogReadResolution(10);
  return tmp; 
}


float Read_Temp1()
{
  SET_MUX16(9);
  delay(1);
  float tmp = map(analogRead(ANALOG_READ_PIN), 0, 544 , -40, 125);     // diese Funktion muss wahrscheinlich noch angepasst werden, irgendwo gibt es einen Spannungsabfall
  return tmp; 
}

float Read_Temp2()
{
  SET_MUX16(10);
  delay(1);
  float tmp = map(analogRead(ANALOG_READ_PIN), 0, 544 , -40, 125);
  return tmp; 
}

float Read_Temp3()
{
  SET_MUX16(11);
  delay(1);
  float tmp = map(analogRead(ANALOG_READ_PIN), 0, 544 , -40, 125);
  return tmp; 
}

float Read_Temp4()
{
  SET_MUX16(12);
  delay(1);
  float tmp = map(analogRead(ANALOG_READ_PIN), 0, 544 , -40, 125);
  return tmp; 
}
