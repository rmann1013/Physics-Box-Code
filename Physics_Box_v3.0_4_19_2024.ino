/*****************************************************************/
/*                       Physics Box Program v3.0
 *                        
 *    Past Contributors:
 *    - Kyle Diekhoff (May 2016)
 *    - Mahesh Gorantla (May 2017)
 *    - Rodolfo Leiva (Summer 2017)
 *    - Andrew Lee-Au (Spring 2024)
 *    - Alec Peng (Spring 2024)
 *    
 *               This program last updated on 4/18/2024
 *               Modifications on v2.1 by: Rodolfo Leiva
 *               Contact Email: rodolfoed94@gmail.com             
 *               Modifications on v3.0 by: Alec Peng
 *               Contact Email: peng340@purdue.edu
 *****************************************************************/

/* --------------------------------------------------------------
  |.........................INCLUDE LIBRARIES................... |
   --------------------------------------------------------------  */
#include <Wire.h>
#include <SD.h>
#include<NeoSWSerial.h>
// This is from the NeoGPS library
// Make sure NMEAGPS_cfg and GPSfix_cfg.h are configured properly
// - NMEAGPS_cfg: NMEAGPS_PARSE_RMC should be uncommented
// - GPSfix_cfg.h: GPS_FIX_DATE and GPS_FIX_TIME should be uncommented
// You can obviously uncomment more but that costs more performance
#include <NMEAGPS.h>

/* --------------------------------------------------------------
  |.............DEFINE CONSTANTS AND GLOBAL VARIABLES........... |
   --------------------------------------------------------------  */
// The ADXL345 sensor I2C address
#define ADXL345 0x53
#define NEWTONS_PER_G 9.8
#define LSB_PER_G 256 // The IMU is 10bit resolution, so 1g = 256

#define LOG_INT 200 // Period of time (milliseconds) for each sample to be logged in the SD card (Energy is exeption)
#define LCD_INT 1000 //Period of time for LCD display to update
#define RPM_INT 1000 // Period of time (milliseconds) over which RPM (Revolutions per Minute) is calculated 
#define MAGNETS 4 // number of magnets attached to axel for hall effect sensor to sense

#define CurrIn A0 // Current Sense input pin
#define VoltIn A1 // Voltage Sense input pin
#define AnalogA A2 // Optional Analog input A pin (temperature)

#define ChangeMode 2 // Button to clear energy input pin
#define HallEffect 3 // Hall Effect Sensor input pin
#define RXPIN 5 // Reads messages from GPS
#define TXPIN 6 // Transmits messages to display
#define StartStop 7 // Start/stop button
#define ChipSelect 10 // SD comunications output pin

// These two inputs have been deprecated so that the holes can be used as buttons
#define AnalogB A3 // Optional Analog input B pin
#define Digital 4 // Optional Digital input pin

// Starting NeoSWSerial for the GPS and display
NMEAGPS gps; // This parses the GPS characters
gps_fix fix; // This holds on to the latest values
NeoSWSerial softSerial(RXPIN, TXPIN);

// Button debounce constants
#define DEBOUNCE_TIME 200 // Debounce time to prevent flickering, in millis
#define DATA_MODE 0 // Shows energy used and card status, allows start/stop data logging
#define WIFI_MODE 1 // Shows WiFi name and PASS_COLON, allows downloading

// The Logging File
char logFileName[13]; // The file name will be ddhhmmss
bool loggingData = false; //Whether or not the box is in data logging mode
bool cardInitialized = false; //Whether or not an SD card is initialized

float Energy = 0; // Collective energy added up since restart

float revolutions = 0; // This is used to keep track of the no. of revolutions made
float rpm = 0; // This is used to store the RPM (Revolutions Per Minute) of the Kart
float rpm_factor = 0; // This is used to multiply the impulses computed for every 'x' ms as specified by RPM_INT

unsigned long millisValPrev = 0; // variable that captures the miliseconds that have PASS_COLONed since the program started
unsigned long millisStartLog = 0; // variable that captures when the box starts logging

// Button variables
byte mode = 0;
byte modeButtonState = HIGH; // Stores the state of the mode button (HIGH when unpressed)
byte startStopButtonState = HIGH; // Stores the state of the start/stop button (HIGH when unpressed)
unsigned long lastModePress = 0; // Stores the last time the mode button was pressed
unsigned long lastStartStopPress = 0; // Stores the last time the start/stop button was pressed

// WiFi
const char PROGMEM WIFI_NAME[] = "PhysBox";
const char PROGMEM WIFI_PASS[] = "12345678";
byte state = 0;
String fileName;

// Dynamic memory can be furthered freed by putting strings in softSerial in setup as PROGMEM
void setup() {

/* --------------------------------------------------------------
  |.............BEGIN SERIAL OUTPUT AT 9600 BAUD................ |
   --------------------------------------------------------------  */
  // This is the output to the ESP32
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

/* --------------------------------------------------------------
  |.............BEGIN SOFTWARE SERIAL AT 9600 BAUD............... |
   --------------------------------------------------------------  */
  // The RX pin receives data from the GPS
  // The TX pin writes data to the display
  //Both are 9600 Baud by default
  softSerial.begin(9600);
  //  while (!softSerial) {
  //  ; // wait for software serial port to connect
  //}

/* --------------------------------------------------------------
  |......................SET UP LCD DISPLAY..................... |
   --------------------------------------------------------------  */
  setBacklightBrightness(2); //set brightness level from 1 to 8: 1 being backlight off, 8 being brightest.
  clearLCD(); // Clear the Contents of LCD.
  displayOn(); // LCD Display On
  cursorHome(); // Set the LCD

/* --------------------------------------------------------------
  |...................Initializing program message.............. |
   --------------------------------------------------------------  */
  clearLCD();
  cursorSet(0,0);
  softSerial.print("Initializing");
  cursorSet(1,0);
  softSerial.print("Program");
  delay(1000);

/* --------------------------------------------------------------
  |.............DECLARE AND SET UP I/O PIN MODES................ |
   --------------------------------------------------------------  */
  // Setting the pin Configurations
  pinMode(VoltIn, INPUT);   // Analog Pin 0
  pinMode(CurrIn, INPUT); // Analog Pin 1
  pinMode(HallEffect, INPUT_PULLUP); // Configuring the Hall Effect Sensor Pin to the INPUT Mode
  pinMode(ChangeMode, INPUT_PULLUP); // Configuring the Clear Energy Button
  pinMode(StartStop, INPUT_PULLUP); // Configuring the Start/stop Button
  pinMode(Digital, INPUT); // Configuring Digital Pin 4 as an input
  
  // These pins will read 5V when there is no input from external interface or the no inputs are connected
  pinMode(AnalogA, INPUT); // Activating the Pull-Up Resistors for Analog Pin 2
  pinMode(AnalogB, INPUT); // Activating the Pull-Up Resistors for Analog Pin 3

  // Make sure that default chip select pin is set to output
  // even if you don't use it:
  pinMode(ChipSelect, OUTPUT);

/* --------------------------------------------------------------
  |...........SET UP HALL EFFECT SENSOR INTERRUPT............... |
   --------------------------------------------------------------  */  
  // Setting the Clear for Power Outage LED
  attachInterrupt(digitalPinToInterrupt(HallEffect), count_revolutions, FALLING);

/* --------------------------------------------------------------
  |......SET UP DATETIMECALLBACK FUNCTION FOR TIMESTAMPING...... |
   --------------------------------------------------------------  */
  SdFile::dateTimeCallback(timestamp);

/* --------------------------------------------------------------
  |....................SET UP ACCELEROMETER..................... |
   --------------------------------------------------------------  */   
  // Writing our own I2C messages instead of using the Adafruit library
  // frees up some dynamic memory, which we need
  // See https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
  Wire.begin(); // Initiate the Wire library
  
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  
  // Set ADXL345 to measuring mode
  Wire.write(0x2D); // Access to POWER_CTL Register - 0x2D
  Wire.write(8); // Write 8dec -> 0000 1000 binary (sets Bit D3 to HIGH, which enables measuring) 
  Wire.endTransmission();
  delay(10);

/* --------------------------------------------------------------
  |.................SET ACCELEROMETER'S RANGE................... |
   --------------------------------------------------------------  */
  // Setting the range of g's the accelerometer can measure
 
  // Set ADXL345 to +/-2g
  Wire.write(0x31); // Access DATA_FORMAT Register - 0x31
  Wire.write(8); // Write 0dec -> 0000 0000 binary (sets Bits D0 and D1 to LOW which is 2g resolution) 
  Wire.endTransmission();
  delay(10);

/* --------------------------------------------------------------
  |.....................DEFINE RPM FACTOR....................... |
   --------------------------------------------------------------  */
  // This is used to compute RPM of the vehicle from the count of impulses
  rpm_factor = ((1000. * 60.) / RPM_INT);

/* --------------------------------------------------------------
  |........CHECK IF SD CARD IS PRESENT AND INITIALIZED.......... |
   --------------------------------------------------------------  */
  // Checking if the SD Card is present and can be initialized:
  if (!SD.begin(ChipSelect)) {
    cardInitialized = false;
    clearLCD();
    cursorSet(0,0);
    softSerial.print("Card failed, or");
    cursorSet(1,0);
    softSerial.print("not present");
    delay(2000);
    return;
  }

  //Messaging that the SD card works and has initialized
  cardInitialized = true;
  clearLCD();
  cursorSet(0,0);
  softSerial.print("Card Initialized");
  delay(2000);
  clearLCD();
}

void loop() {
  
/* --------------------------------------------------------------
  |.......................CAPTURE TIME.......................... |
   --------------------------------------------------------------  */  
  unsigned long millisVal = millis();

/* --------------------------------------------------------------
  |........................UPDATE GPS........................... |
   --------------------------------------------------------------  */
  while (gps.available(softSerial)) {
    fix = gps.read();
  }

/* --------------------------------------------------------------
  |......................CHECK MODE BUTTON...................... |
   --------------------------------------------------------------  */  
  // If the button input used to be HIGH and now it's LOW, AND it's been long enough since the
  // last press (for debouncing purposes), trigger the function associated with the button
  
  if (((modeButtonState == HIGH) && (digitalRead(ChangeMode) == LOW))
      && ((lastModePress - millis()) > DEBOUNCE_TIME)) {
    lastModePress = millis(); // update last press time
    change_mode(); // call the function associated with the button
  }
  // Always update the button state
  modeButtonState = digitalRead(ChangeMode);



/* --------------------------------------------------------------
  |/------------------------------------------------------------\|
  ||............................................................||
  ||......COMPLETE THE FOLLOWING IF BOX IS IN DATA MODE.........||
  ||............................................................||
  |\------------------------------------------------------------/|
   -------------------------------------------------------------- */
  if (mode == DATA_MODE) {// ---------------------------------------------------------------Display Data mode switch------------------
/* --------------------------------------------------------------
//  |..................CHECK START/STOP BUTTON.................... |
//   --------------------------------------------------------------  */
    if (((startStopButtonState == HIGH) && (digitalRead(StartStop) == LOW))
        && ((lastStartStopPress - millis()) > DEBOUNCE_TIME)) {
      lastStartStopPress = millis(); // update last press time
      start_stop(); // call the function associated with the button
    }
    startStopButtonState = digitalRead(StartStop);
/* --------------------------------------------------------------
  |.....................CALCULATE RPM........................... |
   --------------------------------------------------------------  */
  // Computing the RPM for every second
  if(millisVal/RPM_INT != millisValPrev/RPM_INT)
  {
    rpm = (revolutions * rpm_factor);
    revolutions = 0;
  }

/* --------------------------------------------------------------
  |......COMPLETE FOLLOWING SERIES OF TASKS EVERY 200ms......... |
   --------------------------------------------------------------  */
    // Logging the Moving Window average of the Voltage and Current for every 200 ms.
    if (millisVal/LOG_INT != millisValPrev/LOG_INT) { //the 200ms period between samples is marked by a change in the value of millisVal/200

/* --------------------------------------------------------------
  |...CALL FUNCTION TO MEASURE VOLTAGE AND CURRENT EVERY 200ms.. |
   --------------------------------------------------------------  */    
      float volt = readVoltage();
      float current = -1 * readCurrent();

/* --------------------------------------------------------------
  |........CALCULATE POWER AND NEW ENERGY EVERY 200ms........... |
   --------------------------------------------------------------  */    
      float Power = volt * current;
      float newEnergy = Power * LOG_INT / 1000; // This always the newly computed Energy
      Energy += newEnergy / 1000; // Here we are computing cumulative energy value in kJ

/* --------------------------------------------------------------
  |..........RETRIEVE X,Y,Z ACCELERATION EVERY 200ms............ |
   --------------------------------------------------------------  */
      Wire.beginTransmission(ADXL345);
      Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
      Wire.endTransmission(false);
      Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
    
      float accelX = ( Wire.read()| Wire.read() << 8); // X-axis value
      accelX = NEWTONS_PER_G * accelX/LSB_PER_G; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
      float accelY = ( Wire.read()| Wire.read() << 8); // Y-axis value
      accelY = NEWTONS_PER_G * accelY/LSB_PER_G;
      float accelZ = ( Wire.read()| Wire.read() << 8); // Z-axis value
      accelZ = NEWTONS_PER_G * accelZ/LSB_PER_G;

/* --------------------------------------------------------------
  |......CALL FUNCTION TO MEASURE ANALOG INPUTS EVERY 200ms..... |
   --------------------------------------------------------------  */    
      float analog_input_2 = readAnalogInput2();
      float analog_input_3 = readAnalogInput3();

      boolean dig_in_4 = digitalRead(Digital);

/* --------------------------------------------------------------
  |.....CALL FUNCTION TO LOG DATA TO '.csv' FILE EVERY 200ms.... |
   --------------------------------------------------------------  */
      // Saving the Data to a LogFile.
      if (loggingData == true) {
        saveLogData(volt, current, Power, millisVal - millisStartLog, Energy + newEnergy / 1000, analog_input_2, analog_input_3, rpm, dig_in_4, accelX, accelY, accelZ);      
      }

/* --------------------------------------------------------------
  |......CALL FUNCTION TO DISPLAY INFO ON LCD EVERY SECOND...... |
   --------------------------------------------------------------  */ 
      if (millisVal/LCD_INT != millisValPrev/LCD_INT){
        LCD_Display_Data();
      }
    }
  }



/* --------------------------------------------------------------
  |/------------------------------------------------------------\|
  ||............................................................||
  ||......COMPLETE THE FOLLOWING IF BOX IS IN WIFI MODE.........||
  ||............................................................||
  |\------------------------------------------------------------/|
   -------------------------------------------------------------- */
  else { // mode == WIFI_MODE
    
     if (Serial.available() > 0) {
       char receivedChar = Serial.read();
       //Serial.print(state);
       switch(state){
         case 0: // Idle and Send List
           //Serial.print(receivedChar);
           if(receivedChar == 'L'){ // Send File List
             //fetchFiles();
             listFile();
             Serial.write('\n');
             flushSerial();
           }
           if(receivedChar == 'F'){ // File requested
             state = 1;
           }
           break;
         case 1: // Get File name
           if(receivedChar != '\n'){
             fileName += receivedChar;
           }
           else { // Send File
             sendFile();
             Serial.print('\0'); // To signify end of file
             state = 0;
             fileName = "";
           }
           break;
       }
     }

/* --------------------------------------------------------------
  |......CALL FUNCTION TO DISPLAY INFO ON LCD EVERY SECOND...... |
   --------------------------------------------------------------  */ 
    if (millisVal/LCD_INT != millisValPrev/LCD_INT){
      LCD_Display_WiFi();
    }
  }
  millisValPrev = millisVal; //store time capture so it can be compared with next time capture
}

/* --------------------------------------------------------------
  |...............INTERRUPT FUNCTIONS START HERE................ |
   --------------------------------------------------------------  */

// This keeps count of the no. of revolutions for every 'x' ms as specified by LOG_INT 
void count_revolutions() {
  revolutions += 1.0 / MAGNETS;  
}

/* --------------------------------------------------------------
  |................BUTTON FUNCTIONS START HERE................. |
   --------------------------------------------------------------  */
// This cycles between Data and WiFi mode
void change_mode() {
  // Only change modes if not logging data
  // Do NOT change modes if we are in data mode and currently logging data
  if (mode == 0 && !loggingData)
  {
    mode = 1;

  } else {
    mode = 0;
  }
}

// Starts or stops logging data when button is pressed
// Offload literal strings to PROGMEM (our version of F() for softSerial
const PROGMEM char CARD_FAILED_MESSAGE[]  = "Card failed, or";
const PROGMEM char NOT_PRESENT_MESSAGE[] = "not present";
const PROGMEM char STOPPING_DATA_MESSAGE[] = "Stopping data";
const PROGMEM char LOGGING_MESSAGE[] = "logging...";
const PROGMEM char LOGGING_TO_MESSAGE[] = "Logging to: ";
void start_stop() {  
  // If the card is not initialized, warn the user and return
  if (cardInitialized == false) {
    clearLCD();
    cursorSet(0,0);
    LCD_Write_From_PROGMEM(CARD_FAILED_MESSAGE);
    cursorSet(1,0);
    LCD_Write_From_PROGMEM(NOT_PRESENT_MESSAGE);
    
    smartDelay(1000);
    return;
  }
  
  // Otherwise, if we're already logging data, stop logging
  if (loggingData == true) {
    //messaging name of file the data is being logged to in sd card
    clearLCD();
    cursorSet(0,0);
    LCD_Write_From_PROGMEM(STOPPING_DATA_MESSAGE);
    cursorSet(1,0);
    LCD_Write_From_PROGMEM(LOGGING_MESSAGE);
    
    smartDelay(1000);
    
    loggingData = false;
  // Otherwise, create a new file and start logging
  } else {
    //messaging name of file the data is being logged to in sd card
    newFile();
    
    clearLCD();
    cursorSet(0,0);
    LCD_Write_From_PROGMEM(LOGGING_TO_MESSAGE);
    cursorSet(1,0);
    softSerial.print(logFileName);
    smartDelay(1000);
    
    loggingData = true;
    Energy = 0; // Clearing the Cumulative Energy
    millisStartLog = millis();
  }
}

/* --------------------------------------------------------------
  |................REGULAR FUNCTIONS START HERE................. |
   --------------------------------------------------------------  */
// This function acts like a normal delay, but rather than stopping everything it
// still allows GPS signals to be read during the delay
// It also doesn't use interrupts (the NeoGPS library doesn't seem to like interrupts)
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gps.available(softSerial)) {
      fix = gps.read();
    }
  } while (millis() - start < ms);
}

// Always gives a value between -320 and 320 Amperes with 0.5V => -320A and 4.5V => 320A
float readCurrent() {
  long Current = analogRead(CurrIn); // Always returns a value between 0 and 1023
  float CurrentVal = Current * 5.0 / 1023.0 ; // Converting Digital Values (0 to 1023) to Analog Voltages
  CurrentVal = (CurrentVal - 2.5) * 100 / 0.625; // convert analog voltage value to current in Amps
                                  
  return CurrentVal;
}

// Always returns a value between 0.00 and 100.00 Volts
float readVoltage() {
  return (analogRead(VoltIn) * 100.0 / 1023.0) ;
}

// Always returns a value between 0.00 and 5.00 Volts
float readAnalogInput2() {
  return (analogRead(A2) * 5.0 / 1023.0);
}

// Always returns a value between 0.00 and 5.00 Volts
float readAnalogInput3() {
  return (analogRead(A3) * 5.0 / 1023.0);
}

// Returns a timestamp for dateTimeCallback
// This makes it so that Windows Explorer can read the last time a file is modified
void timestamp(uint16_t* date, uint16_t* time) {
  int year;
  byte month, day, hour, minute, second;
  
  // If date/time is valid, use date/time data
  // If date/time is invalid, replace data with placeholder (1/1/2000)
  if (fix.valid.date && fix.valid.time) {
    year = fix.dateTime.full_year();
    month = fix.dateTime.month;
    day = fix.dateTime.date;
    hour = fix.dateTime.hours;
    minute = fix.dateTime.minutes;
    second = fix.dateTime.seconds;
  } else {
    year = 2000;
    month = 1;
    day = 1;
    hour = 1;
    minute = 0;
    second = 0;
  }
  
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year, month, day);

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour, minute, second);
}

// Creates a new data logging file every time data logging is started
// Returns the name of the file
void newFile() {
  // Generate a filename based on date and time
  
  // This below piece of code inserts the Time Stamp for new Logging every time when Power is RESET.
  int year;
  byte month, day, hour, minute, second;

  // If valid, write the time as the filename
  if (fix.valid.date && fix.valid.time) {
    year = fix.dateTime.year;
    month = fix.dateTime.month;
    day = fix.dateTime.date;
    hour = fix.dateTime.hours;
    minute = fix.dateTime.minutes;
    second = fix.dateTime.seconds;
    
    sprintf(logFileName, "%02d%02d%02d%02d.csv", day, hour, minute, second); // There's an 8 char limit on filenames
  
  // Otherwise, write "XXXXXXXX" as the file name
  } else {
    strcpy(logFileName, "XXXXXXXX.csv");
  }

  File logFile = SD.open(logFileName, FILE_WRITE);

  // If the time is valid, add a timestamp into the file
  if (fix.valid.date && fix.valid.time) {
    logFile.print(year, DEC);
    logFile.print('-');
    logFile.print(month, DEC);
    logFile.print('-');
    logFile.print(day, DEC);
    logFile.print(' ');
    logFile.print(hour, DEC);
    logFile.print(':');
    logFile.print(minute, DEC);
    logFile.print(':');
    logFile.println(second, DEC);
  }

  // Print header columns
  // Split the print statement into two parts because there's a character limit
  logFile.print(F("millis,Volt,Current,Power(kW),Energy(kJ),Analog Input 2,Analog Input 3,"));
  logFile.println(F("RPM,X-Accel(m/s^2),Y-Accel(m/s^2),Z-Accel(m/s^2),DIGIN_4"));
  logFile.close();
}

// stores all relevant data in sd card
void saveLogData(float Volt, float Current, float Power, unsigned long millisVal, float Energy, float AI2, float AI3, float rpm, boolean D, float accelX, float accelY, float accelZ) {
  // This opens a new file for writing if it doesn't exist
  // Else it will append the data to the end of the file
  File logFile = SD.open(logFileName, FILE_WRITE);

  // Logging the PhotoVolt data to the SD Card
  logFile.print(millisVal);
  logFile.print(",");

  logFile.print(Volt, 1);
  logFile.print(",");

  logFile.print(Current, 1);
  logFile.print(",");

  logFile.print(Power / 1000, 1); // Logging the Power in kW
  logFile.print(",");

  // Logging the Energy every 2 Seconds in the LogFile
  if (millisVal / LOG_INT != millisValPrev / LOG_INT) {
    logFile.print(Energy, 0); // 2s Interval and Logging the Energy in kJ
    logFile.print(",");
  }
  else{    
    logFile.print(F(" ,"));
  }

  // Logging the Analog Input 2
  logFile.print(AI2);
  logFile.print(",");
  
  // Logging the Analog Input 3
  logFile.print(AI3);
  logFile.print(",");
  
  // Logging the RPM
  logFile.print(rpm);
  logFile.print(",");
  
  // X-Axis g-Force (Acceleration)
  logFile.print(accelX);
  logFile.print(",");

  // Y-Axis g-Force (Acceleration)
  logFile.print(accelY);
  logFile.print(",");

  // Z-Axis g-Force (Acceleration)
  logFile.print(accelZ);
  logFile.print(",");
  
  // Digital Input 4 
  logFile.print(D);
  logFile.print(",");
    
  logFile.println();
  logFile.close();
}

/* --------------------------------------------------------------
  |................DISPLAY FUNCTIONS START HERE................. |
   --------------------------------------------------------------  */
// Writes a message from PROGMEM onto the display
void LCD_Write_From_PROGMEM(char message[]) {
  int numChars;
  numChars = strlen_P(message);

  char character;
  for (byte i = 0; i < numChars; i++) {
    character = pgm_read_byte_near(message + i);
    softSerial.print(character);
  }
}

// Displays the WiFi name and PASS_COLONword
const PROGMEM char WIFI_MESSAGE[] = "WiFi:";
const PROGMEM char PASS_MESSAGE[] = "Pass:";
void LCD_Display_WiFi() {
  clearLCD();
  cursorSet(0,0);
  LCD_Write_From_PROGMEM(WIFI_MESSAGE);
  LCD_Write_From_PROGMEM(WIFI_NAME);
  cursorSet(1,0);
  LCD_Write_From_PROGMEM(PASS_MESSAGE);
  LCD_Write_From_PROGMEM(WIFI_PASS);
}

// Displays the data/"trip summary" page
const PROGMEM char ENERGY_MESSAGE[] = "E(kJ): ";
const PROGMEM char NO_CARD_MESSAGE[] = "No card loaded";
const PROGMEM char CARD_STANDBY_MESSAGE[] = "Card on standby";
const PROGMEM char FILE_MESSAGE[] = "File: ";

void LCD_Display_Data() {
  clearLCD();
  cursorSet(0,0);
  LCD_Write_From_PROGMEM(ENERGY_MESSAGE);
  softSerial.print(Energy, 2);
  cursorSet(1,0);
  // If there is no card, display it
  if (cardInitialized == false) {
    LCD_Write_From_PROGMEM(NO_CARD_MESSAGE);
  // If there is a card but data is not being logged
  } else if (loggingData == false) {
    LCD_Write_From_PROGMEM(CARD_STANDBY_MESSAGE);
  } else {
    LCD_Write_From_PROGMEM(FILE_MESSAGE);
    // Only print first 8 char
    for(int i = 0; i < 8; i++) {
      softSerial.print(logFileName[i]);
    }
  }
}

//  LCD  FUNCTIONS-- keep the ones you need.
// More Details at http://www.newhavendisplay.com/specs/NHD-0216K3Z-FL-GBW-V3.pdf on Page [7]
void clearLCD() {
  softSerial.write(254); // Prefix: 0xFE => 254
  softSerial.write(81); // 0x51 => 81
}

void displayOn() {
  softSerial.write(254); // Prefix 0xFE
  softSerial.write(65);  // 0x41
}

void displayOff() {
  softSerial.write(254); // Prefix 0xFE
  softSerial.write(66); // 0x42
}


// start a new line
void newLine() {
  cursorSet(1, 0); // 1st Row and Column 0.
}

// move the cursor to the home position
void cursorHome() {
  softSerial.write(254);
  softSerial.write(70); // Set the Cursor to (0,0) position on the LCD
}

//set LCD backlight brightness level with number between 1 and 8: 8 being brightest, 1 being backlight off.
//If going to change brightness, for some reason requires to set brightness to 8 first and then reset it to whatever brightness you desire.
void setBacklightBrightness(int brightness) {
   softSerial.write(0xFE); //command prefix
   softSerial.write(0x53); //command to set backlight
   softSerial.write(8);    //preset brightness to 8
 
   softSerial.write(0xFE);
   softSerial.write(0x53);
   softSerial.write(brightness); //set brightness to desired level

}

// move the cursor to a specific place
// Here is xpos is Column No. and ypos is Row No.
void cursorSet(int ypos, int xpos) {
  softSerial.write(254);
  softSerial.write(69);

  // Bounding the X-Position
  if (xpos >= 15) {
    xpos = 15;
  }
  else if (xpos <= 0) {
    xpos = 0;
  }

  // Bounding the Y-Position
  if (ypos >= 1) {
    ypos = 1;
  }
  else if (ypos <= 0) {
    ypos = 0;
  }

  int finalPosition = ypos * 64 + xpos;

  softSerial.write(finalPosition);
}

// move cursor left by one Space
void cursorLeft() {
  softSerial.write(254);
  softSerial.write(73);  // 0x49
}

// move cursor right by One Space
void cursorRight() {
  softSerial.write(254);
  softSerial.write(74); // 0x4A
}

/* --------------------------------------------------------------
  |..................WIFI FUNCTIONS START HERE.................. |
   --------------------------------------------------------------  */
void sendFile(){
  if(SD.exists(fileName)){
    File dataFile = SD.open(fileName);
    while(dataFile.available()){
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  else{
//    Serial.print("Files doesn't exist"); // Show in display
//    dataFile.close();
  }
}

void listFile(){
    File root = SD.open("/");
    if(!root){
      Serial.println(F("no root"));
    }
    while (true) {
      File entry = root.openNextFile();
    if (!entry) {
      break;
    }
    fileName = String(entry.name());
    if (entry.isDirectory()) { // If it's a directory, skip
    } else { // File name
      if(fileName.indexOf(".CSV") >= 0){ // Prints link to just CSV files
          Serial.print(F("<li>"));
          Serial.print(F("<a href=\""));
          Serial.print(fileName); // Link destination
          Serial.print(F("\">"));
          Serial.print(fileName);
          Serial.print(F("</a>"));
          Serial.print(F("</li>"));
      }
    }
    entry.close();
  }
  root.close();
}

void flushSerial(){
  while(Serial.available() > 0) {
    Serial.read();
  }
}
