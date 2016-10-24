/* --------------------------------------------------------------------------
   Welcome to SSI's starter code for Teensy <--> Kythera Communication

   This starter code defines a set of functions you can use to communicate
   with Kythera. You just need to decide what to do with the functions!
   We'll walk you through the entire file. Each step is labeled from "1!"
   to "27!".

   You should only have to modify 2!,4!,6!,11!, and 27!. All other modifications
   you make will be done from scratch
*/

/* 1! - Importing Communication/Logging libraries
   --------------------------------------------------------------------------
   First order of business is to import all the functions you need to
   use the CAN bus and talk to the Pi. Don't delete these!
*/
#include <Metro.h>
#include <FlexCAN.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/* 2! - Importing other Libraries
   --------------------------------------------------------------------------
   If you have any other libraries to import, do so here:
*/


/* 3! - Setting Necessary Constants
   --------------------------------------------------------------------------
   Here are some Constants you shouldn't change, redefine, or otherwise
   modify.
*/
#define PISKY_SERVER_ID 0x202
#define PISKY_CLIENT_ID 0x102
#define DATA_ARRAY_SIZE_BYTES 6
#define MESSAGE_ARRAY_SIZE_BYTES 102
#define DATA_ARRAY_SIZE_BITS 48
#define MESSAGE_ARRAY_SIZE_BITS 816
#define CAN_SPEED 500000

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
#define chipSelect 10
#define ledPin 13

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)

/* 4! - Setting Additional Constants
   --------------------------------------------------------------------------
   Here is space for you to define any other constants
*/


/* 5! - Setting Necessary Global variables
   --------------------------------------------------------------------------
   Here are some globals needed by the base implementation. Don't change these!
*/
FlexCAN   CANbus(CAN_SPEED);
static    CAN_message_t dataIn, messageOut;
int       loops = 0; //number of loops made
File      dataFile;
File      errorLog;
bool      logging = false; //are we logging to files?
bool      healthy = true;  // overall health bit
bool      recieved = false; // did we get a full message?

int txCount, rxCount;
unsigned int txTimer, rxTimer;

/* the mask that defines a message from the master */
CAN_filter_t clientMask;
// clientMask.rtr = 0;
//clientMask.ext = 0;
//clientMask.id = PISKY_CLIENT_ID;
//decltype struct CAN_filter_t clientMask{
//
// };

/* the id of the server */
uint32_t serverID = PISKY_SERVER_ID;

/* the predefined structure of all messages from the server */
typedef struct data
{
  //float heading;        /* current vehicle heading */
  uint8_t rollRate_major; /* 00.-- first 2 digits of current vehicle roll in RADIANS per second*/
  uint8_t rollRate_minor; /* --.00 last 2 digits of current vehicle roll in RADIANS per second*/
  //float pitch;          /* current vehicle pitch */
  //float accelx;         /* current x-axis vehicle acceleration */
  //float accely;         /* current y-axis vehicle acceleration */
  //float accelz;         /* current z-axis vehicle acceleration */
  uint8_t altitude_major; /* 000--- first 3 digits of current altitude -- FEET */
  uint8_t altitude_minor; /* ---000 last 3 digits of current altitude -- FEET */
  uint8_t temp;           /* current interior pressure -- CELCIUS */

  /* current state of flight (defined from 0 to 256)
     0:   startup
     5:   idle-prelaunch (all nodes reporting prelaunch status bit)
     10:  motor-ignition (vehicle z-acceleration exceedes 2g's)
     15:  burnout (vehicle velocity begins to decrease)
     20:  coast   (reported until vehicle dips below 10 meters per second)
     25:  slow-coast (<10 meters per second)
     30:  near-apogee (<5 meters per second)
     35:  apogee (<1 meter per second)
     40:  early-descent (<5 meters per second)
     45:  programmable-recovery-event-1 (user specified)
     50:  programmable-recovery-event-2 (user specified)
     55:  slow-descent  (<6 meters per second)
     60:  landing (<0.5 meters per second)
  */
  uint8_t   flight_status;
  //uint32_t  flight_time;    /* miliseconds elapsed since launch */
} data;

struct data currentData;

typedef struct message
{
  uint8_t   new_status;    /* user specified node status */
  bool      health;        /* user specified health bit */ /* monitoring stepper connection */
  char      downlink[100]; /* user specified message to downlink */
} message;

struct message currentMessage;

/* 6! - Setting Additional Global variables
   --------------------------------------------------------------------------
   Here is space for you to set any other global variables
*/

/* 7! - The error function
   --------------------------------------------------------------------------
   Use this function to print out errors to the errorlog and serial. Do
   not change the implementation of this function.
*/
void error(char const* str)
{
  Serial.print(millis()); Serial.print(" - Error: "); Serial.println(str);

  if (logging) {
    errorLog.print(millis()); errorLog.print(" - Error: "); dataFile.println(str);
    dataFile.flush();
    errorLog.flush();
    delay(50);
  }

  /* set health bit to false */
  healthy = false;
}

/* 8! - Printing the current time
   --------------------------------------------------------------------------
   Use this function to print out the current time since startup to the
   datafile. Do not change the implementation of this function.
*/
void printTime()
{
  unsigned long val = millis() / 1000;
  int hours = numberOfHours(val);
  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

  dataFile.println();
  dataFile.print(hours); dataFile.print(':');
  dataFile.print(minutes); dataFile.print(':'); dataFile.print(seconds);
  dataFile.print("    ");
  dataFile.print('(');
  dataFile.print(val * 1000);
  dataFile.print("ms)   (loop#: "); dataFile.print(loops);
  dataFile.print(')');
  dataFile.println();
}

/* 9! - SD card setup function
   --------------------------------------------------------------------------
   This function connects to the SD card. You don't need to use this!
*/
void initSDCard()
{
  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(10)) { //, 11, 12, 13)) {
    /*cannot use the error function here since dataFile is not initialized */
    Serial.println("Card failed, or not present");
  } else {
    Serial.println("card initialized.");
    logging = true;
  }
}

/* 10! - File setup function
   --------------------------------------------------------------------------
   Use this function to open an error log and datafile on the SD card. Don't
   change the implementation!
*/
void initFiles()
{
  /* Initialize the SD card*/
  initSDCard();

  /* Open log file*/
  if (logging) {
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    errorLog = SD.open("errorlog.txt", FILE_WRITE);
    if (! dataFile) {
      Serial.println("error: could not open datalog.txt");
      logging = false;
    }
    else if (! errorLog) {
      Serial.println("error: could not open errorlog.txt");
      logging = false;
    }
    else {
      dataFile.println("\n");
      dataFile.println("——————————————————————————————————————————————");
      dataFile.println("-------------Starting New Session-------------");
      printTime(); /* indicate start of new session */
      dataFile.println("——————————————————————————————————————————————");
    }
  }
}

/* 11! - Initialize any local sensors
   --------------------------------------------------------------------------
   Generic init functions is called to initialize anything needed by
   user sensors.
*/
void initMore()
{
  Serial.print(F("Initializing sensors... "));
  if (logging) dataFile.print("Initializing sensors... ");

  ///////////////////////////////////////////////////////
  //////                                           //////
  //////   Define any other setup functions here   //////
  //////                                           //////
  ///////////////////////////////////////////////////////

  Serial.println("COMPLETE");
  if (logging) dataFile.println("COMPLETE");
  healthy = true;
}



/* 12! - Setup
   --------------------------------------------------------------------------
   This is the unified setup function. You can add what you need where shown
*/
void setup(void)
{

  clientMask.rtr = 0;
  clientMask.ext = 0;
  clientMask.id = PISKY_CLIENT_ID;
  CANbus.setFilter(clientMask, 7);

  CANbus.begin();
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 1);
  delay(1000);
  initFiles();
  // Add additional functions to initMore()
  initMore();

}


/* 13! - Get Message from Server
   --------------------------------------------------------------------------
   This is the unified get message function. Do not modify the implementation
   of this method.
*/
void updateData()
{
  uint8_t reading[DATA_ARRAY_SIZE_BITS];
  //struct data reading;
  int index = 0;
  while ( CANbus.read(dataIn) ) {
    for (int i = 0; i < dataIn.len; i++) {
      reading[index] = dataIn.buf[i];
      index++;
    }
    rxCount++;
  }

  if (index++ != DATA_ARRAY_SIZE_BITS) {
    String ind = "full data message not recieved...only recieved "+ String(index)+ " bits";
    error(ind.c_str());
    recieved = false;
    return;
  } 
    
  recieved = true;
  
  currentData.rollRate_major = reading[0];
  currentData.rollRate_minor = reading[1];
  currentData.altitude_major = reading[2];
  currentData.temp = reading[3];
  currentData.flight_status = reading[2];
}

/* 14! - getHeading()
   --------------------------------------------------------------------------
   returns current heading or -1 if the node did not recieve a full message
*/
float getHeading()
{
  //if (recieved) return currentData.heading;
  return -1;
}

/* 15! - getRoll()
   --------------------------------------------------------------------------
   returns current roll or -1 if the node did not recieve a full message
*/
float getRollRate()
{
  if (recieved){
    String make =  String(currentData.rollRate_major) + "." + String(currentData.rollRate_major); 
    return make.toFloat();
  }
  return -1;
}


float getAltitude()
{
  if (recieved){
    String make =  String(currentData.altitude_major) + "." + String(currentData.altitude_major); 
    return make.toInt();
  }
  return -1;
}


/* 16! - getPitch()
   --------------------------------------------------------------------------
   returns current pitch or -1 if the node did not recieve a full message
*/
float getPitch()
{
  //if (recieved) return currentData.pitch;
  return -1;
}

/* 17! - getAccelX()
   --------------------------------------------------------------------------
   returns current acceleration in the x direction or -1 if the node did not
   recieve a full message
*/
float getAccelX()
{
  //if (recieved) return currentData.accelx;
  return -1;
}

/* 18! - getAccelY()
   --------------------------------------------------------------------------
   returns current acceleration in the y direction or -1 if the node did not
   recieve a full message
*/
float getAccelY()
{
  //if (recieved) return currentData.accely;
  return -1;
}

/* 19! - getAccelZ()
   --------------------------------------------------------------------------
   returns current acceleration in the z direction or -1 if the node did not
   recieve a full message
*/
float getAccelZ()
{
  //if (recieved) return currentData.accelz;
  return -1;
}

/* 20! - getPressure()
   --------------------------------------------------------------------------
   returns current pressure or -1 if the node did not
   recieve a full message
*/
float getPressure()
{
  //if (recieved) return currentData.pressure;
  return -1;
}

/* 21! - getTemp()
   --------------------------------------------------------------------------
   returns current temperature or -1 if the node did not
   recieve a full message
*/
float getTemp()
{
  if (recieved) return currentData.temp;
  return -1;
}

/* 22! - getStatus()
   --------------------------------------------------------------------------
   returns current flight status or -1 if the node did not
   recieve a full message
*/
uint8_t getStatus()
{
  if (recieved) return currentData.flight_status;
  return -1;
}

///* 23! - getStatus()
//   --------------------------------------------------------------------------
//   returns current flight status or -1 if the node did not
//   recieve a full message
//*/
//uint8_t getStatus()
//{
//  if (recieved) return currentData.flight_status;
//  return -1;
//}

/* 24! - getFlightTime()
   --------------------------------------------------------------------------
   returns current time since launch or -1 if the node did not
   recieve a full message
*/
uint16_t getFlightTime()
{
  //if (recieved) return currentData.flight_time;
  return -1;
}

/* 25! - Update Message to send to Server
   --------------------------------------------------------------------------
   This is the unified change message function. Do not modify the implementation
   of this method.
*/
void updateMessage(int new_status, String new_message)
{
  new_message.toCharArray(currentMessage.downlink, 100);
  currentMessage.health = healthy;
  currentMessage.new_status = new_status;
}

/* 26! - send Message to Server
   --------------------------------------------------------------------------
   This is the unified send message function. Do not modify the implementation
   of this method.
*/
/*void sendMessage(int new_status, String new_message)
{
  messageOut.id = serverID;
  messageOut.len = 8;

  uint8_t sending[MESSAGE_ARRAY_SIZE_BITS];
  sending = currentMessage;
  int index = 0;
  txCount = MESSAGE_ARRAY_SIZE_BYTES;

  digitalWrite(ledPin, 1);
  while (txCount--) {
    for ( int idx = 0; idx < 8; ++idx ) {
      messageOut.buf[idx] = sending[index];
      index++;
    }
    CANbus.write(messageOut);
  }
  digitalWrite(ledPin, 0);
  delay(5);
}*/

/* 27! - Loop
   --------------------------------------------------------------------------
   This is the unified loop function. You can add what you need where shown
*/
void loop(void)
{
  ///////////////////////////////////////////////////////
  //////                Do not modify              //////
  void updateData();                               //////
  delay(100);                                      //////
  void sendMessage();                              //////
  //////                                           //////
  ///////////////////////////////////////////////////////

  /* YOUR CODE AFTER HERE */


}

/* DEFINE YOUR FUNCTIONS AFTER HERE */
