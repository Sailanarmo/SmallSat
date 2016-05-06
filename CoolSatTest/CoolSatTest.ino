#include <uCamII.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include "Adafruit_MCP9808.h"
#include "I2Cdev.h"
#include <CoolSatBaro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ADDRESS 0x76 //defining the address for the Barometor?

#define BNO055_SAMPLERATE_DELAY_MS (100) //Delay for the Gyro


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE //Not sure why this is needed
  #include "Wire.h"
#endif

//=========== Creating Instances of our sensors ==========//
UCAMII camera;  //instance for the camera
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808(); //instance for the outer temperature sensor
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808(); //instance for the inner temperature sensor
Adafruit_BNO055 bno = Adafruit_BNO055(); //instance for the Gyro
CoolSatBaro myBaro; //instance for the barometer 
//========================================================//

//#define OUTPUT_READABLE_ACCELGYRO

//================ Text Files ==================//
File myGyro;
File myAccel;
File myFile;
File myTemp;
File myTemp2;
File myLight;
File myBarotxt;
//=============================================//

tmElements_t tm; //in the time library somewhere this is called.

//===================== Global Variables =======================//
const int wireCutter = 29; //pin for our wire cutter
const long INTERVAL = 995;  //time for 1 second
const long STAMP = 299995;  //time for 5 minutes
unsigned long previousMillis = 0; //reset the time 
unsigned long previousStamp = 0;  //reset the time

byte serialReadCommand; //needed so the Arduino can read commands sent through the Radio.

bool commandSent = false;
bool commandSent2 = false;
bool commandSent3 = false;

int16_t ax, ay, az; //I don't think we need these anymore...
int16_t gx, gy, gz;

int analogPin = 15; //Light sensor pin, should change the name
int batteryPin = 14;  //Battery reading pin

const int chipSelect = 22; //not sure yet

float val = 0.0;  //Voltage
float RLDR = 0.0; //Resistance in Ohms
float Lux = 0.0;  //Lux reading
float battery = 0.0;  //Battery Voltage
//==============================================================//

void setup() {
  //Serial.begin(9600);
  Serial3.begin(9600);  //Initiating the Serial port to push out the the Radio
  Wire.begin(); //Begining everying on our I2C Bus

  myBaro.initial(ADDRESS);  //Passing the address to initialize the Barometer
  // Disable internal pullups, 10Kohms are on the breakout
  PORTC |= (1 << 4);
  PORTC |= (1 << 5);

  pinMode(wireCutter, OUTPUT);  //Initializing our wirecutter
  
  //Serial.print("Initializing SD card..."); // Initiallizes SD Card in Port 22.
  Serial3.print("Initializing SD card...");

  if (!SD.begin(22)) 
  {
    //Serial.println("initialization failed!"); //if initialization of SD Card fails, should stop here.
    Serial3.println("initialization failed!");
    return;
  }
  //Serial.println("initialization done."); //Done initializing the SD Card.
  Serial3.println("initialization done.");
  //Serial.println("");
  Serial3.println("");
  
  //initializing our temperature sensors
if (!tempsensor.begin()) {
    //Serial.println("Couldn't find MCP9808!");
    while (1);
  }
if (!tempsensor2.begin2()) {
    //Serial.println("Couldn't find MCP9808!");
    while (1);
  }

//initializing the Barometer
if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
  
}


void loop() {
  unsigned long currentMillis = millis(); //millis() is a function for the arduino
  unsigned long timeMillis = millis();
  int bytes = 0;
  bool cameraInit = false;  //we have the camera not initialized so it will only respond once we send it a command
  bool fiveMinute = false;  //Probably have no use for this anymore as we are only reading 1 second of data each time now.


  runGyroSensor();  //Runs the Gyro in the "background" it's not really "tasking" but it runs whenever the other sensors are not running.

  //Enabling the arduino to read whatever commands is sent through the Radio.
  if(Serial3.available()){
    serialReadCommand = Serial3.read();
    Serial3.write(serialReadCommand);

  //If 1 is pressed, taked a picture
  if (serialReadCommand == 49 && !commandSent){
    commandSent = true;
    Serial3.println("Attempting to take Picture...");
    }
  //if 2 is pressed, activate the Wire Cutters
  if (serialReadCommand == 50 && !commandSent2){
    commandSent2 = true;
    Serial3.println("Activating Wire Cutters...");  
    }
  //If 3 is pressed, send us a text file of our data.
  if (serialReadCommand == 51 && !commandSent3){
    commandSent3 = true;
    Serial3.println("Sending you a text file, standby...");  
    }
  }
  //If you hit 1 second, purge yourself and then do the following. 
  if (currentMillis - previousMillis >= INTERVAL){
    previousMillis = currentMillis;

    fiveMinute = false;

    myBaro.readBaro(); //Give me a reading of the pressure
    
    myBarotxt = SD.open("Baro.txt", FILE_WRITE);
    myLight = SD.open("Light.txt", FILE_WRITE);

    //=============== Math ==================//
    val = analogRead(analogPin); //Read the value of the pin
    val = val * .0048; //Convert to Voltage
    battery = analogRead(batteryPin); //Read the value of the pin
    battery = (battery * .00475) * 2; //Convert to voltage while taking the resitor into the equation.

    RLDR = (1000.0 * (5 - val))/val; //convert to Resistance in Ohms
    Lux = (776897.0 * (pow(RLDR, -1.206))); //Convert to Lux
    //=======================================//

    //Print to the Radio and write to the SD Card
    Serial3.print(Lux);
    Serial3.print(" Lux. ");
    Serial3.print(RLDR);
    Serial3.print(" Resistance Ohms. ");
    Serial3.print(val);
    Serial3.print(" Volts."); 
    myLight.print(Lux);
    myLight.print(" Lux. ");
    myLight.print(RLDR);
    myLight.print(" Resistance Ohms. ");
    myLight.print(val);
    myLight.println(" Volts.");
    timeStamp(myLight);
    myLight.close();

    Serial3.print(battery);
    Serial3.println(" Battery Voltage.");

    tempSens1(fiveMinute);
    tempSens2(fiveMinute);

    Serial3.print("Barometor Temp: ");
    Serial3.println(myBaro.getTemp());
    Serial3.print("Actual Pressure: ");
    Serial3.print(myBaro.getPressure());
    Serial3.println("mb.");
    Serial3.print("Corrected Pressure: ");
    Serial3.print(myBaro.getCorrectedPressure());
    Serial3.println("mb.");
    Serial3.print("Altitude: ");
    Serial3.println(myBaro.getAltitude());
    myBarotxt.print("Barometor Temp: ");
    myBarotxt.println(myBaro.getTemp());
    myBarotxt.print("Actual Pressure: ");
    myBarotxt.print(myBaro.getPressure());
    myBarotxt.println("mb.");
    myBarotxt.print("Corrected Pressure: ");
    myBarotxt.print(myBaro.getCorrectedPressure());
    myBarotxt.println("mb.");
    myBarotxt.print("Altitude: ");
    myBarotxt.println(myBaro.getAltitude());
    myBarotxt.close();


    /*TODO: 
    Somewhere, either here, or in the if statements
    We need to have coded an ability to cheack the altitude probably something like
    if(myBaro.getAltitude == 60000){ activate wire cutters } but we need to have something
    small scale so we can actually test it first, so something around the altitude of our mountains so I
    can take a quick hike and test the wire cutter. 
    
    It also needs to take a picture and this is the most CRUCIAL part of the whole mission, no picture = mission fail
    Again, the code we have is very inefecient, and there has to be a better more reliable way of taking a picture.
    We are currently looking into other camera options, but for now, we are just rolling with this camera. 
    
    After it takes a picture, it needs to blast a text file through the radio (see void sendText). I think I still want to keep this on
    request, but I honestly don't know if we'll be able to send a text file on command or if it needs to constantly blast it out 
    every 5 minutes or so. We still need to keep track of data, and if we are constantly blasting out the text file then the sensors 
    will stop taking data. So maybe every 5 minutes AFTER it has taken a photo, it will send us a text file.
    
    That is all I can think of for now, let me know if we need to do anything else.
    
    */

  //Checking to see if a command has been sent through the radio.
  if (commandSent){
    runCamera();
    commandSent = false;
      }
  if (commandSent2){
    cutWire();
    commandSent2 = false;
    }
  if (commandSent3){
    sendText();
    commandSent3 = false;      
    }
  }
//  if (timeMillis - previousStamp >= STAMP){
//    previousStamp = timeMillis;
//
//    fiveMinute = true;
//
//    myLight = SD.open("Light.txt", FILE_WRITE);
//    val = analogRead(analogPin);
//    val = val * .0049; 
//
//    RLDR = (1000.0 * (5 - val))/val;
//    Lux = (776897.0 * (pow(RLDR, -1.206)));
//    
//    Serial3.print(Lux);
//    Serial3.print(" Lux. ");
//    Serial3.print(RLDR);
//    Serial3.print(" Resistance Ohms. ");
//    Serial3.print(val);
//    Serial3.print(" Volts."); 
//    myLight.print(Lux);
//    myLight.print(" Lux. ");
//    myLight.print(RLDR);
//    myLight.print(" Resistance Ohms. ");
//    myLight.print(val);
//    myLight.println(" Volts.");
//    timestamp.timeStamp(myLight);
//    myLight.close();
//
//    tempSens1(fiveMinute);
//    tempSens2(fiveMinute);
//  }
}

//This is the nastiest piece of code I have encountered, I need to make it more efficient so it can take a picture EACH time I ask for one.
void runCamera() {
  int bytes = 0; 
  Serial1.begin(115200); // uCAM-II Default Baud Rate
 
  myFile = SD.open("Picture.txt", FILE_WRITE);
  myFile.println("Picture Begin...");
  timeStamp(myFile);  
  

  bool cameraInit = 0;

  cameraInit = camera.init(); // Initializing the camera, this must be done every time or the camera will go to sleep forever. 

  if(cameraInit == true){
  
  camera.takePicture(); // Taking a picture.

  Serial3.print("Image Size: ");
  
  Serial3.println(camera.imageSize, DEC);

  Serial3.print("Number of Packages: ");
  Serial3.println(camera.numberOfPackages(), DEC);

  while(bytes = camera.getData())
  {
    // while the bytes are getting the data? loop through.
    for (short x = 0; x < bytes; x++)
    {
      Serial3.print("0x");
      Serial3.print(camera.imgBuffer[x], HEX);
      Serial3.print(" ");
      myFile.print("0x"); // printing out to the text file.
      myFile.print(camera.imgBuffer[x], HEX);
      myFile.print(" ");
    }
  }
  Serial3.println("");
  myFile.println("");
  myFile.println("End of picture.");
  timeStamp(myFile);
  myFile.println("");
  Serial3.println("Done Downloading");
  }
  myFile.close(); //closing the text file.  
}

//function call for the wire cutter. Stays on for 3 seconds which is just enough to cut the wires.
void cutWire() {
  Serial3.println("Wire cutter:");
  // turn Cutter on:
  digitalWrite(wireCutter, HIGH);
  Serial3.println("On");
  delay(3000);
    
  // turn LED off:
  digitalWrite(wireCutter, LOW);
  Serial3.println("Off");
  delay(5000);

  Serial3.println("Done cutting!!"); 
}

void runGyroSensor(){

  myGyro = SD.open("Gyro.txt", FILE_WRITE);
  myAccel = SD.open("Accel.txt", FILE_WRITE);

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */
  myAccel.print("X: ");
  myAccel.print(acceleration.x());
  myAccel.print(" Y: ");
  myAccel.print(acceleration.y());
  myAccel.print(" Z: ");
  myAccel.print(acceleration.z());
  myAccel.print(" m/s^2 ");
  timeStamp(myAccel);
  myAccel.println();

  

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    /* Display the floating point data */
  myGyro.print("X: ");
  myGyro.print(euler.x());
  myGyro.print(" Y: ");
  myGyro.print(euler.y());
  myGyro.print(" Z: ");
  myGyro.print(euler.z());
  myGyro.print(" Degrees. ");
  timeStamp(myGyro);
  myGyro.println();
   
  myGyro.close();
  myAccel.close();

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void tempSens1(bool stamp){
    tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
    float c = tempsensor.readTempC();
    float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
    myTemp = SD.open("Temp1.txt", FILE_WRITE);
    Serial3.print("Temp Outer: "); Serial3.print(c); Serial3.print("*C\t"); // Printing to the console
    Serial3.print(f); Serial3.print("*F ");
    myTemp.print("Temp Outer: "); myTemp.print(c); myTemp.print("*C\t");  // Printing to the text file.
    myTemp.print(f); myTemp.print("*F ");
    delay(250);
    //if (stamp == true){
      timeStamp(myTemp);
    //}
    myTemp.close();
    tempsensor.shutdown_wake(1);
}


void tempSens2(bool stamp){
    tempsensor2.shutdown_wake(0);   // Don't remove this line! required before reading temp
    float c = tempsensor2.readTempC();
    float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
    myTemp2 = SD.open("Temp2.txt", FILE_WRITE);
    Serial3.print("Temp Inner: "); Serial3.print(c); Serial3.print("*C\t");
    Serial3.print(f); Serial3.print("*F ");
    myTemp2.print("Temp Inner: "); myTemp2.print(c); myTemp2.print("*C\t");  // Printing to the text file.
    myTemp2.print(f); myTemp2.print("*F ");
    delay(300);
    //if (stamp == true){
      timeStamp(myTemp2);
    //}
    myTemp2.close();
    tempsensor2.shutdown_wake(1);
}

void sendText(){

  char readData;
  byte terminate = 72;
  
  myFile = SD.open("Picture.txt", FILE_READ);
    while (myFile.available()){
    readData = myFile.read();
    if (readData){
      Serial3.print(readData);
    }
  }
  myFile.close();
  delay(200);

  Serial3.println("Done"); 
}

void timeStamp(File file){
    if (RTC.read(tm)) {
    Serial3.print("Time: ");
    file.print("Time: ");
    print2digits(file, tm.Hour);
    Serial3.write(':');
    file.write(':');
    print2digits(file, tm.Minute);
    Serial3.write(':');
    file.write(':');
    print2digits(file, tm.Second);
    Serial3.print(", Date: ");
    file.print(", Date: ");
    Serial3.print(tm.Day);
    file.print(tm.Day);
    Serial3.write('/');
    file.write('/');
    Serial3.print(tm.Month);
    file.print(tm.Month);
    Serial3.write('/');
    file.write('/');
    Serial3.print(tmYearToCalendar(tm.Year));
    file.print(tmYearToCalendar(tm.Year));
    Serial3.println();
    file.println();
  }
}

void print2digits(File file, int number) {
  if (number >= 0 && number < 10) {
    Serial3.write('0');
    file.write('0');
  }
  Serial3.print(number);
  file.print(number);
}
