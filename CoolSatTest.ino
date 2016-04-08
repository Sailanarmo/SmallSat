#include <uCamII.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include "Adafruit_MCP9808.h"
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif


UCAMII camera;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();
Adafruit_MCP9808 tempsensor2 = Adafruit_MCP9808();
MPU6050 accelgyro;

#define OUTPUT_READABLE_ACCELGYRO

File myGyro;
File myAccel;
File myFile;
File myTemp;
File myTemp2;
File myLight;

tmElements_t tm;


const int wireCutter = 29;

const long INTERVAL = 995;
const long STAMP = 299995;
unsigned long previousMillis = 0;
unsigned long previousStamp = 0;

byte serialReadCommand;

bool commandSent = false;
bool commandSent2 = false;
bool commandSent3 = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;

int analogPin = 15;
const int chipSelect = 22;

float val = 0.0;
float RLDR = 0.0;
float Lux = 0.0;

void setup() {
  //Serial.begin(115200);
  Serial3.begin(9600);

  pinMode(wireCutter, OUTPUT);
  
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

if (!tempsensor.begin()) {
    //Serial.println("Couldn't find MCP9808!");
    while (1);
  }
if (!tempsensor2.begin2()) {
    //Serial.println("Couldn't find MCP9808!");
    while (1);
  }

  //Serial.println("Initializing I2C devices...");
  Serial3.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  //Serial.println("Testing device connections...");
  Serial3.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial3.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


}


void loop() {
  unsigned long currentMillis = millis();
  unsigned long timeMillis = millis();
  int bytes = 0;
  bool cameraInit = false;


  runGyroSensor();

  if(Serial3.available()){
    serialReadCommand = Serial3.read();
    Serial3.write(serialReadCommand);

  if (serialReadCommand == 49 && !commandSent){
    commandSent = true;
    Serial3.println("Attempting to take Picture...");
    }

  if (serialReadCommand == 50 && !commandSent2){
    commandSent2 = true;
    Serial3.println("Activating Wire Cutters...");  
    }

  if (serialReadCommand == 51 && !commandSent3){
    commandSent3 = true;
    Serial3.println("Sending you a text file, standby...");  
    }
  }
  if (currentMillis - previousMillis >= INTERVAL){
    previousMillis = currentMillis;

    myLight = SD.open("Light.txt", FILE_WRITE);
    val = analogRead(analogPin);
    val = val * .0049; 

    RLDR = (1000.0 * (5 - val))/val;
    Lux = (776897.0 * (pow(RLDR, -1.206)));
    
    Serial3.print(Lux);
    Serial3.print(" Lux. ");
    Serial3.print(RLDR);
    Serial3.print(" Resistance Ohms. ");
    Serial3.print(val);
    Serial3.println(" Volts."); 
    myLight.print(Lux);
    myLight.print(" Lux. ");
    myLight.print(RLDR);
    myLight.print(" Resistance Ohms. ");
    myLight.print(val);
    myLight.println(" Volts.");
    myLight.close();

    tempSens1();
    tempSens2();

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
  if (timeMillis - previousStamp >= STAMP){
    previousStamp = timeMillis;

    myLight = SD.open("Light.txt", FILE_WRITE);
    val = analogRead(analogPin);
    val = val * .0049; 

    RLDR = (1000.0 * (5 - val))/val;
    Lux = (776897.0 * (pow(RLDR, -1.206)));
    
    Serial3.print(Lux);
    Serial3.print(" Lux. ");
    Serial3.print(RLDR);
    Serial3.print(" Resistance Ohms. ");
    Serial3.print(val);
    Serial3.println(" Volts."); 
    myLight.print(Lux);
    myLight.print(" Lux. ");
    myLight.print(RLDR);
    myLight.print(" Resistance Ohms. ");
    myLight.print(val);
    myLight.println(" Volts.");
    timeStamp();
    myLight.close();

    tempSens1();
    tempSens2();
  }
}

void runCamera() {
  int bytes = 0; 
  Serial1.begin(115200); // uCAM-II Default Baud Rate
 
  myFile = SD.open("Picture.txt", FILE_WRITE);
  myFile.println("Picture Begin...");
  writeToSDCamera();
  

  bool cameraInit = 0;

  cameraInit = camera.init(); // Initializing the camera, this must be done every time or the camera will go to sleep forever. 
    
  // TODO add a loop that will loop 60 times, if it reaches 60, we should receive an error to try to initiaze the camera again.

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
  writeToSDCamera();
  myFile.println("");
  Serial3.println("Done Downloading");
  }
  myFile.close(); //closing the text file.  
}

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
  
  int z = 0;
  
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
      
    #ifdef OUTPUT_READABLE_ACCELGYRO
      myAccel.print("Acceleration X, Y, Z: ");
      myAccel.print(ax); myAccel.print(", ");
      myAccel.print(ay); myAccel.print(", ");
      myAccel.print(az); myAccel.println("");
      
      myGyro.print("Gyro X, Y, Z: ");
      myGyro.print(gx); myGyro.print(", ");
      myGyro.print(gy); myGyro.print(", ");
      myGyro.print(gz); myGyro.println("");
    #endif
   
  myGyro.close();
  myAccel.close();
}

void tempSens1(){
    tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
    float c = tempsensor.readTempC();
    float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
    myTemp = SD.open("Temp1.txt", FILE_WRITE);
    Serial3.print("Temp Inner: "); Serial3.print(c); Serial3.print("*C\t"); // Printing to the console
    Serial3.print(f); Serial3.println("*F");
    myTemp.print("Temp Inner: "); myTemp.print(c); myTemp.print("*C\t");  // Printing to the text file.
    myTemp.print(f); myTemp.println("*F");
    delay(250);
    myTemp.close();
    tempsensor.shutdown_wake(1);
}


void tempSens2(){
    tempsensor2.shutdown_wake(0);   // Don't remove this line! required before reading temp
    float c = tempsensor2.readTempC();
    float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
    myTemp2 = SD.open("Temp2.txt", FILE_WRITE);
    Serial3.print("Temp Outer: "); Serial3.print(c); Serial3.print("*C\t");
    Serial3.print(f); Serial3.println("*F");
    myTemp2.print("Temp Outer: "); myTemp2.print(c); myTemp2.print("*C\t");  // Printing to the text file.
    myTemp2.print(f); myTemp2.println("*F");
    delay(300);
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

void writeToSDCamera(){
    if (RTC.read(tm)) {
    Serial3.print("Time: ");
    myFile.print("Time: ");
    print2digitsC(tm.Hour);
    Serial3.write(':');
    myFile.write(':');
    print2digitsC(tm.Minute);
    Serial3.write(':');
    myFile.write(':');
    print2digitsC(tm.Second);
    Serial3.print(", Date: ");
    myFile.print(", Date: ");
    Serial3.print(tm.Day);
    myFile.print(tm.Day);
    Serial3.write('/');
    myFile.write('/');
    Serial3.print(tm.Month);
    myFile.print(tm.Month);
    Serial3.write('/');
    myFile.write('/');
    Serial3.print(tmYearToCalendar(tm.Year));
    myFile.print(tmYearToCalendar(tm.Year));
    Serial3.println();
    myFile.println();
  }
}

void print2digitsC(int number) {
  if (number >= 0 && number < 10) {
    Serial3.write('0');
    myFile.write('0');
  }
  Serial3.print(number);
  myFile.print(number);
}

void timeStamp(){
    if (RTC.read(tm)) {
    Serial3.print("Time: ");
    myLight.print("Time: ");
    print2digitsL(tm.Hour);
    Serial3.write(':');
    myLight.write(':');
    print2digitsL(tm.Minute);
    Serial3.write(':');
    myLight.write(':');
    print2digitsL(tm.Second);
    Serial3.print(", Date: ");
    myLight.print(", Date: ");
    Serial3.print(tm.Day);
    myLight.print(tm.Day);
    Serial3.write('/');
    myLight.write('/');
    Serial3.print(tm.Month);
    myLight.print(tm.Month);
    Serial3.write('/');
    myLight.write('/');
    Serial3.print(tmYearToCalendar(tm.Year));
    myLight.print(tmYearToCalendar(tm.Year));
    Serial3.println();
    myLight.println();
  }
}

void print2digitsL(int number) {
  if (number >= 0 && number < 10) {
    Serial3.write('0');
    myLight.write('0');
  }
  Serial3.print(number);
  myLight.print(number);
}

