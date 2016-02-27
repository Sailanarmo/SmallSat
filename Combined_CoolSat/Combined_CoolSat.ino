#include <uCamII.h>#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include "Adafruit_MCP9808.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


const int buttonPinCam = 24;     // the number of the pushbutton pin for Camera
const int ledPinCam =  26;      // the number of the LED pin for Camera
const int buttonPinTemp = 23;   // the number of the pushbutton pin for Temp Sensor
const int ledPinTemp = 25;      // the number of the LED pin for the Temp Sensor
const int buttonPinGy = 28;   
const int ledPinGy = 27;
const int lightSensorPin = 15;  //light sensor pin.

const int boomSwitchPin = 30; // the number of the pushbutton pin for boom switch

const int wireCutter = 29;

int buttonStateCam = 0;         // variable for reading the pushbutton status for Camera
int buttonStateTemp = 0;        // variable for reading the pushbutton status for Temperature
int buttonStateGy = 0;
int boomSwitchState = 0;        

String tempData = "Temp_0.txt"; // initializer? for text file for Temperature
String pic = "Pic_0.txt";       // initializer? for text file for Picture
String Gyro = "Gyro_0.txt";
String Accel = "Accel_0.txt";

int16_t ax, ay, az;
int16_t gx, gy, gz;

UCAMII camera; //Creating instance of Camera
File myFile; //Creating files for SD Card
File myGyro;
File myAccel;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808(); //Creating instace of Temperature Sensor
MPU6050 accelgyro; // Creating instance of Gyro 

#define OUTPUT_READABLE_ACCELGYRO

void setup() {
 
  pinMode(ledPinCam, OUTPUT);  // initialize the LED pin as an output:
  pinMode(ledPinTemp, OUTPUT); // for Temp Sensor Initialization
  pinMode(ledPinGy, OUTPUT);
  pinMode(wireCutter, OUTPUT); //for the wire cutter
 
  
  pinMode(buttonPinCam, INPUT);  // initialize the pushbutton pin as an input:
  pinMode(buttonPinTemp, INPUT); // for Temp Sensor Initialization
  pinMode(buttonPinGy, INPUT);  
  pinMode(boomSwitchPin, INPUT); // for the Boom Switch
  

  delay(5000); 
  
  Serial.begin(115200); // Arduino IDE Monitor
  

  Serial.print("Initializing SD card..."); // Initiallizes SD Card in Port 22.

  if (!SD.begin(22)) {
    Serial.println("initialization failed!"); //if initialization of SD Card fails, should stop here.
    return;
  }
  Serial.println("initialization done."); //Done initializing the SD Card.
  Serial.println("");


  Serial.print("Initializing Temperature Sensor..."); //Initializing the Temperature Sensor
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!"); //if initilization fails, will stop here.
    while (1);
  }
  Serial.println("Initialization done.");  // done initializing the Temperature Sensor
  Serial.println("");

  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  Serial.print("Waiting for a command..."); // Waiting for a command from the user, can be used to take a picture, take the temperature, or speed.
  Serial.println(""); 

  
}

void loop(){
  
  short x = 0;
  int bytes = 0; 
  int start = 0;

  bool lightSensor = false;

  float lightReading = 0.0;
  
  

  buttonStateCam = digitalRead(buttonPinCam); // reading the states of the buttons?
  buttonStateTemp = digitalRead(buttonPinTemp);
  buttonStateGy = digitalRead(buttonPinGy);
  boomSwitchState = digitalRead(boomSwitchPin);

    // Camera Function
    if (buttonStateCam == LOW) //if LED is off, wait for picture to take
  {
    Serial1.begin(115200); // uCAM-II Default Baud Rate

    myFile = SD.open(pic.c_str(), FILE_WRITE);  // open a text file to prepare for the data to be stored in a text file. 
    //myFile = SD.open("picture.txt", FILE_WRITE);

    camera.init(); // Initializing the camera, this must be done every time or the camera will go to sleep forever. 
    
    // TODO add a loop that will loop 60 times, if it reaches 60, we should receive an error to try to initiaze the camera again.
    
    digitalWrite(ledPinCam, HIGH); // Turning LED on to say the picture data is being received.
    camera.takePicture(); // Taking a picture.
    Serial.print("Image Size: ");
    Serial.println(camera.imageSize, DEC);
    Serial.print("Number of Packages: ");
    Serial.println(camera.numberOfPackages(), DEC); // printing us the data of the image size and how many packages there are.

    while(bytes = camera.getData()){ // while the bytes are getting the data? loop through.
      for (x = 0; x < bytes; x++){
        Serial.print("0x"); // printing out to the console
        Serial.print(camera.imgBuffer[x], HEX);
        Serial.print(" ");
        myFile.print("0x"); // printing out to the text file.
        myFile.print(camera.imgBuffer[x], HEX);
        myFile.print(" ");
      }
    }
    Serial.println(""); 
    myFile.println("");
    Serial.println("Done Downloading"); // telling us that we have successfully received the photo.
    digitalWrite(ledPinCam, LOW); //turns LED off to tell us that the picture is done downloading.
    myFile.close(); //closing the text file.
    Serial.println("Waiting for a command..."); // asking for another user input.
    pic[4] = pic[4] + 1;  // increments the file so we can have more than one text file.
    // TODO: add something here that will delete a text file if it's reached more than n (n will be what we decide) files.
  }
  
  // Temperature Sensor Function
  if (buttonStateTemp == LOW){  // if the Temperature Sensor button is pressed, then turn on the LED
      digitalWrite(ledPinTemp, HIGH);

      
      myFile = SD.open(tempData.c_str(), FILE_WRITE); // opening a text file for reading Temperature Data
      //myFile = SD.open("Temp.txt", FILE_WRITE);
      
      // Read and print out the temperature, then convert to *F
      

      for (int i = 0; i < 5; ++i){  // For loop to increment how many times we are taking temperature data.
      tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
      float c = tempsensor.readTempC();
      float f = c * 9.0 / 5.0 + 32;  // conversion to Farenheight
      Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t"); // Printing to the console
      Serial.print(f); Serial.println("*F");
      myFile.print("Temp: "); myFile.print(c); myFile.print("*C\t");  // Printing to the text file.
      myFile.print(f); myFile.println("*F");
      delay(250);
      tempsensor.shutdown_wake(1);  // Not 100% sure on this, but I believe it's to tell the sensor to stop collection data.
      delay(2000);  // Adding a delay so it can have a little time to think between taking temperatures.
      }
      myFile.close(); // Closing Temperature Text File
      Serial.println("Done taking Temperature.");
      Serial.println("Waiting for a command...");
      tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere
  
      digitalWrite(ledPinTemp, LOW); // Turn the LED OFF to tell us it's done recording data.
      tempData[5] = tempData[5] + 1;  // increments the file so we can have more than one text file.

      lightSensor = true;

      // TODO: add something here that will delete a text file if it's reached more than n (n will be what we decide) files.
  }

  if (buttonStateGy == LOW){
    digitalWrite(ledPinGy, HIGH);
    int z = 0;
    myGyro = SD.open(Gyro.c_str(), FILE_WRITE);
    myAccel = SD.open(Accel.c_str(), FILE_WRITE);
    // TODO Write text file
    
    for(int i = 0; i < 20; ++i){
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 
      
        #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.println("Acceleration X, Y, Z:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.println("");
        myAccel.println("Acceleration X, Y, Z: ");
        myAccel.print(ax); myAccel.print(",");
        myAccel.print(ay); myAccel.print(",");
        myAccel.print(az); myAccel.println("");

        Serial.println("Gyro X, Y, Z:\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz); Serial.println("");
        myGyro.println("Gyro X, Y, Z: ");
        myGyro.print(gx); myGyro.print(",");
        myGyro.print(gy); myGyro.print(",");
        myGyro.print(gz); myGyro.println("");
        #endif
  
        #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
        #endif
        delay(2000);
      }
      Serial.println("Waiting for a command...");
      digitalWrite(ledPinGy, LOW);
      myGyro.close();
      myAccel.close();
      Gyro[5] = Gyro[5] + 1;
      Accel[5] = Accel[5] +1;
      
  } 

  if (boomSwitchState == HIGH){
    // turn Cutter on:
    digitalWrite(wireCutter, HIGH);
    Serial.println("On");

    delay(3000);
    
    // turn LED off:
    digitalWrite(wireCutter, LOW);
    Serial.println("Off");

    delay(5000);
  }

  if (lightSensor == true){
    for(int i = 0; i < 5; ++i){
    lightReading = analogRead(lightSensorPin);
    lightReading = lightReading * .0049;
    if (lightReading > 1){ 
      Serial.print(lightReading);
      Serial.println(" Volts.");
      Serial.println("The Sun is up.");
      } else {
        Serial.print(lightReading);
        Serial.println(" Volts.");
        Serial.println("The Sun is down.");
      }
      delay(2000);
    }
    lightSensor = false;
  }
}
