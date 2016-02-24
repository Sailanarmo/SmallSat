#include <uCamII.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "Adafruit_MCP9808.h"


const int buttonPin = 24;     // the number of the pushbutton pin for Camera
const int ledPin =  26;      // the number of the LED pin for Camera
const int buttonPin2 = 23;   // the number of the pushbutton pin for Temp Sensor
const int ledPin2 = 25;      // the number of the LED pin for the Temp Sensor
int buttonStateCam = 0;         // variable for reading the pushbutton status
int buttonStateTemp = 0;
String tempData = "Temp_0.txt";
String pic = "Pic_0.txt";

UCAMII camera; //Creating instance of Camera
File myFile; //Creating files for SD Card

Adafruit_MCP9808 tempsensor = Adafruit_MCP9808(); //Creating instace of Temperature Sensor


void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT); // for Temp Sensor Initialization
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin2, INPUT); // for Temp Sensor Initialization

  delay(5000);
  Serial.begin(115200); // Arduino IDE Monitor
  // Serial1 is HARDCODED in the uCAM-II library for the Camera

  Serial.print("Initializing SD card..."); // Initiallizes SD Card in Port 22.

  if (!SD.begin(22)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  Serial.println("");


  Serial.print("Initializing Temperature Sensor...");
  if (!tempsensor.begin()) {
    Serial.println("Couldn't find MCP9808!");
    while (1);
  }
  Serial.println("Initialization done.");
  Serial.println("");
  
  

  Serial.print("Waiting for a command..."); //waiting for initialization of Camera.
  Serial.println(""); 

  
}

void loop() 
{
  //UCAMII camera;
  short x = 0;
  int bytes = 0; //maybe can change this to always be = getData()? maybe move bytes down to the while loop?
  int start = 0;
  
  

  buttonStateCam = digitalRead(buttonPin);
  buttonStateTemp = digitalRead(buttonPin2);

    if (buttonStateCam == LOW) //if LED is off, wait for picture to take
  {
    Serial1.begin(115200); // uCAM-II Default Baud Rate

    myFile = SD.open(pic.c_str(), FILE_WRITE);
    //myFile = SD.open("picture.txt", FILE_WRITE);

    camera.init();
    digitalWrite(ledPin, HIGH); // Turning LED on to say the picture is being taken.
    camera.takePicture(); //TODO should set image buffer?
    Serial.print("Image Size: ");
    Serial.println(camera.imageSize, DEC);
    Serial.print("Number of Packages: ");
    Serial.println(camera.numberOfPackages(), DEC);
//    Serial.println("I am outside the loop");
    //I think it breaks here?
    //while (bytes = camera.getData() )
    while(bytes = camera.getData())
    {
  
      //Serial.println("I am here talking to you.");
      //bytes = camera.getData();
      for (x = 0; x < bytes; x++)
      {
        Serial.print("0x");
        Serial.print(camera.imgBuffer[x], HEX);
        Serial.print(" ");
        myFile.print("0x");
        myFile.print(camera.imgBuffer[x], HEX);
        myFile.print(" ");
        //Serial1.end();
      }
      //Serial.println("");
    }
    Serial.println("");
    myFile.println("");
    Serial.println("Done Downloading");
    digitalWrite(ledPin, LOW); //turns LED off to tell us that the picture is done downloading.
    myFile.close();
    Serial.println("Push the button to talk.");
    //Serial1.end();
    //camera.init();
    //Serial1.flush();
    pic[4] = pic[4] + 1;
  }

  if (buttonStateTemp == LOW){
      digitalWrite(ledPin2, HIGH);

      
      myFile = SD.open(tempData.c_str(), FILE_WRITE);
      //myFile = SD.open("Temp.txt", FILE_WRITE);
      
      // Read and print out the temperature, then convert to *F
      

      for (int i = 0; i < 5; ++i){
      tempsensor.shutdown_wake(0);   // Don't remove this line! required before reading temp
      float c = tempsensor.readTempC();
      float f = c * 9.0 / 5.0 + 32;  
      Serial.print("Temp: "); Serial.print(c); Serial.print("*C\t"); 
      Serial.print(f); Serial.println("*F");
      myFile.print("Temp: "); myFile.print(c); myFile.print("*C\t");
      myFile.print(f); myFile.println("*F");
      delay(250);
      tempsensor.shutdown_wake(1);
      delay(2000);
      }
      myFile.close();
      Serial.println("Done taking Temperature.");
      tempsensor.shutdown_wake(1); // shutdown MSP9808 - power consumption ~0.1 mikro Ampere
  
      digitalWrite(ledPin2, LOW);
      tempData[5] = tempData[5] + 1;
  }
    
}
