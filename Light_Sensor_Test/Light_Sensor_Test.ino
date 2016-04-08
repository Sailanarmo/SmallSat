#include <SPI.h>
#include <SD.h>

int analogPin = 15;
const int chipSelect = 22;

const int buttonPin = 28;

int buttonState = 0;

float val = 0.0;

File myFile;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(buttonPin, INPUT);

  Serial.print("Initializing SD card..."); // Initiallizes SD Card in Port 22.

  if (!SD.begin(22)) {
    Serial.println("initialization failed!"); //if initialization of SD Card fails, should stop here.
    return;
  }
  Serial.println("initialization done."); //Done initializing the SD Card.
  Serial.println("");

  

}

void loop() {

 buttonState = digitalRead(buttonPin); 
 myFile = SD.open("Light.txt", FILE_WRITE);

 while (buttonState == HIGH){ 
 val = analogRead(analogPin);
 val = val * .0049; 
 Serial.print(val);
 Serial.println(" Volts.");
 myFile.print(val);
 myFile.println(" Volts.");

 delay(30000);
 buttonState = digitalRead(buttonPin); 
 if (buttonState == LOW){
   
  Serial.println("Done.");
  myFile.close();
  break;
  }
 }
}
