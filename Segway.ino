//Arduino 1.0+ only

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x;
int y;
int z;
float positionX = 0;
float multiplier = 1.0;

int ledPin = 8;
int b = 5;
int g = 6;
int r = 7;
int buttonPin = 9;
int buttonState = 0;
int previousButtonState = 0;

int mode = 0;

bool buttonReleased = false;
int bufferCount = 0;

void setup(){

  Wire.begin();
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(r, OUTPUT);
  pinMode(buttonPin, INPUT);
  
  Serial.println("starting up L3G4200D");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec

  delay(1500); //wait for the sensor to be ready 
}

void loop(){
  getGyroValues();  // This will update x, y, and z with new values
  if(abs(x)<20){
   x = 0; 
  }
  buttonReleased = false;
  buttonState = previousButtonState;
  buttonState = digitalRead(buttonPin);
  //Serial.println(buttonState);
   if(buttonState == HIGH && previousButtonState == LOW && mode != 2){
   
    previousButtonState = HIGH;
  }
  if(buttonState == LOW && previousButtonState == HIGH && bufferCount < 0){
    previousButtonState = LOW;
    mode++;
    bufferCount = 1000;
  }
 digitalWrite(ledPin, HIGH);
    if(mode == 0){
      Serial.println("0");
      digitalWrite(r, LOW);
      digitalWrite(g, HIGH);
      digitalWrite(b, HIGH);
    }
    if(mode == 1){
      Serial.println("1"); 
      digitalWrite(r, HIGH);
      digitalWrite(g, LOW);
      digitalWrite(b, HIGH);
    }
    if(mode == 2){
      Serial.println("2");
      digitalWrite(r, HIGH);
      digitalWrite(g, HIGH);
      digitalWrite(b, LOW);
      
    }
    if(mode == 3){
      Serial.println("3");
    }
    
     bufferCount--;
     
  if(false){
    positionX += ((double)x)/((double)multiplier);
    Serial.print("X:");
    Serial.println(positionX);

   // Serial.print(" Y:");
    //Serial.print(y);

   // Serial.print(" Z:");
    //Serial.println(z);

    delay(100); //Just here to slow down the serial to make it more readable
  }
}

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
