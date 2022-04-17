https://randomnerdtutorials.com/projects-esp32/

* [Pinout](#1)
* [Input Outputs](#2)
* [PWM](#3)
* [Analog inputs](#4)
* [Iterrupts Timers](#5)
* [Deep Sleep](#6)

## Pinout<a id='1'></a>
![image](https://user-images.githubusercontent.com/60011264/163699930-b38f96b5-03a7-4efd-936f-dbf42315ca7c.png)

![image](https://user-images.githubusercontent.com/60011264/163699105-87e77b72-1bb1-41c8-8c4a-9f569d18cc9b.png)

```
// Complete Instructions: https://RandomNerdTutorials.com/esp32-digital-inputs-outputs-arduino/

// set pin numbers
const int buttonPin = 4;  // the number of the pushbutton pin
const int ledPin =  5;    // the number of the LED pin

// variable for storing the pushbutton status 
int buttonState = 0;

void setup() {
  Serial.begin(115200);  
  // initialize the pushbutton pin as an input
  pinMode(buttonPin, INPUT);
  // initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // read the state of the pushbutton value
  buttonState = digitalRead(buttonPin);
  Serial.println(buttonState);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH
  if (buttonState == HIGH) {
    // turn LED on
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off
    digitalWrite(ledPin, LOW);
  }
}
```
[![Everything Is AWESOME](http://i.imgur.com/Ot5DWAW.png)](https://youtu.be/StTqXEQ2l-Y?t=35s "Everything Is AWESOME")

* [ESP32 Pin30](https://github.com/SittidechL/Documents/blob/main/ESP32/arduino/pin.md)

## PWM<a id='3'></a>
![image](https://user-images.githubusercontent.com/60011264/163656438-23070075-f013-4785-9bdd-573bb49d7a30.png)

```arduino.ino
// the number of the LED pin
const int ledPin = 16;  // 16 corresponds to GPIO16

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
 
void setup(){
  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);
}
 
void loop(){
  // increase the LED brightnss
  for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);
    delay(15);
  }

  // decrease the LED brightness
  for(int dutyCycle = 255; dutyCycle >= 0; dutyCycle--){
    // changing the LED brightness with PWM
    ledcWrite(ledChannel, dutyCycle);   
    delay(15);
  }
}

```
