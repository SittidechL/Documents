https://randomnerdtutorials.com/projects-esp32/
# Topic<a id='0'></a>
* [Input Outputs](#1)
* [PWM](#2)
* [Analog inputs](#3)
* [Iterrupts Timers](#4)
* [Deep Sleep](#5)

## Input output<a id='1'></a>
![image](https://user-images.githubusercontent.com/60011264/163699930-b38f96b5-03a7-4efd-936f-dbf42315ca7c.png)

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
[Topic](#0)

[![Everything Is AWESOME](http://i.imgur.com/Ot5DWAW.png)](https://youtu.be/StTqXEQ2l-Y?t=35s "Everything Is AWESOME")

* [ESP32 Pin30](https://github.com/SittidechL/Documents/blob/main/ESP32/arduino/pin.md)

## PWM<a id='2'></a>
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
[Topic](#0)

## Analog Inputs<a id='4'></a>
![image](https://user-images.githubusercontent.com/60011264/163700286-73d3f76a-b30f-45cd-86d0-de531e4a2196.png)
<iframe width="720" height="405" src="https://www.youtube.com/embed/doEY6yi9src" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
```
// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int potPin = 34;

// variable for storing the potentiometer value
int potValue = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  potValue = analogRead(potPin);
  Serial.println(potValue);
  delay(500);
}
```

[Topic](#0)


## Interrupts Timers<a id='4'></a>
![image](https://user-images.githubusercontent.com/60011264/163700136-122b3202-e9d5-45e3-aebc-6928cc67da85.png)
```

```

[Topic](#0)


## Deep Sleep<a id='5'></a>
![image](https://user-images.githubusercontent.com/60011264/163700136-122b3202-e9d5-45e3-aebc-6928cc67da85.png)
```

```

[Topic](#0)


