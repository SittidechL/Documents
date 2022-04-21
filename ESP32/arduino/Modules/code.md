# Topic<a id='0'></a>
* [Dc Motor](#1)

```
ESP32 39PIN WROOM
M1_EN1 15
M1_IN1 2
M1_IN2 0
M2_IN2 4
M2_IN1 16
M2_EN2 17
ENCODER1_A 12
ENCODER1_B 14
ENCODER2_A 27
ENCODER2_B 26
```
## Dc Motor<a id='1'></a>
https://www.electronicsdna.com/basic-esp32-control-dc-motor-2ch/
![image](https://user-images.githubusercontent.com/60011264/164017831-6d67c101-1570-4738-9dd9-63bc893e0edc.png)

```arduino.ino
/*
code Program ESP32 for Control DC Motor and L298 Module
*https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
*/

int motor1Pin1 = 27;    // Set pin for Motor A
int motor1Pin2 = 26;
int enable1Pin = 14;
int motor2Pin1 = 18;    // Set pin for Motor B
int motor2Pin2 = 5;
int enable2Pin = 19;

// Setting PWM properties
const int freq = 500;
const int pwmChannel = 0;
const int pwmChanne2 = 1;
const int resolution = 8;
int dutyCycle = 200;

void setup() {
// sets the pins as outputs
pinMode(motor1Pin1, OUTPUT);
pinMode(motor1Pin2, OUTPUT);
pinMode(enable1Pin, OUTPUT);
pinMode(motor2Pin1, OUTPUT);

pinMode(motor2Pin2, OUTPUT);
pinMode(enable2Pin, OUTPUT);
// configure LED PWM functionalitites
ledcSetup(pwmChannel, freq, resolution);
// attach the channel to the GPIO to be controlled
ledcAttachPin(enable1Pin, pwmChannel);
ledcAttachPin(enable2Pin, pwmChanne2);
Serial.begin(115200);
Serial.print("Testing DC Motor");

}
void loop() {
// Move the DC motor forward at maximum speed
Serial.println("Moving Forward");
digitalWrite(motor1Pin1, LOW);
digitalWrite(motor1Pin2, HIGH);
digitalWrite(motor2Pin1, LOW);
digitalWrite(motor2Pin2, HIGH);
delay(2000);

// Stop the DC motor
Serial.println("Motor stopped");
digitalWrite(motor1Pin1, LOW);
digitalWrite(motor1Pin2, LOW);
digitalWrite(motor2Pin1, LOW);
digitalWrite(motor2Pin2, LOW);
delay(1000);

// Move DC motor backwards at maximum speed
Serial.println("Moving Backwards");
digitalWrite(motor1Pin1, HIGH);
digitalWrite(motor1Pin2, LOW);
digitalWrite(motor2Pin1, HIGH);
digitalWrite(motor2Pin2, LOW);
delay(2000);

// Stop the DC motor
Serial.println("Motor stopped");
digitalWrite(motor1Pin1, LOW);
digitalWrite(motor1Pin2, LOW);
digitalWrite(motor2Pin1, LOW);
digitalWrite(motor2Pin2, LOW);
delay(1000);

// Move DC motor forward with increasing speed
digitalWrite(motor1Pin1, HIGH);
digitalWrite(motor1Pin2, LOW);
digitalWrite(motor2Pin1, HIGH);
digitalWrite(motor2Pin2, LOW);

while (dutyCycle <= 255){
ledcWrite(pwmChannel, dutyCycle);
ledcWrite(pwmChanne2, dutyCycle);
Serial.print("Forward with duty cycle: ");
Serial.println(dutyCycle);
dutyCycle = dutyCycle + 5;
delay(300);
}
dutyCycle = 150;
}
```
[Topic](#0)
