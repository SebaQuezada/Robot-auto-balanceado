#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <SoftwareSerial.h> // Libreria Bluetooth

SoftwareSerial BT(7, 8);

#define leftMotorPWMPin   10
#define leftMotorDirPin   11
#define rightMotorPWMPin  6
#define rightMotorDirPin  9

#define Kp  10
#define Kd  0.125
#define Ki  100
#define sampleTime  0.005
float targetAngle = 3.0; // Ahora targetAngle es una variable global

#define leftSensorPin A0
#define rightSensorPin A1

#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'

int motorL1 = 10;
int motorL2 = 11;
int motorR1 = 6;
int motorR2 = 9;


MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile byte count = 0;

int velocidad = 100;
const int velocidadMaxima = 255;
const int velocidadMinima = 0;
const int incrementoVelocidad = 10;
const int decrementoVelocidad = 10; // Velocidad de decremento para frenado gradual


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if (leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  } else {
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }

  if (rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  } else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

void init_PID() {
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  OCR1A = 9999;   // set compare match register to set sample time 5ms
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11);  // Set CS11 bit for prescaling by 8
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();          // enable global interrupts
}

void setup() {
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  pinMode(13, OUTPUT);
  
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);

  mpu.initialize();
  mpu.setYAccelOffset(723);
  mpu.setZAccelOffset(-4500);
  mpu.setXGyroOffset(-83);

  init_PID();
  BT.begin(9600); // Set the baud rate for serial communication

}

void loop() {
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();

  motorPower = constrain(motorPower, -180, 180);
  
  int leftSensorValue = analogRead(leftSensorPin);
  int rightSensorValue = analogRead(rightSensorPin);
  
  // Simple line following logic
  if (leftSensorValue < 500 && rightSensorValue > 500) {
    // Line is to the right
    setMotors(motorPower + 35, motorPower - 35);
  } else if (leftSensorValue > 500 && rightSensorValue < 500) {
    // Line is to the left
    setMotors(motorPower - 35, motorPower + 35);
  } else {
    // Line is straight ahead
    setMotors(motorPower, motorPower);
  }

    if (BT.available() > 0) {
    char command = BT.read();
    executeCommand(command);

  }

}

ISR(TIMER1_COMPA_vect) {
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;  
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

  motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;

  count++;
  if (count == 200) {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}

void executeCommand(char command) {
  switch (command) {
   
    case FORWARD:
      digitalWrite(motorL1, velocidad);
      digitalWrite(motorL2, LOW);
      digitalWrite(motorR1, velocidad);
      digitalWrite(motorR2, LOW);
      break;
   
    case BACKWARD:
      digitalWrite(motorL1, LOW);
      digitalWrite(motorL2, velocidad);
      digitalWrite(motorR1, LOW);
      digitalWrite(motorR2, velocidad);
      break;

    case LEFT:
      digitalWrite(motorL1, LOW);
      digitalWrite(motorL2, velocidad);
      digitalWrite(motorR1, velocidad);
      digitalWrite(motorR2, LOW);
      break;
   
    case RIGHT:
      digitalWrite(motorL1, velocidad);
      digitalWrite(motorL2, LOW);
      digitalWrite(motorR1, LOW);
      digitalWrite(motorR2, velocidad);
      break;
     
    case CIRCLE:
      break;
   
    case CROSS:
      // Decrease speed
      velocidad = max(velocidad - incrementoVelocidad, velocidadMinima);
      break;
   
    case TRIANGLE:
      // Increase speed
      velocidad = min(velocidad + incrementoVelocidad, velocidadMaxima);
      break;
   
    case SQUARE:
      break;
     
    case START:
      break;
      
    case PAUSE:
      stopMotors();
      break;
     
    default:
      stopMotors();
      break;
  }
}

void stopMotors() {
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);
}

void stopMotorsGradually() {
  while (velocidad > 0) {
    velocidad = max(velocidad - decrementoVelocidad, 0);
    digitalWrite(motorL1, velocidad);
    digitalWrite(motorL2, velocidad);
    digitalWrite(motorR1, velocidad);
    digitalWrite(motorR2, velocidad);
    delay(100); // Ajusta el delay para controlar la rapidez de la desaceleración
  }
  velocidad = 100;
 
}
