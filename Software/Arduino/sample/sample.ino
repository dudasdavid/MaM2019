// testing a stepper motor with a Pololu A4988 driver board
// on an Uno the onboard led will flash with each step
// as posted on Arduino Forum at http://forum.arduino.cc/index.php?topic=208905.0

byte directionPinX = 5;
byte stepPinX = 2;
byte directionPinY = 6;
byte stepPinY = 3;
byte directionPinZ = 7;
byte stepPinZ = 4;
byte directionPinA = 13;
byte stepPinA = 12;

byte enaPin = 8;

int numberOfSteps = 2000;

//byte ledPin = 13;
int pulseWidthMicros = 100;  // microseconds
int microsBetweenSteps = 1000; // microseconds

int selectedMotor = 3;

void setup() 
{ 
  pinMode(directionPinX, OUTPUT);
  pinMode(stepPinX, OUTPUT);
  pinMode(directionPinY, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(directionPinZ, OUTPUT);
  pinMode(stepPinZ, OUTPUT);
  pinMode(directionPinA, OUTPUT);
  pinMode(stepPinA, OUTPUT);
  //pinMode(ledPin, OUTPUT);
  pinMode(enaPin, OUTPUT);  

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
}

void loop() 
{ 

  delay(2000);
  //digitalWrite(ledPin, LOW);
  digitalWrite(enaPin, LOW);

  if (selectedMotor == 1)
  {
    digitalWrite(directionPinX, HIGH);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinX, LOW);
      
      delayMicroseconds(microsBetweenSteps);    
    }
  }

  if (selectedMotor == 2)
  {
    digitalWrite(directionPinY, HIGH);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinY, LOW);
      
      delayMicroseconds(microsBetweenSteps*2);    
    }
  }

  if (selectedMotor == 3)
  {
    digitalWrite(directionPinZ, HIGH);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinZ, LOW);
      
      delayMicroseconds(microsBetweenSteps*3);    
    }
  }

  if (selectedMotor == 4)
  {
    digitalWrite(directionPinA, HIGH);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinA, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinA, LOW);
      
      delayMicroseconds(microsBetweenSteps*4);    
    }
  }
  
  digitalWrite(enaPin, HIGH);
  delay(2000);
  digitalWrite(enaPin, LOW);

  if (selectedMotor == 1)
  {
    digitalWrite(directionPinX, LOW);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinX, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinX, LOW);
      
      delayMicroseconds(microsBetweenSteps);
    }
  }

  if (selectedMotor == 2)
  {
    digitalWrite(directionPinY, LOW);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinY, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinY, LOW);
      
      delayMicroseconds(microsBetweenSteps*2);
    }
  }

  if (selectedMotor == 3)
  {
    digitalWrite(directionPinZ, LOW);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinZ, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinZ, LOW);
      
      delayMicroseconds(microsBetweenSteps*3);
    }
  }

  if (selectedMotor == 4)
  {
    digitalWrite(directionPinA, LOW);
    for(int n = 0; n < numberOfSteps; n++) {
      digitalWrite(stepPinA, HIGH);
      delayMicroseconds(pulseWidthMicros);
      digitalWrite(stepPinA, LOW);
      
      delayMicroseconds(microsBetweenSteps*4);
    }
  }

  digitalWrite(enaPin, HIGH);

}
