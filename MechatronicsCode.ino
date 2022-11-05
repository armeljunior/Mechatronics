#include <Servo.h>

Servo myservo;  // create servo object to control a servo

//Motor shield connections
  // Pins for all inputs
#define pinAI1 7                //channel A input
#define pinBIN1 12              //channel B input
#define pinAI2 8                //channel A
#define pinBIN2 13              //channel B2 input
#define PWMA 5                  //pwm motor1
#define PWMB 6                  //pwm motor2 
#define pinStandBy 9            //pin for standby

//Motor speed
int MotorSpeed = 50;            //MAX 255
int turnspeed = 110;            //MAX 255

int counter = 0;                //Set a counter to decide for when gripping has occured
#define turndelay 10            //small rotating delay for left and right

boolean standBy = 0;            // standBy pin Value

//Sensor Connection

const int leftsensorpin= A0;    //sensor connection left pin
const int rightsensorpin= A5;   //sensor connection right pin
const int triggersensorpin= A1; //sensor connection trigger pin

//Sensor threshold values

#define rightsensorthreshold 500//Right sensor threshold to adjust
#define leftsensorthreshold 500 //Left sensor threshold to adjust
#define TapeValue 500           //value of tape being detected instantly

//intitalisation for serial monitor printing

int leftsensorstate;            //left sensor state
int rightsensorstate;           //right sensor state
int triggersensorstate;         //trigger sensor state

void setup() {

  OpenClaws();
  // Assign the digital I/O pin directions
  pinMode(pinAI1, OUTPUT);      //set pinAI1 as output for motor
  pinMode(pinAI2, OUTPUT);      //set pinAI2 as output for motor
  pinMode(PWMA, OUTPUT);        //PWMA as output for motor A
  pinMode(PWMB, OUTPUT);        //PWMBd as output for motor A
  pinMode(pinStandBy, OUTPUT);  //set pinA1 as output for motor

  Serial.begin(9600);
  // Drive the standby pin high to enable the output
  standBy = true;
  digitalWrite(pinStandBy, standBy);

  // set an initial value for the PWM value

}

void loop() {
  SensorReadings();
  //MotorTest(); //for testing 
  LineFollowing();
  //delay(100);
}

//Sensor Readings

void SensorReadings() {
  leftsensorstate = analogRead(leftsensorpin);      //set variable to grab the analogue values
  rightsensorstate = analogRead(rightsensorpin);    //set variable to grab the analogue values
  triggersensorstate = analogRead(triggersensorpin);//set variable to grab the analogue values

  Serial.println("Left Sensor: ");
  Serial.print(leftsensorstate);

  Serial.println("Right Sensor: ");
  Serial.print(rightsensorstate);

  Serial.println("Trigger Sensor: ");
  Serial.print(triggersensorstate);
  Serial.println("\n");
  delay(50);

}

//Line following and Motors

void LineFollowing() {

  if ((rightsensorstate > rightsensorthreshold) && (leftsensorstate < leftsensorthreshold)) //turning right if threshold of right > left
  {
    Serial.println("turning right");

    digitalWrite(pinAI1, LOW); //set pin to low
    digitalWrite(pinAI2, HIGH);

    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, HIGH);

    analogWrite(PWMA, turnspeed);
    analogWrite(PWMB, MotorSpeed);
    delay(turndelay);

  } else {
    Serial.println("going forward");
    
    digitalWrite(pinAI1, HIGH); 
    digitalWrite(pinAI2, LOW); 

    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, HIGH);

    analogWrite(PWMA, MotorSpeed);
    analogWrite(PWMB, MotorSpeed);
  }

  if ((rightsensorstate < rightsensorthreshold) && (leftsensorstate > leftsensorthreshold)) //turning left if threshold of left > right
  {
    Serial.println("turning left");

    digitalWrite(pinAI1, HIGH);
    digitalWrite(pinAI2, LOW);

    digitalWrite(pinBIN1, HIGH);
    digitalWrite(pinBIN2, LOW);

    analogWrite(PWMA, turnspeed);
    analogWrite(PWMB, MotorSpeed);

    delay(turndelay);
  }
  if ((triggersensorstate > TapeValue)) {

    counter += 1;                         //Tape has been detected increase counter by one
    Serial.println("stop detected tape");
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    delay(2000);

    if (counter < 2) {                  //If tape is not detected counter < 2 close claws
      Serial.println("going forward");
      CloseClaws();
      digitalWrite(pinAI1, HIGH); //
      digitalWrite(pinAI2, LOW); //

      digitalWrite(pinBIN1, LOW);
      digitalWrite(pinBIN2, HIGH);
    
      
    } else {                            //Else go forward
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
      OpenClaws();
      delay(10000);
     
    }
    analogWrite(PWMA, MotorSpeed);
    analogWrite(PWMB, MotorSpeed);
  }
  if (counter == 2) {                   //if counter is 2 stop and close claws 
    Serial.println("stop counter is 2 the end");
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    delay(10000);
  }

}

void MotorTest() {
  //GO FORWARD
  Serial.println("going forward");
  
  digitalWrite(pinAI1, HIGH); 
  digitalWrite(pinAI2, LOW); 

  digitalWrite(pinBIN1, LOW);
  digitalWrite(pinBIN2, HIGH);

  analogWrite(PWMA, MotorSpeed);
  analogWrite(PWMB, MotorSpeed);
}

//Claws / Clamp
void CloseClaws() {
  myservo.attach(3);                          // attaches the servo on pin 3 to the servo object
  for (int pos = 0; pos <= 180; pos += 1) {   // goes from 0 degrees to 180 degrees
    myservo.write(pos);                       //servo go through all angles 0 to 180 
    delay(15);                                // waits 15 ms for claws to reach the position
  }
}

void OpenClaws() {
  myservo.attach(3);                          // attaches the servo on pin 3 to the servo object
  for (int pos = 180; pos >= 0; pos -= 1) {   // goes from 180 degrees to 0 degrees
    myservo.write(pos);                       //servo go through all angles 0 to 180 
    delay(15);                                // waits 15 ms for claws to reach the position
  }
}
