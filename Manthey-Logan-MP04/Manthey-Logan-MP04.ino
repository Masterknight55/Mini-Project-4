

/* -------------------------------------------------------------------------- */
/*                               Mini Project 4                               */
/* -------------------------------------------------------------------------- */

/* --------------------------------- Header --------------------------------- */

/* -------------------------------------------------------------------------- */

/* --------------------------------- Imports -------------------------------- */

#include <SR04.h>
#include <Servo.h>

/* -------------------------------------------------------------------------- */

/* ----------------------------- Global Varibles ---------------------------- */

int buttonA = 12;
int redLED = 7;
int yellowLED = 6;
int greenLED = 5;
int triggerPin = 3;
int echoPin = 2;

bool buttonA_State;
long distance;

int servoPin = 10;
int DCMotorPin = 11;

double joystickXPin = A1;
double joystickyPin = A0;
double joystickButton = 4;

double joystickX;
double joystickY;

double servoPosition;
double DCMotorPower;

Servo myServo;
SR04 mySonar = SR04(echoPin, triggerPin);

#define IDLE 0 // defining our states
#define GO 1
#define STOP 2
#define TURNING 3

#define AUTO 3
#define MANUAL 4

int MState = AUTO;
int ASTATE = IDLE; // start in the IDLE state

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;     // previous state of the button

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                    Setup                                   */
/* -------------------------------------------------------------------------- */

void setup()
{

  pinMode(buttonA, INPUT);

  pinMode(redLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  /** This section sets the pin modes for the joystick and sets the default 
   * state of the button to be off */
  pinMode(joystickButton, INPUT);
  digitalWrite(joystickButton, HIGH);

  //Set the default for the button to be "off"
  digitalWrite(joystickButton, HIGH);

  /** This line uses the attach function in the servo object we imported to set
   * which pin the servo uses. In this case we are using a global variable 
   * called servoPin.
   */
  myServo.attach(servoPin);

  //This sets the DC Motor pin to be an output.
  pinMode(DCMotorPin, OUTPUT);

  Serial.begin(9600);
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                                  Main Loop                                 */
/* -------------------------------------------------------------------------- */

void loop()
{
  read_sensors();

  switch (MState)
  {
  case MANUAL:
    manualState();
    Serial.println("I am in MANUAL State!");
    break;
  case AUTO: // if you are in state GO, run the function run_go
    autoState();
    Serial.println("I am in AUTO State!");
    break;
  default: // default case, do nothing and go back
    break;
  }
}

/* -------------------------------------------------------------------------- */
/*                             Auto State Machine                             */
/* -------------------------------------------------------------------------- */

void autoState()
{

  read_sensors();

  if (buttonA_State == 1)
  {
    MState = MANUAL;
  }
  else
  {
    switch (ASTATE)
    {
    case IDLE: // if you are in state IDLE, run the function run_idle
      run_idle();
      Serial.println("I am in IDLE State!");
      break;
    case GO: // if you are in state GO, run the function run_go
      run_go();
      Serial.println("I am in GO State!");
      break;
    case STOP: // if you are in state STOP, run the function run_stop
      run_stop();
      Serial.println("I am in STOP State!");
      break;
    case TURNING:
      run_turning();
      Serial.println("I am in TURNING State");
    default: // default case, do nothing and go back
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                             Manual Sate Machine                            */
/* -------------------------------------------------------------------------- */

void manualState()
{

  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);

  

  read_sensors();

  if (buttonA_State == 1)
  {
    MState = AUTO;
  }
  else
  {

    //TODO Add Sonarr Stop

    /** This section of code sets the global variables for the soystick x and y 
 * value. This is accomplished through the analog read function on the x and 
 * y pins. 
 * */
    joystickX = analogRead(joystickyPin);
    joystickY = analogRead(joystickXPin);

    /** This line of code maps the servo position (This is the angle which the servo 
 * will rotate to) to the joystick values. This is accomplished through using the
 * map function. The reason for starting at for the low parameters is that the 
 * joystick x values are inverse if you directly map the joystick's x value to 
 * the angle. This way the angle is correct from the perspective of the user.  */
    servoPosition = map(joystickX, 0, 1023, 0, 180);

    //This Constrains the values so you don't try to set the servo to an unobtainable
    //value
    constrain(servoPosition, 0, 180);

    /** This section of code sets the DC motor power from the joystick Y value after 
 * it accounts for a dead zone on the joystick of values below 500. The reasoning 
 * behind this is the joytick y value range is 0 to 1023. The center point of the 
 * joystick is 500. So by setting the deadzone to make all values below 500 equal
 * to zero the motor only proportionally moves when you push the joystick forward. 
 */
    if (joystickY > 500 && distance >=20)
    {
      //This constrains the DCMotorPower to its usable values
      constrain(DCMotorPower, 0, 255);
      DCMotorPower = map(joystickY, 500, 1023, 0, 100);
    }
    else
    {
      DCMotorPower = 0;
    }

    /** Functions to move the DC Motor and the Servo Motor based on the 
   * above calcuated values */
    analogWrite(DCMotorPin, DCMotorPower);
    myServo.write(servoPosition);

    /** This section of code uses the serial monitor to print off the the x and y 
 * values of the joytick along with the Servo and DC Motor Power variables for 
 * debugging purposes.
  */

 /**
  * Serial.println("X VALUE:");
    Serial.print(joystickX);
    Serial.println("Y VALUE:");
    Serial.print(joystickY);

    Serial.println("Servo Position:");
    Serial.print(servoPosition);
    Serial.println("DC Motor Power:");
    Serial.print(DCMotorPower);
  */
    
  }
}

/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/*                               Other Functions                              */
/* -------------------------------------------------------------------------- */

/** More Funactions!! */

/* ------------------------------ Read Sensors ------------------------------ */

void read_sensors()
{
  buttonA_State = buttonValue();
  //buttonA_State = digitalRead(buttonA); // read the button
  distance = mySonar.Distance();        // read the sonar sensor
  Serial.println(buttonA_State);
  delay(200); // wait 200 ms
}

/* -------------------------------------------------------------------------- */

/* ------------------------------ Idle Function ----------------------------- */

void run_idle()
{
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);

  if (distance > 20 && distance != 0)
  {
    ASTATE = GO;
  }
  else if (distance <= 20 && distance != 0)
  {
    ASTATE = STOP;
  }
  else
  {
    ASTATE = IDLE;
  }
}

/* -------------------------------------------------------------------------- */

/* ------------------------------- Go Function ------------------------------ */

void run_go()
{
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, HIGH);

  if ((distance <= 20) && (distance != 0))
  {
    ASTATE = STOP;
  }
  else
  {
    myServo.write(90);
    delay(1000);
    analogWrite(DCMotorPin, 100);
    

    ASTATE = GO;
  }
}

/* -------------------------------------------------------------------------- */

/* ------------------------------ Stop Function ----------------------------- */

void run_stop()
{
  digitalWrite(redLED, HIGH);
  digitalWrite(yellowLED, LOW);
  digitalWrite(greenLED, LOW);

  analogWrite(DCMotorPin, 0);

  delay(1000);

  ASTATE = TURNING;
}

/* -------------------------------------------------------------------------- */

/* ---------------------------- Turning Function ---------------------------- */

void run_turning()
{
  digitalWrite(redLED, LOW);
  digitalWrite(yellowLED, HIGH);
  digitalWrite(greenLED, LOW);

  myServo.write(180);
  delay(1000);
  analogWrite(DCMotorPin, 100);

  delay(2000);

  ASTATE = IDLE;
}

/* -------------------------------------------------------------------------- */


/* -------------------------- Button Value Function ------------------------- */



int buttonValue()
{
// read the pushbutton input pin:
  buttonState = digitalRead(buttonA);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == 1) {
      
      return 0;
      // if the current state is HIGH then the button went from off to on:
      
    } else {
      // if the current state is LOW then the button went from on to off:
      
      return 1;
      
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;



  

}

/* -------------------------------------------------------------------------- */



