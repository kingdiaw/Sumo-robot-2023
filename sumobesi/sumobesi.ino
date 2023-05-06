/*
  https://github.com/sparkfun/Monster_Moto_Shield/blob/master/Firmware/MonsterMoto_Shield_Example_Sketch/MonsterMoto_Shield_Example_Sketch.ino
  MonsterMoto Shield Example Sketch
  date: 10/7/19
  code by: Jim Lindblom & Dennis Irrgang
  hardware by: Nate Bernstein
  SparkFun Electronics

  This is really simple example code to get you some basic
  functionality with the MonsterMoto Shield. The MonsterMote uses
  two VNH2SP30 high-current full-bridge motor drivers.

  Use the motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
  function to control the motors. Available options are CW, CCW,
  BRAKEVCC, or BRAKEGND. Use motorOff(uint8_t motor) to turn a specific motor off.

  The motor variable in each function should be a Motor enum, alternatively 0 or a 1.
  pwm in the motorGo function should be a value between 0 and 255.

  This code is beerware; if you see the authors (or any SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given. */

#define TRAINING

#define BRAKEVCC 0
#define CW  1
#define CCW 2
#define BRAKEGND 3

#define MOTOR_A 0
#define MOTOR_B 1


const uint8_t PWM_MAX = 255;
#ifdef TRAINING
const uint8_t PWM_HALF = 90;
const uint8_t TURN_DLY = 150;
const uint16_t REV_DLY = 1000;
#else
const uint8_t PWM_HALF = PWM_MAX / 2;
const uint8_t TURN_DLY = 130;
const uint16_t REV_DLY = 800;
#endif
const int currentSensingThreshhold = 100;

/* Voltage controlled input pins with hysteresis, CMOS compatible. These two pins
  control the state of the bridge in normal operation according to the truth table (brake
  to VCC , brake to GND, clockwise and counterclockwise).

  Table 1.    Truth table in normal operating conditions
  +------+-----+------+-----+-------+-------+---------------------+------------------------+
  | INA  | INB | ENA  | ENB | OUTA  | OUTB  |         CS          |          MODE          |
  +------+-----+------+-----+-------+-------+---------------------+------------------------+
  |    1 |   1 |    1 |   1 | H     | H     | High Imp.           | Brake to VCC           |
  |    1 |   0 |    1 |   1 | H     | L     | I_Sense = I_OUT / K | Clockwise (CW)         |
  |    0 |   1 |    1 |   1 | L     | H     | I_SENSE = I_OUT / K | Counterclockwise (CCW) |
  |    0 |   0 |    1 |   1 | L     | L     | High Imp.           | Brake to GND           |
  +------+-----+------+-----+-------+-------+---------------------+------------------------+
  For more information, see the VNH2SP30-E motor driver datasheet */
const int inAPin[2] = {7, 4};
const int inBPin[2] = {8, 9};

/* Voltage controlled input pins with hysteresis, CMOS compatible. Gates of low side
  FETs are modulated by the PWM signal during their ON phase allowing speed
  control of the motors. */
const int pwmPin[2] = {5, 6};

/* When pulled low, disables the half-bridge of the VNH2SP30-E of the motor. In case of
  fault detection (thermal shutdown of a high side FET or excessive ON state voltage drop
  across a low side FET), this pin is pulled low by the device.

  Table 2.    Truth table in fault conditions (detected on OUTA)
  +------+-----+------+-----+-------+-------+------------+
  | INA  | INB | ENA  | ENB | OUTA  | OUTB  |     CS     |
  +------+-----+------+-----+-------+-------+------------+
  | 1    | 1   |    1 |   1 | OPEN  | H     | High Imp.  |
  | 1    | 0   |    1 |   1 | OPEN  | L     | High Imp.  |
  | 0    | 1   |    1 |   1 | OPEN  | H     | I_OUTB / K |
  | 0    | 0   |    1 |   1 | OPEN  | L     | High Imp.  |
  | X    | X   |    0 |   0 | OPEN  | OPEN  | High Imp.  |
  | X    | 1   |    0 |   1 | OPEN  | H     | I_OUTB / K |
  | X    | 0   |    0 |   1 | OPEN  | L     | High Imp.  |
  +------+-----+------+-----+-------+-------+------------+
  For more information, see the VNH2SP30-E motor driver datasheet */
const int enPin[2] = {0, 1};

/* Analog current sense input. This input senses a current proportional to the motor
  current. The information can be read back as an analog voltage. */
const int csPin[2] = {2, 3};

// On-Board LED used as status indication
const int statPin = 13;

#define FL  A2
#define FR  A3

const int TIMEOUT = 1500; //1.5sec
const int Left = 2;
const int Right = 11;
const int Front = 10;
const int Back = 1;
const int Pb1 = 0;
const int Pb2 = 3;
const int Buzzer = 13;

int InpArray[] = {FL, FR, Left, Right, Front, Back, Pb1, Pb2};
int OutArray[] = {Buzzer};

void setup()
{
  // Set input pins as INPUT
  for (byte i = 0; i < sizeof(InpArray) / sizeof(InpArray[0]); i++)
    pinMode(InpArray[i], INPUT);

  // Set output pins as OUTPUT
  for (byte i = 0; i < sizeof(OutArray) / sizeof(OutArray[0]); i++)
    pinMode(OutArray[i], OUTPUT);

  digitalWrite(Buzzer, LOW);
  delay(100);
  // Turn off the buzzer
  digitalWrite(Buzzer, HIGH);
  motorSetup();
}

// Define the Input structure using typedef
typedef struct {
  int StateOld = 0;
  int StateNew = 0;
} Input_t;

typedef struct {
  int detect;
} process_t;

Input_t sLeft;
Input_t sRight;
Input_t sFront;
Input_t sBack;
Input_t sFL;
Input_t sFR;
Input_t sPb1;
Input_t sPb2;
process_t sLine;
process_t sObstacle;

//Global Variable
long time_stamp;

void loop() {
  sPb1.StateNew = digitalRead(Pb1);
  sPb2.StateNew = digitalRead (Pb2);
  if ( sPb2.StateNew == LOW)
    progA();
  else if (sPb1.StateNew == LOW)
    progB();
}

//====================================
//user define function
//====================================
void progA(void) {
  while (true) {
    sLeft.StateNew = digitalRead(Left);
    sRight.StateNew = digitalRead(Right);
    sFront.StateNew = digitalRead(Front);
    sBack.StateNew = digitalRead(Back);
    sFL.StateNew = digitalRead(FL);
    sFR.StateNew = digitalRead(FR);

    sLine.detect = (sFL.StateNew) || (sFR.StateNew);
    sObstacle.detect = !(sLeft.StateNew && sRight.StateNew && sFront.StateNew && sBack.StateNew);

    if (sLine.detect) {
      reverse();
      delay(REV_DLY);
      turn();
      delay(TURN_DLY);
    }
    else {
      if (sObstacle.detect) {
        time_stamp = millis();
        if (sFront.StateNew == LOW) {
          //Kejar depan
          forward_max();
        }
        else if (sBack.StateNew == LOW) {
          // turn 180
          turn180();
          beep();
        }
        else if (sLeft.StateNew == LOW) {
          //Turn Left dan kejar
          turn_left_fight();
          beep();
        }
        else if (sRight.StateNew == LOW) {
          //Turn Right dan kejar
          turn_right_fight();
          beep();
        }
      }
      else {
        forward_half();
      }
    }
  }
}

void progB(void) {
  forward_max();
  beep();
  if (digitalRead(FL) || digitalRead(FR))
    progA();
}

void beep() {
  digitalWrite(Buzzer, LOW);
  delay(100);
  digitalWrite(Buzzer, HIGH);
}
void reverse(void) {
  motorGo(MOTOR_A, CCW, PWM_HALF);
  motorGo(MOTOR_B, CW, PWM_HALF);
}

void forward_half(void) {
  motorGo(MOTOR_A, CW, PWM_HALF); //kanan
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri
}

void forward_max(void) {
  motorGo(MOTOR_A, CW, PWM_MAX);
  motorGo(MOTOR_B, CCW, PWM_MAX);
}

void turn(void) {
  motorGo(MOTOR_A, CCW, PWM_HALF); //kanan-reverse
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri-forward
}

void turn180(void) {
  motorGo(MOTOR_A, CCW, PWM_HALF); //kanan-reverse
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri-forward
  while ((digitalRead(Front)) && ((millis() - time_stamp) < TIMEOUT));
  motorGo(MOTOR_A, CW, PWM_HALF); //kanan-forward
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri-forward
}

void turn_left_fight(void) {
  motorGo(MOTOR_A, CW, PWM_HALF); //kanan-forward
  motorGo(MOTOR_B, CW, PWM_HALF);  //kiri-reverse
  while ((digitalRead(Front)) && ((millis() - time_stamp) < TIMEOUT));
  motorGo(MOTOR_A, CW, PWM_HALF); //kanan-forward
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri-forward
}

void turn_right_fight(void) {
  motorGo(MOTOR_A, CCW, PWM_HALF); //kanan-reverse
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri-forward
  while ((digitalRead(Front)) && ((millis() - time_stamp) < TIMEOUT));
  motorGo(MOTOR_A, CW, PWM_HALF); //kanan-forward
  motorGo(MOTOR_B, CCW, PWM_HALF);  //kiri-forward
}
void motorSetup()
{
  pinMode(statPin, OUTPUT);

  // Initialize digital pins as outputs
  for (int i = 0; i < 2; i++)
  {
    pinMode(inAPin[i], OUTPUT);
    pinMode(inBPin[i], OUTPUT);
    pinMode(pwmPin[i], OUTPUT);
  }
  // Initialize with brake applied
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inAPin[i], LOW);
    digitalWrite(inBPin[i], LOW);
  }
}

void motorOff(uint8_t motor)
{
  // Initialize brake to Vcc
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inAPin[i], LOW);
    digitalWrite(inBPin[i], LOW);
  }
  analogWrite(pwmPin[motor], 0);
}

/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either MOTOR_A (0) or MOTOR_B (1), will select
  which of the two motors to be controlled. Invalid values are ignored.

  mode: Should be one of the following values
  BRAKEVCC (0): Brake to VCC
  CW (1): Turn Clockwise
  CCW (2): Turn Counter-Clockwise
  BRAKEGND (3): Brake to GND

  speed: should be a value between 0 and PWM_MAX (255), higher the number, the faster
*/
void motorGo(uint8_t motor, uint8_t mode, uint8_t speed)
{

  if (motor == MOTOR_A || motor == MOTOR_B)
  {
    switch (mode)
    {
      case BRAKEVCC: // Brake to VCC
        digitalWrite(inAPin[motor], HIGH);
        digitalWrite(inBPin[motor], HIGH);
        break;
      case CW: // Turn Clockwise
        digitalWrite(inAPin[motor], HIGH);
        digitalWrite(inBPin[motor], LOW);
        break;
      case CCW: // Turn Counter-Clockwise
        digitalWrite(inAPin[motor], LOW);
        digitalWrite(inBPin[motor], HIGH);
        break;
      case BRAKEGND: // Brake to GND
        digitalWrite(inAPin[motor], LOW);
        digitalWrite(inBPin[motor], LOW);
        break;

      default:
        // Invalid mode does not change the PWM signal
        return;
    }
    analogWrite(pwmPin[motor], speed);
  }
  return;
}
