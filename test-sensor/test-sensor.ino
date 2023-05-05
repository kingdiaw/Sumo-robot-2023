//Test code: Sensor sumobesi
/*
  Line sensor:
  Line Front Left -> FL -> A0
  Line Front Right -> FR -> A1
  Obstacle sensor:
  Left -> 2
  Right -> 11
  Front -> 10
  Back -> 1
  Switch Remote:
  PB1 -> 0
  PB2 -> 3
  Buzzer:
  Buzzer -> 13 (active low)
*/
#define FL  A0
#define FR  A1
const int Left = 2;
const int Right = 11;
const int Front = 10;
const int Back = 1;
const int Pb1 = 0;
const int Pb2 = 3;
const int Buzzer = 13;

int InpArray[] = {FL, FR, Left, Right, Front, Back, Pb1, Pb2};
int OutArray[] = {Buzzer};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(10);
  // Set input pins as INPUT
  for (byte i = 0; i < sizeof(InpArray) / sizeof(InpArray[0]); i++)
    pinMode(InpArray[i], INPUT);

  // Set output pins as OUTPUT
  for (byte i = 0; i < sizeof(OutArray) / sizeof(OutArray[0]); i++)
    pinMode(OutArray[i], OUTPUT);

  // Turn off the buzzer
  digitalWrite(Buzzer, HIGH);
  while (digitalRead(Pb2) == HIGH);
}

// Define the Input structure using typedef
typedef struct {
  int StateOld = 0;
  int StateNew = 0;
} Input_t;

Input_t sLeft;
Input_t sRight;
Input_t sFront;
Input_t sBack;
Input_t sFL;
Input_t sFR;

void loop() {

  sLeft.StateNew = digitalRead(Left);
  sRight.StateNew = digitalRead(Right);
  sFront.StateNew = digitalRead(Front);
  sBack.StateNew = digitalRead(Back);
  sFL.StateNew = digitalRead(FL);
  sFR.StateNew = digitalRead(FR);

  if (sLeft.StateOld != sLeft.StateNew) {
    Serial.print("Sensor Left:");
    Serial.println(sLeft.StateNew);
    sLeft.StateOld = sLeft.StateNew;
  }
  if (sRight.StateOld != sRight.StateNew) {
    Serial.print("Sensor Right:");
    Serial.println(sRight.StateNew);
    sRight.StateOld = sRight.StateNew;
  }
  if (sFront.StateOld != sFront.StateNew) {
    Serial.print("Sensor Front:");
    Serial.println(sFront.StateNew);
    sFront.StateOld = sFront.StateNew;
  }
  /*
    if (sBack.StateOld != sBack.StateNew) {
    Serial.print("Sensor Back:");
    Serial.println(sBack.StateNew);
    sBack.StateOld = sBack.StateNew;
    }
  */
  if (sFL.StateOld != sFL.StateNew) {
    Serial.print("Line Left:");
    Serial.println(sFL.StateNew);
    sFL.StateOld = sFL.StateNew;
  }
  if (sFR.StateOld != sFR.StateNew) {
    Serial.print("Line Right:");
    Serial.println(sFR.StateNew);
    sFR.StateOld = sFR.StateNew;
  }
}
