// ########################## DEFINES ##########################
// Hoverboard params
#define HOVER_SERIAL_BAUD   115200      // Baud rate for HoverSerial (Serial1)
#define START_FRAME         0xABCD      // Start frame definition
#define TIME_SEND           100         // [ms] Sending time interval

// Actuator directions
#define STOP 0
#define UP   1
#define DOWN 2

// DEBUG_LEVEL
// -1 = no debug
// 0  = system information
// 1  = all info (received command from APP, all received values from front hoverboard)
#define DEBUG_LEVEL 0
#if (DEBUG_LEVEL != -1)
  #define SERIAL_BAUD 115200  // Baud rate for USB debug
#endif

// Global variables
uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;


int CurrSens = 2048;
int fil = 0;
int currentTime = 0;
int lastTime = 0;
int liftState = 0;
bool unlockMotor = 0;
bool motor_running = 0;

// ########################## SETUP ##########################
void setup() 
{
  if (DEBUG_LEVEL != -1) Serial.begin(SERIAL_BAUD);
  Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1); // Initialize Serial1 for hoverboard
  Serial2.begin(9600);    // Serial to communicate with HM-10 module

  pinMode(PC13, OUTPUT);  // Indicator LED pin

  pinMode(PB8, OUTPUT);   // UP directon pin
  pinMode(PB9, OUTPUT);   // DOWN direction pin
  pinMode(PA0, INPUT);    // Ammeter pin
  digitalWrite(PB8, 1);   // Actuator pins are inverted
  digitalWrite(PB9, 1);   // Actuator pins are inverted
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  Serial1.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
    if (Serial1.available()) {
        incomingByte     = Serial1.read();
        bufStartFrame    = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;
    } else {
        return;
    }

    // Copy received data
    if (bufStartFrame == START_FRAME) {
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;  
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
        *p++    = incomingByte; 
        idx++;
    }   
    
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
            
            switch (DEBUG_LEVEL) {
              case 0:
                Serial.print(Feedback.batVoltage);
                Serial.print("\t");
                Serial.println((Feedback.speedL_meas + Feedback.speedR_meas)/2);
              case 1:
                Serial.print(NewFeedback.start);
                Serial.print("\t");
                Serial.print(NewFeedback.cmd1);
                Serial.print("\t");
                Serial.print(NewFeedback.cmd2);
                Serial.print("\t");
                Serial.print(NewFeedback.speedR_meas);
                Serial.print("\t");
                Serial.print(NewFeedback.speedL_meas);
                Serial.print("\t");
                Serial.print(NewFeedback.batVoltage);
                Serial.print("\t");
                Serial.print(NewFeedback.boardTemp);
                Serial.print("\t");
                Serial.println(NewFeedback.cmdLed);
            }
        }
        idx = 0;
    }

    incomingBytePrev = incomingByte;
}

void controlMotors(int leftSpeed, int rightSpeed) {
  int speed = (leftSpeed + rightSpeed) / 2;
  int steer = rightSpeed - leftSpeed;
  
  Send(steer, speed);
}

void runLift(){
  if (unlockMotor== 0) {
    lastTime = millis();
  } else {
      currentTime = millis();
      if (currentTime - lastTime >= 2000) unlockMotor = 0;
    }

  if (liftState == STOP){
    digitalWrite(PB8, 1);
    digitalWrite(PB9, 1);
    unlockMotor= 0;
  } else if (liftState == UP){
    if (unlockMotor == 1 || motor_running == 1){
      digitalWrite(PB8, 0);
      digitalWrite(PB9, 1);
    } else {
      unlockMotor = 0;
      liftState = 0;
      digitalWrite(PB8, 1);
      digitalWrite(PB9, 1);
    }
  } else if (liftState == DOWN){
    if (unlockMotor == 1 || motor_running == 1){
      digitalWrite(PB8, 1);
      digitalWrite(PB9, 0);
    } else {
      unlockMotor = 0;
      liftState = 0;
      digitalWrite(PB8, 1);
      digitalWrite(PB9, 1);
    }
  }
}

unsigned long iTimeSend = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int leftMS = 0;
int rightMS = 0;

void loop(void)
{ 
  unsigned long timeNow = millis();

  Receive();
  CurrSens = analogRead(PA0);
  fil += (CurrSens - fil) >> 4;
  motor_running = (fil < 1540);
  if (Serial2.available() > 0) {
    String input = Serial2.readStringUntil('\n');
    input.trim();
    
    int left, right, b;
    
    int result = sscanf(input.c_str(), "%d:%d:%d", &left, &right, &b);
    
    if (result == 3) {
      rightMS = right;
      leftMS = left;
      if (b == 1){
        unlockMotor = 1;
        liftState = UP;
      } else if (b == -1){
        unlockMotor = 1;
        liftState = DOWN;
      } else if (b == 0){
        unlockMotor = 0;
        liftState = STOP;
      }
    } else {
      Serial2.println("Error: Invalid format. Use: left:right:b");
    }
  }

  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  
  controlMotors(rightMS, leftMS);
  runLift();

  digitalWrite(PC13, (timeNow % 1000) < 500); // Indicator
}
