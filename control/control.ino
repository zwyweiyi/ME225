// control 5 motors
// Weiyi Zhang
// Before running the code, check all places marked with [!] .

// include Ethernet for UDP communication
#include <Ethernet.h>
// Stepper lib
#include <Stepper.h>
// For the ATOMIC_BLOCK macro
#include <util/atomic.h> 

// network setup
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x6A, 0x29};
IPAddress ip (192, 168, 200,3);
unsigned int localPort = 8888;
byte remoteip [4] = {192, 168, 200,2};

EthernetUDP Udp; //An EthernetUDP instance to let us send and receive packets over UDP
unsigned char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incomings

///////////// Stepper 1///////////////
// Number of steps per output rotation
const int stepsPerRevolution1 = 200; // [!] steps per revolution Gearbox 5.18:1
double resolution1 = 1.8; // [!] resolution of the motor, in deg
// Create Instance of Stepper library
Stepper myStepper1(stepsPerRevolution1, 22, 23, 24, 25); // [!] remeber to change the pins!

///////////// Stepper 2 /////////////
// Number of steps per output rotation
const int stepsPerRevolution2 = 200; // [!] steps per reolution
double resolution2 = 1.8; // [!] resolution of the motor, in deg
// Create Instance of Stepper library
Stepper myStepper2(stepsPerRevolution2, 26, 27, 28, 29); // [!] remeber to change the pins!

///////////// DC 1 //////////////////
#define dc1_encoderPinA 2 // input: connect to PinA of encoder [!] interrupt
#define dc1_encoderPinB 42 // input: connect to PinB of encoder [!]
#define dc1_PWM         7 // output: PWM
#define dc1_M1          32 // output: motor PWM: L298N in1 [!]
#define dc1_M2          33 // output: motor PWM: L298N in2 [!]

volatile int dc1_posi = 0; // specify position as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long dc1_prevT = 0;
float dc1_eprev = 0;
float dc1_eintegral = 0;

///////////// DC 2 /////////////////（）
#define dc2_encoderPinA 3 // input: connect to PinA of encoder [!] interrupt
#define dc2_encoderPinB 43 // input: connect to PinB of encoder [!]
#define dc2_PWM         8 // output: PWM
#define dc2_M1          34 // output: motor PWM: L298N in1 [!]
#define dc2_M2          35 // output: motor PWM: L298N in2 [!]

volatile int dc2_posi = 0; // specify position as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long dc2_prevT = 0;
float dc2_eprev = 0;
float dc2_eintegral = 0;



///////////// DC 3 /////////////////
#define dc3_encoderPinA 18 // input: connect to PinA of encoder [!] interrupt
#define dc3_encoderPinB 44 // input: connect to PinB of encoder [!]
#define dc3_PWM         9 // output: PWM
#define dc3_M1          36 // output: motor PWM: L298N in1 [!]
#define dc3_M2          37 // output: motor PWM: L298N in2 [!]

volatile int dc3_posi = 0; // specify position as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long dc3_prevT = 0;
float dc3_eprev = 0;
float dc3_eintegral = 0;

///////////// DC 4 /////////////////
#define dc4_encoderPinA 20 // input: connect to PinA of encoder [!] interrupt
#define dc4_encoderPinB 45 // input: connect to PinB of encoder [!]
#define dc4_PWM         11 // output: PWM
#define dc4_M1          46 // output: motor PWM: L298N in1 [!]
#define dc4_M2          47 // output: motor PWM: L298N in2 [!]

volatile int dc4_posi = 0; // specify position as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long dc4_prevT = 0;
float dc4_eprev = 0;
float dc4_eintegral = 0;

int angle_stepper1=0, angle_stepper2=0, angle_dc1=0, angle_dc2=0, angle_dc3=0, angle_dc4=0;
int angle_stepper1_current = 0, angle_stepper2_current=0, angle_dc1_current=0, angle_dc2_current=0, angle_dc3_current=0, angle_dc4_current=0;


void setup() 
{
  // Setup network
  Ethernet.begin(mac, ip ); // start the Ethernet and UDP
  Udp.begin( localPort );// launch UDP listening on the localPort
  delay(10); // for switching delay
  Serial.begin(9600); // initialize the serial port:
  Serial.print("Hello Robot\n");

  // Stepper1
  // set the speed at 100 rpm:
  myStepper1.setSpeed(13);
  //myStepper1.step(-1);


  // Stepper2
  // set the speed at 20 rpm:
  myStepper2.setSpeed(15);

  // DC 1
  pinMode(dc1_encoderPinA, INPUT);
  pinMode(dc1_encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(dc1_encoderPinA), dc1_readEncoder, RISING);
  pinMode(dc1_PWM, OUTPUT);
  pinMode(dc1_M1, OUTPUT);
  pinMode(dc1_M2, OUTPUT);
//  

  // DC 2
  pinMode(dc2_encoderPinA, INPUT);
  pinMode(dc2_encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(dc2_encoderPinA), dc2_readEncoder, RISING);
  pinMode(dc2_PWM, OUTPUT);
  pinMode(dc2_M1, OUTPUT);
  pinMode(dc2_M2, OUTPUT);


//   DC 3
  pinMode(dc3_encoderPinA, INPUT);
  pinMode(dc3_encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(dc3_encoderPinA), dc3_readEncoder, RISING);
  pinMode(dc3_PWM, OUTPUT);
  pinMode(dc3_M1, OUTPUT);
  pinMode(dc3_M2, OUTPUT);

  // DC 4
  pinMode(dc4_encoderPinA, INPUT);
  pinMode(dc4_encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(dc4_encoderPinA), dc4_readEncoder, RISING);
  pinMode(dc4_PWM, OUTPUT);
  pinMode(dc4_M1, OUTPUT);
  pinMode(dc4_M2, OUTPUT);
}


void loop() 
{
  int packetSize = Udp.parsePacket();// define the packetSize for communication
  if (packetSize)// if packetSize is nonezero, i.e., successfully defined
  {
    // read angle from UDP
    Udp.read(packetBuffer, 6);
    angle_stepper1 = ((int)packetBuffer[0])*8;
    
    angle_stepper2 = -((int)packetBuffer[1])*2;
    
    angle_dc1 = ((int)packetBuffer[2])*3;
    
    angle_dc2 = -((int)packetBuffer[3]);
    
    angle_dc3 = ((int)packetBuffer[4])*5;
    
    angle_dc4 = ((int)packetBuffer[5])*7;
  }
    // Stepper1
    // step some angle in one direction:
    int step1 = angle_stepper1 - angle_stepper1_current;
    if (step1 != 0)
    {
//      Serial.print("stepper_1 moving to ");
//      Serial.println(angle_stepper1);
      myStepper1.step(step1); 
      angle_stepper1_current += step1;
    }

    // Stepper2
    // step some angle in one direction:
    int step2 = angle_stepper2 - angle_stepper2_current;
    if (step2 != 0)
    {
//      Serial.print("stepper_2 moving to ");
//      Serial.println(angle_stepper2);
      myStepper2.step(step2); 
      angle_stepper2_current += step2;
    }

  // DC 1
  int dc1_target = angle_dc1;

  // PID constants
  float dc1_kp = 1;
  float dc1_kd = 0.025;
  float dc1_ki = 0.0;

  // time difference
  long dc1_currT = micros();
  float dc1_deltaT = ((float) (dc1_currT - dc1_prevT))/( 1.0e6 );
  dc1_prevT = dc1_currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int dc1_pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    dc1_pos = dc1_posi;
  }
  
  // error
  int dc1_e = dc1_pos - dc1_target;

  // derivative
  float dc1_dedt = (dc1_e-dc1_eprev)/(dc1_deltaT);

  // integral
  dc1_eintegral = dc1_eintegral + dc1_e*dc1_deltaT;

  // control signal
  float dc1_u = dc1_kp*dc1_e + dc1_kd*dc1_dedt + dc1_ki*dc1_eintegral;

  // motor power
  float dc1_pwr = fabs(dc1_u);
  if( dc1_pwr > 120 ){
    dc1_pwr = 120;
  }

  // motor direction
  int dc1_dir = 1;
  if(dc1_u<0){
    dc1_dir = -1;
  }

  // signal the motor
  setMotor(dc1_dir,dc1_pwr,dc1_PWM,dc1_M1,dc1_M2);


  // store previous error
  dc1_eprev = dc1_e;

//  Serial.print(dc1_target);
//  Serial.print(" ");
//  Serial.print(dc1_pos);
//  Serial.println();

  // DC 2
  int dc2_target = angle_dc2;

  // PID constants
  float dc2_kp = 1;
  float dc2_kd = 0.025;
  float dc2_ki = 0.0;

  // time difference
  long dc2_currT = micros();
  float dc2_deltaT = ((float) (dc2_currT - dc2_prevT))/( 1.0e6 );
  dc2_prevT = dc2_currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int dc2_pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    dc2_pos = dc2_posi;
  }
  
  // error
  int dc2_e = dc2_pos - dc2_target;

  // derivative
  float dc2_dedt = (dc2_e-dc2_eprev)/(dc2_deltaT);

  // integral
  dc2_eintegral = dc2_eintegral + dc2_e*dc2_deltaT;

  // control signal
  float dc2_u = dc2_kp*dc2_e + dc2_kd*dc2_dedt + dc2_ki*dc2_eintegral;

  // motor power
  float dc2_pwr = fabs(dc2_u);
  if( dc2_pwr > 255 ){
    dc2_pwr = 255;
  }

  // motor direction
  int dc2_dir = 1;
  if(dc2_u<0){
    dc2_dir = -1;
  }

  // signal the motor
  setMotor(dc2_dir,dc2_pwr,dc2_PWM,dc2_M1,dc2_M2);


  // store previous error
  dc2_eprev = dc2_e;

//  Serial.print(dc2_target);
//  Serial.print(" ");
//  Serial.print(dc2_pos);
//  Serial.println();

   //DC 3
  int dc3_target = angle_dc3;

  // PID constants
  float dc3_kp = 1;
  float dc3_kd = 0.025;
  float dc3_ki = 0.0;

  // time difference
  long dc3_currT = micros();
  float dc3_deltaT = ((float) (dc3_currT - dc3_prevT))/( 1.0e6 );
  dc3_prevT = dc3_currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int dc3_pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    dc3_pos = dc3_posi;
  }
  
  // error
  int dc3_e = dc3_pos - dc3_target;

  // derivative
  float dc3_dedt = (dc3_e-dc3_eprev)/(dc3_deltaT);

  // integral
  dc3_eintegral = dc3_eintegral + dc3_e*dc3_deltaT;

  // control signal
  float dc3_u = dc3_kp*dc3_e + dc3_kd*dc3_dedt + dc3_ki*dc3_eintegral;

  // motor power
  float dc3_pwr = fabs(dc3_u);
  if( dc3_pwr > 255 ){
    dc3_pwr = 255;
  }

  // motor direction
  int dc3_dir = 1;
  if(dc3_u<0){
    dc3_dir = -1;
  }

  // signal the motor
  setMotor(dc3_dir,dc3_pwr,dc3_PWM,dc3_M1,dc3_M2);

  // store previous error
  dc3_eprev = dc3_e;

  Serial.print(dc3_target);
  Serial.print(" ");
  Serial.print(dc3_pos);
  Serial.println();

  // DC 4
  int dc4_target = angle_dc4;

  // PID constants
  float dc4_kp = 1;
  float dc4_kd = 0.025;
  float dc4_ki = 0.0;

  // time difference
  long dc4_currT = micros();
  float dc4_deltaT = ((float) (dc4_currT - dc4_prevT))/( 1.0e6 );
  dc4_prevT = dc4_currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int dc4_pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    dc4_pos = dc4_posi;
  }
  
  // error
  int dc4_e = dc4_pos - dc4_target;

  // derivative
  float dc4_dedt = (dc4_e-dc4_eprev)/(dc4_deltaT);

  // integral
  dc4_eintegral = dc4_eintegral + dc4_e*dc4_deltaT;

  // control signal
  float dc4_u = dc4_kp*dc4_e + dc4_kd*dc4_dedt + dc4_ki*dc4_eintegral;

  // motor power
  float dc4_pwr = fabs(dc4_u);
  if( dc4_pwr > 255 ){
    dc4_pwr = 255;
  }

  // motor direction
  int dc4_dir = 1;
  if(dc4_u<0){
    dc4_dir = -1;
  }

  // signal the motor
  setMotor(dc4_dir,dc4_pwr,dc4_PWM,dc4_M1,dc4_M2);


  // store previous error
  dc4_eprev = dc4_e;
//
//  Serial.print(dc4_target);
//  Serial.print(" ");
//  Serial.print(dc4_pos);
//  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
}
}

void dc1_readEncoder()
{
  int b = digitalRead(dc1_encoderPinB);
  if(b > 0)
  {
     dc1_posi++;
  }
  else
  {
     dc1_posi--;
  }
}

void dc2_readEncoder()
{
  int b = digitalRead(dc2_encoderPinB);
  if(b > 0)
  {
     dc2_posi++;
  }
  else
  {
     dc2_posi--;
  }
}

void dc3_readEncoder()
{
  int b = digitalRead(dc3_encoderPinB);
  if(b > 0)
  {
     dc3_posi++;
  }
  else
  {
     dc3_posi--;
  }
}

void dc4_readEncoder()
{
  int b = digitalRead(dc4_encoderPinB);
  if(b > 0)
  {
     dc4_posi++;
  }
  else
  {
     dc4_posi--;
  }
}
