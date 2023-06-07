//Motor conveyor

int IN1 = 16;      // pin digital 8 de Arduino a IN1 de modulo controlador
int IN2 = 17;      // pin digital 9 de Arduino a IN2 de modulo controlador
int IN3 = 18;     // pin digital 10 de Arduino a IN3 de modulo controlador
int IN4 = 19;     // pin digital 11 de Arduino a IN4 de modulo controlador
int demora = 2;      // demora entre pasos, no debe ser menor a 10 ms.
// paso completo simple
int paso [4][4] =   // matriz (array bidimensional) con la secuencia de pasos
{
  {1, 0, 0, 0},
  {0, 1, 0, 0},
  {0, 0, 1, 0},
  {0, 0, 0, 1}
};

int paso2 [4][4] =   // matriz (array bidimensional) con la secuencia de pasos
{
  {0, 0, 0, 1},
  {0, 0, 1, 0},
  {0, 1, 0, 0},
  {1, 0, 0, 0}
};

int ds = 1;

//Step Motors

#include <Arduino.h>
#include "MultiDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 30
#include "A4988.h"

//Motor Nema 1
#define MS1 32
#define MS2 31
#define MS3 30

#define STEP 29
#define DIR 28

A4988 stepper1(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

//Motor Nema 2
#define MS12 8
#define MS22 7
#define MS32 6

#define STEP2 5
#define DIR2 4

A4988 stepper2(MOTOR_STEPS, DIR2, STEP2, MS12, MS22, MS32);

//Motor Nema 3
#define MS13 3
#define MS23 2
#define MS33 1

#define STEP3 34
#define DIR3 14

A4988 stepper3(MOTOR_STEPS, DIR3, STEP3, MS13, MS23, MS33);

MultiDriver controller(stepper1,stepper2,stepper3);

float f = 4.5;

int micro = 4;

int accel = 150;

//FC

int fc_b;
int fc_a;
int fc_f;
int fc_g;
int fc_c;

//ZERO

#define fc_bb 22
#define fc_aa 23
#define fc_ff 24

float z1 = 0;
float z2 = 0;
float z3 = 0;

float pos1 = 0;
float pos2 = 0;
float pos3 = 0;

void setup() {

    Serial.begin(115200);

    //Motor Conveyour

    pinMode(IN1, OUTPUT);   // todos los pines como salida
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    //Speed and acceleration configuration
  
    stepper1.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);
    stepper2.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);
    stepper3.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);

    //FC
    //Zero
    pinMode(fc_bb, INPUT_PULLUP);
    pinMode(fc_aa, INPUT_PULLUP);
    pinMode(fc_ff, INPUT_PULLUP);
    pinMode(25, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);

    //Set velocity
    
    
    stepper1.begin(RPM);
    stepper2.begin(RPM);
    stepper3.begin(RPM);

    //stepper1.enable();
    
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);

    // Set microstep mode 1:1 1:2 1:4 1:8 1:16
    
    stepper1.setMicrostep(micro);  
    stepper2.setMicrostep(micro); 
    stepper3.setMicrostep(micro);

    //Serial.println("START");
    
    //stepper1.startMove(-100 * MOTOR_STEPS * micro);
    
    //stepper1.startRotate(100*360);
    //stepper2.startRotate(100*360);
    //homep();

    //zero();
    
}

/*
void homep(){
  stepper1.startRotate(100*360);
  if(digitalRead(fc_bb)==0){
    stepper1.stop();
    delay(100);
    stepper1.rotate(90*f);
  }
}
*/

void loop() 
{
  
  
  //FC();
  //conveyor();

  
  delay(2000);

  controller.rotate(360.0, 45.5*f, 360.0);
  delay(2000);
  controller.rotate(-360.0, -45.5*f, -360.0); 
  /*

  Serial.print(pos1);
  Serial.print("\t");
  Serial.print(pos2);
  Serial.print("\t");
  Serial.println(pos3);
  
    */
}


void zero()
{
  
  int del = 500;
  //Joint 1
  while(digitalRead(fc_bb) == HIGH)
  {
    z1 = z1 + 0.005;
    stepper1.rotate(-z1*f);
  }
  delay(del);
  stepper1.rotate(45*f);
  pos1 = 45;

  //Joint 2
  while(digitalRead(fc_aa) == HIGH)
  {
    z2 = z2 + 0.005;
    stepper2.rotate(-z2*f);
  }
  delay(del);
  stepper2.rotate(45*f);
  pos2 = 45;
  
  //Joint 3
  while(digitalRead(fc_ff) == HIGH)
  {
    z3 = z3 + 0.005;
    stepper3.rotate(-z3*f);
  }
  delay(del);
  stepper3.rotate(60*f);
  pos3 = 60;
  
}


void conveyor()
{
  for (int i = 0; i < 512; i++) // 512*4 = 2048 pasos
  {
    for (int i = 0; i < 4; i++)   // bucle recorre la matriz de a una fila por vez
    {         // para obtener los valores logicos a aplicar
      if (ds == 1)
      {
        digitalWrite(IN1, paso[i][0]);  // a IN1, IN2, IN3 e IN4
        digitalWrite(IN2, paso[i][1]);
        digitalWrite(IN3, paso[i][2]);
        digitalWrite(IN4, paso[i][3]);
        delay(demora);
      }
      if (ds == 0)
      {
        digitalWrite(IN1, paso2[i][0]);  // a IN1, IN2, IN3 e IN4
        digitalWrite(IN2, paso2[i][1]);
        digitalWrite(IN3, paso2[i][2]);
        digitalWrite(IN4, paso2[i][3]);
        delay(demora);
      }
      
    }
  }

  /*

  digitalWrite(IN1, LOW); // detiene por 5 seg.
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(1000);

  */ 
}

void FC()
{
  fc_b = digitalRead(22);
  fc_a = digitalRead(23);
  fc_f = digitalRead(24);
  fc_g = digitalRead(25);
  fc_c = digitalRead(26);

  Serial.print("b: ");
  Serial.print(fc_b); Serial.print("\t");
  Serial.print("a: ");
  Serial.print(fc_a); Serial.print("\t");
  Serial.print("f: ");
  Serial.print(fc_f); Serial.print("\t");
  Serial.print("g: ");
  Serial.print(fc_g); Serial.print("\t");
  Serial.print("c: ");
  Serial.println(fc_c);
}
