#include <IRremote.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Arduino.h>
#include "MultiDriver.h"
#include <Stepper.h>
#include <Servo.h>

#define PI 3.141592654

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
#define RPM 30
#include "A4988.h"

//Motor Nema 1
#define MS1 13
#define MS2 12
#define MS3 11

#define STEP 10
#define DIR 9

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

#define ena3 16

A4988 stepper3(MOTOR_STEPS, DIR3, STEP3, MS13, MS23, MS33);

//Motor Nema 4 Conveyor
#define MS14 32
#define MS24 31
#define MS34 30

#define STEP4 29
#define DIR4 28


A4988 stepper4(MOTOR_STEPS, DIR4, STEP4, MS14, MS24, MS34);

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
#define fc_gg 25
#define fc_cc 26

float z1 = 0;
float z2 = 0;
float z3 = 0;

float pos1 = 0;
float pos2 = 0;
float pos3 = 0;

int choco_counter = 1;
int type = 1;
double paso_dec = 0.01;
int paso_coord = 1;
int set = 0;
int warning_geo = 0;
int paso_conveyor = 1020;

//Conveyor Motor
const int steps_Conveyor = 2048;
Stepper conveyor = Stepper(steps_Conveyor,52,51,50,49);

//Gripper Servo
Servo gripper;
int gripper_const = 2;

//Forward Kinematics
double pos_x, pos_y, pos_z = 0.0;
double sigma_1, sigma_2, sigma_3, sigma_4, sigma_5 = 0.0;
double theta_1, theta_2, theta_3, theta_4 = 0.0;
double L2 = 120;
double L3 = 120;
double x = 58.49;
double y = -28.45;

//Inverse Kinematics by Geometry
double x_input, y_input, z_input, x_inv, y_inv, z_inv, z_1 = 0.0;
int mt, ve = 0;
double q1, q2, q3, q30, q4 = 0.0;
double h_p, a1_1, h_1;
double alpha, beta, ang1_inv = 0.0;
int c, j, er = 0;


const int RECV_PIN = 27;
IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long key_value = 0;
int menu_counter = 0;
int var_menu = 0;
int selection = 0;

double ang1_input = 0;
double ang2_input = 0;
double ang3_input = 0;
double ang1_output = 0;
double ang2_output = 0;
double ang3_output = 0;
double ang1_actual = 0;
double ang2_actual = 0;
double ang3_actual = 0;
double ang2_muestra = 0;
double ang3_muestra = 0;
int paso_input = 1;

int counter_easter = 0;

LiquidCrystal_I2C lcd(0x27,20,4);

void setup(){
  Serial.begin(9600);
  irrecv.enableIRIn();
  irrecv.blink13(true);


  //pinMode(22, INPUT_PULLUP);
  pinMode(fc_b, INPUT_PULLUP);
  pinMode(fc_a, INPUT_PULLUP);
  pinMode(fc_f, INPUT_PULLUP);
  pinMode(fc_g, INPUT_PULLUP);
  pinMode(fc_c, INPUT_PULLUP);

  //Enable motor 3

  pinMode(ena3, OUTPUT);

  //Set velocity
  stepper1.begin(RPM);
  stepper2.begin(RPM);
  stepper3.begin(RPM);
  stepper4.begin(RPM);

  stepper1.setMicrostep(micro);  
  stepper2.setMicrostep(micro); 
  stepper3.setMicrostep(micro);
  stepper4.setMicrostep(micro);

  //Gripper
  gripper.attach(15);

  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,1);
  lcd.print("Choco-bot v7.4");
  lcd.setCursor(5,2);
  lcd.print("Loading...");

  //Homing8
  zero();
  
  delay(800);
  controller.rotate(90*f, 60*f, 60*f);
  stepper4.rotate(paso_conveyor);
  delay(800);
  controller.rotate(-90*f, -60*f, -60*f);
  
  lcd.clear();
  menu_1();
  set = 1;
}

void loop(){
  
  button_recognition();
}

void zero(){

  //Speed and Acceleration Configuration
  stepper1.setSpeedProfile(A4988::CONSTANT_SPEED);
  stepper2.setSpeedProfile(A4988::CONSTANT_SPEED);
  stepper3.setSpeedProfile(A4988::CONSTANT_SPEED);
  stepper4.setSpeedProfile(A4988::CONSTANT_SPEED);
  
  int del = 500;

  //Joint 2

  digitalWrite(ena3, HIGH);
  
  while(digitalRead(fc_aa) == HIGH){
    z2 = z2 + 0.005;
    stepper2.move(-z2*f);
  }
  delay(del);
  pos2 = 45;
  stepper2.rotate(pos2*f);
  ang2_input = pos2;
  ang2_muestra = 90;

  digitalWrite(ena3, LOW);

  //Angulo inicial 90°
  
  //Joint 1
  while(digitalRead(fc_bb) == HIGH){
    z1 = z1 + 0.005;
    stepper1.move(-z1*f);
  }
  delay(del);
  pos1 = 90;
  stepper1.rotate(pos1*f);
  ang1_input = pos1;

  //Angulo inicial 90°
  
  //Joint 3
  while(digitalRead(fc_ff) == HIGH){
    z3 = z3 + 0.005;
    stepper3.move(-z3*f);
  }
  delay(del);
  stepper3.rotate(50*f);
  pos3 = -25;
  stepper3.rotate(pos3*f);
  ang3_input = 50 + pos3;
  ang3_muestra = 90;

  //Angulo inicial 90°

  ang1_output = ang1_input - ang1_actual;
  ang1_actual = ang1_input;
  ang2_output = ang2_input - ang2_actual;
  ang2_actual = ang2_input;
  ang3_output = ang3_input - ang3_actual;
  ang3_actual = ang3_input;
   
  while(digitalRead(fc_gg) == HIGH){
    gripper.write(0);
  }
  gripper.write(90);
  gripper_const = 1;
  delay(1000);

  x_input = 0.0;
  y_input = 178.49;
  z_input = 91.55;

  //Speed and Acceleration Configuration
  stepper1.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);
  stepper2.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);
  stepper3.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);
  stepper4.setSpeedProfile(A4988::LINEAR_SPEED, accel, accel);

  
}

void menu_1(){
  lcd.setCursor(0,0);
  lcd.print("Mode Selection:");
  lcd.setCursor(0,2);
  lcd.print("1) Automatic Mode");
  lcd.setCursor(0,3);
  lcd.print("2) Manual Mode");
  menu_counter = 1;
  var_menu = 0;
}

void menu_auto(){
  lcd.setCursor(0,0);
  lcd.print("Automatic Mode");
  lcd.setCursor(0,1);
  lcd.print("1) Start Routine");
  lcd.setCursor(0,2);
  lcd.print("2) Inventory");
  lcd.setCursor(0,3);
  lcd.print("3) Auto Home");
  menu_counter = 2;
  var_menu = 1;
}

void menu_manual(){
  lcd.setCursor(0,0);
  lcd.print("Manual Mode");
  lcd.setCursor(0,1);
  lcd.print("1) Movement Control");
  lcd.setCursor(0,2);
  lcd.print("2) Gripper Control");
  lcd.setCursor(0,3);
  lcd.print("3) Conveyor Control");
  menu_counter = 2;
  var_menu = 2;
}

void menu_motorsforw(){
  lcd.setCursor(0,0);
  lcd.print("1) Base Angle");
  lcd.setCursor(17,0);
  lcd.print(int(ang1_input));
  lcd.setCursor(0,1);
  lcd.print("2) Arm Angle");
  lcd.setCursor(17,1);
  lcd.print(int(ang2_muestra));
  lcd.setCursor(0,2);
  lcd.print("3) Forearm Angle");
  lcd.setCursor(17,2);
  lcd.print(int(ang3_muestra));
  lcd.setCursor(0,3);
  lcd.print("4) Start Movement");
  menu_counter = 3;
  var_menu = 1;
}

void menu_motorsinv(){
  lcd.setCursor(0,0);
  lcd.print("1) Coor. X");
  lcd.setCursor(12,0);
  lcd.print(x_input);
  lcd.setCursor(0,1);
  lcd.print("2) Coor. Y");
  lcd.setCursor(12,1);
  lcd.print(y_input);
  lcd.setCursor(0,2);
  lcd.print("3) Coor. Z");
  lcd.setCursor(12,2);
  lcd.print(z_input);
  lcd.setCursor(0,3);
  lcd.print("4) Start Movement");
  menu_counter = 3;
  var_menu = 1;
}

void menu_infokinematics(){
  lcd.setCursor(0,0);
  lcd.print("Forward Kinematics");
  lcd.setCursor(0,1);
  lcd.print("Pos. X: ");
  lcd.setCursor(8,1);
  lcd.print(pos_x);
  lcd.setCursor(16,1);
  lcd.print(int(ang1_actual));
  lcd.setCursor(0,2);
  lcd.print("Pos. Y: ");
  lcd.setCursor(8,2);
  lcd.print(pos_y);
  lcd.setCursor(16,2);
  lcd.print(int(ang2_muestra));
  lcd.setCursor(0,3);
  lcd.print("Pos. Z: ");
  lcd.setCursor(8,3);
  lcd.print(pos_z);
  lcd.setCursor(16,3);
  lcd.print(int(ang3_muestra));
  menu_counter = 4;
  var_menu = 5;
}

void menu_gripper(){
  lcd.setCursor(0,0);
  lcd.print("Gripper Control");
  lcd.setCursor(0,2);
  lcd.print("1) Open Gripper");
  lcd.setCursor(0,3);
  lcd.print("2) Close Gripper");
  menu_counter = 3;
  var_menu = 2;
}

void menu_conveyor(){
  lcd.setCursor(0,0);
  lcd.print("Conveyor Control");
  lcd.setCursor(0,1);
  lcd.print("1) Move Forward");
  lcd.setCursor(0,2);
  lcd.print("2) Move Backwards");
  lcd.setCursor(0,3);
  lcd.print("3) Stop Movement");
  menu_counter = 3;
  var_menu = 3;
}

void announcement(){
  lcd.setCursor(6,0);
  lcd.print("Warning!");
  lcd.setCursor(4,2);
  lcd.print("This Point Is");
  lcd.setCursor(3,3);
  lcd.print("Not Reachable!");
  delay(3000);
  warning_geo = 0;
  var_menu = 1;
}

void easter_egg(){
  lcd.setCursor(1,0);
  lcd.print("Phrase Of The Day:");
  lcd.setCursor(2,1);
  lcd.print("Notice, That For");
  lcd.setCursor(1,2);
  lcd.print("Instance, This Is");
  lcd.setCursor(1,3);
  lcd.print("Very True, Really?");
}

void Denavit(double a1, double a2, double a3){
  theta_1 = a1;
  theta_2 = a2;
  theta_3 = a3;
  theta_4 = abs(theta_3)-theta_2;
  sigma_1 = sin(theta_1*PI/180);
  sigma_2 = cos(theta_1*PI/180);
  sigma_4 = sin((theta_2-theta_3+theta_4)*PI/180);
  sigma_5 = cos((theta_2-theta_3+theta_4)*PI/180);
  sigma_3 = L2*cos(theta_2*PI/180)+L3*cos(theta_4*PI/180)+x*sigma_5-y*sigma_4;
  pos_x = sigma_2*sigma_3;
  pos_y = sigma_1*sigma_3;
  pos_z = L2*sin(theta_2*PI/180)+L3*sin((theta_2-theta_3)*PI/180)+y*sigma_5+x*sigma_4;
}

void InverseKin(double xy, double yy, double zy){
  x_inv = xy;
  y_inv = yy;
  z_inv = zy;
  z_1 = z_inv+28.45;
  mt = 0;
  ve = 0;
  q1 = atan2(y_inv,x_inv)*180/PI;
  if (q1 < 0){
    q1 = 180;
  }
  h_p = sqrt(pow(x_inv,2)+pow(y_inv,2))-58.49;
  a1_1 = atan2(z_1,h_p)*180/PI;
  h_1 = sqrt(pow(z_1,2)+pow(h_p,2));
  alpha = acos((pow(h_1,2))/(2*h_1*120))*180/PI;
  q2 = a1_1+alpha;
  if (q2 <= 0){
    q2 = 0;
    ve + ve + 1;
  }
  else if (q2 >= 139){
    q2 = 139;
    ve = ve + 1;
  }
  beta = acos((pow(120,2)+pow(120,2)-pow(h_1,2))/(2*120*120))*180/PI;
  q30 = 180-beta;
  if (q30 <= 35){
    q30 = 35;
    ve = ve + 1;
  }
  else if (q30 >= 138){
    q30 = 138;
    ve = ve + 1;
  }
  q3 = q30;
  if (ve > 0){
    mt = 1;
    warning_geo = 1;
    Serial.println("Geometric method didn't work! Point not reachable!");
  }
  if (mt == 0){
    Serial.println("Geometric method worked!");
    ang1_input = q1;
    ang2_muestra = q2;
    ang3_muestra = q3;
  }
}

void button_recognition(){
  if (irrecv.decode(&results)){ 
        if (results.value == 0XFFFFFFFF)
          results.value = key_value;
        switch(results.value){
          case 0xFF22DD: //Prev
            if(menu_counter == 2){
              lcd.clear();
              menu_1();
            }
            else if(counter_easter == 4){
              lcd.clear();
              menu_1();
            }
            else if(menu_counter == 3 && var_menu == 1){
              lcd.clear();
              menu_manual();
            }
            else if(menu_counter == 3 && var_menu == 2){
              lcd.clear();
              menu_manual();
            }
            else if(menu_counter == 3 && var_menu == 3){
              lcd.clear();
              menu_manual();
            }
            else if(menu_counter == 4 && var_menu == 5){
              lcd.clear();
              if(type == 1){
                menu_motorsforw();
              }
              else if(type == 2){
                menu_motorsinv();
              }
            }
          break; 
          
          case 0xFFC23D: //Play
            if(selection == 1 || selection == 2 || selection == 3)
            {
              selection = 0;
              lcd.clear();
              if(type == 1){
                menu_motorsforw();
              }
              else if(type == 2){
                menu_motorsinv();
              }
            }
          break ;               
          
          case 0xFFE01F: //-
            switch(selection){
              case 1:
                if(type == 1){
                  if(ang1_input > 0){
                    ang1_input = ang1_input - paso_input;
                  }
                  lcd.clear();
                  menu_motorsforw();
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
                if(type == 2){
                  if(x_input > -250 && x_input < 250){
                    if(set == 1){
                      x_input = x_input - paso_coord;
                    }
                    else if(set == 2){
                      x_input = x_input - paso_dec;
                    }
                  }
                  lcd.clear();
                  menu_motorsinv();
                  if(set == 1){
                    lcd.setCursor(18,3);
                    lcd.print(" ");
                  }
                  else if(set == 2){
                    lcd.setCursor(18,3);
                    lcd.print(".");
                  }
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
              break;
              case 2:
                if(type == 1){
                  if(ang2_muestra > 0){
                    ang2_input = ang2_input + paso_input;
                    ang2_muestra = ang2_muestra - paso_input;
                    ang3_input = ang3_input + paso_input;
                  }
                  lcd.clear();
                  menu_motorsforw();
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
                if(type == 2){
                  if(y_input > -250 && y_input < 250){
                    if(set == 1){
                      y_input = y_input - paso_coord;
                    }
                    else if(set == 2){
                      y_input = y_input - paso_dec;
                    }
                  }
                  lcd.clear();
                  menu_motorsinv();
                  if(set == 1){
                    lcd.setCursor(18,3);
                    lcd.print(" ");
                  }
                  else if(set == 2){
                    lcd.setCursor(18,3);
                    lcd.print(".");
                  }
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
              break;
              case 3:
                if(type == 1){
                  if(ang3_muestra > 0){
                    ang3_input = ang3_input - paso_input;
                    ang3_muestra = ang3_muestra - paso_input;
                  }
                  lcd.clear();
                  menu_motorsforw();
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
                if(type == 2){
                  if(z_input > -100 && z_input < 130){
                    if(set == 1){
                      z_input = z_input - paso_coord;
                    }
                    else if(set == 2){
                      z_input = z_input - paso_dec;
                    }
                  }
                  lcd.clear();
                  menu_motorsinv();
                  if(set == 1){
                    lcd.setCursor(18,3);
                    lcd.print(" ");
                  }
                  else if(set == 2){
                    lcd.setCursor(18,3);
                    lcd.print(".");
                  }
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
              break;
            }
          break ;  
          
          case 0xFFA857: //+
            switch(selection){
              case 1:
                if(type == 1){
                  if(ang1_input > 0){
                    ang1_input = ang1_input + paso_input;
                  }
                  lcd.clear();
                  menu_motorsforw();
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
                if(type == 2){
                  if(x_input > -250 && x_input < 250){
                    if(set == 1){
                      x_input = x_input + paso_coord;
                    }
                    else if(set == 2){
                      x_input = x_input + paso_dec;
                    }
                  }
                  lcd.clear();
                  menu_motorsinv();
                  if(set == 1){
                    lcd.setCursor(18,3);
                    lcd.print(" ");
                  }
                  else if(set == 2){
                    lcd.setCursor(18,3);
                    lcd.print(".");
                  }
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
              break;
              case 2:
                if(type == 1){
                  if(ang2_muestra > 0){
                    ang2_input = ang2_input - paso_input;
                    ang2_muestra = ang2_muestra + paso_input;
                    ang3_input = ang3_input - paso_input;
                  }
                  lcd.clear();
                  menu_motorsforw();
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
                if(type == 2){
                  if(y_input > -250 && y_input < 250){
                    if(set == 1){
                      y_input = y_input + paso_coord;
                    }
                    else if(set == 2){
                      y_input = y_input + paso_dec;
                    }
                  }
                  lcd.clear();
                  menu_motorsinv();
                  if(set == 1){
                    lcd.setCursor(18,3);
                    lcd.print(" ");
                  }
                  else if(set == 2){
                    lcd.setCursor(18,3);
                    lcd.print(".");
                  }
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
              break;
              case 3:
                if(type == 1){
                  if(ang3_muestra > 0){
                    ang3_input = ang3_input + paso_input;
                    ang3_muestra = ang3_muestra + paso_input;
                  }
                  lcd.clear();
                  menu_motorsforw();
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
                if(type == 2){
                  if(z_input > -100 && z_input < 130){
                    if(set == 1){
                      z_input = z_input + paso_coord;
                    }
                    else if(set == 2){
                      z_input = z_input + paso_dec;
                    }
                  }
                  lcd.clear();
                  menu_motorsinv();
                  if(set == 1){
                    lcd.setCursor(18,3);
                    lcd.print(" ");
                  }
                  else if(set == 2){
                    lcd.setCursor(18,3);
                    lcd.print(".");
                  }
                  lcd.setCursor(19,3);
                  lcd.print("!");
                }
              break;
            }
          break ;
          
          case 0xFF30CF: //1
            counter_easter = -1;
            if(menu_counter == 1)
            {
              lcd.clear();
              menu_auto();
            }
            else if(menu_counter == 2 && var_menu == 1){
              zero();
              delay(2000);

              //Chocolate 1
              stepper4.rotate(paso_conveyor); //Conveyor Moving Forward
              delay(1000);
              controller.rotate(0*f, 52*f, (14+52)*f);
              delay(2000);
              while(digitalRead(fc_cc) == HIGH){
                gripper.write(180);
              }
              gripper.write(90);
              gripper_const = 0;
              delay(100);
              Serial.println("Gripper Closing"); //Gripping Chocolate
              controller.rotate(0*f, -52*f, (-14-52)*f);
              delay(2000);
              controller.rotate(71*f, 57*f, (33+57)*f);
              delay(2000);
              while(digitalRead(fc_gg) == HIGH){
                gripper.write(0);
              }
              gripper.write(90);
              gripper_const = 1;
              delay(100);
              Serial.println("Gripper Opening"); //Deposit Chocolate
              controller.rotate(0*f, -17*f, (7-17)*f);
              delay(100);
              controller.rotate(-71*f, -40*f, (-40-40)*f);
              delay(2000);
              
              //Chocolate 2
              stepper4.rotate(paso_conveyor); //Conveyor Moving Forward
              delay(1000);
              controller.rotate(0*f, 52*f, (14+52)*f);
              delay(2000);
              while(digitalRead(fc_cc) == HIGH){
                gripper.write(180);
              }
              gripper.write(90);
              gripper_const = 0;
              delay(100);
              Serial.println("Gripper Closing"); //Gripping Chocolate
              controller.rotate(0*f, -52*f, (-14-52)*f);
              delay(2000);
              controller.rotate(80*f, 66*f, (-7+66)*f);
              delay(2000);
              while(digitalRead(fc_gg) == HIGH){
                gripper.write(0);
              }
              gripper.write(90);
              gripper_const = 1;
              delay(100);
              Serial.println("Gripper Opening"); //Deposit Chocolate
              controller.rotate(0*f, -25*f, (5-25)*f);
              delay(100);
              controller.rotate(-80*f, -41*f, (2-41)*f);
              delay(2000);

              //Chocolate 3
              stepper4.rotate(paso_conveyor); //Conveyor Moving Forward
              delay(1000);
              controller.rotate(0*f, 52*f, (14+52)*f);
              delay(2000);
              while(digitalRead(fc_cc) == HIGH){
                gripper.write(180);
              }
              gripper.write(90);
              gripper_const = 0;
              delay(100);
              Serial.println("Gripper Closing"); //Gripping Chocolate
              controller.rotate(0*f, -52*f, (-14-52)*f);
              delay(2000);
              controller.rotate(60*f, 76*f, (-32+76)*f);
              delay(2000);
              while(digitalRead(fc_gg) == HIGH){
                gripper.write(0);
              }
              gripper.write(90);
              gripper_const = 1;
              delay(100);
              Serial.println("Gripper Opening"); //Deposit Chocolate
              controller.rotate(0*f, -41*f, (6-41)*f);
              delay(100);
              controller.rotate(-60*f, -35*f, (26-35)*f);
              delay(2000);

              //Chocolate 4
              
              stepper4.rotate(paso_conveyor); //Conveyor Moving Forward
              delay(1000);
              controller.rotate(0*f, 52*f, (14+52)*f);
              delay(2000);
              while(digitalRead(fc_cc) == HIGH){
                gripper.write(180);
              }
              gripper.write(90);
              gripper_const = 0;
              delay(100);
              Serial.println("Gripper Closing"); //Gripping Chocolate
              controller.rotate(0*f, -52*f, (-14-52)*f);
              delay(2000);
              controller.rotate(50*f, 61*f, (10+61)*f);
              delay(2000);
              while(digitalRead(fc_gg) == HIGH){
                gripper.write(0);
              }
              gripper.write(90);
              gripper_const = 1;
              delay(100);
              Serial.println("Gripper Opening"); //Deposit Chocolate
              controller.rotate(0*f, -10*f, (6-10)*f);
              delay(100);
              controller.rotate(-50*f, -51*f, (-16-51)*f);
              delay(2000);
            }
            else if(menu_counter == 2 && var_menu == 2){
              lcd.clear();
              if(type == 1){
                menu_motorsforw(); 
              }
              else if(type == 2){
                menu_motorsinv();
              }
            }
            else if(menu_counter == 3 && var_menu == 1)
            {
              selection = 1;
              lcd.setCursor(19,3);
              lcd.print("!");
            }
            else if(menu_counter == 3 && var_menu == 2 && gripper_const != 1)
            {
              while(digitalRead(fc_gg) == HIGH){
                gripper.write(0);
              }
              gripper.write(90);
              gripper_const = 1;
              delay(100);
              Serial.println("Gripper Opening");
            }
            else if(menu_counter == 3 && var_menu == 3)
            {
              conveyor.setSpeed(300);
              conveyor.step(steps_Conveyor);
              Serial.println("Conveyor Moving Forward");
            }
          break ;
          
          case 0xFF18E7: //2
            counter_easter = -1;
            if(menu_counter == 1){
              lcd.clear();
              menu_manual();
            }
            else if(menu_counter == 2 && var_menu == 2){
              lcd.clear();
              menu_gripper();
            }
            else if(menu_counter == 3 && var_menu == 1)
            {
              selection = 2;
              lcd.setCursor(19,3);
              lcd.print("!");
            }
            else if(menu_counter == 3 && var_menu == 2 && gripper_const != 0)
            {
              while(digitalRead(fc_cc) == HIGH){
                gripper.write(180);
              }
              gripper.write(90);
              gripper_const = 0;
              delay(100);
              Serial.println("Gripper Closing");
            }
            else if(menu_counter == 3 && var_menu == 3)
            {
              conveyor.setSpeed(300);
              conveyor.step(-steps_Conveyor);
              Serial.println("Conveyor Moving Backwards");
            }
          break ;
          
          case 0xFF7A85: //3
            if(menu_counter == 2 && var_menu == 2){
              lcd.clear();
              menu_conveyor();
            }
            else if(menu_counter == 3 && var_menu == 1)
            {
              selection = 3;
              lcd.setCursor(19,3);
              lcd.print("!");
            }
            else if(menu_counter == 3 && var_menu == 3)
            {
              Serial.println("Conveyor Stopping");
            }
            else if(menu_counter == 2 && var_menu == 1)
            {
              zero();
              Serial.println("Homing Robot!");
            }
          break ;
          
          case 0xFF10EF: //4
            if(menu_counter == 2 && var_menu == 2){
              lcd.clear();
              menu_conveyor();
            }
            else if(menu_counter == 3 && var_menu == 1){
              if(type == 1){
                ang1_output = ang1_input - ang1_actual;
                ang1_actual = ang1_input;
                ang2_output = ang2_input - ang2_actual;
                ang2_actual = ang2_input;
                ang3_output = ang3_input - ang3_actual;
                ang3_actual = ang3_input;
                Serial.print("Angle 1: ");
                Serial.println(ang1_output);
                Serial.print("Angle 2: ");
                Serial.println(ang2_output);
                Serial.print("Angle 3: ");
                Serial.println(ang3_output);
                Serial.println("Robot Moving!!");
                delay(2000);
                controller.rotate(ang1_output*f, ang2_output*f, ang3_output*f);
                delay(500);
              }
              if(type == 2 && warning_geo == 0){
                InverseKin(x_input, y_input, z_input);
                ang2_input = 135 - ang2_muestra;
                ang3_input = ang3_muestra - 65;
                ang1_output = ang1_input - ang1_actual;
                ang1_actual = ang1_input;
                ang2_output = ang2_input - ang2_actual;
                ang2_actual = ang2_input;
                ang3_output = ang3_input - ang3_actual;
                ang3_actual = ang3_input;
                Serial.print("Angle 1: ");
                Serial.println(ang1_output);
                Serial.print("Angle 2: ");
                Serial.println(ang2_output);
                Serial.print("Angle 3: ");
                Serial.println(ang3_output);
                Serial.println("Robot Moving!!");
                delay(2000);
                controller.rotate(ang1_output*f, ang2_output*f, (ang2_output + ang3_output)*f);
                delay(500);
              }
              else if(type == 2 && warning_geo == 1){
                lcd.clear();
                var_menu = 6;
                announcement();
                lcd.clear();
                menu_motorsinv();
              }
            }
          break ;  

          case 0xFF38C7: //5
            if(menu_counter == 3 && var_menu == 1){
              lcd.clear();
              Denavit(ang1_actual, ang2_muestra, ang3_muestra);
              menu_infokinematics();
            }
          break;

          case 0xFFE21D: //CH+
            if(menu_counter == 3 && var_menu == 1 && type == 1){
              lcd.clear();
              menu_motorsinv();
              type = 2;
            }
          break;

          case 0xFFA25D: //CH-
            if(menu_counter == 3 && var_menu == 1 && type == 2){
              lcd.clear();
              menu_motorsforw();
              type = 1;
            }
          break;

          case 0xFF629D: //CH
            if(menu_counter == 3 && var_menu == 1 && type == 2){
              if(set == 1){
                set = 2;
              }
            }
          break;

          case 0xFF906F: //6
            if(menu_counter == 3 && var_menu == 1 && type == 2){
              if(set == 2){
                set = 1;
              }
            }
          break;

          //Useless Variables
          case 0xFF5AA5: //6
            if(counter_easter == 0){
              counter_easter = 1;
            }
          break;

          case 0xFF52AD: //9
            if(counter_easter == 1){
              counter_easter = 2;
            }
          break;

          case 0xFF9867: //100+
            if(counter_easter == 2){
              counter_easter = 3;
            }
          break;

          case 0xFF02FD: //Next
            if(counter_easter == 3 && menu_counter == 1 && var_menu == 0){
              lcd.clear();
              easter_egg();
              counter_easter = 4;
            }
          break;
        }
        key_value = results.value;
        irrecv.resume();
  }
}
