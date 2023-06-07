#include <Servo.h>

Servo servo1;

//Motor 1
const int dirPin1 = 8;
const int stepPin1 = 9;
const int der1 = 2; 
int dere1;
const int izq1 = 3; 
int izqu1;

//Motor 2
const int dirPin2 = 10;
const int stepPin2 = 11;
const int der2 = 4; 
int dere2;
const int izq2 = 5; 
int izqu2;

//Motor 3
const int dirPin3 = 12;
const int stepPin3 = 13;
const int der3 = 6; 
int dere3;
const int izq3 = 7; 
int izqu3;

const int steps = 200;
int stepDelay = 800;


const int sd = 22;
int ssd;
const int si = 24;
int ssi;

void setup() {
   // Marcar los pines como salida
   pinMode(dirPin1, OUTPUT);
   pinMode(stepPin1, OUTPUT);
   pinMode(der1, INPUT);
   pinMode(izq1, INPUT);

   pinMode(dirPin2, OUTPUT);
   pinMode(stepPin2, OUTPUT);
   pinMode(der2, INPUT);
   pinMode(izq2, INPUT);

   pinMode(dirPin3, OUTPUT);
   pinMode(stepPin3, OUTPUT);
   pinMode(der3, INPUT);
   pinMode(izq3, INPUT);

   servo1.attach(1);
   pinMode(sd, INPUT);
   pinMode(si, INPUT);
   servo1.write(90);
}



void loop() {
  dere1 = digitalRead(der1);
  izqu1 = digitalRead(izq1);

  if(dere1 == LOW || izqu1 == LOW)
  { 
    //Activar una direccion y fijar la velocidad con stepDelay
     dire1();
     // Giramos 200 pulsos para hacer una vuelta completa
     for (int x = 0; x < steps * 1; x++) {
        digitalWrite(stepPin1, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin1, LOW);
        delayMicroseconds(stepDelay);
     }
  }

  dere2 = digitalRead(der2);
  izqu2 = digitalRead(izq2); 

  if(dere2 == LOW || izqu2 == LOW)
  {
    
    //Activar una direccion y fijar la velocidad con stepDelay
     dire2();
     // Giramos 200 pulsos para hacer una vuelta completa
     for (int x = 0; x < steps * 1; x++) {
        digitalWrite(stepPin2, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin2, LOW);
        delayMicroseconds(stepDelay);
     }
  }

  dere3 = digitalRead(der3);
  izqu3 = digitalRead(izq3); 

  if(dere3 == LOW || izqu3 == LOW)
  {
    
    //Activar una direccion y fijar la velocidad con stepDelay
     dire3();
     // Giramos 200 pulsos para hacer una vuelta completa
     for (int x = 0; x < steps * 1; x++) {
        digitalWrite(stepPin3, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(stepPin3, LOW);
        delayMicroseconds(stepDelay);
     }
  }

  ssd = digitalRead(sd);
  ssi = digitalRead(si);

  serr();

}

void dire1()
{
  if(dere1 == LOW && izqu1 == HIGH )
  {
    digitalWrite(dirPin1, LOW);
  }
  if(izqu1 == LOW && dere1 == HIGH)
  {
    digitalWrite(dirPin1, HIGH);
  }
}

void dire2()
{
  if(dere2 == LOW && izqu2 == HIGH )
  {
    digitalWrite(dirPin2, LOW);
  }
  if(izqu2 == LOW && dere2 == HIGH)
  {
    digitalWrite(dirPin2, HIGH);
  }
}

void dire3()
{
  if(dere3 == LOW && izqu3 == HIGH )
  {
    digitalWrite(dirPin3, LOW);
  }
  if(izqu3 == LOW && dere3 == HIGH)
  {
    digitalWrite(dirPin3, HIGH);
  }
}

void serr()
{
  if(ssd == LOW)
  {
    servo1.write(70);
  }
  if(ssi == LOW)
  {
    servo1.write(115);
  }
  if(ssd == HIGH && ssi == HIGH)
  {
    servo1.write(90);
  }

  delay(1000);
}
