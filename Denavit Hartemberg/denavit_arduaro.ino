#define PI 3.141592654

double pos_x, pos_y, pos_z;
double sigma_1, sigma_2, sigma_3, sigma_4, sigma_5 = 0;

double L2 = 120;
double L3 = 120;
double x = 58.49;
double y = -28.45;
double theta_1 = 0;
double theta_2 = 58.1;
double theta_3 = 92.8;
double theta_4 = theta_3-theta_2;
  
void setup() {
  Serial.begin(9600);
}

void loop() {
    Denavit();
    Serial.print("Position X: ");
    Serial.println(pos_x);
    Serial.print("Position Y: ");
    Serial.println(pos_y);
    Serial.print("Position Z: ");
    Serial.println(pos_z);
    delay(1000);
}

void Denavit(){
  sigma_1 = sin(theta_1*PI/180);
  sigma_2 = cos(theta_1*PI/180);
  sigma_4 = sin((theta_2-theta_3+theta_4)*PI/180);
  sigma_5 = cos((theta_2-theta_3+theta_4)*PI/180);
  sigma_3 = L2*cos(theta_2*PI/180)+L3*cos(theta_4*PI/180)+x*sigma_5-y*sigma_4;
  pos_x = sigma_2*sigma_3;
  pos_y = sigma_1*sigma_3;
  pos_z = L2*sin(theta_2*PI/180)+L3*sin((theta_2-theta_3)*PI/180)+y*sigma_5+x*sigma_4;
}
