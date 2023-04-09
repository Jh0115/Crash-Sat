#include <TM1637Display.h>
#include "HX711.h"
#include "ms4525do.h"

const int LC1D = 2; //load cell data pins
const int LC2D = 4;
const int LC3D = 6;

const int LC1K = 3; //load cell clock pins
const int LC2K = 5;
const int LC3K = 7;

const int disp1D = 9; // 7 segment display 1 pins
const int disp1K = 8;

const int disp2D = 10; // 7 segment display 2 pins
const int disp2K = 11;

const int LED1 = 12; //LED pins
const int LED2 = 13;
const int LED3 = 1;

long R1;
long R2;
long R3;
long q;

float rho = 1.1; //THIS NEEDS TO BE UPDATED EVERY FEW HOURS

float spd = 0;
float spdcmd = 0;

//Uncertainty tracking variables for all variables
float R1_var_prev;
float R1_avg_prev;
float R1_var_curr;
float R1_avg_curr;

float R2_var_prev;
float R2_avg_prev;
float R2_var_curr;
float R2_avg_curr;

float R3_var_prev;
float R3_avg_prev;
float R3_var_curr;
float R3_avg_curr;

float q_var_prev;
float q_avg_prev;
float q_var_curr;
float q_avg_curr;

float R1_mu;
float R2_mu;
float R3_mu;
float q_mu;

float R1_mu_goal = 0.0015; // load cell 1 uncertainty goal (0.15%)
float R2_mu_goal = 0.0014; // load cell 2 uncertainty goal (0.14%)
float R3_mu_goal = 0.0019; // load cell 3 uncertainty goal (0.19%)
float q_mu_goal = 0.00065;  // dynamic pressure uncertainty goal (0.065%)

float N;

HX711 scale1;
HX711 scale2;
HX711 scale3;
bfs::Ms4525do pres;
TM1637Display dis_tar(disp1K,disp1D);
TM1637Display dis_act(disp2K,disp2D);

void setup() {
  Serial.begin(57600);
  scale1.begin(LC1D,LC1K);
  scale2.begin(LC2D,LC2K);
  scale3.begin(LC3D,LC3K);
  Wire.begin();

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  
  Wire.begin();
  Wire.setClock(400000);

  pres.Config(&Wire, 0x28, 1.0f, -1.0f);
  pres.Begin();

}

void loop() {
  // grab 3 measurements from 3 loadcells
  delay(100);
  
  if (scale1.is_ready()) { //Scale 1
    R1 = scale1.read();
  }
  else {
    R1 = 0;
  }

  if (scale2.is_ready()) { //Scale 2
    R2 = scale2.read();
  }
  else {
    R2 = 0;
  }

  if (scale3.is_ready()) { //Scale 3
    R3 = scale3.read();
  }
  else {
    R3 = 0;
  }

  if (pres.Read()){
    q = abs(pres.pres_pa());
    spd = sqrt(2*q/rho);
  }
  else {
    q = 0;
    spd = 0;
  }

  // grab data from pitot tube and update the display
  dis_act.setBrightness(2);
  dis_act.showNumberDecEx((spd*223.694),0b01000000,false,4,0);
  
  // print data to the Serial monitor
  Serial.print(millis());
  Serial.print(',');
  Serial.print(R1);
  Serial.print(',');
  Serial.print(R2);
  Serial.print(',');
  Serial.print(R3);
  Serial.print(',');
  Serial.println(spd);

  // feedback LED settings
  //step 1: Update averaged uncertainties
  N = N+1;
  R1_avg_curr = R1_avg_prev+(R1-R1_avg_prev)/N;
  R2_avg_curr = R2_avg_prev+(R2-R2_avg_prev)/N;
  R3_avg_curr = R3_avg_prev+(R3-R3_avg_prev)/N;
  q_avg_curr = q_avg_prev+(q-q_avg_prev)/N;

  R1_var_curr = (((N-2)*R1_var_prev)+((R1-R1_avg_curr)*(R1-R1_avg_prev)))/(N-1);
  R2_var_curr = (((N-2)*R2_var_prev)+((R2-R2_avg_curr)*(R2-R2_avg_prev)))/(N-1);
  R3_var_curr = (((N-2)*R3_var_prev)+((R3-R3_avg_curr)*(R3-R3_avg_prev)))/(N-1);
  q_var_curr = (((N-2)*q_var_prev)+((q-q_avg_curr)*(q-q_avg_prev)))/(N-1);
  
  R1_mu = 1.96*sqrt(R1_var_curr/N);
  R2_mu = 1.96*sqrt(R2_var_curr/N);
  R3_mu = 1.96*sqrt(R3_var_curr/N);
  q_mu = 1.96*sqrt(q_var_curr/N);

  R1_var_prev = R1_var_curr;
  R2_var_prev = R2_var_curr;
  R3_var_prev = R3_var_curr;
  q_var_prev = q_var_curr;

  R1_avg_prev = R1_avg_curr;
  R2_avg_prev = R2_avg_curr;
  R3_avg_prev = R3_avg_curr;
  q_avg_prev = q_avg_curr;

  //step 2: Track progress
    //red LED will always be on when collecting data
  
  if ((R1_mu<R1_mu_goal)||(R2_mu<R2_mu_goal)||(R3_mu<R3_mu_goal)||(q_mu<q_mu_goal)){
    //light the LED yellow
  }
  if ((R1_mu<R1_mu_goal)&&(R2_mu<R2_mu_goal)&&(R3_mu<R3_mu_goal)&&(q_mu<q_mu_goal)){
    //light the LED Green
  }

  //Step 3: Red means lower 50%, yellow is upper 50%, green  is complete

  
  // check for a new speed setting, update the displays, reset the uncertainty numbers
  if (Serial.available()) {
    String rx = Serial.readString();
    spdcmd = rx.toFloat();
    
    // After reading the serial data do a flush
    Serial.flush();

    // update the display
    dis_tar.setBrightness(2);
    dis_tar.showNumberDecEx((spdcmd*223.694),0b01000000,false,4,0);

    // reset uncertainty counter
    R1_var_prev = 0;
    R2_var_prev = 0;
    R3_var_prev = 0;
    q_var_prev = 0;

    R1_avg_prev = R1;
    R2_avg_prev = R2;
    R3_avg_prev = R3;
    q_avg_prev = q;

    N = 1;
    
  }
}
