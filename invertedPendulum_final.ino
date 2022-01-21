#include<Wire.h>
const int motorPin = 9;
const int controlPin1 = 7;
const int controlPin2 = 6;

const int MPU_addr=0b1101000;  // I2C address of the MPU-6050
int16_t AcX,GyZ;
int acc_calibration_value;

double pendAngle; //state 1 =x1
double rotZ;      //state 2 = x2
double wRot;      // state 3= x3

//pid 
unsigned long lastTime;
double Input;
double Output=0;
double Setpoint = 0;
double errSum, lastInput;
double sampleTime = 15; // milisec
double kp = 3.0; // 0.7;
double ki = 4.5*(sampleTime/1000.0); 
double kd = 0.0004/(float)(sampleTime/1000.0);
double prop,dterm; 

//pid 2
unsigned long lastTime2;
double Input2;
double Output2=0;
double errSum2, lastInput2;
double sampleTime2 = 15; // milisec
double kp2 = 0.18; //0.2
double ki2 = 0;//0.0001*(sampleTime2/1000.0); //0.1
double kd2 = 0.0006/(sampleTime2/1000.0); //0.3;
double prop2,dterm2; 

//state 
double x1;
double x2;
double x3;

double u;
double input;

int encoderOutput = 11;
int hallsen = 3;
volatile long encoderValue = 0;
double interval = 1000;
unsigned long prevMillis = 0;
unsigned long currMillis = 0;
double rps;
int receive_counter=0;
unsigned long loop_timer;
double gyroZcali;
byte start, received_byte;

void setup() {
  // put your setup code here, to run once:
      Wire.begin();
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      Wire.beginTransmission(MPU_addr); //I2C address of the MPU
      Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
      Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
      Wire.endTransmission(true); 
      Wire.beginTransmission(MPU_addr); //I2C address of the MPU
      Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
      Wire.write(0x08); //Setting the accel to +/- 2g
      Wire.endTransmission(true);
      Wire.beginTransmission(MPU_addr);                                     
      Wire.write(0x1A);                                                         
      Wire.write(0x03);           //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
      Wire.endTransmission();       
      Serial.begin(115200);

      pinMode(motorPin, OUTPUT);
      pinMode(controlPin1, OUTPUT);
      pinMode(controlPin2,OUTPUT);
//      pinMode(onOff,INPUT);
      digitalWrite(controlPin1,HIGH);
      digitalWrite(controlPin2,LOW);
      pinMode(hallsen,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallsen), updateEncoder, RISING);
  // gyro calibration
for(receive_counter = 0; receive_counter < 100; receive_counter++){       //Create 500 loops
    Wire.beginTransmission(MPU_addr);                                   //Start communication with the gyro
    Wire.write(0x47);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(MPU_addr, 2);                                      //Request 2 bytes from the gyro
    gyroZcali += Wire.read()<<8|Wire.read();               //Combine the two bytes to make one integer
    delayMicroseconds(3700);                               //Wait for 3700 microseconds to simulate the main program loop time
 // Serial.print("calibration gyro: ");Serial.println(gyroZcali);
  }
   gyroZcali /= 100;
   //Serial.print("final calibration: ");Serial.println(gyroZcali);
/*for(receive_counter = 0; receive_counter < 100; receive_counter++){       //Create 500 loops
     Wire.beginTransmission(MPU_addr);                                   //Start communication with the gyro
    Wire.write(0x3B);                                                       //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                                 //End the transmission
    Wire.requestFrom(MPU_addr, 2);                                      //Request 2 bytes from the gyro
   acc_calibration_value += Wire.read()<<8|Wire.read()*-1;               //Combine the two bytes to make one integer
    delayMicroseconds(3700);    
    Serial.print("calibration acc: ");Serial.println(acc_calibration_value);           
   }
   acc_calibration_value/=100;
*/
   acc_calibration_value= 200;
  //  Serial.print("calibration acc: ");Serial.println(acc_calibration_value);
    
  loop_timer = micros() + 15000;
}
void loop() {
  // put your main code here, to run repeatedly:
//unsigned long ct = micros();
//Serial.print("start time: ");Serial.print(ct);
  /*
  //wheel angular speed
  currMillis = millis();
if(currMillis - prevMillis > interval){
  prevMillis = currMillis;
 rps = (encoderValue/encoderOutput); 
 //Serial.print(" rps: "); Serial.println(rps); 
 //Serial.print("  | hallsen: ");Serial.println(encoderValue);
 encoderValue = 0;
}
wRot = (rps*360)/(interval/1000); //deg/sec
x3 = wRot*(3.14/180);   
*/
//angle calc
 Wire.beginTransmission(MPU_addr);                //Start communication with the gyro
  Wire.write(0x3B);                            //Start reading at register 3F
  Wire.endTransmission();                                        //End the transmission
  Wire.requestFrom(MPU_addr, 2);                          //Request 2 bytes from the gyro
  AcX = Wire.read()<<8|Wire.read();                      //Combine the two bytes to make one integer
  AcX += acc_calibration_value;                          //Add the accelerometer calibration value
  if( AcX > 8200) AcX = 8200;           //Prevent division by zero by limiting the acc data to +/-8200;
  if( AcX < -8200) AcX = -8200;         //Prevent division by zero by limiting the acc data to +/-8200;
  pendAngle = asin((float)AcX/8200.0)*(180/3.14);      //deg     //Calculate the current angle according to the accelerometer(rad)
  //x1 = asin((float)AcX/8200.0)*(180/3.14);         //deg
if(pendAngle>-0.5 && pendAngle<0.5){
  x1 = pendAngle;
}
//pend angular speed calc
  Wire.beginTransmission(MPU_addr);                //Start communication with the gyro
  Wire.write(0x47);                            //Start reading at register 3F
  Wire.endTransmission();                                        //End the transmission
  Wire.requestFrom(MPU_addr, 2); 
  GyZ = Wire.read()<<8|Wire.read();
  GyZ -= gyroZcali;   //raw
  rotZ = (GyZ / (float)131); // deg/sec
x2 = rotZ*(3.14/(float)180);  //rad/sec
x1 += GyZ*(float)0.0001145;    //deg
x1 = x1*0.6 + pendAngle*0.4;  

//PID 2
Input2 = Output;

    errSum2 += (float)(ki2*Input2);
     if((errSum2)>10)errSum2 = 10;
else if((errSum2)<-10)errSum2 = -10;
    double dInput2 = (float)(Input2-lastInput2);
    Output2 = (float)kp2*Input2 + (float)errSum2 + (float)kd2*dInput2;
 if(Output2>10)Output2 = 10;
 if(Output2<-10)Output = -10;

    lastInput2 = Input2;
    prop2 = kp2*Input2;
    dterm2 = kd*dInput2;
    
  Setpoint = 0 - Output2;
  
//PID calc
Input = x1;
    double error = Setpoint - Input;
    errSum += (ki*error);
     if((errSum)>12)errSum = 12;
else if((errSum)<-12)errSum = -12;
    double dInput = (Input-lastInput);
    Output = kp*error + errSum - kd*dInput;
 if(Output>12)Output = 12;
else if(Output< -12)Output = -12;

    lastInput = Input;
    //lastTime = now;
    prop = kp*error;
    dterm = kd*dInput;
  //}
  
if(Output<3 && Output>-3){
 Output=0;
}
if(pendAngle>8 || pendAngle<-8){
  input = 0;
  Output = 0;
  errSum= 0;
  Output2 = 0;
  errSum2 = 0;
}

u = abs(Output);
input = constrain(map(u,0,12,0,255),0,255);

if(Output <(0.3)){
  PORTD = B10000000;
 // digitalWrite(controlPin1,HIGH);
 // digitalWrite(controlPin2,LOW);
  analogWrite(motorPin,input);
}else if(Output > (0.3)){
  //u = u*(-0.3);
  PORTD = B01000000;
  //digitalWrite(controlPin1,LOW);
  //digitalWrite(controlPin2,HIGH);
  analogWrite(motorPin,input);
}else{
 // PORTD = B11000000;
  //digitalWrite(motorPin,HIGH);
analogWrite(motorPin,0);
}



Serial.print(" | x1 ");
Serial.print(x1);
Serial.print(" | voltage: "); 
Serial.println(Output);
//Serial.print(" | Ref ");
//Serial.println(Setpoint);

  while(loop_timer>= micros()){
  }
  loop_timer += 15000;
 // ct = micros();
 // Serial.print("| time end: ");Serial.println(ct);
}

void updateEncoder()
{
  encoderValue++;
}
