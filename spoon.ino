#include <Wire.h>
#include <Servo.h>
#include <math.h>

#define n 1                            //標本数を決める

Servo myservo;
long accelX, accelY, accelZ;

double gForceX[2]={0,0},gForceX_after[2]={0,0}, gForceY[2]={0,0},gForceY_after[2]={0,0},  gForceZ[2]={0,0},gForceZ_after[2]={0,0};
float gx[n]={0},gy[n]={0},gz[n]={0};

long gyroX, gyroY, gyroZ;
float rotX, rotX_after[2]={0,0},rotY, rotZ;
float rx[n]={0},ry[n]={0},rz[n]={0};
float rotX2, rotY2, rotZ2;

//サンリング周期
float T=0.01;
//時定数
float t=0.225;

void setup() {
  Serial.begin(9600);
  //9ピンからサーボモーターの回転信号をPWM出力
  myservo.attach(9); 
  Wire.begin();
  setupMPU6050();
  //シリアルモニタに出力
  myservo.write(90); //0度に回転
  
}
 
 
void loop() {
//  for(int i=n-1;i>0;i--){
//    gx[i] = gx[i-1];//過去１０回分の値を記録
//    gy[i] = gy[i-1];//過去１０回分の値を記録
//    gz[i] = gz[i-1];//過去１０回分の値を記録
//  }
  recordAccelRegisters();
  recordGyroRegisters();
  filter_x();
  filter_z();
  printData();
  moter();
  delay(10);
}
 
void setupMPU6050() {
  //MPU6050との通信を開始し、ジャイロと加速度の最大範囲を指定
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x6B); //Accessing the register 6B
  Wire.write(0b00000000); //SLEEP register to 0
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00000000); //gyro to full scale ± 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration
  Wire.write(0b00000000); //accel to +/- 2g
  Wire.endTransmission();
}
 
void recordAccelRegisters() {
  //加速度読み取り
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B); // Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  calculateAccelData();
}
 
void calculateAccelData() {
  //読み取った値をgに変換
  gForceX[1] = accelX / 16384.0;
  gForceY[1] = accelY / 16384.0;
  gForceZ[1] = accelZ / 16384.0;
}
 
void recordGyroRegisters() {
  //ジャイロの値を読み取る
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
  calculateGyroData();
}
 
void calculateGyroData() {
  //読み取った値をdeg/secに変換
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

 void filter_x(){
  gForceX_after[1]=T/(T+2*t)*(gForceX[0]+gForceX[1])-(T-2*t)/(T+2*t)*gForceX_after[0];
  gForceX[0] =gForceX[1] ;
  gForceX_after[0]=gForceX_after[1];
 }
 void filter_y(){
  gForceY_after[1]=T/(T+2*t)*(gForceY[0]+gForceY[1])-(T-2*t)/(T+2*t)*gForceY_after[0];
  gForceY[0] =gForceY[1] ;
  gForceY_after[0]=gForceY_after[1];
 }
  void filter_z(){
  gForceZ_after[1]=T/(T+2*t)*(gForceZ[0]+gForceZ[1])-(T-2*t)/(T+2*t)*gForceZ_after[0];
  gForceZ[0]=gForceZ[1];
  gForceZ_after[0]=gForceZ_after[1];
 }
void moter(){ 
  double theta=atan2(gForceZ_after[1],gForceX_after[1])*180/M_PI;
  double theta_nonfil=atan2(gForceZ[1],gForceX[1])*180/M_PI;
//  plot
  Serial.print(theta); //フィルタあり
  Serial.print(",");
  Serial.print(theta_nonfil);//フィルタなし
  Serial.println("");
  myservo.write(theta); //sita度に回転
//  delay(10);      //1000㎳待つ
  
}
void printData() {
  //シリアルモニタに出力
//  Serial.print(rotX );
//  Serial.print(",");
//  Serial.print(rotY );
//  Serial.print(",");
//  Serial.print(rotZ );
//  Serial.print(",");
//    Serial.print(" X=");
//    Serial.print(gForceY[1]);
//    Serial.print(",");
//    Serial.println(gForceY_after[1]);
//    Serial.println(gForceY[0] );
//  Serial.print(",");
//  Serial.println(gForceZ );  //シリアルモニタに出力
}
