#include<Wire.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void  setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(115200);
}
void  loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();  
  GyX=Wire.read()<<8|Wire.read();  
  GyY=Wire.read()<<8|Wire.read();  
  GyZ=Wire.read()<<8|Wire.read();  
  
  
//  Serial.print("Accelerometer: ");
  bool FLAG_ACC  = false;
  bool FLAG_GYRO = true;
  if(FLAG_ACC)
  {
    Serial.print(AcX);
    Serial.print(" "); Serial.print(AcY);
    Serial.print(" "); Serial.println(AcZ);
  }
  if(FLAG_GYRO)
  { 
  //Serial.print("Gyroscope: ");
    Serial.print(GyX);
    Serial.print(" "); Serial.print(GyY);
    Serial.print(" "); Serial.println(GyZ);
  }
  delay(20);
  
}
