/******加速度计无法计算偏航角，陀螺仪计算偏航有漂移，需与磁力计融合******/
#include <Wire.h>
#include <Kalman.h>
#define RESTRICT_PITCH
#define fRad2Deg  57.295779513f //将弧度转为角度的乘数
#define fDeg2Rad  0.0174532925f
#define MPU 0x68 //MPU-6050的I2C地址
#define nValCnt 7 //一次读取寄存器的数量

#define HMC 0x1E //001 1110b(0x3C>>1), HMC5883的7位i2c地址  
#define MagnetcDeclination 0.0698131701f //所在地磁偏角，根据情况自行百度  
#define CalThreshold 2  //最大和最小值超过此值，则计算偏移值
float offsetX,offsetY,offsetZ;  //磁场偏移
float kalAngleX, kalAngleY,kalAngleZ; //横滚、俯仰、偏航角的卡尔曼融合值
float Gryoyaw; //偏航角的地磁测量值
Kalman kalmanX; // 实例化卡尔曼滤波
Kalman kalmanY;
//Kalman kalmanZ;
long timer;

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050
  //WriteMPUReg(0x6A, ReadMPUReg(0x6A)&0xDF);
  WriteMPUReg(0x37, ReadMPUReg(0x37)|0x02);  //开启mpu6050的IIC直通，连接磁场传感器
  float realVals[nValCnt];
  for(int i=0;i<500;i++)ReadAccGyr(realVals); //读出测量值
  float roll,pitch;
  GetRollPitch(realVals,&roll,&pitch);
  roll *= fRad2Deg; pitch *= fRad2Deg;
  kalmanX.setAngle(roll); // 设置初始角
  kalmanY.setAngle(pitch);
  
  //设置HMC5883模式
  Wire.beginTransmission(HMC); //开始通信
  Wire.write(0x00); //选择配置寄存器A
  Wire.write(0x70); //0111 0000b，具体配置见数据手册
  Wire.endTransmission();
  Wire.beginTransmission(HMC);
  Wire.write(0x02); //选择模式寄存器
  Wire.write(0x00); //连续测量模式:0x00,单一测量模式:0x01
  Wire.endTransmission();
  calibrateMag();  //地磁计校准
//  int x,y,z;
//  getRawData(&x,&y,&z); //获取地磁数据
//  kalmanZ.setAngle(calculateYaw(pitch,roll,x,y,z) * fRad2Deg);  //设定卡尔曼滤波初始值
  timer = micros(); //计时
}

void loop() {
  float realVals[nValCnt];
  ReadAccGyr(realVals); //读出测量值
  int x,y,z;
  getRawData(&x,&y,&z);//获取地磁数据
  
  double dt = (double)(micros() - timer) / 1000000; // 计算时间差
  timer = micros();  //更新时间
  
  float roll,pitch;
  GetRollPitch(realVals,&roll,&pitch);
  float Geoyaw = calculateYaw(pitch,roll,x,y,z);
  float gyroXrate = realVals[4] / 131.0; // 转换到角度/秒
  float gyroYrate = realVals[5] / 131.0;
  float gyroZrate = realVals[6] / 131.0;
  Gryoyaw += gyroZrate * dt; //由陀螺仪获取偏航角
  if(Gryoyaw>360)Gryoyaw=0;  //大于360则变为0
  // 解决加速度计角度在-180度和180度之间跳跃时的过渡问题
  roll *= fRad2Deg; pitch *= fRad2Deg; Geoyaw *= fRad2Deg;
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } 
  else{
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // 卡尔曼融合
  }
  if (abs(kalAngleX) > 90) gyroYrate = -gyroYrate; // 限制的加速度计读数
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  //对俯仰角滤波
//  kalAngleZ = kalmanZ.getAngle(Geoyaw, gyroZrate, dt); //对偏航角滤波
  
  //float temperature = realVals[3] / 340.0 + 36.53;  //计算温度
  Serial.print("accangleX:");Serial.print(roll);
  Serial.print(" kalAngleX:");Serial.print(kalAngleX);
  Serial.print(" accangleY:");Serial.print(pitch);
  Serial.print(" kalAngleY:");Serial.print(kalAngleY);
//  Serial.print(" Gryoyaw:");Serial.print(Gryoyaw);
  Serial.print(" Geoyaw:");Serial.print(Geoyaw);
////  Serial.print(" kalAngleZ:");Serial.print(kalAngleZ);
  Serial.print("\r\n");
//  int yrp[] = {kalAngleX,kalAngleY,Geoyaw};
//  Serial.print(String(yrp[0])+','+String(yrp[1])+','+String(yrp[2])+'\n');
  delay(100);
}

//根据俯仰角和偏航角修正地磁传感器
float calculateYaw(float Pitch,float Roll,int x ,int y,int z)
{
  x = x-offsetX;
  y = y-offsetY;
  z = z-offsetZ;
  float Hx,Hy;
  float Out;
//  Hx = cos(Pitch)*x+sin(Pitch)*z;
//  Hy = sin(Roll)*sin(Pitch)*x + cos(Roll)*y - sin(Roll)*cos(Pitch)*z;
//  Out = atan2(Hx,Hy);
  Hx = cos(Pitch)*x+sin(Roll)*sin(Pitch)*y-cos(Roll)*sin(Pitch)*z;
  Hy = cos(Roll)*y - sin(Roll)*z;
  Out = atan2(Hy,Hx);
  if(Out<0)Out+=2*PI;
  Out = Out + MagnetcDeclination;
  if(Out > 2*PI) Out -= 2*PI;
  return Out;
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(float *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (int i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//读取地磁数据
void getRawData(int* x ,int* y,int* z)
{
  Wire.beginTransmission(HMC);
  Wire.write(0x03); //从寄存器3开始读数据
  //每轴的数据都是16位的
  Wire.requestFrom(HMC, 6);
  Wire.endTransmission();
  if(6<=Wire.available()){
    *x = Wire.read()<<8| Wire.read(); //X msb，X轴高8位
    *z = Wire.read()<<8| Wire.read(); //Z msb
    *y = Wire.read()<<8| Wire.read(); //Y msb
  }
}
//根据xy分量计算方向角
float calculateHeading(int* x ,int* y,int* z)
{
  float headingRadians = atan2((double)((*y)-offsetY),(double)((*x)-offsetX));
  // 保证数据在0-2*PI之间
  if(headingRadians < 0)
    headingRadians += 2*PI;
 
  float headingDegrees = headingRadians * fRad2Deg;
  headingDegrees += MagnetcDeclination; //磁偏角
 
  // <span style="font-family: Arial, Helvetica, sans-serif;">保证数据在0-360之间</span>
  if(headingDegrees > 360)
    headingDegrees -= 360;
 
  return headingDegrees;
}
//校正传感器
void calibrateMag()
{
  int x,y,z; //三轴数据
  int xMax, xMin, yMax, yMin, zMax, zMin;
  //初始化
  getRawData(&x,&y,&z);  
  xMax=xMin=x;
  yMax=yMin=y;
  zMax=zMin=z;
  offsetX = offsetY = offsetZ = 0;
 
  Serial.println("Starting Calibration......");
  Serial.println("Please turn your device around in 20 seconds");
 
  for(int i=0;i<200;i++)
  {
    getRawData(&x,&y,&z);
    // 计算最大值与最小值
    // 计算传感器绕X,Y,Z轴旋转时的磁场强度最大值和最小值
    if (x > xMax)
      xMax = x;
    if (x < xMin )
      xMin = x;
    if(y > yMax )
      yMax = y;
    if(y < yMin )
      yMin = y;
    if(z > zMax )
      zMax = z;
    if(z < zMin )
      zMin = z;
 
    delay(100);
 
    if(i%10 == 0)
    {
      Serial.print(xMax);
      Serial.print(" ");
      Serial.println(xMin);
    }
  }
  //计算修正量
  if(abs(xMax - xMin) > CalThreshold )
    offsetX = (xMax + xMin)/2;
  if(abs(yMax - yMin) > CalThreshold )
    offsetY = (yMax + yMin)/2;
  if(abs(zMax - zMin) > CalThreshold )
    offsetZ = (zMax +zMin)/2;
 
  Serial.print("offsetX:");
  Serial.print("");
  Serial.print(offsetX);
  Serial.print(" offsetY:");
  Serial.print("");
  Serial.print(offsetY);
  Serial.print(" offsetZ:");
  Serial.print("");
  Serial.println(offsetZ);
}


//算得Roll角。
void GetRollPitch(float *pRealVals,float* roll,float* pitch) {
#ifdef RESTRICT_PITCH
  float fNorm = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  *pitch = atan2(-pRealVals[0],fNorm);
  *roll = atan2(pRealVals[1],pRealVals[2]);  //atan2和atan作用相同，但atan2在除数是0时也可以计算，所以尽量使用atan2
#else
  float fNorm = sqrt(pRealVals[2] * pRealVals[2] + pRealVals[0] * pRealVals[0]);
  *roll = atan2(pRealVals[1],fNorm);
  *pitch = atan2(-pRealVals[0],pRealVals[2]);
#endif
}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}
