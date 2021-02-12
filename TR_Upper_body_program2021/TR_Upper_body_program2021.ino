//学ロボ2021 上半身プログラム ver1.2.0
#include "MsTimerTPU3.h"
#include "ISO.h"
#include "define.h"
#include "Arduino.h"
#include "RoboClaw.h"
#include "PIDclass.h"
#include "AMT203read.h"
#define timer_time 10
#define Serial_fm Serial1

uint8_t mfs_p[6]; //自己位置データ[Y,Y,X,X,Z,Z]　mfs=master from serial p=position
uint8_t mfs[2];   //命令　チェックサム
uint8_t chks;     //チェックサム
char mfs_n;       //改行コード
bool flag_10ms;
bool flag1 = false,flag2 = false,flag3 = false,flag4 = false;
bool flag5 = false,flag6 = false,flag7 = false,flag8 = false,flag9 = false;
int count=0;
int AE1=0,AE2=0;//アブソリュートエンコーダーの値
int M1max,M2max,M5max;
double azimuth,shotdeg,slide;//方位角,射角,スライドレール
double mtspeedM1,enc1,encpast1,encsabn1,encval1,encrM1;
double mtspeedM2,enc2,encpast2,encsabn2,encval2,encrM2;
double mtspeedM5,encrM5;
double mtspeed_shot;

RoboClaw roboclaw(&Serial2,10000);
AMT203read AbsoluteENC(true);
PID M1pid(0.0, 0.0, 0.0,0.01);
PID M2pid(0.0, 0.0, 0.0,0.01);
PID M5pid(0.0, 0.0, 0.0,0.01);

uint8_t mts[2]; //マスターに送るデーター　チェックサム mts=master to serial
void timer()
{
  count++;
  if (count == 1)
  {
    flag_10ms = true;
    count=0;
  }
  else
  {
    flag_10ms = false;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial_fm.begin(115200);
  roboclaw.begin(38400);
  AbsoluteENC.AMT203_SPI_set(10,30);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  pinMode(AS1_PIN, OUTPUT);
  pinMode(AS2_PIN, OUTPUT);
  pinMode(AS3_PIN, OUTPUT);
  pinMode(AS4_PIN, OUTPUT);
  pinMode(AS5_PIN, OUTPUT);
  pinMode(AS6_PIN, OUTPUT);
  pinMode(AS7_PIN, OUTPUT);
  pinMode(AS8_PIN, OUTPUT);
  MsTimerTPU3::set((int)timer_time, timer);
  MsTimerTPU3::start();
  ISO::ISOkeisu_SET();
  ISO::ISOkeisu_MTU1(M5ppr);//M5
  ISO::ISOkeisu_MTU2(400);
}
bool read_mfs()
{
  bool success_r = false;
  if (Serial_fm.available())
  {
    mfs_p[0] = (uint8_t)Serial_fm.read();
    mfs_p[1] = (uint8_t)Serial_fm.read();
    mfs_p[2] = (uint8_t)Serial_fm.read();
    mfs_p[3] = (uint8_t)Serial_fm.read();
    mfs_p[4] = (uint8_t)Serial_fm.read();
    mfs_p[5] = (uint8_t)Serial_fm.read();
    mfs[0] = (uint8_t)Serial_fm.read();
    mfs[1] = (uint8_t)Serial_fm.read();
    mfs_n = (char)Serial_fm.read();
  }
  if (mfs_n == '\n')
  {
    chks = mfs_p[0] ^ mfs_p[1] ^ mfs_p[2] ^ mfs_p[3] ^ mfs_p[4] ^ mfs_p[5] ^ mfs[0];
    if (mfs[1] == chks)
    {
      success_r = true;
      Serial.print("success!!");
      Serial.print(mfs_p[0]);
      Serial.print("-");
      Serial.print(mfs_p[1]);
      Serial.print("-");
      Serial.print(mfs_p[2]);
      Serial.print("-");
      Serial.print(mfs_p[3]);
      Serial.print("-");
      Serial.print(mfs_p[4]);
      Serial.print("-");
      Serial.print(mfs_p[5]);
      Serial.print("-");
      Serial.print(mfs[0]);
      Serial.print("-");
      Serial.print(mfs[1]);
      Serial.print("-");
      Serial.print(chks);
      Serial.println(mfs_n);
    }
  }
  else
  {
    /*Serial.print("failure");Serial.print(mfs_p[0]);Serial.print("-");Serial.print(mfs_p[1]);Serial.print("-");Serial.print(mfs_p[2]);Serial.print("-");Serial.print(mfs_p[3]);Serial.print("-");Serial.print(mfs_p[4]);Serial.print("-");Serial.print(mfs_p[5]);Serial.print("-");Serial.print(mfs[0]);Serial.print("-");Serial.print(mfs[1]);Serial.print("-");Serial.print(chks);Serial.println(mfs_n);*/
    success_r = false;
  }
  return success_r;
}
bool write_mts()
{
  bool success_w = false;
  mts[0] = 82;
  mts[1] = mts[0];
  Serial_fm.write(mts[0]);
  Serial_fm.write(mts[1]);
  Serial_fm.write('\n');
  success_w = true;

  return success_w;
}

void loop()////////////////////////////////////////////////////////////////////////////
{
  digitalWrite(PIN_LED1, flag_10ms);
  if (flag_10ms == true)
  {
    digitalWrite(PIN_LED3, read_mfs());
    digitalWrite(PIN_LED2, write_mts());
    if(mfs[0] == master_collection_order){//回収作業
      //↓[1]方位角調整
      azimuth = 3.14;//方位角設定

      AE2 = AbsoluteENC.AMT203_read2(0);//AE2読み取り
      encsabn2=AE2-encpast2;
      if(encsabn2>3500){
        encsabn2=encsabn2-4095;
      }
      if(encsabn2<-3500){
        encsabn2=encsabn2+4095;
      }
      encval2+=encsabn2;
      encpast2 = AE2;
      encrM2=(encval2/4095)*6.28;
      mtspeedM2 = M2pid.getCmd(azimuth,encrM2,M2max);
      roboclaw.ForwardBackwardM2(RC3_ad,mtspeedM2);//M2　PID角度制御をAE2で決めた角度に行う
      if(encrM2<=azimuth+0.15&&encrM2>=azimuth-0.15){
        flag2 = true;
      }
      //↑[1]
      //↓[2]スライドレール調整
      if(flag2 == true){
      slide = 3.14;//スライドレール調整
      encrM5 = (ISO::ISOkeisu_read_MTU1(0)/M5ppr)*6.28;
      mtspeedM5 = M5pid.getCmd(slide,encrM5,M5max);
      roboclaw.ForwardBackwardM2(RC2_ad,mtspeedM5);//M5　スライドレール調整　PID角度調整

      digitalWrite(AS1_PIN,0);
      digitalWrite(AS2_PIN,0);
      digitalWrite(AS3_PIN,1);
      digitalWrite(AS4_PIN,1);
      }
      if(encrM5<=slide+0.15&&encrM5>=slide-0.15){
        flag2 = false;
        flag3 = true;
      }
      //↑[2]
      //↓[3]回収
      if(flag3 == true){
      digitalWrite(AS3_PIN,0);
      flag3 = false;
      flag4 = true;
      }
      //↑[3]
      //↓[4]回収完了
      if(flag4 == true){
      digitalWrite(AS1_PIN,1);
      digitalWrite(AS4_PIN,0);
      flag4 = false;
      flag5 = true;
      }
      //↑[4]
    }
    
    else if(mfs[0] == master_shot_order){
     //発射作業
     //↓[5]角度計算
      if(flag5 = true){
        azimuth = 3.14;
        shotdeg = 3.14;
        slide = 3.14;
        flag5 = false;
        flag6 = true;
      }
      //↑[5]
      //↓[6]射角・方位角調整
      if(flag6 = true){
      AE2 = AbsoluteENC.AMT203_read2(0);//AE2読み取り
      encsabn2=AE2-encpast2;
      if(encsabn2>3500){
        encsabn2=encsabn2-4095;
      }
      if(encsabn2<-3500){
        encsabn2=encsabn2+4095;
      }
      encval2+=encsabn2;
      encpast2 = AE2;
      encrM2=(encval2/4095)*6.28;
      mtspeedM2 = M2pid.getCmd(azimuth,encrM2,M2max);
      roboclaw.ForwardBackwardM2(RC3_ad,mtspeedM2);//M2　PID角度制御をAE2で計算した結果に行う

      //射角調整
      AE2 = AbsoluteENC.AMT203_read1(0);//AE2読み取り
      encsabn1=AE1-encpast1;
      if(encsabn1>3500){
        encsabn1=encsabn1-4095;
      }
      if(encsabn1<-3500){
        encsabn1=encsabn1+4095;
      }
      encval1+=encsabn1;
      encpast1 = AE1;
      encrM1=(encval1/4095)*6.28;
      mtspeedM1 = M1pid.getCmd(shotdeg,encrM1,M1max);
      roboclaw.ForwardBackwardM1(RC2_ad,mtspeedM1);//M1　PID角度制御をAE1で計算した結果に行う
      digitalWrite(AS5_PIN,1);
      if(encrM2<=azimuth+0.15&&encrM2>=azimuth-0.15&&encrM1<=shotdeg+0.15&&encrM1>=shotdeg-0.15){
        flag6 = false;
        flag7 = true;
      }
      }
      //↑[6]
      //↓[7]スライドレール作動
      if(flag7 == true){
      digitalWrite(AS3_PIN,0);
      encrM5=(ISO::ISOkeisu_read_MTU1(0)/M5ppr)*6.28;
      mtspeedM5 = M5pid.getCmd(slide,encrM5,M5max);
      roboclaw.ForwardBackwardM2(RC2_ad,mtspeedM5);//M5　スライドレール調整　PID角度調整
      
      roboclaw.ForwardBackwardM1(RC1_ad,mtspeed_shot);//M3発射　PID速度制御　計算した結果から行う
      roboclaw.ForwardBackwardM2(RC1_ad,mtspeed_shot);//M4発射　PID速度制御　計算した結果から行う
      if(encrM5<=slide+0.15&&encrM5>=slide-0.15){
        flag7 = false;
        flag8 = true;
      }
      }
      //↑[7]
      //↓[8]発射
      if(flag8 == true){
      digitalWrite(AS5_PIN,0);
      flag8 = false;
      }
      //↑[8]
    }
    else if(mfs[0] == master_initialize_order){//初期化
    //↓[9]初期化
    digitalWrite(AS1_PIN,1);
    digitalWrite(AS2_PIN,0);
    //↑[9]
    }
    //安全停止
    /*digitalWrite(AS1_PIN,0);
    digitalWrite(AS2_PIN,1);*/
    flag_10ms = false;
  } 
}