//学ロボ2021 上半身プログラム ver1.0.1
#include "MsTimerTPU3.h"
#include "ISO.h"
#include <Arduino.h>
#define timer_time 10
#define Serial_fm Serial1
uint8_t mfs_p[6]; //自己位置データ　mfs=master from serial p=position
uint8_t mfs[2];   //命令　チェックサム
uint8_t chks;     //チェックサム
char mfs_n;       //改行コード
bool flag_10ms;
int count=0;

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
  pinMode(PIN_LED3, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED1, OUTPUT);
  MsTimerTPU3::set((int)timer_time, timer);
  MsTimerTPU3::start();
  ISO::ISOkeisu_SET();
  ISO::ISOkeisu_MTU1();
  ISO::ISOkeisu_MTU2();
  ISO::ISOkeisu_TPU1();
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
    /*Serial.print("failure");
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
     Serial.println(mfs_n);*/
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

void loop()
{
  digitalWrite(PIN_LED1, flag_10ms);
  if (flag_10ms == true)
  {
    digitalWrite(PIN_LED3, read_mfs());
    digitalWrite(PIN_LED2, write_mts());
    //Serial.print("ISO:");
    //int val =ISO::ISOkeisu_read_MTU1(false);
    //Serial.println(val);
    flag_10ms = false;
  }

}

