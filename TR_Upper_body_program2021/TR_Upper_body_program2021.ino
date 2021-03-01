//学ロボ2021 上半身プログラム ver1.5.0
//制御の大方の流れは"https://docs.google.com/presentation/d/1agclaAzMRlz074pZXtvcgefBO3hHuOG_t2Uiqf9cAfk/edit?usp=sharing"で確認できます
#include "MsTimerTPU3.h"
#include "ISO.h"
#include "define.h"
#include "Arduino.h"
#include "RoboClaw.h"
#include "PIDclass.h"
#include "AMT203read.h"
#define timer_time 10
#define Serial_fm Serial5

uint8_t mfs_p[6]; //自己位置データ[Y,Y,X,X,Z,Z]　mfs=master from serial p=position
uint8_t mfs[4];   //命令1 命令2　チェックサム 改行コード
uint8_t mfs_pre;  //成型用の箱
uint8_t chks;     //チェックサム
int master_pod1_1 = 0, master_pod1_2 = 0, master_pod2_1 = 0, master_pod2_2 = 0, master_pod3_0 = 0;
int master_pic_order = 0, master_release_order = 0, master_prepare_order = 0, master_shot_order = 0;
bool flag_10ms, flag10s;
bool flag0 = true, flag1 = false, flag2 = false, flag3 = false, flag4 = false;
bool flag5 = false, flag6 = false, flag7 = false, flag8 = false, flag9 = false;
bool flag_slide = false;
int count10ms = 0, count10s = 0, count_shot = 0;
int AE1 = 0, AE2 = 0; //アブソリュートエンコーダーの値
int M1max, M2max, M5max;
double azimuth_tgt = 0, shotdeg_tgt = 0, slide_tgt = 0;                                            //方位角,射角(0~1.70[rad](0~100°)),スライドレール
double speed_azimuth, enc_azimuth, encpast_azimuth, encsabn_azimuth, encval_azimuth, encr_azimuth; //AE1
double speed_shotdeg, enc_shotdeg, encpast_shotdeg, encsabn_shotdeg, encval_shotdeg, encr_shotdeg; //AE2
double speed_slide, encr_slide;                                                                    //スライドレール
double speed_shot;

RoboClaw roboclaw(&Serial1, 10000);
AMT203read AbsoluteENC(true);
PID pid_shotdeg(0.0, 0.0, 0.0, 0.01);
PID pid_azimuth(0.0, 0.0, 0.0, 0.01);
PID pid_slide(0.0, 0.0, 0.0, 0.01);
bool flag_1s = false;
uint8_t mts[2]; //マスターに送るデーター　チェックサム mts=master to serial
void timer()
{
  count10ms++;
  count10s++;
  if (count10ms >= 10)
  {
    flag_10ms = true;
    count10ms = 0;
  }
  static int count1s = 0;
  count1s++;
  if (count1s >= 100)
  {
    flag_1s = true;
    count1s = 0;
  }
  if (count10s >= 1000)
  {
    flag10s = true;
    count10s = 0;
  }
  /*else
  {
    flag_10ms = false;
    flag10s = false;
  }*/
}

void setup()
{
  Serial.begin(115200);
  Serial_fm.begin(115200);
  roboclaw.begin(115200);
  AbsoluteENC.AMT203_SPI_set(10, 30);

  pid_slide.PIDinit(0.0, 0.0);
  pid_slide.setPara(10, 0, 0);
  pid_azimuth.PIDinit(0.0, 0.0);
  pid_azimuth.setPara(150, 0, 20);
  pid_shotdeg.PIDinit(0.0, 0.0);
  pid_shotdeg.setPara(10, 0, 0);

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
  pinMode(KOUDEN, INPUT_PULLUP);
  pinMode(boardLED1, OUTPUT);
  pinMode(boardLED2, OUTPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);

  MsTimerTPU3::set((int)timer_time, timer);
  MsTimerTPU3::start();
  ISO::ISOkeisu_SET();
  ISO::ISOkeisu_MTU1(M5ppr); //M5
  ISO::ISOkeisu_MTU2(400);
  AbsoluteENC.AMT203_set_zero1();
  AbsoluteENC.AMT203_set_zero2();
}
bool read_mfs()
{
  bool success_r = false;
  if (Serial_fm.available() > 0)
  {
    mfs[0] = (uint8_t)Serial_fm.read();
    mfs[1] = (uint8_t)Serial_fm.read();
    mfs[2] = (uint8_t)Serial_fm.read();
    mfs[3] = (uint8_t)Serial_fm.read();
  }
  for (int seikei = 0; seikei <= 3; seikei++)
  {
    if (mfs[3] == 0xB4)
    {
      chks = mfs[0] ^ mfs[1];
      if (mfs[2] == chks)
      {
        master_pod2_1 = mfs[1] & 0x01;             //2-1ポッドを狙う命令
        master_pod1_1 = mfs[2] >> 7 & 0x01;        //1-1ポッドを狙う命令
        master_pod3_0 = mfs[2] >> 6 & 0x01;        //3型ポッドを狙う命令
        master_pod1_2 = mfs[2] >> 5 & 0x01;        //1-2ポッドを狙う命令
        master_pod2_2 = mfs[2] >> 4 & 0x01;        //2-2ポッドを狙う命令
        master_pic_order = mfs[2] >> 3 & 0x01;     //回収命令 ハンド
        master_release_order = mfs[2] >> 2 & 0x01; //解放命令
        master_prepare_order = mfs[2] >> 1 & 0x01; //射出準備命令　棒
        master_shot_order = mfs[2] & 0x01;         //射出命令
        success_r = true;
      }
      else
      {
        success_r = false;
      }
    }
    else //並び替え
    {
      mfs_pre = mfs[0];
      for (int nchk = 0; nchk < 3; nchk++)
      {
        mfs[nchk] = mfs[nchk + 1];
      }
      mfs[3] = mfs_pre;
    }
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

void loop() ////////////////////////////////////////////////////////////////////////////
{
  if (flag_10ms == true)
  {
    Serial.print(" flag_slide ");
    Serial.print(flag_slide);
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    digitalWrite(PIN_LED3, read_mfs());
    digitalWrite(PIN_LED2, write_mts());
    //エンコーダー読み取り/////////////////////////////////////////////////
    encr_slide = ((ISO::ISOkeisu_read_MTU1(0, false) / 1000) * 6.28);
    //初期化//////////////////////////////////////////////////////////////
    if (flag0 == true)
    {
      /*if (analogRead(KOUDEN) > 100 && flag_slide == false)
      {
        roboclaw.ForwardM1(RC2_ad, 20);
      }
      else if (analogRead(KOUDEN) < 100 && flag_slide == false)
      {
        flag_slide = true;
        ISO::ISOkeisu_read_MTU1(0, true);
      }
      else if (flag_slide == true)
      {
        slide_tgt = -5;
        if (encr_slide <= slide_tgt + 0.5 && encr_slide >= slide_tgt - 0.5)
        {
          digitalWrite(AS4_PIN, 0);
          digitalWrite(AS3_PIN, 1);
          flag0 = false;
          flag1 = true;
        }
      }*/
      shotdeg_tgt = 1.60;
    }
    //手動装填/////////////////////////////////////////////////////////////
    if (digitalRead(S1) < 1 && flag1 == true)
    {
      //shotdeg_tgt = 1.60; //要変更
      digitalWrite(AS1_PIN, 1);
      digitalWrite(AS4_PIN, 0);
      flag1 = false;
      flag2 = true;
    }
    //発射////////////////////////////////////////////////////////////////
    if (master_shot_order > 0 && flag2 == true)
    {
      shotdeg_tgt = 1.60;
      azimuth_tgt = 3.14;
      slide_tgt = 25;
      if (encr_slide <= slide_tgt + 0.1 && encr_slide >= slide_tgt - 0.1 && encr_azimuth <= azimuth_tgt + 0.1 && encr_azimuth >= azimuth_tgt - 0.1 && encr_shotdeg <= shotdeg_tgt + 0.1 && encr_shotdeg >= shotdeg_tgt - 0.1)
      {
        digitalWrite(AS5_PIN, 1);
      }
      count_shot++;
    }
    if (count_shot >= 5)
    {
      flag2 = false;
      flag3 = true;
    }
    //回収////////////////////////////////////////////////////////////////
    if (master_pic_order > 0 && flag4 == true)
    {
      shotdeg_tgt = 0.5;
      digitalWrite(AS2_PIN, 0);
      digitalWrite(AS1_PIN, 0);
      digitalWrite(AS4_PIN, 1);
      flag4 = false;
      flag5 = true;
    }
    if (master_pic_order > 0 && flag5 == true)
    {
      digitalWrite(AS3_PIN, 1);
      flag5 = false;
      flag6 = true;
    }
    if (master_pic_order > 0 && flag6 == true)
    {
      digitalWrite(AS4_PIN, 0);
    }
    //PID常時動作////////////////////////////////////////////////////////
    enc_azimuth = AbsoluteENC.AMT203_read1(0);
    encsabn_azimuth = enc_azimuth - encpast_azimuth;
    if (encsabn_azimuth > 3500)
    {
      encsabn_azimuth = encsabn_azimuth - 4095;
    }
    if (encsabn_azimuth < -3500)
    {
      encsabn_azimuth = encsabn_azimuth + 4095;
    }
    encval_azimuth += encsabn_azimuth;
    encpast_azimuth = enc_azimuth;
    encr_azimuth = ((encval_azimuth / 4095) * 6.28);
    speed_azimuth = pid_azimuth.getCmd(azimuth_tgt, encr_azimuth, 20);
    /*if (speed_azimuth >= 0)
    {
      roboclaw.BackwardM1(RC1_ad, speed_azimuth);
    }
    else if (speed_azimuth < 0)
    {
      speed_azimuth = speed_azimuth * (-1);
      roboclaw.ForwardM1(RC1_ad, speed_azimuth);
    }*/

    enc_shotdeg = AbsoluteENC.AMT203_read2(0);
    encsabn_shotdeg = enc_shotdeg - encpast_shotdeg;
    if (encsabn_shotdeg > 3500)
    {
      encsabn_shotdeg = encsabn_shotdeg - 4095;
    }
    if (encsabn_shotdeg < -3500)
    {
      encsabn_shotdeg = encsabn_shotdeg + 4095;
    }
    encval_shotdeg += encsabn_shotdeg;
    encpast_shotdeg = enc_shotdeg;
    encr_shotdeg = ((encval_shotdeg / 4095) * 6.28);
    speed_shotdeg = pid_shotdeg.getCmd(shotdeg_tgt, encr_shotdeg, 20);
    speed_shotdeg = min(speed_shotdeg, 5);
    if (speed_shotdeg >= 0)
    {
      roboclaw.BackwardM2(RC1_ad, speed_shotdeg);
    }
    else if (speed_shotdeg < 0)
    {
      speed_shotdeg = speed_shotdeg * (-1);
      roboclaw.ForwardM2(RC1_ad, speed_shotdeg);
    }

    if (flag_slide == 1)
    {
      speed_slide = pid_slide.getCmd(slide_tgt, encr_slide, 20);
      if (speed_slide >= 0)
      {
        roboclaw.ForwardM1(RC2_ad, speed_slide);
      }
      else if (speed_slide < 0)
      {
        speed_slide = speed_slide * (-1);
        roboclaw.BackwardM1(RC2_ad, speed_slide);
      }
    }
    if (flag_1s)
    {
      digitalWrite(A0, !digitalRead(A0));
      flag_1s = false;
    }
    flag_10ms = false;
  }
  /////////////////////////////////////////////////////////////////////
  Serial.print("tuusin-");
  if (read_mfs() == true)
  {
    Serial.print("success ");
  }
  else if (read_mfs() == false)
  {
    Serial.print("failure ");
  }
  Serial.print(" flag0 ");
  Serial.print(flag0);
  Serial.print(" flag_slide ");
  Serial.print(flag_slide);
  Serial.print(" flag1 ");
  Serial.print(flag1);
  Serial.print(" flag2 ");
  Serial.print(flag2);
  /*Serial.print(" flag3 ");
    Serial.print(flag3);
    Serial.print(" flag4 ");
    Serial.print(flag4);
    Serial.print(" flag5 ");
    Serial.print(flag5);
    Serial.print(" flag6 ");
    Serial.print(flag6);*/
  Serial.print(" azimuth_tgt ");
  Serial.print(azimuth_tgt);
  Serial.print(" shotdeg_tgt ");
  Serial.print(shotdeg_tgt);
  Serial.print(" slide_tgt ");
  Serial.print(slide_tgt);
  Serial.print(" KOUDEN ");
  Serial.print(analogRead(KOUDEN));
  Serial.print(" encr_slide ");
  Serial.print(encr_slide);
  Serial.print(" encr_shotdeg ");
  Serial.print(encr_shotdeg);
  Serial.print(" encr_azimuth ");
  Serial.print(encr_azimuth);
  Serial.print(" speed_slide ");
  Serial.print(speed_slide);
  Serial.print(" speed_shotdeg ");
  Serial.println(speed_shotdeg);
  /*Serial.print(" master_pic_order ");
    Serial.print(master_pic_order);
    Serial.print(" master_release_order ");
    Serial.print(master_release_order);
    Serial.print(" master_prepare_order ");
    Serial.print(master_prepare_order);
    Serial.print(" master_shot_order ");
    Serial.println(master_shot_order);*/
  digitalWrite(PIN_LED1, 0);
}