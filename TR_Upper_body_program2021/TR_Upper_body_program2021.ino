//学ロボ2021 上半身プログラム ver1.5.0
//制御の大方の流れは"https://docs.google.com/presentation/d/1agclaAzMRlz074pZXtvcgefBO3hHuOG_t2Uiqf9cAfk/edit?usp=sharing"で確認できます
#include "MsTimerTPU3.h"
#include "ISO.h"
#include "define.h"
#include "Filter.h"
#include "Arduino.h"
#include "RoboClaw.h"
#include "PIDclass.h"
#include "AMT203read.h"
#define timer_time 10
#define Serial_fm Serial2

uint8_t mfs_p[6]; //自己位置データ[Y,Y,X,X,Z,Z]　mfs=master from serial p=position
uint8_t mfs[4];   //命令1 命令2　チェックサム 改行コード
uint8_t mfs_pre;  //成型用の箱
uint8_t chks;     //チェックサム
int master_pod1_1 = 0, master_pod1_2 = 0, master_pod2_1 = 0, master_pod2_2 = 0, master_pod3_0 = 0;
int master_pic_order = 0, master_release_order = 0, master_prepare_order = 0, master_shot_order = 0;
bool flag_10ms, flag_SWs, flag_100ms, flag_shotsec;
bool flag0 = true, flag1 = false, flag2 = false, flag3 = false, flag4 = false;
bool flag5 = false, flag6 = false, flag7 = false, flag8 = false, flag9 = false;
bool AS1 = false, AS2 = false, AS3 = false, AS4 = false, AS5 = false;
bool flag_slide = false, flag_shot = false, flag_serial = false;
int count10ms = 0, count_SWs = 0, count100ms = 0, count_shot = 0, count_shotsec;
int AE1 = 0, AE2 = 0; //アブソリュートエンコーダーの値
int programcheck = 0;
int M1max, M2max, M5max;
double azimuth_tgt, shotdeg_tgt, slide_tgt, roller_tgt = 0;                                        //方位角,射角(0~1.70[rad](0~100°)),スライドレール
double speed_azimuth, enc_azimuth, encpast_azimuth, encsabn_azimuth, encval_azimuth, encr_azimuth; //AE1
double speed_shotdeg, enc_shotdeg, encpast_shotdeg, encsabn_shotdeg, encval_shotdeg, encr_shotdeg; //AE2
double speed_slide, encr_slide, shotdeg_lowlimit = 0;
double shotkp, shotki, shotkd; //スライドレール
double speed_shot;
//double shot_azimuth[10] = {2.552, 2.177, 2.479, 2.107, 2.324, 1.013, 0.642, 0.637, 1.050, 0.804};
//double shot_azimuth[10] = {1.57,1.57,1.57,1.57,1.57,1.57,1.57,1.57,1.57,1.57};
double shot_azimuth[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//double shot_shotdeg[10] = {1.6, 1.5, 1.6, 1.5, 1.6, 1.5, 1.6, 1.5, 1.6, 1.5};
//double shot_shotdeg[10] = {1.5, 1.4, 1.3, 1.5, 1.4, 1.3, 1.5, 1.4, 1.3, 1.5};
double shot_shotdeg[10] = {1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6};
double shot_slide[10] = {-4.5, -30, -55.5, -79.5, -104, -4.5, -30, -55.5, -79.5, -103};
//double shot_roller[10] = {6, 6, 6, 6, 6, 6, 6, 6, 6, 6};
double shot_roller[10] = {5, 6, 7, 8, 9, 3, 3, 3, 3, 3};

RoboClaw roboclaw(&Serial1, 10000);
AMT203read AbsoluteENC(true);
Filter PIDFilter_azimuth(0.01);
Filter PIDFilter_shotdeg(0.01);
Filter PIDFilter_slide(0.01);
PID pid_shotdeg(0.0, 0.0, 0.0, 0.01);
PID pid_azimuth(0.0, 0.0, 0.0, 0.01);
PID pid_slide(0.0, 0.0, 0.0, 0.01);
uint8_t mts[2]; //マスターに送るデーター　チェックサム mts=master to serial
void timer()
{
  count10ms++;
  count100ms++;
  count_SWs++;
  count_shotsec++;
  if (count10ms >= 1)
  {
    flag_10ms = true;
    count10ms = 0;
    programcheck++;
  }
  count100ms++;
  if (count100ms >= 10)
  {
    flag_100ms = true;
    count100ms = 0;
  }
  if (count_SWs >= 20)
  {
    flag_SWs = true;
    count_SWs = 0;
  }
  if (count_shotsec >= 200)
  {
    flag_shotsec = true;
    count_shotsec = 0;
  }
  /*else
  {
    flag_10ms = false;
    flag10s = false;
  }*/
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
  //encr_azimuth = PIDFilter_azimuth.LowPassFilter(encr_azimuth);
  speed_azimuth = pid_azimuth.getCmd(azimuth_tgt, encr_azimuth, 15);
  speed_azimuth = PIDFilter_azimuth.LowPassFilter(speed_azimuth);

  pid_shotdeg.setPara(shotkp, shotki, shotkd);
  enc_shotdeg = AbsoluteENC.AMT203_read2(0);
  encsabn_shotdeg = enc_shotdeg - encpast_shotdeg;
  /*if (encsabn_shotdeg > 3500)
  {
    encsabn_shotdeg = encsabn_shotdeg - 4095;
  }
  if (encsabn_shotdeg < -3500)
  {
    encsabn_shotdeg = encsabn_shotdeg + 4095;
  }
  encval_shotdeg += encsabn_shotdeg;
  encpast_shotdeg = enc_shotdeg;*/
  encr_shotdeg = ((enc_shotdeg / 4095) * 6.28);
  speed_shotdeg = pid_shotdeg.getCmd(shotdeg_tgt, encr_shotdeg, 10); //20
  speed_shotdeg = min(speed_shotdeg, shotdeg_lowlimit);
  speed_shotdeg = PIDFilter_shotdeg.LowPassFilter(speed_shotdeg);

  encr_slide = (ISO::ISOkeisu_read_MTU1(0, false) / 1000) * 6.28;
  speed_slide = pid_slide.getCmd(slide_tgt, encr_slide, 20);
  speed_slide = PIDFilter_slide.LowPassFilter(speed_slide);

  speed_shot = ((roller_tgt / rollerR) * rollerppr) / (2 * PI);
  ///////////////////////////////////////////////////////////////////////////////
  digitalWrite(PIN_LED3, read_mfs());
  digitalWrite(PIN_LED2, write_mts());
}

void setup()
{
  Serial.begin(115200);
  Serial_fm.begin(23400);
  roboclaw.begin(115200);
  AbsoluteENC.AMT203_SPI_set(10, 30);

  pid_slide.PIDinit(0.0, 0.0);
  pid_slide.setPara(10, 1, 1);
  pid_azimuth.PIDinit(0.0, 0.0);
  pid_azimuth.setPara(180, 18, 18);
  pid_shotdeg.PIDinit(0.0, 0.0);
  //pid_shotdeg.setPara(90, 5, 5);
  //pid_shotdeg.setPara(45,7,7);

  PIDFilter_azimuth.setLowPassPara(0.005, 0.0);
  PIDFilter_shotdeg.setLowPassPara(0.05, 0.0);
  PIDFilter_slide.setLowPassPara(0.2, 0.0);

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
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(boardSW, INPUT);
  pinMode(boardLED1, OUTPUT);
  pinMode(boardLED2, OUTPUT);
  MsTimerTPU3::set((int)timer_time, timer);
  MsTimerTPU3::start();
  ISO::ISOkeisu_SET();
  ISO::ISOkeisu_MTU1(M5ppr); //M5
  digitalWrite(boardLED2, 1);
  shotdeg_tgt = 1.7; //回収=0.5
  azimuth_tgt = 0;
  enc_azimuth = AbsoluteENC.AMT203_read1(0);
  /*if(enc_azimuth > 2000){
    enc_azimuth = enc_azimuth - 4095;
  }*/
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
  mts[1] = mts[0] ^ 0xB4;
  Serial_fm.write(mts[0]);
  Serial_fm.write(mts[1]);
  Serial_fm.write(0xB4);
  success_w = true;

  return success_w;
}
void loop() ////////////////////////////////////////////////////////////////////////////
{
  if (flag_10ms == true)
  {
    digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
    //初期化//////////////////////////////////////////////////////////////
    if (flag0 == true)
    {
      shotdeg_lowlimit = 0; //10
      shotkp = 90;
      shotki = 5;
      shotkd = 5;
      if (analogRead(KOUDEN) > 100 && flag_slide == false)
      {
        roboclaw.ForwardM1(RC2_ad, 20);
      }
      else if (analogRead(KOUDEN) < 100 && flag_slide == false)
      {
        flag_slide = true;
        ISO::ISOkeisu_read_MTU1(0, true);
        roboclaw.ForwardM1(RC2_ad, 0);
      }
      else if (flag_slide == true)
      {
        slide_tgt = -60;
        if (encr_slide <= slide_tgt + 0.15 && encr_slide >= slide_tgt - 0.15)
        {
          AS3 = true;
          flag0 = false;
          flag1 = true;
        }
      }
    }
    //発射////////////////////////////////////////////////////////////////
    //if (master_shot_order > 0 && flag2 == true)
    if (digitalRead(S2) < 1 && flag1 == true && flag_shot == false && count_shot == 0)
    {
      azimuth_tgt = shot_azimuth[count_shot];
      shotdeg_tgt = shot_shotdeg[count_shot];
      slide_tgt = shot_slide[count_shot];
      roller_tgt = shot_roller[count_shot];
      flag_shot = true;
    }
    if (flag1 == true && flag_shot == true && encr_slide <= slide_tgt + 0.3 && encr_slide >= slide_tgt - 0.3 && encr_azimuth <= azimuth_tgt + 0.1 && encr_azimuth >= azimuth_tgt - 0.1 && encr_shotdeg <= shotdeg_tgt + 0.1 && encr_shotdeg >= shotdeg_tgt - 0.1)
    {
      AS2 = true;
      AS5 = true;
      flag1 = false;
      flag2 = true;
      count_shot++;
    }
    if (digitalRead(S2) < 1 && flag2 == true && flag_shot == true && flag_shotsec == true && count_shot > 0)
    {
      AS5 = false;
      count_shotsec = 0;
      flag_shotsec = false;
      flag_shot = false;
    }
    if (flag2 == true && flag_shot == false && flag_shotsec == true)
    {
      AS5 = true;
      azimuth_tgt = shot_azimuth[count_shot];
      shotdeg_tgt = shot_shotdeg[count_shot];
      slide_tgt = shot_slide[count_shot];
      roller_tgt = shot_roller[count_shot];
      if (encr_slide <= slide_tgt + 0.3 && encr_slide >= slide_tgt - 0.3 && encr_azimuth <= azimuth_tgt + 0.1 && encr_azimuth >= azimuth_tgt - 0.1 && encr_shotdeg <= shotdeg_tgt + 0.1 && encr_shotdeg >= shotdeg_tgt - 0.1)
      {
        AS2 = true;
        AS5 = true;
        count_shot++;
        flag_shot = true;
        if (count_shot >= 6)
        {
          shotdeg_lowlimit = 10;
          shotkp = 0;
          shotki = 0;
          shotkd = 0;
          slide_tgt = -60;
          azimuth_tgt = 0;
          roller_tgt = 0;
          flag2 = false;
          flag3 = true;
          AS5 = false;
          AS2 = false;
        }
      }
    }
    //回収////////////////////////////////////////////////////////////////
    //if (master_pic_order > 0 && flag4 == true)
    if (digitalRead(S1) < 1 && flag3 == true)
    {
      shotdeg_lowlimit = 10;
      shotkp = 45;
      shotki = 7;
      shotkd = 7;
      shotdeg_tgt = 0.5;
      AS2 = false;
      AS1 = true;
      AS4 = false;
      count_SWs = 0;
      flag_SWs = false;
      flag3 = false;
      flag4 = true;
    }
    //if (master_pic_order > 0 && flag5 == true)
    if (digitalRead(S2) < 1 && flag4 == true && flag_SWs == true)
    {
      AS3 = !AS3;
      count_SWs = 0;
      flag_SWs = false;
    }
    //if(master_prepare_order > 0 && flag5 == true)
    if (digitalRead(S1) < 1 && flag4 == true && flag_SWs == true)
    {
      AS4 = !AS4;
      count_SWs = 0;
      flag_SWs = false;
    }
    //////////////////////////////////////////////////////////////////////
    if (speed_azimuth >= 0)
    {
      roboclaw.BackwardM1(RC1_ad, speed_azimuth);
    }
    else if (speed_azimuth < 0)
    {
      speed_azimuth = speed_azimuth * (-1);
      roboclaw.ForwardM1(RC1_ad, speed_shotdeg);
    }

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
    roboclaw.SpeedM1(RC3_ad, speed_shot);
    roboclaw.SpeedM2(RC3_ad, speed_shot);
    digitalWrite(AS1_PIN, AS1);
    digitalWrite(AS2_PIN, AS2);
    digitalWrite(AS3_PIN, AS3);
    digitalWrite(AS4_PIN, AS4);
    digitalWrite(AS5_PIN, AS5);
    //エンコーダーリセット/////////////////////////////////////////////////
    if (digitalRead(boardSW) < 1)
    {
      /*AbsoluteENC.AMT203_set_zero1();
      AbsoluteENC.AMT203_set_zero2();
      encval_azimuth = 0;
      encval_shotdeg = 0;
      encr_azimuth = 0;
      encr_shotdeg = 0;
      encpast_azimuth = 0;
      encpast_shotdeg = 0;*/
      flag_serial = !flag_serial;
    }
    /////////////////////////////////////////////////////////////////////
    if (flag_serial == true)
    {
      Serial.print("tuusin-");
      if (read_mfs() == true)
      {
        Serial.print("success ");
      }
      else if (read_mfs() == false)
      {
        Serial.print("failure ");
      }
      /*Serial.print(" flag0 ");
    Serial.print(flag0);
    Serial.print(" flag_slide ");
    Serial.print(flag_slide);
    Serial.print(" flag1 ");
    Serial.print(flag1);
    Serial.print(" flag2 ");
    Serial.print(flag2);
    Serial.print(" flag3 ");
    Serial.print(flag3);
    Serial.print(" flag4 ");
    Serial.print(flag4);
    Serial.print(" flag_shot ");
    Serial.print(flag_shot);
    Serial.print(" flag5 ");
    Serial.print(flag5);
    Serial.print(" flag6 ");
    Serial.print(flag6);
    Serial.print(" count_shot ");
    Serial.print(count_shot);
    Serial.print(" speed_shot ");
    Serial.println(speed_shot);*/

      /*Serial.print(" azimuth_tgt ");
    Serial.print(azimuth_tgt);
    Serial.print(" shotdeg_tgt ");
    Serial.print(shotdeg_tgt);
    Serial.print(" slide_tgt ");
    Serial.print(slide_tgt);*/
      //Serial.print(" KOUDEN ");
      //Serial.print(analogRead(KOUDEN));
      Serial.print(" encr_slide ");
      Serial.print(encr_slide);
      Serial.print(" encr_shotdeg ");
      Serial.print(encr_shotdeg);
      Serial.print(" encr_azimuth ");
      Serial.print(encr_azimuth);
      Serial.print(" speed_slide ");
      Serial.print(speed_slide);
      Serial.print(" speed_shotdeg ");
      Serial.print(speed_shotdeg);
      Serial.print(" speed_azimuth ");
      Serial.println(speed_azimuth);

      /*Serial.print(" mfs[0] ");
    Serial.print(mfs[0]);
    Serial.print(" mfs[1] ");
    Serial.print(mfs[1]);
    Serial.print(" mfs[2] ");
    Serial.print(mfs[2]);
    Serial.print(" mfs[3] ");
    Serial.print(mfs[3]);*/

      /*Serial.print(" master_pic_order ");
    Serial.print(master_pic_order);
    Serial.print(" master_release_order ");
    Serial.print(master_release_order);
    Serial.print(" master_prepare_order ");
    Serial.print(master_prepare_order);
    Serial.print(" master_shot_order ");
    Serial.print(master_shot_order);
    Serial.print("  programcheck ");
    Serial.println(programcheck);*/
    }
    if (flag_100ms)
    {
      digitalWrite(boardLED1, !digitalRead(boardLED1));
      flag_100ms = false;
    }
    flag_10ms = false;
  }
  digitalWrite(PIN_LED1, 0);
}