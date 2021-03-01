#ifndef _define_h_
#define _define_h_
/*#define master_collection_order 1 //回収命令
#define master_shot_order 2       //発射命令
#define master_initialize_order 4 //初期化命令*/

#define upper_collection_report 1 //回収完了報告
#define upper_stop_report 2       //停止命令
#define upper_shot_report 4       //発射完了報告
#define upper_initialize_report 8 //初期化完了報告

#define RC1_ad 129 //RoboClaw1のアドレス 方位角，射角
#define RC2_ad 128 //RoboClaw2のアドレス スライドレール
#define RC3_ad 130 //RoboClaw3のアドレス ローラー

#define KOUDEN 20//光電センサー
#define AS1_PIN 51//関節下(AS1)
#define AS2_PIN 50//関節上(AS2)
#define AS3_PIN 49//回収エアシリンダ(AS3)
#define AS4_PIN 48//射出用意エアシリンダ(AS4)
#define AS5_PIN 47//射出用エアシリンダ(AS5)
#define AS6_PIN 46
#define AS7_PIN 45
#define AS8_PIN 44

#define boardLED1 A0
#define boardLED2 A0

#define S1 55
#define S2 54

#define LED_status 43

#define M5ppr 1000

#endif