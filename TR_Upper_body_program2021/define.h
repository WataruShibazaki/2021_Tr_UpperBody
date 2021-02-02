#ifndef _define_h_
#define _define_h_
#define master_collection_order 1 //回収命令
#define master_shot_order 2       //発射命令
#define master_initialize_order 4 //初期化命令

#define upper_collection_report 1 //回収完了報告
#define upper_stop_report 2       //停止命令
#define upper_shot_report 4       //発射完了報告
#define upper_initialize_report 8 //初期化完了報告

#define RC1_ad 0x80 //RoboClaw1のアドレス　128(DEC)
#define RC2_ad 0x80 //RoboClaw2のアドレス　128(DEC)
#define RC3_ad 0x80 //RoboClaw3のアドレス　128(DEC)

#define AS1_PIN 24;//関節下(AS1)
#define AS2_PIN 25;//関節上(AS2)
#define AS3_PIN 26;//回収エアシリンダ(AS3)
#define AS4_PIN 27;//射出用意エアシリンダ(AS4)
#define AS5_PIN 28;//射出用エアシリンダ(AS5)
#define AS6_PIN 29;
#define AS7_PIN 30;
#define AS8_PIN 31;

#endif