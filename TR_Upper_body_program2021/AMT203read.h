/*
このライブラリはGr-SAKURAでのAMT203をSPI通信を利用しての読み取りを省略するものです．
MTU1はP25,P24. MTU2はP23,P22. TPU1はP2,P3.TPU2はP0,P1で読み取れます．
詳しい使い方は[AMT203read_sample_proglam.ino]やWikiを見てください．
制作:柴崎　亘
制作日:2021/2/3
version:1.0
*/
#ifndef _AMT203read_h_
#define _AMT203read_h_

#include"SPI.h"
#include"Arduino.h"


class AMT203read{
    public:
    AMT203read(bool b);
     void AMT203_SPI_set(int CS1,int CS2);
     void AMT203_set_zero1();
     void AMT203_set_zero2();
     double  AMT203_read1(int mode);
     double  AMT203_read2(int mode);

    private:
    bool _b;
    uint8_t SPIWrite1(uint8_t sendByte);
    uint8_t SPIWrite2(uint8_t sendByte);
};
#endif
