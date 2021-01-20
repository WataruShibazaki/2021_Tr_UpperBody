/*
このライブラリはGr-SAKURAでの位相係数のセットアップを省略するものです．
MTU1はP25,P24. MTU2はP23,P22. TPU1はP2,P3.TPU2はP0,P1で読み取れます．
詳しい使い方は[ISO_sample_proglam.ino]やWikiを見てください．
制作:柴崎　亘
制作日:2021/1/14
version:1.2
*/
#ifndef _ISO_h_
#define _ISO_h_

namespace ISO{
    //public:
     void ISOkeisu_SET();
     void ISOkeisu_MTU1(int ppr);
     void ISOkeisu_MTU2(int ppr);
     void ISOkeisu_TPU1(int ppr);
     void ISOkeisu_TPU2(int ppr);
     double  ISOkeisu_read_MTU1(int mode);
     double  ISOkeisu_read_MTU2(int mode);
     double  ISOkeisu_read_TPU1(int mode);
     double  ISOkeisu_read_TPU2(int mode);

    //private:
}
#endif