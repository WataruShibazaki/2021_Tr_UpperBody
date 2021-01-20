#include "Arduino.h"
#include "ISO.h"
#define over 30000
double ppr_MTU1,ppr_MTU2,ppr_TPU1,ppr_TPU2;
void ISO::ISOkeisu_SET()//セットアップ
{
    SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; 
    SYSTEM.MSTPCRA.BIT.MSTPA13 = 0;
    MPC.PWPR.BIT.B0WI=0;
    MPC.PWPR.BIT.PFSWE=1;
}
void ISO::ISOkeisu_MTU1(int ppr)//P24 P25
{
    PORT2.PMR.BIT.B4=1;
    PORT2.PMR.BIT.B5=1;

    MPC.P24PFS.BIT.PSEL=0b00010;
    MPC.P25PFS.BIT.PSEL=0b00010; 

    MTU.TSTR.BIT.CST1=0;
    MTU1.TCR.BYTE=0;
    MTU1.TMDR.BIT.MD=0b0100;
    MTU1.TCNT=0;
    MTU1.TIOR.BIT.IOA=0b1010;
    MTU1.TIOR.BIT.IOB=0b1010;
    MTU.TSTR.BIT.CST1=1;

    ppr_MTU1 = ppr*4;
}
void ISO::ISOkeisu_MTU2(int ppr)//P22 P23
{
    PORT2.PMR.BIT.B2=1;
    PORT2.PMR.BIT.B3=1; 

    MPC.P22PFS.BIT.PSEL=0b00010;
    MPC.P23PFS.BIT.PSEL=0b00010;

    MTU.TSTR.BIT.CST2=0;
    MTU2.TCR.BYTE=0;
    MTU2.TMDR.BIT.MD=0b0100;
    MTU2.TCNT=0;
    MTU2.TIOR.BIT.IOA=0b1010;
    MTU2.TIOR.BIT.IOB=0b1010;
    MTU.TSTR.BIT.CST2=1;

    ppr_MTU2 = ppr*4;
}
void ISO::ISOkeisu_TPU1(int ppr)//P2 P3　Mstimer2に干渉あり
{
    PORTC.PMR.BIT.B2=1;
    PORTC.PMR.BIT.B3=1;

    MPC.PC2PFS.BIT.PSEL=0b00011;
    MPC.PC3PFS.BIT.PSEL=0b00011;

    TPUA.TSTR.BIT.CST1=0;
    TPU1.TCR.BYTE=0;
    TPU1.TMDR.BIT.MD=0b0100;
    TPU1.TCNT=0;
    TPU1.TIOR.BIT.IOA=0b1010;
    TPU1.TIOR.BIT.IOB=0b1010;
    TPUA.TSTR.BIT.CST1=1;

    ppr_TPU1 = ppr*4;
}
void ISO::ISOkeisu_TPU2(int ppr)//P0 P1  P0がSDカード回路に接続あり
{
    PORTC.PMR.BIT.B0=1;
    PORTC.PMR.BIT.B1=1;

    MPC.P16PFS.BIT.PSEL=0b00100;
    MPC.P17PFS.BIT.PSEL=0b00100; 

    TPUA.TSTR.BIT.CST2=0;
    TPU2.TCR.BYTE=0;
    TPU2.TMDR.BIT.MD=0b0100;
    TPU2.TCNT=0;
    TPU2.TIOR.BIT.IOA=0b1010;
    TPU2.TIOR.BIT.IOB=0b1010;
    TPUA.TSTR.BIT.CST2=1;

    ppr_TPU2 = ppr*4;
}
double ISO::ISOkeisu_read_MTU1(int mode)
{
    int isosabn1=0,isopast1=0;
    double isoval1=0,isoout1;
    if(mode == 0|mode==2){
        unsigned int iso1 = MTU1.TCNT;
        isosabn1=iso1-isopast1;
        if(isosabn1>over){
            isosabn1=isosabn1-65535;
        }
        if(isosabn1<-over){
            isosabn1=isosabn1+65535;
        }
        isoval1+=isosabn1;
        if(mode==2){
            isoout1 = isoval1*(360/ppr_MTU1);
        }
        else{
            isoout1 = isoval1;
        }
    }
    else{
        isoout1 = MTU1.TCNT;
    }
    return isoout1;
}
double ISO::ISOkeisu_read_MTU2(int mode)
{
    int isosabn2=0,isopast2=0;
    double isoval2=0,isoout2;
    if(mode==0|mode==2){
        unsigned int iso2 = MTU2.TCNT;
         isosabn2=iso2-isopast2;
         if(isosabn2>over){
             isosabn2=isosabn2-65535;
         }
         if(isosabn2<-over){
             isosabn2=isosabn2+65535;
         }
         isoval2+=isosabn2;
        if(mode==2){
            isoout2 = isoval2*(360/ppr_MTU2);
        }
        else{
            isoout2=isoval2;
        }
    }
    else{
        isoout2 = MTU2.TCNT;
    }
    return isoout2;
}
double ISO::ISOkeisu_read_TPU1(int mode)
{
    int isosabn3=0,isopast3=0;
    double isoval3=0,isoout3;
    if(mode==0|mode==2){
        unsigned int iso3 = TPU1.TCNT;
         isosabn3=iso3-isopast3;
         if(isosabn3>over){
             isosabn3=isosabn3-65535;
         }
         if(isosabn3<-over){
             isosabn3=isosabn3+65535;
         }
         isoval3+=isosabn3;
        if(mode==2){
            isoout3 = isoval3*(360/ppr_TPU1);
        }
        else{
            isoout3=isoval3;
        }
    }
    else{
        isoout3 = TPU1.TCNT;
    }
    return isoout3;
}
double ISO::ISOkeisu_read_TPU2(int mode)
{
    int isosabn4=0,isopast4=0;
    double isoval4=0,isoout4;
    if(mode==0|mode==2){
        unsigned int iso4 = TPU2.TCNT;
         isosabn4=iso4-isopast4;
         if(isosabn4>over){
             isosabn4=isosabn4-65535;
         }
         if(isosabn4<-over){
             isosabn4=isosabn4+65535;
         }
         isoval4+=isosabn4;
        if(mode==2){
            isoout4 = isoval4*(360/ppr_TPU2);
        }
        else{
            isoout4=isoval4;
        }
    }
    else{
       isoout4 = TPU2.TCNT;
    }
    return isoout4;
}
