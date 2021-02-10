
#include"SPI.h"
#include"Arduino.h"
#include"AMT203read.h"

#define timoutLimit 500
#define over 3500

#define nop 0x00
#define rd_pos 0x10
#define set_zero_point 0x70

int CS1;
int CS2;

AMT203read::AMT203read(bool b)
{
  _b=b; 
}
void AMT203read::AMT203_SPI_set(int cs1,int cs2)//繧ｻ繝�繝医い繝�繝�
{
  CS1=cs1;
  CS2=cs2;
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CS1, OUTPUT);
  pinMode(CS2, OUTPUT);
  SPISettings Settings(500000, MSBFIRST, SPI_MODE0);
  SPI.begin();
  digitalWrite(CS1, HIGH);
  digitalWrite(CS2, HIGH);
}
double AMT203read::AMT203_read1(int mode)
{
  uint8_t data1;
  uint8_t timeoutCounter1;
  uint16_t currentPosition1; 

  while(true)
  {

    timeoutCounter1 = 0;

    data1 = SPIWrite1(rd_pos);

    while (data1 != rd_pos && timeoutCounter1++ < timoutLimit)
    {
      data1 = SPIWrite1(nop);
      return 65535;
    }

    if (timeoutCounter1 < timoutLimit)
    {
      currentPosition1 = (SPIWrite1(nop)& 0x0F) << 8;

      currentPosition1 |= SPIWrite1(nop);
    }
    else
    {

    }
    if(mode==0){
    }
    else if(mode==1){
      currentPosition1 = currentPosition1 * 0.08789;
    }
  }
}

double AMT203read::AMT203_read2(int mode)
{
  uint8_t data2;
  uint8_t timeoutCounter2;
  uint16_t currentPosition2;
  
  while(true)
  {

    timeoutCounter2 = 0;

    data2 = SPIWrite2(rd_pos);

    while (data2 != rd_pos && timeoutCounter2++ < timoutLimit)
    {
      data2 = SPIWrite2(nop);
    }

    if (timeoutCounter2 < timoutLimit)
    {
      currentPosition2 = (SPIWrite2(nop)& 0x0F) << 8;

      currentPosition2 |= SPIWrite2(nop);
    }
    else
    {

    }
    if(mode==0){
      return currentPosition2;
    }
    else if(mode==1){
      currentPosition2 = currentPosition2 * 0.08789;
      return currentPosition2;
    }
  }
}

void AMT203read::AMT203_set_zero1()
{
  digitalWrite(CS1, LOW);
  SPI.transfer(set_zero_point);
  digitalWrite(CS1, HIGH);
  delayMicroseconds(10);
}

void AMT203read::AMT203_set_zero2()
{
  digitalWrite(CS2, LOW);
  SPI.transfer(set_zero_point);
  digitalWrite(CS2, HIGH);
  delayMicroseconds(10);

}

uint8_t AMT203read::SPIWrite1(uint8_t sendByte)
{
  uint8_t data;
  digitalWrite(CS1, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS1, HIGH);
  delayMicroseconds(10);
  return data;
}
uint8_t AMT203read::SPIWrite2(uint8_t sendByte)
{
  uint8_t data;
  digitalWrite(CS2, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS2, HIGH);
  delayMicroseconds(10);
  return data;
}




