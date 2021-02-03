#include<SPI.h>
#include<Arduino.h>

#define timelimit 500

#define nop 0x00
#define rd_pos 0x10
#define set_zero_point 0x70

int CS1;
int CS2;

uint8_t timeoutCounter;

void AMT203read::AMT203_SPI_set(int cs1 int cs2)//セットアップ
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
    uint8_t data1;               //this will hold our returned data from the AMT20
    uint16_t currentPosition1;   //this 16 bit variable will hold our 12-bit position

  while(true)
  {

    timeoutCounter = 0;
    
    data1 = SPIWrite1(rd_pos);
  
    while (data1 != rd_pos && timeoutCounter++ < timoutLimit)
    {
      data1 = SPIWrite1(nop);
    }
  
    if (timeoutCounter < timoutLimit)
    {
      currentPosition1 = (SPIWrite1(nop)& 0x0F) << 8;
  
      currentPosition1 |= SPIWrite1(nop);
    }
    else
    {

    }
    if(mode 0){
    }
    else if(mode 1){
        currentPosition1 = currentPosition1 * 0.08789;
    }
    return currentPosition1;
  }
}

double AMT203read::AMT203_read2(int mode)
{
    uint8_t data2;               //this will hold our returned data from the AMT20
    uint16_t currentPosition2;   //this 16 bit variable will hold our 12-bit position

  while(true)
  {

    timeoutCounter = 0;
    
    data2 = SPIWrite2(rd_pos);
  
    while (data2 != rd_pos && timeoutCounter++ < timoutLimit)
    {
      data2 = SPIWrite2(nop);
    }
  
    if (timeoutCounter < timoutLimit)
    {
      currentPosition2 = (SPIWrite2(nop)& 0x0F) << 8;
  
      currentPosition2 |= SPIWrite2(nop);
    }
    else
    {

    }
    if(mode 0){
    }
    else if(mode 1){
        currentPosition2 = currentPosition2 * 0.08789;
    }
    return currentPosition2;
  }
}

void AMT203read::AMT203_set_zero1()
{
    data1 = SPIWrite1(nop);
}

void AMT203read::AMT203_set_zero2()
{
    data2 = SPIWrite2(nop);
}

uint8_t SPIWrite1(uint8_t sendByte)
{
    uint8_t data;
    digitalWrite(CS1, LOW);
    data = SPI.transfer(sendByte);
    digitalWrite(CS1, HIGH);
    delayMicroseconds(10);
    return data;
}
uint8_t SPIWrite2(uint8_t sendByte)
{
    uint8_t data;
    digitalWrite(CS2, LOW);
    data = SPI.transfer(sendByte);
    digitalWrite(CS2, HIGH);
    delayMicroseconds(10);
    return data;
}