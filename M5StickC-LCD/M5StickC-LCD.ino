#include <Wire.h>
#include <SPI.h>
#define AXP192ADDR 0x34
#define BM8563ADDR 0x51

#include "image2.h"
#include "picture.h"
#include "font8x16.h"
#include "bitmapdata.h"

#define TFT_CS     5  // Data/Command
#define TFT_MOSI  15  // Master In Slave Out
#define TFT_CLK   13  // Serial Clock
#define TFT_DC    23  // Data/Command
#define TFT_RST   18  // RESET

#define TFT_WIDTH 135
#define TFT_HIGH  240

#define HBYTE(x) ((x >> 8) & 0xFF)
#define LBYTE(x) (x & 0xFF)

int LCD_DIR = 0 ;
int LCD_XZPoint = 52 ;
int LCD_YZPoint = 40 ;
int LCD_W = TFT_WIDTH ;
int LCD_H = TFT_HIGH ;

uint8_t SPIBuf[1024] ;   // SPI transfer buffer
uint8_t fontColorF[2] ; // TEXT FOR GROND COLOR
uint8_t fontColorB[2] ; // TEXT BACK GROND COLOR
char str[64] ;
uint8_t SEGDISP[1024] ;  // 7SEG DISP buffer

const int btnPin = 37;
const int rstPin = 39;

SPISettings settings = SPISettings(32000000, MSBFIRST, SPI_MODE0);

// --- Recive I2C Data
uint8_t ReadByte(uint8_t i2cAddr , uint8_t addr) {
  Wire1.beginTransmission(i2cAddr);
  Wire1.write(addr);
  Wire1.endTransmission();
  Wire1.requestFrom(i2cAddr, 1);
  while (Wire1.available() < 1) ;
  uint8_t val = Wire1.read();
  return val;
}

// --- Send I2C Data
void WriteByte(uint8_t i2cAddr , uint8_t addr, uint8_t data) {
  Wire1.beginTransmission(i2cAddr);
  Wire1.write(addr);
  Wire1.write(data);
  Wire1.endTransmission();
}

// --- Send command and data to TFT
void SPISendCommand(uint8_t command, uint8_t n=0,
                    uint8_t data1=0, uint8_t data2=0, uint8_t data3=0,
                    uint8_t data4=0, uint8_t data5=0, uint8_t data6=0) {
  // Send command
  digitalWrite(TFT_CS,LOW); 
  digitalWrite(TFT_DC, LOW);
  SPI.transfer(command);
  // Send data
  digitalWrite(TFT_DC, HIGH);
  if (n >= 1) SPI.transfer(data1);
  if (n >= 2) SPI.transfer(data2);
  if (n >= 3) SPI.transfer(data3);
  if (n >= 4) SPI.transfer(data4);
  if (n >= 5) SPI.transfer(data5);
  if (n >= 6) SPI.transfer(data6);
  digitalWrite(TFT_CS,HIGH);
}

// Send command to TFT
void tftSendCommand(uint8_t command) {
  SPISendCommand(command) ;
}

// Send command + 1 byte data to TFT
void tftSendCommand(uint8_t command, uint8_t data1) {
  SPISendCommand(command,1,data1) ;
}

// Send command + 2 byte data to TFT
void tftSendCommand(uint8_t command, uint8_t data1, uint8_t data2) {
  SPISendCommand(command,2,data1,data2) ;
}

// Send command + 3 byte data to TFT
void tftSendCommand(uint8_t command, uint8_t data1, uint8_t data2, uint8_t data3) {
  SPISendCommand(command,3,data1,data2,data3) ;
}

// Send command + 4 byte data to TFT
void tftSendCommand(uint8_t command, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4) {
  SPISendCommand(command,4,data1,data2,data3,data4) ;
}

// Send command + 6 byte data to TFT
void tftSendCommand(uint8_t command,uint8_t data1,uint8_t data2,uint8_t data3,uint8_t data4,uint8_t data5,uint8_t data6) {
  SPISendCommand(command,4,data1,data2,data3,data4,data5,data6) ;
}

// screen clear
void cls( ) {
//  tftSendCommand(0x2A,HBYTE(LCD_XZPoint),LBYTE(LCD_XZPoint),HBYTE(LCD_XZPoint+LCD_W-1),LBYTE(LCD_XZPoint+LCD_W-1)) ; // Colmun Address
//  tftSendCommand(0x2B,HBYTE(LCD_YZPoint),LBYTE(LCD_YZPoint),HBYTE(LCD_YZPoint+LCD_H-1),LBYTE(LCD_YZPoint+LCD_H-1)) ; // Row Address
  tftSendCommand(0x2A,HBYTE(0),LBYTE(0),HBYTE(239),LBYTE(239)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(0),LBYTE(0),HBYTE(319),LBYTE(319)) ; // Row Address

  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
//  for (int i=0;i<LCD_H;i++) {
//    for (int j=0;j<LCD_W;j++) {
  for (int i=0;i<320;i++) {
    for (int j=0;j<240;j++) {
      SPI.transfer(0x00);
      SPI.transfer(0x00);
    }
  }
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS, HIGH);  //
}

// fill
void fill(int xs,int ys , int xe,int ye , uint8_t c1 , uint8_t c2) {
  tftSendCommand(0x2A,HBYTE(xs+LCD_XZPoint),LBYTE(xs+LCD_XZPoint),HBYTE(xe+LCD_XZPoint),LBYTE(xe+LCD_XZPoint)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(ys+LCD_YZPoint),LBYTE(ys+LCD_YZPoint),HBYTE(ye+LCD_YZPoint),LBYTE(ye+LCD_YZPoint)) ; // Row Address

  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
  for (int i=0;i<xe-xs+1;i++) {
    for (int j=0;j<ye-ys+1;j++) {
      uint8_t dw = c1 ;
      SPI.transfer(dw);
      dw = c2 ;
      SPI.transfer(dw);
    }
  }
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS,HIGH);  //
}

// fill Box
void fillBox(int xs,int ys , int xe,int ye , uint8_t c1 , uint8_t c2 , uint8_t c3 , uint8_t c4) {
  tftSendCommand(0x2A,HBYTE(xs+LCD_XZPoint),LBYTE(xs+LCD_XZPoint),HBYTE(xe+LCD_XZPoint),LBYTE(xe+LCD_XZPoint)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(ys+LCD_YZPoint),LBYTE(ys+LCD_YZPoint),HBYTE(ye+LCD_YZPoint),LBYTE(ye+LCD_YZPoint)) ; // Row Address

  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
  for (int i=0;i<ye-ys+1;i++) {
    for (int j=0;j<xe-xs+1;j++) {
      if (i==0 || j== (xe-xs) || j == 0 || i == (ye-ys)) {
        uint8_t dw = c1 ;
        SPI.transfer(dw);
        dw = c2 ;
        SPI.transfer(dw);
      } else {
        uint8_t dw = c3 ;
        SPI.transfer(dw);
        dw = c4 ;
        SPI.transfer(dw);
      }
    }
  }
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS,HIGH);  //
}

// BIT map display
void drowBitMap(int offsetX , int offsetY , int width, int hight, uint8_t bitMap[]) {
  uint8_t endX = offsetX + width - 1;
  uint8_t endY = offsetY + hight - 1;
  tftSendCommand(0x2A,HBYTE(offsetX+LCD_XZPoint),LBYTE(offsetX+LCD_XZPoint),HBYTE(endX+LCD_XZPoint),LBYTE(endX+LCD_XZPoint)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(offsetY+LCD_YZPoint),LBYTE(offsetY+LCD_YZPoint),HBYTE(endY+LCD_YZPoint),LBYTE(endY+LCD_YZPoint)) ; // Row Address
  delayMicroseconds(500);

  int pos = 0 ;
  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
  for (int row = 0 ; row < hight ; row ++) {
    // display data transfer
    // With Arduino, it is not possible to transfer one screen at a time due to lack of memory
    // So, we are sending 1 line at a time
    for (int col = 0 ; col < width * 2 ; col ++ ) {
       SPIBuf[col] = bitMap[pos];
       pos ++ ;
    }
    SPI.transfer(SPIBuf, width * 2);
  }
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS,HIGH);  //
}

// PICTURE display
void drowPicture( ) {
  tftSendCommand(0x2A,HBYTE(0),LBYTE(0),HBYTE(239),LBYTE(239)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(0),LBYTE(0),HBYTE(319),LBYTE(319)) ; // Row Address
  delayMicroseconds(500);
  int pos = 0 ;
  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
  for (int row = 0 ; row < 320 ; row ++) {
    // display data transfer
    // With Arduino, it is not possible to transfer one screen at a time due to lack of memory
    // So, we are sending 1 line at a time
    for (int col = 0 ; col < 240 * 2 ; col ++ ) {
       SPIBuf[col] = pictureData1[pos];
       pos ++ ;
    }
    SPI.transfer(SPIBuf, 240 * 2);
  }
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS,HIGH);  //
}

void DrowFont(uint8_t code , uint8_t x, uint8_t y) {
  int pos = (code - 0x20) * 16 ;
  memset(SPIBuf,0,8*16*2) ;
  for (int i=0;i<16;i++) {
    uint8_t data = ssd1306xled_font8x16[pos + i];
    uint8_t bitMast = 0x01 ; 
    for (int j=0;j<8;j++) {
      int fpos = 0 ;
      if (i<8) {
        fpos = (j*8+i)*2 ;
      } else {
        fpos = ((j+8)*8+(i-8))*2 ;
      }
      if ((data & bitMast) == 0) {
        SPIBuf[fpos] = fontColorB[0] ;
        SPIBuf[fpos+1] = fontColorB[1] ;
      } else {
        SPIBuf[fpos] = fontColorF[0] ;
        SPIBuf[fpos+1] = fontColorF[1] ;
      }
      bitMast <<= 1 ;
    }
  }

  uint8_t endX = x + 7 ;
  uint8_t endY = y + 15;

  tftSendCommand(0x2A,HBYTE(x+LCD_XZPoint),LBYTE(x+LCD_XZPoint),HBYTE(endX+LCD_XZPoint),LBYTE(endX+LCD_XZPoint)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(y+LCD_YZPoint),LBYTE(y+LCD_YZPoint),HBYTE(endY+LCD_YZPoint),LBYTE(endY+LCD_YZPoint)) ; // Row Address
  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
  SPI.transfer(SPIBuf, 8*16*2);
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS,HIGH);  // 
  delayMicroseconds(500);
}

void DrowStr(char str[] , uint8_t x , uint8_t y) {
  for (int i=0;str[i] != 0 ; i++) {
      DrowFont(str[i],x,y) ;
      x += 8 ;
      if (x+8>=96) {
        x = 0 ;
        y += 16 ;
        if (y + 16 >= 64) {
          y = 0 ; 
        }
      }
  }
}

// 7セグメント調フォント表示
void disp7SEG(int x , int y , int num) {
  uint8_t bitMapBuff[24*4];  // 7セグっぽいフォントの合成用バッファ
  int width = 16 ;
  int heigh = 4 ;

  if (num < 10) {
    if (num < 0) { num = 10 ; }
    bool flag7[7] ;
    for (int i=0;i<7;i++) {
      flag7[i] = addMap[num][i] ; 
    }
    for (int i = 0; i < 16*4; i++) {
        bitMapBuff[i] = 0 ;
      if (flag7[0]) { bitMapBuff[i] |= SEGa[i] ; }  // a
      if (flag7[1]) { bitMapBuff[i] |= SEGb[i] ; }  // b
      if (flag7[2]) { bitMapBuff[i] |= SEGc[i] ; }  // c
      if (flag7[3]) { bitMapBuff[i] |= SEGd[i] ; }  // d
      if (flag7[4]) { bitMapBuff[i] |= SEGe[i] ; }  // e
      if (flag7[5]) { bitMapBuff[i] |= SEGf[i] ; }  // f
      if (flag7[6]) { bitMapBuff[i] |= SEGg[i] ; }  // g
    }
  } else
  if (num > 100) { // CLEAR
    width = num % 100 ;
    heigh = num / 100 ;
    memset(bitMapBuff,0,width*heigh) ;
  } else {
    uint8_t *bitMap ;
    switch(num) {
      case 10: bitMap = SEGdot ; break ; // DOT
      case 11: bitMap = SEGg ; break ; // -
      case 12: bitMap = base7SEG ; break ; // Body
      case 13: bitMap = SEGcoron ; break ; // coron
    }
    for (int i = 0; i < width*heigh; i++) {
      bitMapBuff[i] = bitMap[i] ; 
    } 
  }
  // <- bitMapBuff [2][16]

  // --- Drow FONT
  memset(SEGDISP,0,16*32*2) ;
  int fpos = 0 ;
  for (int w=0;w<width;w++) {
    for (int h=0;h<heigh;h++) {
      uint8_t bitMast = 0x01 ; 
      uint8_t data = bitMapBuff[h*width+w];
      for (int j=0;j<8;j++) {
        fpos = ((h*8+j)*width + w) * 2 ;
        if ((data & bitMast) == 0) {
          SEGDISP[fpos] = fontColorB[0] ;
          SEGDISP[fpos+1] = fontColorB[1] ;
        } else {
          SEGDISP[fpos] = fontColorF[0] ;
          SEGDISP[fpos+1] = fontColorF[1] ;
        }
        bitMast <<= 1 ;
      }
    }
  }

  uint8_t endX = x + width - 1;
  uint8_t endY = y + heigh * 8 -1 ;
  int datalen = width*(heigh*8)*2 ;

  tftSendCommand(0x2A,HBYTE(x+LCD_XZPoint),LBYTE(x+LCD_XZPoint),HBYTE(endX+LCD_XZPoint),LBYTE(endX+LCD_XZPoint)) ; // Colmun Address
  tftSendCommand(0x2B,HBYTE(y+LCD_YZPoint),LBYTE(y+LCD_YZPoint),HBYTE(endY+LCD_YZPoint),LBYTE(endY+LCD_YZPoint)) ; // Row Address
  digitalWrite(TFT_CS,LOW);
  digitalWrite(TFT_DC, LOW); // Command mode
  SPI.transfer(0x2C);
  digitalWrite(TFT_DC, HIGH); // Data mode
  SPI.transfer(SEGDISP, datalen);
  digitalWrite(TFT_DC, LOW); // Command mode
  digitalWrite(TFT_CS,HIGH);  // 
  delayMicroseconds(500);
}

void SET_LCD_DIR(int dir) {
  switch(dir) {
  case 0:
  default:
    tftSendCommand(0x36,B00000000) ; // MX MY MV ML RGB MH x x：Portrait-1
    LCD_DIR = 0 ;
    LCD_XZPoint = 52 ;
    LCD_YZPoint = 40 ;
    LCD_W = TFT_WIDTH ;
    LCD_H = TFT_HIGH ;
    break ;

  case 1:
    tftSendCommand(0x36,B10100000) ; // MX MY MV ML RGB MH x x:sideways -1
    LCD_DIR = 1 ;
    LCD_XZPoint = 40 ;
    LCD_YZPoint = 52 ;
    LCD_W = TFT_HIGH ;
    LCD_H = TFT_WIDTH ;
    break ;

  case 2:
    tftSendCommand(0x36,B11000000) ; // MX MY MV ML RGB MH x x：Portrait-2
    LCD_DIR = 2 ;
    LCD_XZPoint = 53 ;
    LCD_YZPoint = 40 ;
    LCD_W = TFT_WIDTH ;
    LCD_H = TFT_HIGH ;
    break ;

  case 3:
    tftSendCommand(0x36,B01100000) ; // MX MY MV ML RGB MH x x：sideways -1
    LCD_DIR = 3 ;
    LCD_XZPoint = 40 ;
    LCD_YZPoint = 53 ;
    LCD_W = TFT_HIGH ;
    LCD_H = TFT_WIDTH ;
    break ;
  }
}

void setup() {
  Serial.begin(9600);

  SPI.begin(TFT_CLK,-1,TFT_MOSI,TFT_CS);  //SPI.begin(SCK, MISO, MOSI, SS);
  SPI.beginTransaction(settings);

  // --- POWER CONTROL ---
  Wire1.begin(21, 22);
  // EXT Power ON
  uint8_t val = ReadByte(AXP192ADDR,0x12);
  val |= 0x4C;  // EXT / BACK LIGHT /LCD - ON
  WriteByte(AXP192ADDR , 0x12, val);
  // TFT & BAACK LIGHT POWER ON
  WriteByte(AXP192ADDR , 0x28, 0x8F); // BakcLight-4Bit(0~F) : LCD-4Bit(0~F)

  // -----  ST3375 INITIAL -----
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);

  // --- HARD Ware Reset
  digitalWrite(TFT_RST, HIGH);
  delay(500);                  // VDD goes high at start, pause for 500 ms
  digitalWrite(TFT_RST, LOW);  // Bring reset low
  delay(100);                  // Wait 100 ms
  digitalWrite(TFT_RST, HIGH); // Bring out of reset
  delay(500);                  // Wait 500 ms, more then 120 ms
  // --- SOFT Ware Reset
  tftSendCommand(0x01) ;       // SOFTWARE RESET
  delay(50);

  // --- Initial Comands
  bool r = false ;
  tftSendCommand(0x28) ;            // Display OFF : 0x29 - Display ON
  delay(500);

  tftSendCommand(0x11) ;            // Sleep Out
  delay(500);
  tftSendCommand(0x3A,05) ;        // 16Bit Pixel Mode
  delay(10);

  tftSendCommand(0xB6,0x15,0x02) ; // Display settings #5
  tftSendCommand(0xB4,0x00) ;      // Display inversion control
  tftSendCommand(0x13) ;            // NomalDisplayMode
  tftSendCommand(0x21) ;            // Display Inversion Off

  SET_LCD_DIR(0) ;
  cls( ) ;
  delay(500);
  tftSendCommand(0x29) ;            // Display ON : 0x28 - Display OFF
  delay(500);

  // ----
  fontColorB[0] = 0x00 ; fontColorB[1] = 0x00 ; // TEXT BACK GROND COLOR
  fontColorF[0] = 0xFF ; fontColorF[1] = 0xFF ; // TEXT FOR GROND COLOR - WHITE

  // ---
  pinMode(btnPin, INPUT);
  pinMode(rstPin, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  // ----- SCROLL
  SET_LCD_DIR(0) ;
  cls( ) ;
  tftSendCommand(0x12) ;            // Partial Display Mode
  tftSendCommand(0x33,HBYTE(0),LBYTE(0),HBYTE(320),LBYTE(320),HBYTE(0),LBYTE(0)) ; // SCROLL AREA
  drowPicture( ) ;
  delay(2000) ;
  for (int i=0;i<320;i++) {
    tftSendCommand(0x37,HBYTE(i),LBYTE(i)) ; // SCROLL START ADDR
    delay(10) ;
  }
  for (int i=319;i>=0;i--) {
    tftSendCommand(0x37,HBYTE(i),LBYTE(i)) ; // SCROLL START ADDR
    delay(10) ;
  }
  delay(1000) ;
  tftSendCommand(0x13) ;            // NomalDisplayMode

  for (int i=0;i<4;i++) {
    delay(1000);

    SET_LCD_DIR(i) ;
    cls( ) ;
    fillBox(0,0,LCD_W-1,LCD_H-1,B11111111,B11111111,B00000000,B00000000) ;
    fillBox(2,2,18,18,B11111000,B00000000,B00000000,B00000000) ;

    sprintf(str,"DIR-%d",i);
    DrowStr(str,18,2) ;
    drowBitMap(5,23,96,64,bitmapData2) ;
    fill(5, 90,101, 99,B11111000,B00000000) ;
    fill(5,102,101,111,B00000111,B11100000) ;
    fill(5,113,101,122,B00000000,B00011111) ;
 }

 // --- RYC
 SET_LCD_DIR(3) ;
 for (int i=0;i<60;i++) {
   uint8_t sec = ReadByte(BM8563ADDR,0x02);
   uint8_t min = ReadByte(BM8563ADDR,0x03);
   uint8_t hou = ReadByte(BM8563ADDR,0x04);
   uint8_t houH = (hou & 0x30) >> 4 ;
   uint8_t houL = hou & 0x0F ;
   uint8_t minH = (min & 0x70) >> 4;
   uint8_t minL = min & 0x0F ;
   uint8_t selH = (sec & 0x70) >> 4;
   uint8_t selL = sec & 0x0F ;
  
   disp7SEG(110,23,houH) ;
   disp7SEG(126,23,houL) ;
   disp7SEG(140,23,13) ;
   disp7SEG(152,23,minH) ;
   disp7SEG(170,23,minL) ;
   disp7SEG(184,23,13) ;
   disp7SEG(196,23,selH) ;
   disp7SEG(214,23,selL) ;

   delay(1000) ;
 }
 
}
