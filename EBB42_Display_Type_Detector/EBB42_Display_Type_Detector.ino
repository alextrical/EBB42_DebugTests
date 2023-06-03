#include <U8g2lib.h>
#include <Wire.h>

#define SDAPin          PB4
#define SCLPin          PB3

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_1(U8G2_R0, /* reset=*/U8X8_PIN_NONE);  //SSD1306
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2_2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);   //SH1106
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2_3(U8G2_R0, /* reset=*/ PD3);   //SSD1309
U8G2 *u8g2;

void setup() {
  Serial.begin(9600);
  Wire.setSDA(SDAPin);
  Wire.setSCL(SCLPin);
  Wire.begin();
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  Serial.println("Display Type Detector V0.1");

  byte OLEDid = getOLEDid(0x3C); //Address of the display to be checked
  
  Serial.print("/nDisplay type: ");
  if (OLEDid == 0x6) { //SD1306 128x64 
    u8g2 = &u8g2_1;
    Serial.print("SSD1306 128x64");
  } else if (OLEDid == 0x8) { //SH1106
    u8g2 = &u8g2_2;
    Serial.print("SH1106");
  } else if (OLEDid == 0x1) { //SSD1309
    u8g2 = &u8g2_3;
    Serial.print("SSD1309");
  } else {
    Serial.print("UNKNOWN");
  }
  
  u8g2->begin();

  u8g2->firstPage();
  do {
    u8g2->setFont(u8g2_font_ncenB10_tr);
    u8g2->drawStr(15, 24, "Hello World!");

    //Show the display type
    if (OLEDid == 0x6) { //SD1306 128x64 
      u8g2->drawStr(32, 48, "SSD1306!");
    } else if (OLEDid == 0x8) {
      u8g2->drawStr(35, 48, "SH1106!");
    } else if (OLEDid == 0x1) { //SSD1309
      u8g2->drawStr(35, 48, "SSD1309!");
    }

  } while (u8g2->nextPage());
}

void loop() {
}

byte getOLEDid(int address) {
  byte buffer[0];

  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.requestFrom(address, 1);  // This register is 8 bits = 1 byte long
  if (Wire.available() > 0) {
    Wire.readBytes(buffer, 1);
  }
  Wire.endTransmission();
  
  buffer[0] &= 0x0f;        // mask off power on/off bit
  Serial.println(buffer[0],HEX);
  return buffer[0];  //0x1 = SSD1309, 0x3 = SSD1306 128x32, 0x6 = SSD1306 128x64, 0x7 || 0xf = SH1107, 0x8 = SH1106
}