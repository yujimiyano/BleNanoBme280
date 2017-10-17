// 2017.10.11 notifyで1秒毎に送信

/*
   Copyright (c) 2016 RedBear

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/
#include <BLE_API.h>
#include <Wire.h>
//#include<stdlib.h>  // for dtostrf
//#include<avr/dtostrf.h> // for dtostrf

#define DEVICE_NAME     "BLE_Peripheral"
#define TXRX_BUF_LEN    20
#define BME280_ADDRESS  0x76  // SD0->GNDなら0x76

BLE                     ble;
Timeout                 timeout;

static uint8_t rx_buf[TXRX_BUF_LEN];
static uint8_t rx_buf_num;
static uint8_t rx_state = 0;

// The uuid of service and characteristics
static const uint8_t service1_uuid[]        = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]     = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]     = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]   = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,};
uint8_t rx_value[TXRX_BUF_LEN] = {0,};
//char rx_value[TXRX_BUF_LEN] = {0,};

// Initialize value of chars
GattCharacteristic  characteristic1(
  service1_tx_uuid,
  tx_value, 1, TXRX_BUF_LEN,
  GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
  GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE
);
GattCharacteristic  characteristic2(
  service1_rx_uuid,
  rx_value, 1, TXRX_BUF_LEN,
  GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);
GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};
GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

///* Health Thermometer Service */
//uint8_t thermTempPayload[5] = { 0, 0, 0, 0, 0 };
//GattCharacteristic tempChar (GattCharacteristic::UUID_TEMPERATURE_MEASUREMENT_CHAR,
//                             thermTempPayload, 5, 5,
//                             GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE);
//GattCharacteristic *htmChars[] = {&tempChar, };
//GattService htmService(service1_uuid, htmChars,
//                       sizeof(uartChars) / sizeof(GattCharacteristic *));
////GattService htmService(GattService::UUID_HEALTH_THERMOMETER_SERVICE, htmChars,
////                       sizeof(htmChars) / sizeof(GattCharacteristic *));

// Variables for BME280
unsigned long int hum_raw, temp_raw, pres_raw;
signed long int   t_fine;
uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;

// union
union float2bytes {
  float f;
  byte s[sizeof(float)];
};

// functions for BLE
void disconnectionCallBack(const Gap::DisconnectionCallbackParams_t *params) {
  Serial.println("Disconnected!");
  Serial.println("Restarting the advertising process");
  ble.startAdvertising();
}

void gattServerWriteCallBack(const GattWriteCallbackParams *Handler) {
  uint8_t  buf[TXRX_BUF_LEN];
  uint16_t bytesRead, index;

  Serial.println("onDataWritten : ");
  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
    Serial.print("bytesRead: ");
    Serial.println(bytesRead, HEX);
    for (index = 0; index < bytesRead; index++) {
      Serial.write(buf[index]);
    }
    Serial.println("");
  }
}

void m_uart_rx_handle() {   //update characteristic data
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);
  memset(rx_buf, 0x00, 20);
  rx_state = 0;
}

void uart_handle(uint32_t id, SerialIrq event) {   /* Serial rx IRQ */
  if (event == RxIrq) {
    if (rx_state == 0) {
      rx_state = 1;
      timeout.attach_us(m_uart_rx_handle, 100000);
      rx_buf_num = 0;
    }
    while (Serial.available()) {
      if (rx_buf_num < 20) {
        rx_buf[rx_buf_num] = Serial.read();
        rx_buf_num++;
      }
      else {
        Serial.read();
      }
    }
  }
}

// functions for BME280
void bme280setup()
{
  uint8_t osrs_t =    1;           //Temperature oversampling x 1
  uint8_t osrs_p =    1;           //Pressure oversampling x 1
  uint8_t osrs_h =    1;           //Humidity oversampling x 1
  uint8_t mode =      3;           //Normal mode
  uint8_t t_sb =      5;           //Tstandby 1000ms
  uint8_t filter =    0;           //Filter off
  uint8_t spi3w_en =  0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;

  Wire.begin();

  writeReg(0xF2, ctrl_hum_reg);
  writeReg(0xF4, ctrl_meas_reg);
  writeReg(0xF5, config_reg);
  readTrim();
}

void readTrim()
{
  uint8_t data[32], i = 0;
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 24);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xA1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 1);
  data[i] = Wire.read();
  i++;

  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xE1);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 7);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  dig_H1 = data[24];
  dig_H2 = (data[26] << 8) | data[25];
  dig_H3 = data[27];
  dig_H4 = (data[28] << 4) | (0x0F & data[29]);
  dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
  dig_H6 = data[31];
}
void writeReg(uint8_t reg_address, uint8_t data)
{
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(reg_address);
  Wire.write(data);
  Wire.endTransmission();
}


void readData()
{
  int i = 0;
  uint32_t data[8];
  Wire.beginTransmission(BME280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BME280_ADDRESS, 8);
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
  temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
  hum_raw  = (data[6] << 8) | data[7];
}


signed long int calibration_T(signed long int adc_T)
{

  signed long int var1, var2, T;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

unsigned long int calibration_P(signed long int adc_P)
{
  signed long int var1, var2;
  unsigned long int P;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
    return 0;
  }
  P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (P < 0x80000000)
  {
    P = (P << 1) / ((unsigned long int) var1);
  }
  else
  {
    P = (P / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
  P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
  return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
  signed long int v_x1;

  v_x1 = (t_fine - ((signed long int)76800));
  v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
            ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
                ((signed long int) dig_H2) + 8192) >> 14));
  v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
  v_x1 = (v_x1 < 0 ? 0 : v_x1);
  v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
  return (unsigned long int)(v_x1 >> 12);
}

void setup() {
  // put your setup code here, to run once
  Serial.begin(9600);
  Serial.attach(uart_handle); // 割り込み処理

  bme280setup();

  ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onDataWritten(gattServerWriteCallBack);

  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)"TXRX", sizeof("TXRX") - 1);
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                   (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev));
  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  // set device name
  ble.setDeviceName((const uint8_t *)"Simple Chat");
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();
  Serial.println("Advertising Start!");
}

void loop() {

  //  double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  float temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
  signed long int temp_cal;
  unsigned long int press_cal, hum_cal;

  readData();

  temp_cal = calibration_T(temp_raw);
  press_cal = calibration_P(pres_raw);
  hum_cal = calibration_H(hum_raw);
  temp_act = (double)temp_cal / 100.0;
  press_act = (double)press_cal / 100.0;
  hum_act = (double)hum_cal / 1024.0;
  Serial.print("TEMP : ");
  Serial.print(temp_act);
  Serial.print(" DegC  PRESS : ");
  Serial.print(press_act);
  Serial.print(" hPa  HUM : ");
  Serial.print(hum_act);
  Serial.println(" %");

  delay(1000);

  // test
//    byte buf[20];                             // byte = uint8_t
//    float tmpNum = 16.11;
//    String tmp = String(tmpNum, (unsigned char)2);      // String(実数?, 桁数)
//    String str = "TEMP : -16.11";
//    str.getBytes(buf, sizeof(buf));           // String->byteに変換
//    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (byte *)buf, sizeof(buf));  // 19だとOK、20だと表示間隔が増える

  //  int n = sprintf(buf, "%f", tmpNum);

  //  sprintf(buf,"%f", 16.11); // errorが出る invalid conversion from 'byte* {aka unsigned char*}' to 'char*' [-fpermissive]
  //  *((float *)buf) = tmpNum; // http://forum.arduino.cc/index.php?topic=180456.0 // nullになる

  //  buf[0] = (float)((tmpNum >> 24) & 0xFF) ; // errorが出る invalid operands of types 'float' and 'int' to binary 'operator>>'
  //  buf[1] = (float)((tmpNum >> 16) & 0xFF) ;
  //  buf[2] = (float)((tmpNum >> 8) & 0XFF);
  //  buf[3] = (float)((tmpNum & 0XFF));

  //// https://os.mbed.com/forum/helloworld/topic/2053/?page=1#comment-53016
  //float f = 1;  // null -> 4 byte little endianで格納される
  //uint8_t *p = (uint8_t*)&f;
  //for (int i = 0; i < sizeof(f); i++) printf("0x%02x ", p[i]);

//  // https://os.mbed.com/forum/helloworld/topic/2053/?page=1#comment-53016
    byte buf[20];                             // byte = uint8_t
//    float f = 16.11;
//    float2bytes f2b;
//    f2b.f = f;
//    for (int i = 0; i < sizeof(buf); i++) {
//      if (i == 0) buf[i] = f2b.s[4];
//      if (i == 1) buf[i] = f2b.s[3];
//      if (i == 2) buf[i] = f2b.s[2];
//      if (i == 3) buf[i] = f2b.s[1];
//      if (i == 4) buf[i] = f2b.s[0];
//      if (i == 5) buf[i] = 0x00;
//      if (i == 6) buf[i] = 0x01;
//    }
//    Serial.println(f2b.f);
//    Serial.print(f2b.s[0]);
//    Serial.print(f2b.s[1]);
//    Serial.print(f2b.s[2]);
//    Serial.print(f2b.s[3]);
//    Serial.println(f2b.s[4]);
//    Serial.println(buf[0]);
//    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (byte *)buf, sizeof(buf));  // 19だとOK、20だと表示間隔が増える

// 10/12に試してたやつ
//  // http://bradsduino.blogspot.jp/2012/11/converting-float-to-array-of-byte-s.html
//  float x = -16.11;
//  byte* b = (byte*) &x; // 4 byte little endianで格納される
//  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (byte *)b, sizeof(b));  // 19だとOK、20だと表示間隔が増える

    for (int i = 0; i < sizeof(buf); i++) {
      if (i == 0) buf[i] = byte('T');
      if (i == 1) buf[i] = byte('E');
      if (i == 2) buf[i] = byte('M');
      if (i == 3) buf[i] = byte('P');
      if (i == 4) buf[i] = byte(' ');
      if (i == 5) buf[i] = byte(':');
      if (i == 6) buf[i] = byte(' ');
      if (i == 7) buf[i] = byte('-');
      if (i == 8) buf[i] = byte('1');
      if (i == 9) buf[i] = byte('6');
      if (i == 10) buf[i] = byte('.');
      if (i == 11) buf[i] = byte('1');
      if (i == 12) buf[i] = byte('1');
      if (i == 13) buf[i] = 0x00;
      if (i == 14) buf[i] = 0x01;
  //    else buf[i] = 0x00;
    }
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (byte *)buf, sizeof(buf));  // 19だとOK、20だと表示間隔が増える


//  char buf[20];
//  float x = -16.11;
//  //  (byte *)buf = tempToAscii(x);
//  //  tempToAscii(x, buf);
//  sprintf(buf, "%f", x);
//  //  dtostrf(x, sizeof(x), 2, buf);

//  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (byte *)buf, sizeof(buf));  // 19だとOK、20だと表示間隔が増える

  //  uint32_t temp_ieee11073 = quick_ieee11073_from_float(11.1);
  //  memcpy(thermTempPayload+1, &temp_ieee11073, 4);
  //  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), thermTempPayload, sizeof(thermTempPayload)); //getHandle->getValueAttribute
  //  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t *)&temp_ieee11073, sizeof(temp_ieee11073)); //getHandle->getValueAttribute

  ble.waitForEvent();
}

//// https://playground.arduino.cc/Main/TempToAscii
//byte* tempToAscii(double temp)
//{
////  char ascii[32];
//  byte ascii[32];
//  int frac;
//  frac=(unsigned int)(temp*1000)%1000;  //get three numbers to the right of the deciaml point
//
//  itoa((int)temp,ascii,10);
//  strcat(ascii,".");
//  itoa(frac,&ascii[strlen(ascii)],10); //put the frac after the deciaml
//
//  return ascii;
//}

//// http://blog.automated.it/tag/arduino/
////void tempToAscii(double temp, char *buff) {
//void tempToAscii(double temp, byte *buff) {
//  int frac;
//  //get three numbers to the right of the decimal point
//  frac = (unsigned int)(temp * 1000) % 1000;
////  itoa((int)temp,buff,10);
//  itoa((int)temp, buff);
//
////  strcat(buff, ".");
//  my_strcat(buff, ".");
//  //put the frac after the decimal
//  itoa(frac, &buff[strlen(buff)]);
//}

//// https://os.mbed.com/forum/bugs-suggestions/topic/2319/
///* itoa:  convert n to characters in s */
//// void itoa(int n, char s[])
//void itoa(int n, char s[])
//{
//  int i, sign;
//
//  if ((sign = n) < 0)  /* record sign */
//    n = -n;          /* make n positive */
//  i = 0;
//  do {       /* generate digits in reverse order */
//    s[i++] = n % 10 + '0';   /* get next digit */
//  } while ((n /= 10) > 0);     /* delete it */
//  if (sign < 0)
//    s[i++] = '-';
//  s[i] = '\0';
//  reverse(s);
//}
//
//// http://wktcoder.blogspot.jp/2012/07/cstrcat.html
//// str1の後ろにstr2を連結する
//byte * my_strcat(byte* str1, const char* str2){
//    byte *top;
//
//    // (1) 返り値用に先頭アドレスを保持しておく
//    top=str1;
//
//    // (2) str1のポインタを最後まで進める
//    while(*str1++ != '¥0');
//    *str1--;
//
//    // (3)str1の背後から、str2の文字をコピーしていく
//    while( (*str1++ = *str2++) != '¥0');
//
//    return top;
//}

// https://forum.arduino.cc/index.php?topic=170564.0
// https://github.com/arduino/Arduino/blob/a2e7413d229812ff123cb8864747558b270498f1/hardware/arduino/sam/cores/arduino/avr/dtostrf.c
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
