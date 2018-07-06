/*
   BME280.ino
   Created by Yuji Miyano on 2017/10/20
   description  : Pressure sensing with piezo.
   Use hardware : BLE Nano
   Use library  : BLE_API.h, Wire.h
   Reference    : BLE program based on    https://github.com/RedBearLab/nRF51822-Arduino/blob/S130/arduino-1.6.x/hardware/RBL/RBL_nRF51822/libraries/BLE_Examples/examples/SimpleChat/SimpleChat.ino
                  BME280 program based on https://github.com/SWITCHSCIENCE/BME280/blob/master/Arduino/BME280_I2C/BME280_I2C.ino
                  Float to series of bytes program based on http://forum.arduino.cc/index.php?topic=180456.0    
                  BME280 calculation based on http://akizukidenshi.com/download/ds/bosch/BST-BME280_DS001-10.pdf (p.23)
                  battery level based on http://bril-tech.blogspot.jp/2014/06/bluetoothsmartmbed-4.html
                  Changing analog ref voltage based on http://www.analog.com/jp/education/landing-pages/003/iot_lectureship/content_03.html
                                                   and https://lowreal.net/2016/07/18/3
*/

// #include <BLE_API.h>     // For v1.5 https://github.com/sandeepmistry/arduino-BLEPeripheral <-多分
#include <nRF5x_BLE_API.h>  // For v2.0 https://github.com/redbear/nRF5x/blob/master/nRF52832/arduino/arduino-1.8.0/hardware/RBL/RBL_nRF52832/cores/RBL_nRF52832/FEATURE_BLE/nRF5x_BLE_API.h
#include <Wire.h>

#define DEVICE_NAME     "CHAKUYOBAKO_0001"
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


/* Complete list of 16-bit Service IDs */
uint16_t    uuid16_list[] = {GattService::UUID_BATTERY_SERVICE};

/* Battery Level Service */
uint8_t            batt      = 72;    /* Battery level */
GattCharacteristic battLevel(GattCharacteristic::UUID_BATTERY_LEVEL_CHAR, (uint8_t *)&batt, sizeof(batt), sizeof(batt),
                           GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ);
 
/* GattService Infomation */
GattCharacteristic *BattChars[] = {&battLevel };
GattService        BattService(GattService::UUID_BATTERY_SERVICE , BattChars, sizeof(BattChars) / sizeof(GattCharacteristic *));

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
  Serial.begin(9600);
  Serial.attach(uart_handle); // 割り込み処理

//  adc_init_setting(); // アナログ入力の基準電圧変更
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
  ble.addService(BattService);  // バッテリー監視
  // set device name
//  ble.setDeviceName((const uint8_t *)"Simple Chat");
  ble.setDeviceName((const uint8_t *)DEVICE_NAME);
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
//  Serial.print("TEMP : ");
//  Serial.print(temp_act);
//  Serial.print(" DegC  PRESS : ");
//  Serial.print(press_act);
//  Serial.print(" hPa  HUM : ");
//  Serial.print(hum_act);
//  Serial.println(" %");

  delay(5000);
  
  byte byteArray[20] = {0x00};        // byte = uint8_t

  byteArray[0]  = (int)((temp_cal  >> 24) & 0xFF);
  byteArray[1]  = (int)((temp_cal  >> 16) & 0xFF);
  byteArray[2]  = (int)((temp_cal  >>  8) & 0xFF);
  byteArray[3]  = (int)( temp_cal         & 0xFF);
  byteArray[4]  = (int)((press_cal >> 24) & 0xFF);
  byteArray[5]  = (int)((press_cal >> 16) & 0xFF);
  byteArray[6]  = (int)((press_cal >>  8) & 0xFF);
  byteArray[7]  = (int)( press_cal        & 0xFF);
  byteArray[8]  = (int)((hum_cal   >> 24) & 0xFF);
  byteArray[9]  = (int)((hum_cal   >> 16) & 0xFF);
  byteArray[10] = (int)((hum_cal  >>  8)  & 0xFF);
  byteArray[11] = (int)( hum_cal          & 0xFF);

  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (byte *)byteArray, sizeof(byteArray));  // swift側でエラーが出る。
  
  batt = analogRead(A4) * 1.2 * 3;  // バッテリー電圧入力
//  Serial.print("  Battery Level : ");
//  Serial.print(batt);
//  Serial.println(" %");
  ble.updateCharacteristicValue(battLevel.getValueAttribute().getHandle(), (byte *)batt, sizeof(batt));
 
  ble.waitForEvent();
  
}

// https://devzone.nordicsemi.com/question/49292/battery-service/
//static void battery_level_update(void)
//{
//        uint32_t err_code;
//        uint8_t  battery_level;
//
//        battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
//        //set the condition for sending notification
//        err_code = ble_bas_battery_level_update(&m_bas, battery_level);
//        if ((err_code != NRF_SUCCESS) &&
//                (err_code != NRF_ERROR_INVALID_STATE) &&
//                (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
//                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
//        )
//        {
//                APP_ERROR_HANDLER(err_code);
//        }
//}

//void adc_init_setting()   // ADC setting
//{
//    NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
//    NRF_ADC->CONFIG =
//    (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
//    (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
//    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
//    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
//
////   NRF_ADC->CONFIG = (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos) |
////                      (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
////                      (ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling << ADC_CONFIG_REFSEL_Pos) |
////                      (analogInputPin << ADC_CONFIG_PSEL_Pos) |
//
//}

//void tickerCallback(void)
//{
//     
//    /* Update battery level */
////    batt = ana_bat.read_u16() >> 2; // nRF51822 ADC is 10 bit resolution
//    batt = analogRead(A5);  // バッテリー電圧入力
//    ble.updateCharacteristicValue(battLevel.getValueAttribute().getHandle(), (byte *)batt, sizeof(batt));
// 
//}

