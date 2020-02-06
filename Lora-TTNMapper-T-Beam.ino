 /*
 * Copyright 2019 sbiermann - https://github.com/sbiermann/Lora-TTNMapper-ESP32
 * Copyright 2019 hottimuc  - https://github.com/hottimuc/Lora-TTNMapper-T-Beam
 * Copyright 2020 noppingen - https://www.onderka.com
 * 
 * UPDATE the gps.h    file in the same folder with your gateway GPS coordinates.
 * UPDATE the config.h file in the same folder with your TTN keys and device address.
 * 
 * Node:         TTGO T-Beam "T22_v1.0, 20190612"
 * ---------------------------------------------------------------------------------------------
 * LoRa-Modul:   Reset GPIO 23
 * GPS-Modul:    RX GPIO 34 and TX GPIO 12.
 * AXP192:       I2C is at SDA GPIO 21 and SCK GPIO 22
 * Button PWR:   2s: AXP192 on / 6s: AXP192 off (Charging via USB stays active)
 * LED Rot:      GPS fix, also on GPIO37

 * SF-CHANGE       < 2 SEC.
 * INTERVAL-CHANGE = BUTTON PRESS FOR MIN. 2 / MAX 4 SEC.
 * ADR             = BUTTON PRESS FOR MIN. 4 / MAX 6 SEC.
 * PORT-CHANGE     = BUTTON PRESS FOR MIN. 6 / MAX 8 SEC.
 * NOTHING         > 8 SEC.
 * 
 */

#include <lmic.h>
#include <hal/hal.h>
#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <axp20x.h>
#include "config.h"
#include "gps.h"
#include "gpsicon.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

// T-Beam specific hardware, middle button
#define SELECT_BTN 38

// OLED reset pin
#define OLED_RESET 4 
Adafruit_SSD1306 display(OLED_RESET);

// Power management chip AXP192
AXP20X_Class axp;
bool  axpIrq = 0;
#define AXP192_SLAVE_ADDRESS 0x34
const uint8_t axp_irq_pin = 35;

String LoraStatus;

/* Spread factor and TX interval */
int TX_Mode = 6;          // Default: SF7-14
int TX_Interval_Mode = 1; // Default: 30 second interval

char s[32]="";            // used to sprintf for Serial output
char sd[10]="";           // used to sprintf for Display sf-txpow output
char iv[10]="";           // used to sprintf for Display tx-interval output

// Misc
unsigned int blinkGps = 0;
boolean noFix = true;
boolean redraw = false;
boolean isDimmed = false;
boolean GPSonceFixed = false;
byte adr = 0;
byte port = 1;

// Set LED mode when packet is queued
// AXP20X_LED_OFF       - LED off
// AXP20X_LED_LOW_LEVEL - LED on
// AXP20X_LED_BLINK_1HZ - LED 1 blink/s
// AXP20X_LED_BLINK_4HZ - LED 4 blink/s
axp_chgled_mode_t ledMode = AXP20X_LED_LOW_LEVEL;

uint8_t txBuffer[9];
uint16_t txBuffer2[5];
gps gps;
Preferences prefs;

#ifndef OTAA
// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
#endif

static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
unsigned TX_INTERVAL = 15;
unsigned long lastMillis = 0;
unsigned long lastMillis2 = 0;

// For battery mesurement
float VBAT;  // battery voltage from ESP32 ADC read

const lmic_pinmap lmic_pins = {
  .nss = 18,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 23, // was "14,"
  .dio = {26, 33, 32},
};

void do_send(osjob_t* j) {  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    LoraStatus = "OP_TXRXPEND, not sending";
  } else { 
    if (gps.checkGpsFix()) {
      // Prepare upstream data transmission at the next possible time.
      gps.buildPacket(txBuffer);
      LMIC_setTxData2(port, txBuffer, sizeof(txBuffer), 0);
      Serial.println(F("EV_PKG_QUEUED [LED on]"));
      axp.setChgLEDMode(ledMode);
      LoraStatus = "PKG_QUEUED";
    } else {
      // Try again in 3 seconds
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(3), do_send);
    }
  }
  // Next TX: Scheduled after TX_COMPLETE event.
}

void sf_set() {
    if (TX_Mode==0) {
      LMIC_setDrTxpow(DR_SF7,17);
      sprintf(sd,"SF7-17");
    } else if (TX_Mode==1) {
      LMIC_setDrTxpow(DR_SF8,17);
      sprintf(sd,"SF8-17");
    } else if (TX_Mode==2) {
      LMIC_setDrTxpow(DR_SF9,17);
      sprintf(sd,"SF9-17");
    } else if (TX_Mode==3) {
      LMIC_setDrTxpow(DR_SF10,17);
      sprintf(sd,"SF10-17");
    } else if (TX_Mode==4) {
      LMIC_setDrTxpow(DR_SF11,17);
      sprintf(sd,"SF11-17");
    } else if (TX_Mode==5) {
      LMIC_setDrTxpow(DR_SF12,17);
      sprintf(sd,"SF12-17");
    } else if (TX_Mode==6) {
      LMIC_setDrTxpow(DR_SF7,14);
      sprintf(sd,"SF7-14");
    } else if (TX_Mode==7) {
      LMIC_setDrTxpow(DR_SF8,14);
      sprintf(sd,"SF8-14");
    } else if (TX_Mode==8) {
      LMIC_setDrTxpow(DR_SF9,14);
      sprintf(sd,"SF9-14");
    } else if (TX_Mode==9) {
      LMIC_setDrTxpow(DR_SF10,14);
      sprintf(sd,"SF10-14");
    } else if (TX_Mode==10) {
      LMIC_setDrTxpow(DR_SF11,14);
      sprintf(sd,"SF11-14");
    } else if (TX_Mode==11) {
      LMIC_setDrTxpow(DR_SF12,14);
      sprintf(sd,"SF12-14");
    } 
}

void iv_set() {
    if (TX_Interval_Mode==0) {
      TX_INTERVAL = 15;
      sprintf(iv,"15s");
    } else if (TX_Interval_Mode==1) {
      TX_INTERVAL = 30;
      sprintf(iv,"30s");
    } else if (TX_Interval_Mode==2) {
      TX_INTERVAL = 60;
      sprintf(iv,"60s");
    } else if (TX_Interval_Mode==3) {
      TX_INTERVAL = 120;
      sprintf(iv,"2m");
    } else if (TX_Interval_Mode==4) {
      TX_INTERVAL = 300;
      sprintf(iv,"5m");
    } else if (TX_Interval_Mode==5) {
      TX_INTERVAL = 600;
      sprintf(iv,"10m");
    } else if (TX_Interval_Mode==6) {
      TX_INTERVAL = 1800;
      sprintf(iv,"30m");
    } else if (TX_Interval_Mode==7) {
      TX_INTERVAL = 3600;
      sprintf(iv,"60m");
    } 
}

void sf_select() {
    if (digitalRead(SELECT_BTN) == LOW) {
      display.invertDisplay(true);
      delay(50);
      display.invertDisplay(false);
      ostime_t startTime = os_getTime();
      int selection = 0;

      while (digitalRead(SELECT_BTN) == LOW) {
        if (((os_getTime()-startTime) > sec2osticks(2)) && (os_getTime()-startTime) <= sec2osticks(4)) {
          // INTERVAL-CHANGE = BUTTON PRESS FOR MIN. 2 / MAX 4 SEC.
          display.clearDisplay();
          display.setTextColor(WHITE);
          display.setTextSize(2);
          TX_Interval_Mode++;
          if (TX_Interval_Mode > 7) {
            TX_Interval_Mode = 0;
          }
          iv_set();
          display.setCursor(20,32);
          display.print("IV->");
          display.print(iv);
          display.display();
          TX_Interval_Mode--;
          if (TX_Interval_Mode < 0) {
            TX_Interval_Mode = 7;
          }
          iv_set();
          delay(20);
          selection = 1;
        }

        if (((os_getTime()-startTime) >= sec2osticks(4)) && (os_getTime()-startTime) <= sec2osticks(6)) {
          // ADR = BUTTON PRESS FOR MIN. 4 / MAX 6 SEC.
          display.clearDisplay();
          display.setTextColor(WHITE);
          display.setTextSize(2);
          display.setCursor(8,32);
          if (adr == 0) {
            display.print("ADR -> ON");
          } else {
            display.print("ADR -> OFF");
          }
          display.display();
          delay(20);
          selection = 2;
        }

        if (((os_getTime()-startTime) > sec2osticks(6)) && (os_getTime()-startTime) <= sec2osticks(8)) {
          // PORT-CHANGE = BUTTON PRESS FOR MIN. 6 / MAX 8 SEC.
          display.clearDisplay();
          display.setTextColor(WHITE);
          display.setTextSize(2);
          display.setCursor(10,32);
          display.print("PORT -> " + String(3 - port));
          display.display();
          delay(20);
          selection = 3;
        }
        if ((os_getTime()-startTime) > sec2osticks(8)) {
          // > 8 SEC. = NOTHING...
          display.clearDisplay();
          display.display();
          delay(20);
          selection = 4;
        }
      }

      // Long-Press
      
      if (selection == 1) {
        // Change Interval
        TX_Interval_Mode++;
        if (TX_Interval_Mode > 7) {
          TX_Interval_Mode = 0;
        }
        iv_set();       
        prefs.begin("nvs", false);
        prefs.putString("IV_MODE", String(TX_Interval_Mode));
        prefs.end();
      }

      if (selection == 2) {
        // ADR
        adr = 1 - adr;
        prefs.begin("nvs", false);
        prefs.putString("ADR", String(adr));
        prefs.end();
        LMIC_setAdrMode(adr);
        sf_set();
      }

      if (selection == 3) {
        // Change Port
        port = 3 - port; // 1 -> 2, 2 -> 1
        prefs.begin("nvs", false);
        prefs.putString("PORT", String(port));
        prefs.end();
      }

      // Short-Press (SF&POWER)
      if (selection == 0) {
        TX_Mode++;
        if (TX_Mode >= 12) {
          TX_Mode = 0;
        }
        sf_set();
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);
        prefs.begin("nvs", false);
        prefs.putString("SF_MODE", String(TX_Mode));
        prefs.end();
      }
    }  
}

void onEvent (ev_t ev) {
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      LoraStatus = "EV_SCAN_TIMEOUT";
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      LoraStatus = "EV_BEACON_FOUND";
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      LoraStatus = "EV_BEACON_MISSED";
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      LoraStatus = "EV_BEACON_TRACKED";
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      LoraStatus = "EV_JOINING";
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      LoraStatus = "EV_JOINED";
      // Don't ask for ACKs: Disable link check validation (automatically enabled during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      LoraStatus = "EV_RFU1";
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      LoraStatus = "EV_JOIN_FAILED";
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      LoraStatus = "EV_REJOIN_FAILED";
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE [LED off]"));
      LoraStatus = "EV_TXCOMPLETE";
      axp.setChgLEDMode(AXP20X_LED_OFF);  
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received Ack"));
        LoraStatus = "Received Ack";
      }
      if (LMIC.dataLen) {
        sprintf(s, "Received %i bytes of payload", LMIC.dataLen);
        Serial.println(s);
        sprintf(s, "RSSI %d / SNR %.1d", LMIC.rssi, LMIC.snr);
        Serial.println(s);
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      // do_send(&sendjob);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      LoraStatus = "EV_LOST_TSYNC";
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      LoraStatus = "EV_RESET";
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      LoraStatus = "EV_RXCOMPLETE";
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      LoraStatus = "EV_LINK_DEAD";
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      LoraStatus = "EV_LINK_ALIVE";
      break;
    default:
      Serial.println(F("Unknown event"));
      LoraStatus = "Unknown event";
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("T-Beam TTNMapper"));
  Serial.println(F("----------------------------------------------"));
  
  /* Init I2C, SDA=21 / SCL=22 */
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS)) {
    Serial.println("AXP192 power controller init: Pass");
  } else {
    Serial.println("AXP192 power controller init: Fail");
  }

  // enable all irq channel
  axp.enableIRQ(0xFFFFFFFF, true);

  // attachInterrupt to gpio 35, HI -> LO (falling)
  pinMode(axp_irq_pin, INPUT_PULLUP);
  attachInterrupt(axp_irq_pin, [] {
      axpIrq = 1;
      Serial.println("AXP IRQ R!");
  }, FALLING);
  axp.clearIRQ();

  // Setup AXP192, OLED display at 3.3v
  axp.setDCDC1Voltage(3300);

  // Activate ADC
  axp.adc1Enable(AXP202_BATT_VOL_ADC1, true);
  axp.adc1Enable(AXP202_BATT_CUR_ADC1, true);
  axp.adc1Enable(AXP202_VBUS_VOL_ADC1, true);
  axp.adc1Enable(AXP202_VBUS_CUR_ADC1, true);

  // Activate power rails
  axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC2, AXP202_ON);
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_ON);
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
  
  // Read preferences
  prefs.begin("nvs", false);
  TX_Mode = prefs.getString("SF_MODE", "0").toInt();
  TX_Interval_Mode = prefs.getString("IV_MODE", "0").toInt();
  adr = prefs.getString("ADR", "0").toInt();
  port = prefs.getString("PORT", "1").toInt();
  TX_Interval_Mode = prefs.getString("IV_MODE", "0").toInt();
  prefs.end();
  iv_set();

  // UI Button
  pinMode(SELECT_BTN, INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, 0, 21, 22, 800000);
  display.clearDisplay();

  // Set text color
  display.setTextColor(WHITE);

  // Set font size
  display.setTextSize(2);

  // Set cursor position
  display.setCursor(1,16);

  // Show text
  display.print("TTN Mapper");
  display.setCursor(0,32);
  display.print("----------");
  display.setCursor(0,48);

  // TTN device ID
  display.print("0x26011EBC");
  display.drawLine(0, 9, display.width(), 9, WHITE);
  display.setTextSize(1);
  display.setCursor(86,0);
  if (adr == 0) {
    display.print("ADR:OFF");
  } else {
    display.print("ADR:ON");
  } 
  display.setCursor(42,00);
  display.print("IV:");
  display.print(iv);
  display.setCursor(0,0);
  display.print("PORT:"+String(port));
  display.display();
  
  // Turn off WiFi and Bluetooth
  WiFi.mode(WIFI_OFF);
  btStop();
  gps.init();
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set LMC clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  #ifndef OTAA 
    // Init session
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  #ifdef CFG_eu868
    // 868 MHz SRD channels
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  #endif
  
  #ifdef CFG_us915
    // 915 MHz band
    LMIC_selectSubBand(1);
    // Disable channels 16-72
    for (int i = 16; i < 73; i++) {
      if (i != 10)
        LMIC_disableChannel(i);
    }
  #endif

  // Don't ask for ACKs: Disable link check validation
  LMIC_setLinkCheckMode(0);

  // Disable/Enable ADR 
  LMIC_setAdrMode(adr);

  // TheThingsNetwork uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: Txpow seems to be ignored by the library)
  // LMIC_setDrTxpow(DR_SF10,17); 
  sf_set();

  // Set Interval-Text corresponding to current setting
  iv_set();
  
  do_send(&sendjob);
  axp.setChgLEDMode(AXP20X_LED_OFF);
  display.clearDisplay();
  lastMillis2 = -1000;
  delay(2000);
}

void loop() {
  // AXP power management IRQ-handling
  if (axpIrq) {
      Serial.println("AXP IRQ!");
      axpIrq = 0;
      axp.readIRQ();
      if (axp.isPEKLongtPressIRQ()) {
        // switch LED off
        axp.setChgLEDMode(AXP20X_LED_OFF);
        delay(5000);
      }
      if (axp.isPEKShortPressIRQ()) {
        Serial.printf("AXP202 PEK key Short press\n");
        // Reduce display-voltage to 1.7V
        if (isDimmed) {
          // End dimming OLED
          isDimmed = false;
          axp.setDCDC1Voltage(3300);
          ledMode = AXP20X_LED_LOW_LEVEL;           
        } else {
          // Start dimming OLED
          isDimmed = true;
          axp.setDCDC1Voltage(1700); 
          ledMode = AXP20X_LED_OFF;           
        }
          
      }      
      axp.clearIRQ();
  }

  // Where are we?
  gps.encode();
  sf_select();
  if (lastMillis + 1000 < millis()) {
    lastMillis = millis();
    VBAT = axp.getBattVoltage()/1000;

    os_runloop_once();
    if (gps.checkGpsFix()) {
      // We had at least one GPS fix
      GPSonceFixed = true;
      noFix = false;
      gps.gdisplay(txBuffer2);
      float hdop = txBuffer2[4] / 10.0;
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(1);
      display.setCursor(0,0);
      display.print("SAT: " + String(txBuffer2[0]));
      display.setCursor(104,0);
      display.print(VBAT,1);
      display.setCursor(122,0);
      // Display battery charging state
      if (axp.isChargeing()) {
        // Charging: Uppercase
        display.print("V");
      } else {
        // Discharging: Lowercase
        display.print("v");
      }
      display.setCursor(0,10);
      display.print("Speed: " + String(txBuffer2[1])+ " km/h");
      display.setCursor(0,20);
      display.print("Course: " + String(txBuffer2[2])+(char)247);
      display.setCursor(0,30);
      display.print("Alt: " + String(txBuffer2[3])+ "m");
      display.setCursor(0,40);
      display.print("HDOP: ");
      display.setCursor(35,40);
      display.print(hdop,1);
      display.setCursor(80,20); // SF and TXpow
      display.print(sd);
      display.setCursor(80,30); // Sending interval
      display.print("Iv: ");
      display.print(iv);
      display.setCursor(80,40); // up packet number
      display.print("Up: " + String(LMIC.seqnoUp-1));
      display.drawLine(0, 48, display.width(), 48, WHITE);
      display.setCursor(0,54);
      display.print("LoRa: ");
      display.setCursor(35,54);
      display.print(LoraStatus);

      if (gps.tGps.time.isValid()) {
        // GPS-Zeit vorhanden
        display.setCursor(48,0);
        if (gps.tGps.time.hour() < 10) display.print("0");
        display.print(gps.tGps.time.hour());
        display.print(":");
        if (gps.tGps.time.minute() < 10) display.print("0");
        display.print(gps.tGps.time.minute());
        display.print(":");
        if (gps.tGps.time.second() < 10) display.print("0");
        display.print(gps.tGps.time.second());
      }
    } else {
      // No GPS fix yet
      noFix = true;
      display.clearDisplay();
      display.setTextColor(WHITE);
      display.setTextSize(2);
      display.setCursor(0,16);
      // Change wording after first successful fix
      if (GPSonceFixed) {
        display.print("Lost");
      } else {
        display.print("Missing");
      }
      display.setCursor(0,32);
      display.print("GPS fix");
      display.setCursor(0,48); // SF and TXpow
      display.print(sd);
      display.setTextSize(1);
      display.setCursor(104,0);
      display.print(VBAT,1);
      display.setCursor(122,0);
      // Display battery charging state
      if (axp.isChargeing()) {
        // Charging: Uppercase
        display.print("V");
      } else {
        // Discharging: Lowercase
        display.print("v");
      }
    }
    redraw = true;
  }

  if ((lastMillis2 + 500 < millis()) || redraw) {
    lastMillis2 = millis();
    if (noFix)
    {
      /* Disable ugly satellite icon ;) */
      /*
      blinkGps = 1-blinkGps;
      if (blinkGps == 1) {
        display.fillRect((display.width()  - imageWidthGpsIcon ), (display.height() - imageHeightGpsIcon), imageWidthGpsIcon, imageHeightGpsIcon, 0);
        display.drawBitmap(
                            (display.width()  - imageWidthGpsIcon ),
                            (display.height() - imageHeightGpsIcon),
                            gpsIcon, imageWidthGpsIcon, imageHeightGpsIcon, 1);
      } else {
        display.fillRect((display.width()  - imageWidthGpsIcon ), (display.height() - imageHeightGpsIcon), imageWidthGpsIcon, imageHeightGpsIcon, 0);
        display.drawBitmap(
                            (display.width()  - imageWidthGpsIcon ),
                            (display.height() - imageHeightGpsIcon),
                            gpsIcon2, imageWidthGpsIcon, imageHeightGpsIcon, 1);
      }
      */
    }
    redraw = true;
  }

  if (redraw) {
    redraw = false;
    display.display();
  }
}
