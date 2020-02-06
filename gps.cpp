#include "gps.h"

HardwareSerial GPSSerial(1);

void gps::init() {
  GPSSerial.begin(9600, SERIAL_8N1, GPS_TX, GPS_RX);
  GPSSerial.setTimeout(2);
}

void gps::encode() {       
    int data;
    unsigned long previousMillis = millis();

    while((previousMillis + 100) > millis()) {
        while (GPSSerial.available() ) {
            char data = GPSSerial.read();
            tGps.encode(data);
            //Serial.print(data);
        }
    }
    //Serial.println("");
}

void gps::buildPacket(uint8_t txBuffer[9]) {
  LatitudeBinary  = ((tGps.location.lat() + 90) / 180.0)  * 16777215;
  LongitudeBinary = ((tGps.location.lng() + 180) / 360.0) * 16777215;
  double gw_distance_m = tGps.distanceBetween(tGps.location.lat(), tGps.location.lng(), HOME_LAT, HOME_LNG);
  float gw_distance_km = gw_distance_m/1000;
  double gw_course = tGps.courseTo(tGps.location.lat(), tGps.location.lng(), HOME_LAT, HOME_LNG);
  int hour   = tGps.time.hour();
  int minute = tGps.time.minute();
  int second = tGps.time.second();

  Serial.println();
  Serial.println(F("GPS Data"));
  Serial.println(F("----------------------------------------------"));
  
  sprintf(t, "Latitude:        %f", tGps.location.lat());
  Serial.println(t);
  
  sprintf(t, "Longitude:       %f", tGps.location.lng());
  Serial.println(t);

  sprintf(t, "Altitude:        %.0f m", tGps.altitude.meters());
  Serial.println(t);

  sprintf(t, "Speed:           %.2f km/h", tGps.speed.kmph());
  Serial.println(t);

  sprintf(t, "HDOP:            %.2lf", tGps.hdop.value()/100.0);
  Serial.println(t);

  sprintf(t, "Satellites:      %i", tGps.satellites.value());
  Serial.println(t);

  Serial.println(F("----------------------------------------------"));

  Serial.print("Dst to Gateway:  ");
  Serial.print(gw_distance_m);
  Serial.print(" m to ");
  Serial.println(tGps.cardinal(gw_course));

  Serial.println(F("----------------------------------------------"));
  Serial.println();

  // Build LoRa payload
  txBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
  txBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
  txBuffer[2] = LatitudeBinary & 0xFF;

  txBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
  txBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
  txBuffer[5] = LongitudeBinary & 0xFF;

  altitudeGps = tGps.altitude.meters();
  txBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
  txBuffer[7] = altitudeGps & 0xFF;

  hdopGps = tGps.hdop.value()/10;
  txBuffer[8] = hdopGps & 0xFF;
}

void gps::gdisplay(uint16_t txBuffer2[5]) {
  txBuffer2[0] = tGps.satellites.value();
  txBuffer2[1] = tGps.speed.kmph();
  txBuffer2[2] = tGps.course.deg();
  txBuffer2[3] = tGps.altitude.meters();
  txBuffer2[4] = tGps.hdop.value()/10;
}

bool gps::checkGpsFix() {
  encode();
  if (tGps.location.isValid() && 
      tGps.location.age() < 4000 && 
      tGps.hdop.isValid() && 
      tGps.hdop.value() <= 600 && 
      tGps.hdop.age() < 4000 && 
      tGps.altitude.isValid() && 
      tGps.altitude.age() < 4000 )
  {
    // Serial.println("Valid GPS fix");
    return true;
  } else {
     Serial.println("No GPS fix");
//     sprintf(t, "location valid: %i" , tGps.location.isValid());
//     Serial.println(t);
//     sprintf(t, "location age: %i" , tGps.location.age());
//     Serial.println(t);
//     sprintf(t, "hdop valid: %i" , tGps.hdop.isValid());
//     Serial.println(t);
//     sprintf(t, "hdop age: %i" , tGps.hdop.age());
//     Serial.println(t);
//     sprintf(t, "hdop: %i" , tGps.hdop.value());
//     Serial.println(t);
//     sprintf(t, "altitude valid: %i" , tGps.altitude.isValid());
//     Serial.println(t);
//     sprintf(t, "altitude age: %i" , tGps.altitude.age());
//     Serial.println(t);
    return false;
  }
}
