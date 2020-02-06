// Comment the next line to use ABP authentication on TTN. 
// Leave it as it is to use recommended OTAA
//#define OTAA

#ifndef LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
#define LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED

#ifndef OTAA
// Settings for ABP
static PROGMEM u1_t NWKSKEY[16] = { 0x80, 0x1A, 0xBC, 0xCC, 0x5A, 0x63, 0x78, 0xD8, 0x7D, 0x6E, 0x51, 0x9E, 0xAB, 0x12, 0xEF, 0xA4 }; // LoRaWAN NwkSKey, network session key 
static u1_t PROGMEM APPSKEY[16] = { 0x83, 0x91, 0xD8, 0x82, 0x3A, 0x5A, 0x6C, 0x26, 0xF1, 0x87, 0xD4, 0x0B, 0x77, 0x2F, 0x2A, 0x2A }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x26011EBC ; // LoRaWAN end-device address (DevAddr)
#else
// Settings from OTAA device
static const u1_t PROGMEM DEVEUI[8]={ 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }; // Device EUI, hex, lsb
static const u1_t PROGMEM APPEUI[8]={ 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }; // Application EUI, hex, lsb
static const u1_t PROGMEM APPKEY[16] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 }; // App Key, hex, msb

void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }
#endif

#endif //LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
