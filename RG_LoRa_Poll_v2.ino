
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SoftwareSerial.h>
SoftwareSerial unoSerial = SoftwareSerial(7, 8);
//void Poll();
//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define CFG_in866 1
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// LoRaWAN NwkSKey, network session key
// This should be in big-endian (aka msb).
static const PROGMEM u1_t NWKSKEY[16] = { 0x5E, 0xEF, 0x66, 0x8B, 0x20, 0x10, 0x2C, 0x2D, 0xCA, 0xF3, 0x4B, 0x49, 0x02, 0xE8, 0x23, 0xD8 };

// LoRaWAN AppSKey, application session key
// This should also be in big-endian (aka msb).
static const u1_t PROGMEM APPSKEY[16] = { 0xF8, 0xA6, 0x68, 0x10, 0x1B, 0x60, 0x36, 0x1D, 0x63, 0xFC, 0x08, 0x9C, 0x65, 0x11, 0xCA, 0x2C };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// The library converts the address to network byte order as needed, so this should be in big-endian (aka msb) too.
static const u4_t DEVADDR = 0xFC00978C  ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmic/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[6];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 180;

// Pin mapping
// Adapted for Feather M0 per p.10 of [feather]
const lmic_pinmap lmic_pins = {
  .nss = 6,                       // chip select on feather (rf95module) CS
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 5,                       // reset pin
  .dio = {2, 3, 4}, // assumes external jumpers [feather_lora_jumper]
  // DIO1 is on JP1-1: is io1 - we connect to GPO6
  // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      //Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      //Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      //Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      //Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      //Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      //Serial.println(F("EV_JOINED"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_RFU1:
      ||     Serial.println(F("EV_RFU1"));
      ||     break;
    */
    case EV_JOIN_FAILED:
      //Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      //Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      //Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      //Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      //Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      //Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      //Serial.println(F("EV_LINK_ALIVE"));
      break;
    /*
      || This event is defined but not used in the code. No
      || point in wasting codespace on it.
      ||
      || case EV_SCAN_FOUND:
      ||    Serial.println(F("EV_SCAN_FOUND"));
      ||    break;
    */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      //Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      // Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;
    default:
      // Serial.print(F("Unknown event: "));
      // Serial.println((unsigned) ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    rg();
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{

  while (!Serial);
  Serial.begin(9600);
  unoSerial.begin(9600);
  //    pinMode(13, OUTPUT);
  // wait for Serial to be initialized
  //Serial.begin(115200);
  delay(100);     // per sample code on RF_95 test
  Serial.println(F("Starting"));

#ifdef VCC_ENABLE
  // For Pinoccio Scout boards
  pinMode(VCC_ENABLE, OUTPUT);
  digitalWrite(VCC_ENABLE, HIGH);
  delay(1000);
#endif

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();


#ifdef PROGMEM

  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
#elif defined(CFG_us915) || defined(CFG_au915)
  LMIC_selectSubBand(1);
#elif defined(CFG_as923)
#elif defined(CFG_kr920)

#elif defined(CFG_in866)
  LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
  LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_MILLI);
#else
# error Region not supported
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink
  LMIC_setDrTxpow(DR_SF7, 14);

  // Start job

  do_send(&sendjob);
}

//void Poll()
//{
//  unoSerial.begin(9600);
//  delay(1000);
//  unoSerial.write('r');
//  unoSerial.write('\n');
//  //unoSerial.flush();
//}
void rg() {

  //  Poll();
  unoSerial.write('r');
  unoSerial.write('\n');
  String response = unoSerial.readStringUntil('\n');
  Serial.println(response);
  delay(100);
 // if (response.startsWith("Acc"))
  //{
    //slicing string
    char Acc[7], EventAcc[7], TotalAcc[7], RInt[7], unit[4];
    sscanf (response.c_str(), "%*s %s %[^,] , %*s %s %*s %*s %s %*s %*s %s", &Acc, &unit, &EventAcc, &TotalAcc, &RInt);

    // converting char array to float
    float float_Acc = atof(Acc);
    float float_EventAcc = atof(EventAcc);
    float float_TotalAcc = atof(TotalAcc);
    float float_RInt = atof(RInt);

    Serial.print("accumulation = ");
    Serial.println(atof(Acc));
    Serial.print("eventAccumulation = ");
    Serial.println(atof(EventAcc));
    Serial.print("Totalcommunication = ");
    Serial.println(atof(TotalAcc));
    Serial.print("Rint =");
    Serial.println(atof(RInt));
//    delay(1000);
    int a =  int(float_Acc * 100);
    int b =  int(float_EventAcc * 100);
    int c =  int(float_RInt * 100);

    mydata[0] = highByte(a);
    mydata[1] = lowByte(a);
    mydata[2] = highByte(b);
    mydata[3] = lowByte(b);
    mydata[4] = highByte(c);
    mydata[5] = lowByte(c);
 // }
}
void loop() {

  os_runloop_once();
}
