/**
 * Tally Light control using RFM69 data radios
 * 
 * Inspired by the BMD SDI shield and Skaarhoj's ATEM demo code
 * and work done by kvITko  Consulting back in 2016
 * 
 * This code is a new implementation using Adafruit's Featherwing processors with RFM69 radios and NeoPixels
 * The "transmitter" uses an Ethernet Shield to connect to the network that the ATEM switch is also on,
 * though a WiFi shield/processor could also be used if mobility is a concern.
 * 
 * Copyright 2020, John Plocher
 * www.SPCoast.com
 * Released under the MIT Open Source License
 * 
 * Dependencies:
 *     Adafruit NeoPixel library
 *     Adafruit's fork of RadioHead's RFM69 library
 *
 *
 * Notes:
 * To begin talking to the radio, you will need to download the Adafruit fork of the Radiohead
 * code from their github repository. You can do that by visiting the github repo and manually 
 * downloading or, easier, just download the zip
 * https://github.com/adafruit/RadioHead/archive/master.zip
 *
 * Place the RadioHead library folder in your arduinosketchfolder/libraries/ folder. 
 * You may need to create the libraries subfolder if it's your first library. Restart the IDE.
 * 
 * ---- circuits ---
 * I used the Adafruit Adafruit Feather 32u4 RFM69HCW Packet Radio - 868 or 915 MHz - RadioFruit, P/N 3076
 * which has more than enough speed, code space and memory for this application
 * Any similar radio or board should work.  In particular, the Adafruit Feather M0 RFM69HCW Packet Radio - 868
 * or 915 MHz - RadioFruit (3176) should be a drop in replacement, with 8x to 16x more of everything to play with.
 * 
 * Transmitter:  MCU with I2C connected to pins 4 (SDA), 5 (SCL) & GND on the SDI shield
 *               MCU with SPI connected to the NeoPixel stick on pins 6 (SPI Data), USB (5v) and GND
 * 
 * Tally Light Receivers: MCU connected to the NeoPixel stick on pins 6 (SPI Data), USB (5v) and GND
 *                        TODO: Connect pins 9, 10, 11 and 12 to a dip switch so the camea address can be read in at startup.
 *                        TODO: use address 0 (an invalid SDI Camera number) to display RSSI on the neopixels
 *                        Optional: Connect a LION battery to the Feather MCU to allow hours of mobile untethered operation
 *
 * Configuration comes from a 4 position dip switch bank, where "ON" = 0 and "OFF" = 1
 *    xxx1:  TALLY, xxx = camera number -1  (i.e., binary 0 = camera 1)
 *    0010:  TX Test sequence, no SDI shield
 *    0100   RX RSSI Signal Strength meter
 *    0000   TRANSMITTER connected to SDI shield
 */

//#define DEBUG_SERIAL        // Don't turn on in production
#define TALLY_BRIGHT 100    // how bright should RED/PROGRAM be displayed?
#define TALLY_DIM      3    // Preview and standby don't need to be so bright - save energy and don't blind the cast...
#define STATUS_BRIGHT 10    // the transmitter uses the neopixels to show what cameras are in use...


#define TALLY       0
#define TRANSMITTER 1
#define TX_TEST     2
#define RX_TEST     3

/**
 * Read the dip switches and interpret them for the program
 */
class Config {
  public:
    Config(void) {                    // TX_TEST  SDI   RX_TEST  Tally
        pinMode(12, INPUT_PULLUP);    // -na-     -na-    -na-   Addr 0
        pinMode(11, INPUT_PULLUP);    // GND      GND     5v     Addr 1
        pinMode(10, INPUT_PULLUP);    // GND      5v      5v     Addr 2
        pinMode( 9, INPUT_PULLUP);    // GND      GND     GND    5v
        _camera  = 0;
        _persona = TALLY;

        if (digitalRead(9)) {                                    //  1 xxx  TALLY on Camera
            _persona = TALLY;                                    //    rest of pins are camera address
            _camera  = digitalRead(12) * 1 +
                       digitalRead(11) * 2 +
                       digitalRead(10) * 4 + 1;  // camera IDs are 1..8, not 0..7
        } else {
            if (       (digitalRead(10) == 0) && (digitalRead(11) == 0)) {  // 0 0 0 x TX Test mode
                _persona = TX_TEST;
            } else if ((digitalRead(10) == 1) && (digitalRead(11) == 1)) {  // 0 1 1 x RSSI Meter mode
                _persona = RX_TEST;
            } else if ((digitalRead(10) == 1) && (digitalRead(11) == 0)) {  // 0 1 0 x transmitter base station with SDI Card 
                _persona = TRANSMITTER;
            }
        }
    }

    bool isTally(void)          { return _persona == TALLY; }
    bool isTransmitter(void)    { return _persona == TRANSMITTER; }
    bool isTXTest(void)         { return _persona == TX_TEST; }
    bool isRXTest(void)         { return _persona == RX_TEST; }
    byte camera(void)           { return _camera; }
    byte numCameras(void)       { return 8; }
    
  private:
    byte _persona;
    byte _camera;
};

Config *conf;


// rf69 demo from RadioHead69 / Adafruit:
//     RH_RF69 class does not provide for addressing or
//     reliability, so you should only use RH_RF69  if you do not need the higher
//     level messaging abilities.  
// These limitations are OK for this use case.

#include <SPI.h>
#include <RH_RF69.h>    // 

/************ Radio Setup ***************/

#define RF69_FREQ 915.0

#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS       8
  #define RFM69_INT      7
  #define RFM69_RST      4
  #define LED           13
  // Which pin on the Arduino is connected to the NeoPixels?
  // On a Trinket or Gemma we suggest changing this to 1:
  #define LED_PIN        6
#endif

#if defined(ADAFRUIT_FEATHER_M0) // Feather M0 w/Radio
  #define RFM69_CS       8
  #define RFM69_INT      3
  #define RFM69_RST      4
  #define LED           13
  #define LED_PIN        6
#endif

#if defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM69_INT      3  // 
  #define RFM69_CS       4  //
  #define RFM69_RST      2  // "A"
  #define LED           13
  #define LED_PIN        6
#endif

#if defined(ESP8266)    // ESP8266 feather w/wing
  #define RFM69_CS      2    // "E"
  #define RFM69_IRQ     15   // "B"
  #define RFM69_RST     16   // "D"
  #define LED            0
  #define LED_PIN        PICK ONE
#endif

#if defined(ESP32)    // ESP32 feather w/wing
  #define RFM69_RST     13   // same as LED
  #define RFM69_CS      33   // "B"
  #define RFM69_INT     27   // "A"
  #define LED           13
  #define LED_PIN        PICK ONE
#endif

/* Teensy 3.x w/wing
#define RFM69_RST     9   // "A"
#define RFM69_CS      10   // "B"
#define RFM69_IRQ     4    // "C"
#define RFM69_IRQN    digitalPinToInterrupt(RFM69_IRQ )
*/
 
/* WICED Feather w/wing 
#define RFM69_RST     PA4     // "A"
#define RFM69_CS      PB4     // "B"
#define RFM69_IRQ     PA15    // "C"
#define RFM69_IRQN    RFM69_IRQ
*/

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

/*
 * Packet contents - an array of bytes
 * 
 * [ <4 byte ID> <version> <PROGRAM #> <PREVIEW #> ] 
 * [ 'A' 'T' 'E' 'M' 1 [0-16] [0-16]
 */

struct Payload {
    char id[4];
    byte ver;
    byte program;
    byte preview;
} payload;



/********** NeoPixel Setup **************/
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif


// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 8

#define COLOR_OFF     0   // Black
#define COLOR_STANDBY 1   // Blue Hue
#define COLOR_PROGRAM 2   // RED Hue
#define COLOR_PREVIEW 3   // GREEN Hue

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

/********** ATEM Setup **************/

#include <BMDSDIControl.h> // The library for BMD's SDI Shield
BMD_SDITallyControl_I2C sdiTallyControl(0x6E);            // define the Tally object using I2C using the default shield address


void setup() {
#ifdef DEBUG_SERIAL
    Serial.begin(115200);
    delay(1000);
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
    Serial.println("Tally Light Feather RFM69");
#endif
    
    // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
    // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
    clock_prescale_set(clock_div_1);
#endif
    // END of Trinket-specific code.
    
    strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.fill(strip.Color(  0,   0,   0), 0, 8);
    strip.show();             // Turn OFF all pixels ASAP
    
    strip.setBrightness(10);  // Set BRIGHTNESS (max = 255)

    strip.fill(strip.Color(255, 255, 255), 0, 8); strip.show(); delay(100);
    strip.fill(strip.Color(  0, 255, 255), 0, 8); strip.show(); delay(100);
    strip.fill(strip.Color(255, 255,   0), 0, 8); strip.show(); delay(100);
    strip.fill(strip.Color(255,   0, 255), 0, 8); strip.show(); delay(100);
    strip.fill(strip.Color(  0,   0,   0), 0, 8); strip.show(); 

    conf = new Config();
#ifdef DEBUG_SERIAL
    if (conf->isTally()) {
        Serial.print("Tally Camera: ");Serial.println(conf->camera() );
    }
    if (conf->isTransmitter()) {
        Serial.print("TALLY Transmitter ");
    }
    if (conf->isTXTest()) { 
        Serial.print("TX TEST MODE ");
    }
    if (conf->isRXTest()) { 
        Serial.print("RX TEST MODE ");
    }
    Serial.println();
#endif
    pinMode(RFM69_RST, OUTPUT);
    digitalWrite(RFM69_RST, LOW);
    
   
    // manual reset
    digitalWrite(RFM69_RST, HIGH);
    delay(10);
    digitalWrite(RFM69_RST, LOW);
    delay(10);
    
    if (!rf69.init()) {
#ifdef DEBUG_SERIAL
      Serial.println("RFM69 radio init failed");
#endif
      while (1) {
          // Flash in an attention getting form...
          strip.fill(strip.Color(255,   0,   0), 0, 8); strip.show(); delay(100);
          strip.fill(strip.Color(0,     0, 255), 0, 8); strip.show(); delay(100);
      }
    }
    
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69.setFrequency(RF69_FREQ)) {
#ifdef DEBUG_SERIAL
        Serial.println("setFrequency failed");
#endif
    }
    
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
    
    // The 16 byte encryption key has to be the same for both TRANSMITTER and TALLY receivers...
    uint8_t key[] = { 'B', 'M', 'D', ' ',
                      'T', 'a', 'l', 'l', 'y', ' ',
                      'L', 'i', 'g', 'h', 't', 's' };
    rf69.setEncryptionKey(key);

    payload.id[0] = 'A';
    payload.id[1] = 'T';
    payload.id[2] = 'E';
    payload.id[3] = 'M';
    payload.ver   =  1;
    payload.program = 9; 
    payload.preview = 9;

    strip.setBrightness(STATUS_BRIGHT);

    if (conf->isTransmitter()) {
        sdiTallyControl.begin();   // initialize BMD I2C card tally control
    } else if (conf->isTally()) {
        // Display camera number for a few seconds...
        strip.setPixelColor((8-conf->camera()), strip.Color(0, 0, 255));
        strip.show();
        delay(2000);
        strip.fill(strip.Color(  0,   0,   0), 0, 8); strip.show(); 
    }
}


byte lastprogram = 0;
byte lastpreview = 0;

void loop() {
  
    if (conf->isTXTest()) {
        delay(1000);  // Wait a bit between cycles, could also 'sleep' here if using a battery!
        payload.preview = payload.program;
        payload.program += 1;
        if (payload.program > conf->numCameras()) { payload.program = 1; }      
    } else if (conf->isTransmitter()) {
        delay(50);  // Wait a bit between cycles, could also 'sleep' here!
    
        for (int x = 0; x < conf->numCameras(); x++) {
            bool prog = false, pre=false;
            if (sdiTallyControl.getCameraTally(x, prog, pre)) {
                if (prog) {
                  payload.program = x; 
                } else if (pre) {
                  payload.preview = x;
                }
            }
        }
        
        if ( (lastprogram != payload.program) || (lastpreview != payload.preview)) {
            lastprogram = payload.program;
            lastpreview = payload.preview;
        } else {
            return;
        }
    }
    
    if (conf->isTXTest() || conf->isTransmitter()) {
        strip.clear();
        if (payload.preview == payload.program) {
            strip.setPixelColor(8-payload.program, strip.Color(255,   0,   0));
        }
        strip.setPixelColor(8-payload.preview, strip.Color(0,   255,   0));
        strip.setPixelColor(8-payload.program, strip.Color(255,   0,   0));
        strip.show();                          //  Update strip to match
        
        // Send a message!
        rf69.send((uint8_t *)&payload, sizeof(payload));
        rf69.waitPacketSent();
    } 

    // show RSSI as a scale of green/yellow/red...
    if (conf->isRXTest()) {
        uint32_t c;
        int rssi;

        if (rf69.available()) { // Should be a message for us now   
          uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
          uint8_t len = sizeof(buf);
          if (rf69.recv(buf, &len)) {
              rssi = rf69.lastRssi();
        
              if      (rssi > -50) c = strip.Color(0, 255, 0);    // strong green
              else if (rssi > -60) c = strip.Color(0, 128, 0);    // dim green
              else if (rssi > -69) c = strip.Color(0, 80, 0);     // very dim green
              else if (rssi > -70) c = strip.Color(255, 255, 0);  // strong yellow
              else if (rssi > -74) c = strip.Color(128, 128, 0);  // dim yellow
              else if (rssi > -76) c = strip.Color(80,  80,  0);  // very dim yellow
              else if (rssi > -77) c = strip.Color(255, 0, 0);    // strong red
              else if (rssi > -78) c = strip.Color(128, 0, 0);    // dim red
              else if (rssi > -80) c = strip.Color( 80, 0, 0);    // very dim red
              else c = strip.Color(0, 0, 80);                     // very dim blue
              
              strip.clear();       strip.show();  // short blink
              delay(50);
              strip.fill(c, 0, 8); strip.show();
          }
        }
    }

    if (conf->isTally()) {
        if (rf69.available()) {
            // Should be a message for us now   
            uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
            uint8_t len = sizeof(buf);
            if (rf69.recv(buf, &len)) {
                if (!len) return;
                Payload *p = (Payload *)buf;
                
                if ((p->id[0] == payload.id[0]) &&
                    (p->id[1] == payload.id[1]) && 
                    (p->id[2] == payload.id[2]) && 
                    (p->id[3] == payload.id[3]) && 
                    (p->ver   == payload.ver) ) {  // got good data
    
                    if (p->program == conf->camera()) {           // TALLY RED
                        tally(COLOR_PROGRAM);
                    } else if (p->preview == conf->camera()) {    // TALLY GREEN
                        tally(COLOR_PREVIEW);
                    } else {                              // TALLY OFF
                        tally(COLOR_OFF);
                        delay(50);
                        tally(COLOR_STANDBY);
                    }
                
                } else {
                    // Serial.println("malformed packet");
                }
            } else {
                // Serial.println("Receive failed");
            }
        }
    }
}

/**
 * Light up the Tally LEDs
 * 
 * I chose to have the center two LEDs bright, with dimmer LEDs on the edges
 * and an animated transition between PROGRAM,. PREVIEW and idle
 * Having every LED on full was too bright for close in cameras, but the
 * ones further away needed something to catch the talent's eye...
 * Feel free to play with this - there are cool animation effects possible!
 * 
 * In particular, having PREVIEW be bright and visible confuses some who may
 * tend to think that GREEN means GO...
 * 
 * Finally, idle shows a dim BLUE that blinks whenever a valid radio packet is
 * received.  This debugging aid may also be confusing to those in front of the
 * cameras, and could be toned down or removed.
 */
void tally(uint32_t color) {

    uint32_t hue;
    if (color == COLOR_OFF) {
        strip.clear(); 
        strip.show();
        return;
    }

    if (color == COLOR_STANDBY) {
        strip.setBrightness(TALLY_DIM); // Set BRIGHTNESS (max = 255)
        strip.setPixelColor(3, strip.Color(0, 0, 80));
        strip.setPixelColor(4, strip.Color(0, 0, 80));
        strip.show();                          //  Update strip to match
        return;
    }
    
    
    if (color == COLOR_PROGRAM) {
        hue = 0;       //  RED
        strip.setBrightness(TALLY_BRIGHT); // Set BRIGHTNESS (max = 255)
        strip.setPixelColor(0, strip.ColorHSV(hue, 255, 64));
        strip.setPixelColor(7, strip.ColorHSV(hue, 255, 64));
        strip.show();                          //  Update strip to match
        delay(80);                             //  Pause for a moment
        strip.setPixelColor(1, strip.ColorHSV(hue, 255, 128));
        strip.setPixelColor(6, strip.ColorHSV(hue, 255, 128));
        strip.show();                          //  Update strip to match
        delay(50);                             //  Pause for a moment
        strip.setPixelColor(2, strip.ColorHSV(hue, 255, 200));
        strip.setPixelColor(5, strip.ColorHSV(hue, 255, 200));
        strip.show();                          //  Update strip to match
        delay(30);                             //  Pause for a moment
        strip.setPixelColor(3, strip.ColorHSV(hue, 255, 255));
        strip.setPixelColor(4, strip.ColorHSV(hue, 255, 255));
        strip.show();                          //  Update strip to match
    } else if (color == COLOR_PREVIEW) {
        hue = 65536/3; // GREEN
        strip.clear(); 
        strip.setBrightness(TALLY_DIM); // Set BRIGHTNESS (max = 255)
        strip.setPixelColor(3, strip.ColorHSV(hue, 255, 64));
        strip.setPixelColor(4, strip.ColorHSV(hue, 255, 64));
        strip.show();    
    }
}
