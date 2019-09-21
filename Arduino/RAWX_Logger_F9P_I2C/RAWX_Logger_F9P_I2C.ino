// RAWX_Logger_F9P_I2C

// Logs RXM-RAWX, RXM-SFRBX and TIM-TM2 data from u-blox ZED_F9P GNSS to SD card
// Also logs NAV_PVT messages (which provide the carrSoln status) and NAV-STATUS messages (which indicate a time fix for Survey_In mode)

// This version uses the SparkFun u-blox library by Nate Seidle to configure the RAWX messages via I2C, leaving the UART dedicated for the messages to be logged to SD card
// Feel like supporting open source hardware? Buy a board from SparkFun!
// ZED-F9P RTK2: https://www.sparkfun.com/products/15136

// This code is written for the Adalogger M0 Feather
// https://www.adafruit.com/products/2796
// https://learn.adafruit.com/adafruit-feather-m0-adalogger
// Adafruit invests time and resources providing this open source design, please support Adafruit and open-source hardware by purchasing products from Adafruit!

// Choose a good quality SD card. Some cheap cards can't handle the write rate.
// Ensure the card is formatted as FAT32.

// Changes to a new log file every INTERVAL minutes
bool splitLog = false; // set splitLog to false if you don't want a new file every INTERVAL minutes

// Define how long we should log in minutes before changing to a new file
// Sensible values are: 5, 10, 15, 20, 30, 60
const int INTERVAL = 5;

// Define how long we should log in minutes, after starting a delayed stop logging
const int DELAYED_STOP = 10;

// Define how long we should wait in msec (approx.) for residual RAWX data before closing the last log file
// For a measurement rate of 4Hz (250msec), 300msec is a sensible value. i.e. slightly more than one measurement interval
const int dwell = 300;

// Send serial debug messages
//#define DEBUG // Comment this line out to disable debug messages

// Debug SerialBuffer
// Displays a "Max bufAvail:" message each time SerialBuffer.available reaches a new maximum
#define DEBUGserialBuffer // Comment this to disable serial buffer maximum available debugging

// Connect modePin to GND to select base mode. Leave open for rover mode.
#define modePin 14 // A0 / Digital Pin 14

// Connect a normally-open push-to-close switch between swPin and GND.
// Press it to stop logging and close the log file.
#define swPin 15 // A1 / Digital Pin 15 (0.2" away from the GND pin on the Adalogger)

// Pin A2 (Digital Pin 16) is reserved for the ZED-F9P EXTINT signal
// The code uses this an an interrupt to set the NeoPixel to white
#define ExtIntPin 16 // A2 / Digital Pin 16
#define white_flash 1000 // Flash the NeoPxel white for this many milliseconds on every ExtInt

// Connect A3 (Digital Pin 17) to GND to select SURVEY_IN mode when in BASE mode
#define SurveyInPin 17 // A3 / Digital Pin 17

// Connect A4 (Digital Pin 18) to GND to stop logging with a delay (see DELAYED_STOP to set the duration)
#define DelayedPin  A4 // A4 / Digital Pin 18

// Include the SParkFun u-blox Library
#include <Wire.h> //Needed for I2C to GPS
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS i2cGPS;

// LEDs

//#define NoLED // Uncomment this line to completely disable the LEDs
//#define NoLogLED // Uncomment this line to disable the LEDs during logging only

// NeoPixel Settings
//#define NeoPixel // Uncomment this line to enable a NeoPixel on the same pin as RedLED

// The red LED flashes during SD card writes
#define RedLED 13 // The red LED on the Adalogger is connected to Digital Pin 13
// The green LED indicates that the GNSS has established a fix 
#define GreenLED 8 // The green LED on the Adalogger is connected to Digital Pin 8

// Include the Adafruit NeoPixel Library
#ifdef NeoPixel
#include <Adafruit_NeoPixel.h> // Support for the WB2812B
#define swap_red_green // Uncomment this line if your WB2812B has red and green reversed
#ifdef swap_red_green
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RedLED, NEO_GRB + NEO_KHZ800); // GRB WB2812B
#else
  Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, RedLED, NEO_RGB + NEO_KHZ800); // RGB WB2812B
#endif
#define LED_Brightness 32 // 0 - 255 for WB2812B
#endif

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;

// Fast SD card logging using Bill Greiman's SdFat
// https://github.com/greiman/SdFat
// From FatFile.h:
//   * \note Data is moved to the cache but may not be written to the
//   * storage device until sync() is called.

#include <SPI.h>
#include <SdFat.h>
const uint8_t cardSelect = 4; // Adalogger uses D4 as the SD SPI select
SdFat sd;
SdFile rawx_dataFile;
// The log filename starts with "r_" for the rover and "b_" for the static base
bool base_mode = true; // Flag to indicate if the code is in base or rover mode
char rawx_filename[] = "20000000/b_000000.ubx"; // the b will be replaced by an r if required
char dirname[] = "20000000";
long bytes_written = 0;

bool survey_in_mode = false; // Flag to indicate if the code is in survey_in mode

// Timer to indicate if an ExtInt has been received
volatile unsigned long ExtIntTimer; // Load this with millis plus white_flash to show when the ExtInt LED should be switched off

// Define packet size, buffer and buffer pointer for SD card writes
const size_t SDpacket = 512;
uint8_t serBuffer[SDpacket];
size_t bufferPointer = 0;
int numBytes;

// Battery voltage
float vbat;
#define LOWBAT 3.55 // Low battery voltage

// Include Real Time Clock support for the M0
// https://github.com/arduino-libraries/RTCZero
#include <RTCZero.h> // M0 Real Time Clock
RTCZero rtc; // Create an rtc object
volatile bool alarmFlag = false; // RTC alarm (interrupt) flag

// Count number of valid fixes before starting to log
#define maxvalfix 10 // Collect at least this many valid fixes before logging starts
int valfix = 0;

bool stop_pressed = false; // Flag to indicate if stop switch was pressed to stop logging
bool stop_delayed_pressed = false; // Flag to indicate if delayed stop switch is pressed to stop logging
bool stop_delayed_active = false; // Flag to indicate if delayed stop is active

// Define SerialBuffer as a large RingBuffer which we will use to store the Serial1 receive data
// Actual Serial1 receive data will be copied into SerialBuffer by a timer interrupt
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
// That way, we do not need to increase the size of the Serial1 receive buffer (by editing RingBuffer.h)
// You can use DEBUGserialBuffer to determine how big the buffer should be. Increase it if you see bufAvail get close to or reach the buffer size.
RingBufferN<24576> SerialBuffer; // Define SerialBuffer as a RingBuffer of size 24k bytes

// Loop Steps
#define init          0
#define start_rawx    1
#define open_file     2
#define write_file    3
#define new_file      4
#define close_file    5
#define restart_file  6
int loop_step = init;

// UBX and NMEA Parse State
#define looking_for_B5_dollar   0
#define looking_for_62          1
#define looking_for_class       2
#define looking_for_ID          3
#define looking_for_length_LSB  4
#define looking_for_length_MSB  5
#define processing_payload      6
#define looking_for_checksum_A  7
#define looking_for_checksum_B  8
#define sync_lost               9
int ubx_nmea_state = looking_for_B5_dollar;
int ubx_length = 0;
int ubx_class = 0;
int ubx_ID = 0;
int ubx_checksum_A = 0;
int ubx_checksum_B = 0;
int ubx_expected_checksum_A = 0;
int ubx_expected_checksum_B = 0;

// Definitions for u-blox F9P UBX-format (binary) messages
// As a temporary fix, until multi-setVal is available, these messages are defined in SparkFun ubxPacket format:
// Class, ID, Length, Counter, startingSpot, [payload], checksumA, checksumB, valid
// ** Make sure that Length matches the payload length **
// As these are all UBX-CFG-VALSET messages, the Class is always 0x06 and the ID is always 0x8a
// The payload format is:
// version, layers, reserved, reserved, key1, value1, key2, value2...
// version is 0
// layers: 0x01 for RAM, 0x02 for BBR, 0x04 for Flash. Setting layers to 0x07 would set the value in RAM, BBR and Flash
// The VALSET message can hold a maximum of 64 key and value pairs
// The sendCommand calls used to transmit each message will timeout as the commandAck checking in processUBXpacket expects the packet to be defined in packetCfg, not our custom packet!
// Turn on DEBUG to see if the commands are acknowledged (Received: CLS:5 ID:1 Payload: 6 8A) or not acknowledged (CLS:5 ID:0)

// Disable NMEA output on the I2C port
// UBX-CFG-VALSET message with a key ID of 0x10720002 (CFG-I2COUTPROT-NMEA) and a value of 0
// Setting is updated in RAM only
static uint8_t disableI2cNMEA_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x02, 0x00, 0x72, 0x10,  0x00 };
ubxPacket disableI2cNMEA = { 0x06, 0x8a,  9, 0, 0,  disableI2cNMEA_payload,  0, 0, false };

// Set UART1 to 230400 Baud
// UBX-CFG-VALSET message with a key ID of 0x40520001 (CFG-UART1-BAUDRATE) and a value of 0x00038400 (230400 decimal)
static uint8_t setUART1BAUD_payload_230400[] = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x52, 0x40,  0x00, 0x84, 0x03, 0x00 };
ubxPacket setUART1BAUD_230400 = { 0x06, 0x8a,  12, 0, 0,  setUART1BAUD_payload_230400,  0, 0, false };

// Set UART1 to 460800 Baud
// UBX-CFG-VALSET message with a key ID of 0x40520001 (CFG-UART1-BAUDRATE) and a value of 0x00070800 (460800 decimal)
static uint8_t setUART1BAUD_payload_460800[] = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x52, 0x40,  0x00, 0x08, 0x07, 0x00 };
ubxPacket setUART1BAUD_460800 = { 0x06, 0x8a,  12, 0, 0,  setUART1BAUD_payload_460800,  0, 0, false };

// setRAWXoff: this is the message which disables all of the messages being logged to SD card
// It also clears the NMEA high precision mode for the GPGGA message
// It also sets the main talker ID to 'GP'
// UBX-CFG-VALSET message with key IDs of:
// 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
// 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
// 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)
// 0x2091002a (CFG-MSGOUT-UBX_NAV_POSLLH_UART1)
// 0x20910007 (CFG-MSGOUT-UBX_NAV_PVT_UART1)
// 0x2091001b (CFG-MSGOUT-UBX_NAV_STATUS_UART1)
// 0x10930006 (CFG-NMEA-HIGHPREC)
// 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
// and values (rates) of zero
// 0x20930031 (CFG-NMEA-MAINTALKERID) has value 1 (GP)
static uint8_t setRAWXoff_payload[] = {
  0x00, 0x01, 0x00, 0x00,           // version, layers, reserved, reserved
  0xa5, 0x02, 0x91, 0x20,  0x00,    // CFG-MSGOUT-UBX_RXM_RAWX_UART1
  0x32, 0x02, 0x91, 0x20,  0x00,    // CFG-MSGOUT-UBX_RXM_SFRBX_UART1
  0x79, 0x01, 0x91, 0x20,  0x00,    // CFG-MSGOUT-UBX_TIM_TM2_UART1
  0x2a, 0x00, 0x91, 0x20,  0x00,    // CFG-MSGOUT-UBX_NAV_POSLLH_UART1
  0x07, 0x00, 0x91, 0x20,  0x00,    // CFG-MSGOUT-UBX_NAV_PVT_UART1
  0x1b, 0x00, 0x91, 0x20,  0x00,    // CFG-MSGOUT-UBX_NAV_STATUS_UART1
  0x31, 0x00, 0x93, 0x20,  0x01,    // CFG-NMEA-MAINTALKERID : This line sets the main talker ID to GP
  0x06, 0x00, 0x93, 0x10,  0x00,    // CFG-NMEA-HIGHPREC : This line disables NMEA high precision mode
  0xbb, 0x00, 0x91, 0x20,  0x00 };  // CFG-MSGOUT-NMEA_ID_GGA_UART1 : This line disables the GGA message
ubxPacket setRAWXoff = { 0x06, 0x8a,  49, 0, 0,  setRAWXoff_payload,  0, 0, false };

// setRAWXon: this is the message which enables all of the messages to be logged to SD card in one go
// It also sets the NMEA high precision mode for the GNGGA message
// It also sets the main talker ID to 'GN'
// UBX-CFG-VALSET message with key IDs of:
// 0x209102a5 (CFG-MSGOUT-UBX_RXM_RAWX_UART1)
// 0x20910232 (CFG-MSGOUT-UBX_RXM_SFRBX_UART1)
// 0x20910179 (CFG-MSGOUT-UBX_TIM_TM2_UART1)
// 0x2091002a (CFG-MSGOUT-UBX_NAV_POSLLH_UART1)
// 0x20910007 (CFG-MSGOUT-UBX_NAV_PVT_UART1)
// 0x2091001b (CFG-MSGOUT-UBX_NAV_STATUS_UART1)
// 0x10930006 (CFG-NMEA-HIGHPREC)
// 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
// and values (rates) of 1
// 0x20930031 (CFG-NMEA-MAINTALKERID) has value 3 (GN)
static uint8_t setRAWXon_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0xa5, 0x02, 0x91, 0x20,  0x01,
  0x32, 0x02, 0x91, 0x20,  0x01,
  0x79, 0x01, 0x91, 0x20,  0x01,
  0x2a, 0x00, 0x91, 0x20,  0x01,   // Change the last byte from 0x01 to 0x00 to leave NAV_POSLLH disabled
  0x07, 0x00, 0x91, 0x20,  0x01,   // Change the last byte from 0x01 to 0x00 to leave NAV_PVT disabled
  0x1b, 0x00, 0x91, 0x20,  0x01,   // This line enables the NAV_STATUS message
  0x31, 0x00, 0x93, 0x20,  0x03,   // This line sets the main talker ID to GN
  0x06, 0x00, 0x93, 0x10,  0x01,   // This sets the NMEA high precision mode
  0xbb, 0x00, 0x91, 0x20,  0x01 }; // This (re)enables the GGA mesage
ubxPacket setRAWXon = { 0x06, 0x8a,  49, 0, 0,  setRAWXon_payload,  0, 0, false };

// Enable the NMEA GGA and RMC messages and disable the GLL, GSA, GSV, VTG, and TXT(INF) messages
// UBX-CFG-VALSET message with key IDs of:
// 0x209100ca (CFG-MSGOUT-NMEA_ID_GLL_UART1)
// 0x209100c0 (CFG-MSGOUT-NMEA_ID_GSA_UART1)
// 0x209100c5 (CFG-MSGOUT-NMEA_ID_GSV_UART1)
// 0x209100b1 (CFG-MSGOUT-NMEA_ID_VTG_UART1)
// 0x20920007 (CFG-INFMSG-NMEA_UART1)
// 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
// 0x209100ac (CFG-MSGOUT-NMEA_ID_RMC_UART1)
static uint8_t setNMEAon_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0xca, 0x00, 0x91, 0x20,  0x00,
  0xc0, 0x00, 0x91, 0x20,  0x00,
  0xc5, 0x00, 0x91, 0x20,  0x00,
  0xb1, 0x00, 0x91, 0x20,  0x00,
  0x07, 0x00, 0x92, 0x20,  0x00,
  0xbb, 0x00, 0x91, 0x20,  0x01,
  0xac, 0x00, 0x91, 0x20,  0x01 };
ubxPacket setNMEAon = { 0x06, 0x8a,  39, 0, 0,  setNMEAon_payload,  0, 0, false };

// Disable the NMEA GGA and RMC messages
// UBX-CFG-VALSET message with key IDs of:
// 0x209100bb (CFG-MSGOUT-NMEA_ID_GGA_UART1)
// 0x209100ac (CFG-MSGOUT-NMEA_ID_RMC_UART1)
static uint8_t setNMEAoff_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0xbb, 0x00, 0x91, 0x20,  0x00,
  0xac, 0x00, 0x91, 0x20,  0x00 };
ubxPacket setNMEAoff = { 0x06, 0x8a,  14, 0, 0,  setNMEAoff_payload,  0, 0, false };

// Set the Main NMEA Talker ID to "GP"
// UBX-CFG-VALSET message with a key ID of 0x20930031 (CFG-NMEA-MAINTALKERID) and a value of 1 (GP):
static uint8_t setTALKERID_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x31, 0x00, 0x93, 0x20,  0x01 };
ubxPacket setTALKERID = { 0x06, 0x8a,  9, 0, 0,  setTALKERID_payload,  0, 0, false };

// Set the measurement rate
// UBX-CFG-VALSET message with a key ID of 0x30210001 (CFG-RATE-MEAS)
static uint8_t setRATE_20Hz_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0x32, 0x00 };
static uint8_t setRATE_10Hz_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0x64, 0x00 };
static uint8_t setRATE_8Hz_4c_payload[]  = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30, 0x7d, 0x00, 0x02, 0x00, 0x21, 0x30, 0x04, 0x00 }; // 8hz meas, 4 cycles navigation
static uint8_t setRATE_5Hz_payload[]  = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xc8, 0x00 };
static uint8_t setRATE_4Hz_payload[]  = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xfa, 0x00 };
static uint8_t setRATE_2Hz_payload[]  = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xf4, 0x01 };
static uint8_t setRATE_1Hz_payload[]  = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x21, 0x30,  0xe8, 0x03 };
ubxPacket setRATE_20Hz = { 0x06, 0x8a,  10, 0, 0,  setRATE_20Hz_payload,  0, 0, false };
ubxPacket setRATE_10Hz = { 0x06, 0x8a,  10, 0, 0,  setRATE_10Hz_payload,  0, 0, false };
ubxPacket setRATE_8Hz = { 0x06, 0x8a,  16, 0, 0,  setRATE_8Hz_4c_payload,  0, 0, false };
ubxPacket setRATE_5Hz = { 0x06, 0x8a,  10, 0, 0,  setRATE_5Hz_payload,  0, 0, false };
ubxPacket setRATE_4Hz = { 0x06, 0x8a,  10, 0, 0,  setRATE_4Hz_payload,  0, 0, false };
ubxPacket setRATE_2Hz = { 0x06, 0x8a,  10, 0, 0,  setRATE_2Hz_payload,  0, 0, false };
ubxPacket setRATE_1Hz = { 0x06, 0x8a,  10, 0, 0,  setRATE_1Hz_payload,  0, 0, false };

// Set the navigation dynamic model
// UBX-CFG-VALSET message with a key ID of 0x20110021 (CFG-NAVSPG-DYNMODEL)
static uint8_t setNAVportable_payload[]   = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x00 };
static uint8_t setNAVstationary_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x02 };
static uint8_t setNAVpedestrian_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x03 };
static uint8_t setNAVautomotive_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x04 };
static uint8_t setNAVsea_payload[]        = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x05 };
static uint8_t setNAVair1g_payload[]      = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x06 };
static uint8_t setNAVair2g_payload[]      = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x07 };
static uint8_t setNAVair4g_payload[]      = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x08 };
static uint8_t setNAVwrist_payload[]      = { 0x00, 0x01, 0x00, 0x00,  0x21, 0x00, 0x11, 0x20,  0x09 };
ubxPacket setNAVportable = { 0x06, 0x8a,  9, 0, 0,  setNAVportable_payload,  0, 0, false };
ubxPacket setNAVstationary = { 0x06, 0x8a,  9, 0, 0,  setNAVstationary_payload,  0, 0, false };
ubxPacket setNAVpedestrian = { 0x06, 0x8a,  9, 0, 0,  setNAVpedestrian_payload,  0, 0, false };
ubxPacket setNAVautomotive = { 0x06, 0x8a,  9, 0, 0,  setNAVautomotive_payload,  0, 0, false };
ubxPacket setNAVsea = { 0x06, 0x8a,  9, 0, 0,  setNAVsea_payload,  0, 0, false };
ubxPacket setNAVair1g = { 0x06, 0x8a,  9, 0, 0,  setNAVair1g_payload,  0, 0, false };
ubxPacket setNAVair2g = { 0x06, 0x8a,  9, 0, 0,  setNAVair2g_payload,  0, 0, false };
ubxPacket setNAVair4g = { 0x06, 0x8a,  9, 0, 0,  setNAVair4g_payload,  0, 0, false };
ubxPacket setNAVwrist = { 0x06, 0x8a,  9, 0, 0,  setNAVwrist_payload,  0, 0, false };

// Set UART2 to 230400 Baud
// UBX-CFG-VALSET message with a key ID of 0x40530001 (CFG-UART2-BAUDRATE) and a value of 0x00038400 (230400 decimal)
static uint8_t setUART2BAUD_230400_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x53, 0x40,  0x00, 0x84, 0x03, 0x00 };
ubxPacket setUART2BAUD_230400 = { 0x06, 0x8a,  12, 0, 0,  setUART2BAUD_230400_payload,  0, 0, false };

// Set UART2 to 115200 Baud
// UBX-CFG-VALSET message with a key ID of 0x40530001 (CFG-UART2-BAUDRATE) and a value of 0x0001c200 (115200 decimal)
static uint8_t setUART2BAUD_115200_payload[] = { 0x00, 0x01, 0x00, 0x00,  0x01, 0x00, 0x53, 0x40,  0x00, 0xc2, 0x01, 0x00 };
ubxPacket setUART2BAUD_115200 = { 0x06, 0x8a,  12, 0, 0,  setUART2BAUD_115200_payload,  0, 0, false };

// Set Survey_In mode
// UBX-CFG-VALSET message with a key IDs and values of:
// 0x20030001 (CFG-TMODE-MODE) and a value of 1
// 0x40030011 (CFG-TMODE-SVIN_ACC_LIMIT) and a value of 0x0000c350 (50000 decimal = 5 m)
// 0x40030010 (CFG-TMODE-SVIN_MIN_DUR) and a value of 0x0000003c (60 decimal = 1 min)
static uint8_t setSurveyIn_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x03, 0x20,  0x01,
  0x11, 0x00, 0x03, 0x40,  0x50, 0xc3, 0x00, 0x00,
  0x10, 0x00, 0x03, 0x40,  0x3c, 0x00, 0x00, 0x00 };
ubxPacket setSurveyIn = { 0x06, 0x8a,  25, 0, 0,  setSurveyIn_payload,  0, 0, false };

// Disable Survey_In mode
// UBX-CFG-VALSET message with a key ID of 0x20030001 (CFG-TMODE-MODE) and a value of 0
static uint8_t disableSurveyIn_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x03, 0x20,  0x00 };
ubxPacket disableSurveyIn = { 0x06, 0x8a,  9, 0, 0,  disableSurveyIn_payload,  0, 0, false };

// Enable RTCM message output on UART2
// UBX-CFG-VALSET message with the following key IDs
// Set the value byte to 0x01 to send an RTCM message at RATE_MEAS; set the value to 0x04 to send an RTCM message at 1/4 RATE_MEAS
// (i.e. assumes you will be logging RAWX data at 4 Hz. Adjust accordingly)
// 0x209102bf (CFG-MSGOUT-RTCM_3X_TYPE1005_UART2)
// 0x209102ce (CFG-MSGOUT-RTCM_3X_TYPE1077_UART2)
// 0x209102d3 (CFG-MSGOUT-RTCM_3X_TYPE1087_UART2)
// 0x209102d8 (CFG-MSGOUT-RTCM_3X_TYPE1127_UART2)
// 0x2091031a (CFG-MSGOUT-RTCM_3X_TYPE1097_UART2)
// 0x20910305 (CFG-MSGOUT-RTCM_3X_TYPE1230_UART2)
static uint8_t setRTCMon_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0xbf, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0xce, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0xd3, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0xd8, 0x02, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0x1a, 0x03, 0x91, 0x20,  0x04, // Change the last byte from 0x01 to 0x04 to send an RTCM message at 1/4 RATE_MEAS
  0x05, 0x03, 0x91, 0x20,  0x28 };  // Change the last byte from 0x01 to 0x28 to send an RTCM message at 1/40 RATE_MEAS
ubxPacket setRTCMon = { 0x06, 0x8a,  34, 0, 0,  setRTCMon_payload,  0, 0, false };

// Disable RTCM message output on UART2
// UBX-CFG-VALSET message with the following key IDs and values of 0:
// 0x209102bf (CFG-MSGOUT-RTCM_3X_TYPE1005_UART2)
// 0x209102ce (CFG-MSGOUT-RTCM_3X_TYPE1077_UART2)
// 0x209102d3 (CFG-MSGOUT-RTCM_3X_TYPE1087_UART2)
// 0x209102d8 (CFG-MSGOUT-RTCM_3X_TYPE1127_UART2)
// 0x2091031a (CFG-MSGOUT-RTCM_3X_TYPE1097_UART2)
// 0x20910305 (CFG-MSGOUT-RTCM_3X_TYPE1230_UART2)
static uint8_t setRTCMoff_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0xbf, 0x02, 0x91, 0x20,  0x00,
  0xce, 0x02, 0x91, 0x20,  0x00,
  0xd3, 0x02, 0x91, 0x20,  0x00,
  0xd8, 0x02, 0x91, 0x20,  0x00,
  0x1a, 0x03, 0x91, 0x20,  0x00,
  0x05, 0x03, 0x91, 0x20,  0x00 };
ubxPacket setRTCMoff = { 0x06, 0x8a,  34, 0, 0,  setRTCMoff_payload,  0, 0, false };

// Set TimeGrid for TP1 to GPS (instead of UTC) so TIM_TM2 messages are aligned with GPS time
// UBX-CFG-VALSET message with the key ID 0x2005000c (CFG-TP-TIMEGRID_TP1) and value of 1 (GPS):
static uint8_t setTimeGrid_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0x0c, 0x00, 0x05, 0x20,  0x01 };
ubxPacket setTimeGrid = { 0x06, 0x8a,  9, 0, 0,  setTimeGrid_payload,  0, 0, false };

// Enable NMEA messages on UART2 for test purposes
// UBX-CFG-VALSET message with key ID of 0x10760002 (CFG-UART2OUTPROT-NMEA) and value of 1:
static uint8_t setUART2nmea_payload[] = {
  0x00, 0x01, 0x00, 0x00,
  0x02, 0x00, 0x76, 0x10,  0x01 };
ubxPacket setUART2nmea = { 0x06, 0x8a,  9, 0, 0,  setUART2nmea_payload,  0, 0, false };

// ExtInt interrupt service routine
void ExtInt() {
  ExtIntTimer = millis() + white_flash; // Set the timer value to white_flash milliseconds from now
}

// RTC alarm interrupt
// Must be kept as short as possible. Update the alarm time in the main loop, not here.
void alarmMatch()
{
  alarmFlag = true; // Set alarm flag
}

// TimerCounter3 functions to copy Serial1 receive data into SerialBuffer
// https://gist.github.com/jdneo/43be30d85080b175cb5aed3500d3f989
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 16

// Set TC3 Interval (sec)
void setTimerInterval(float intervalS) {
  int compareValue = intervalS * CPU_HZ / TIMER_PRESCALER_DIV;
  if (compareValue > 65535) compareValue = 65535;
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

// Start TC3 with a specified interval
void startTimerInterval(float intervalS) {
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;

  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 16
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerInterval(intervalS);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_SetPriority(TC3_IRQn, 3); // Set the TC3 interrupt priority to 3 (lowest)
  NVIC_EnableIRQ(TC3_IRQn);

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

// TC3 Interrupt Handler
void TC3_Handler() {
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // copy any available Serial1 data into SerialBuffer
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    int available1 = Serial1.available(); // Check if there is any data waiting in the Serial1 RX buffer
    while (available1 > 0) { 
        SerialBuffer.store_char(Serial1.read()); // If there is, copy it into our RingBuffer
        available1--;
    }
  }
}

// NeoPixel Functions
// WB2812B blue LED has the highest forward voltage and is slightly dim at 3.3V. The red and green values are adapted accordingly (222 instead of 255).

#ifdef NeoPixel

// Define the NeoPixel colors
#define black 0
#define blue 1
#define cyan 2
#define green 3
#define yellow 4
#define red 5
#define magenta 6
#define white 7
#define dim_blue 8
#define dim_cyan 9
#define dim_green 10
#define dim_yellow 11
#define dim_red 12
#define dim_magenta 13
#define dim_white 14

#define to_dim 7 // Offset from bright to dim colors

int write_color = green; // Flash the NeoPixel this color during SD writes (can be set to magenta or yellow too)

volatile int last_color = black;
volatile int this_color = black;

void setLED(int color) // Set NeoPixel color
{
  if (color >= dim_blue)
  {
    pixels.setBrightness(LED_Brightness / 2); // Dim the LED brightness
  }
  if (color == black)
  {
    pixels.setPixelColor(0,0,0,0);
  }
  else if ((color == dim_blue) || (color == blue))
  {
    pixels.setPixelColor(0, pixels.Color(0,0,255)); // Set color
  }
  else if ((color == dim_cyan) || (color == cyan))
  {
    pixels.setPixelColor(0, pixels.Color(0,222,255)); // Set color
  }
  else if ((color == dim_green) || (color == green))
  {
    pixels.setPixelColor(0, pixels.Color(0,222,0)); // Set color
  }
  else if ((color == dim_yellow) || (color == yellow))
  {
    pixels.setPixelColor(0, pixels.Color(222,222,0)); // Set color
  }
  else if ((color == dim_red) || (color == red))
  {
    pixels.setPixelColor(0, pixels.Color(222,0,0)); // Set color
  }
  else if ((color == dim_magenta) || (color == magenta))
  {
    pixels.setPixelColor(0, pixels.Color(222,0,255)); // Set color
  }
  else // must be dim_white or white
  {
    pixels.setPixelColor(0, pixels.Color(222,222,255)); // Set color
  }
  pixels.show();
  if (color >= dim_blue)
  {
    pixels.setBrightness(LED_Brightness); // Reset the LED brightness
  }
  last_color = this_color;
  this_color = color;
}

#endif

// SerialBuffer DEBUG
#ifdef DEBUGserialBuffer
int maxSerialBufferAvailable = 0;
#endif

void setup()
{
#ifdef NeoPixel
  // Initialise the NeoPixel
  pixels.begin(); // This initializes the NeoPixel library.
  delay(100); // Seems necessary to make the NeoPixel start reliably 
  pixels.setBrightness(LED_Brightness); // Initialize the LED brightness
  setLED(black); // Set NeoPixel off
#ifndef NoLED
  setLED(dim_blue); // Set NeoPixel to dim blue
#endif
#else
  // initialize digital pins RedLED and GreenLED as outputs.
  pinMode(RedLED, OUTPUT); // Red LED
  pinMode(GreenLED, OUTPUT); // Green LED
  digitalWrite(RedLED, LOW); // Turn Red LED off
  digitalWrite(GreenLED, LOW); // Turn Green LED off
#ifndef NoLED
  // flash red and green LEDs on reset
  for (int i=0; i <= 4; i++) {
    digitalWrite(RedLED, HIGH);
    delay(200);
    digitalWrite(RedLED, LOW);
    digitalWrite(GreenLED, HIGH);
    delay(200);
    digitalWrite(GreenLED, LOW);
  }
#endif
#endif

  // initialize modePin (A0) as an input for the Base/Rover mode select switch
  pinMode(modePin, INPUT_PULLUP);

  // initialize swPin (A1) as an input for the stop switch
  pinMode(swPin, INPUT_PULLUP);

  // initialise ExtIntPin (A2) as an input for the EVENT switch
  pinMode(ExtIntPin, INPUT_PULLUP);
  // Attach the interrupt service routine
  // Interrupt on falling edge of the ExtInt signal
  attachInterrupt(ExtIntPin, ExtInt, FALLING);
  ExtIntTimer = millis(); // Initialise the ExtInt LED timer

  // initialise SurveyInPin (A3) as an input for the SURVEY_IN switch
  pinMode(SurveyInPin, INPUT_PULLUP);

  // initialise DelayedPin (A4) as an input for the delayed_stop switch
  pinMode(DelayedPin, INPUT_PULLUP);

  delay(10000); // Allow 10 sec for user to open serial monitor (Comment this line if required)
  //while (!Serial); // OR Wait for user to run python script or open serial monitor (Comment this line as required)

  Serial.begin(115200);

  Serial.println("RAWX Logger F9P");
  Serial.println("Log GNSS RAWX data to SD card");
#ifndef NeoPixel
  Serial.println("Green LED = Initial GNSS Fix");
  Serial.println("Red LED Flash = SD Write");
#else
  Serial.println("Blue = Init");
  Serial.println("Dim Cyan = Waiting for GNSS Fix");
  Serial.println("Cyan = Checking GNSS Fix");
  Serial.println("Green flash = SD Write");
  Serial.println("Magenta flash = TIME fix in Survey_In mode");
  Serial.println("Yellow flash = fixed carrier solution");
  Serial.println("White = EVENT (ExtInt) detected");
#endif
  Serial.println("Continuous Red indicates a problem or that logging has been stopped");
  Serial.println("Initializing GNSS...");

#ifndef NoLED
#ifdef NeoPixel
  setLED(blue); // Set NeoPixel to blue
#endif
#endif

  // Initialise UBX communication over I2C
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (i2cGPS.begin(Wire,0x42) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Panic!! Ublox GNSS not detected at default I2C address. Please check wiring. Freezing!"));
#ifndef NoLED
#ifdef NeoPixel
    setLED(red); // Set NeoPixel to red
#else
    digitalWrite(RedLED, HIGH); // Turn red LED on
#endif
#endif    
    while (1);
  }
  Serial.println(F("Ublox GNSS found!"));

#ifdef DEBUG
  //i2cGPS.enableDebugging(); //Enable debug messages over Serial (default)
#endif

  // These sendCommands will timeout as the commandAck checking in processUBXpacket expects the packet to be in packetCfg, not our custom packet!
  // Turn on DEBUG to see if the commands are acknowledged (Received: CLS:5 ID:1 Payload: 6 8A) or not acknowledged (CLS:5 ID:0)
  i2cGPS.sendCommand(disableI2cNMEA); //Disable NMEA messages on the I2C port leaving it clear for UBX messages
  i2cGPS.sendCommand(setNMEAoff); // Disable NMEA messages (need testing, it seems it doesn't disable NMEA output on UART1)
  i2cGPS.sendCommand(setUART1BAUD_460800); // Change the UART1 baud rate to 460800
  i2cGPS.sendCommand(setRAWXoff); // Disable RAWX messages (on UART1). Also disables the NMEA high precision mode
  i2cGPS.sendCommand(setRATE_1Hz); // Set Navigation/Measurement Rate to 1Hz
  i2cGPS.sendCommand(setUART2BAUD_115200); // Set UART2 Baud rate
  i2cGPS.sendCommand(disableSurveyIn); // Disable Survey_In mode
  i2cGPS.sendCommand(setRTCMoff); // Disable RTCM output on UART2
  i2cGPS.sendCommand(setTimeGrid); // Set the TP1 TimeGrid to GPS so TIM_TM2 messages are aligned with GPS time

  // Check the modePin and set the navigation dynamic model
  if (digitalRead(modePin) == LOW) {
    Serial.println("BASE mode selected");
    i2cGPS.sendCommand(setNAVstationary); // Set Static Navigation Mode (use this for the Base Logger)    
  }
  else {
    base_mode = false; // Clear base_mode flag
    Serial.println("ROVER mode selected");
    // Select one mode for the mobile Rover Logger
    //i2cGPS.sendCommand(setNAVportable); // Set Portable Navigation Mode
    //i2cGPS.sendCommand(setNAVpedestrian); // Set Pedestrian Navigation Mode
    //i2cGPS.sendCommand(setNAVautomotive); // Set Automotive Navigation Mode
    //i2cGPS.sendCommand(setNAVsea); // Set Sea Navigation Mode
    i2cGPS.sendCommand(setNAVair1g); // Set Airborne <1G Navigation Mode
    //i2cGPS.sendCommand(setNAVair2g); // Set Airborne <2G Navigation Mode
    //i2cGPS.sendCommand(setNAVair4g); // Set Airborne <4G Navigation Mode
    //i2cGPS.sendCommand(setNAVwrist); // Set Wrist Navigation Mode
  }
  Serial1.begin(460800); // Start Serial1 at 460800 baud
  while(Serial1.available()){Serial1.read();} // Flush RX buffer so we don't confuse Adafruit_GPS with UBX acknowledgements

  Serial.println("GNSS initialized!");

#ifndef NoLED
#ifndef NeoPixel
  // flash the red LED during SD initialisation
  digitalWrite(RedLED, HIGH);
#endif
#endif

  // Initialise SD card
  Serial.println("Initializing SD card...");
  // See if the SD card is present and can be initialized
  if (!sd.begin(cardSelect, SD_SCK_MHZ(50))) {
    Serial.println("Panic!! SD Card Init failed, or not present!");
    Serial.println("Waiting for reset...");
#ifndef NoLED
#ifdef NeoPixel
    setLED(red); // Set NeoPixel to red
#endif
#endif
    // don't do anything more:
    while(1);
  }
  Serial.println("SD Card initialized!");

#ifndef NoLED
#ifdef NeoPixel
  setLED(dim_cyan); // Set NeoPixel to dim cyan now that the SD card is initialised
#else
  // turn red LED off
  digitalWrite(RedLED, LOW);
#endif
#endif

#ifdef NeoPixel
      write_color = green; // Reset the write color to green
#endif
          
  Serial.println("Waiting for GNSS fix...");
}

void loop() // run over and over again
{
  switch(loop_step) {
    case init: {

#ifdef DEBUG
        Serial.print("\nTime: ");
        Serial.print(i2cGPS.getHour(), DEC); Serial.print(':');
        Serial.print(i2cGPS.getMinute(), DEC); Serial.print(':');
        Serial.print(i2cGPS.getSecond(), DEC); Serial.print('.');
        Serial.println(i2cGPS.getMillisecond());
        Serial.print("Date: ");
        Serial.print(i2cGPS.getYear(), DEC); Serial.print("/");
        Serial.print(i2cGPS.getMonth(), DEC);  Serial.print("/");
        Serial.println(i2cGPS.getDay(), DEC);
        Serial.print("Fix: "); Serial.println((int)i2cGPS.getFixType());
        //Serial.print(" Quality: "); Serial.println((int)GPS.fixquality);
        if (i2cGPS.getFixType() == 3) {
          
          Serial.print("Location: ");
          Serial.print(i2cGPS.getLatitude() * 1E-7, 6);
          Serial.print(", ");
          Serial.println(i2cGPS.getLongitude() * 1E-7, 6);
          Serial.print("Speed (m/s): "); Serial.println(i2cGPS.getGroundSpeed() * 1E-3);
          Serial.print("Heading: "); Serial.println(i2cGPS.getHeading() * 1E-5);
          Serial.print("Altitude (m): "); Serial.println(i2cGPS.getAltitude() * 1E-3);
          Serial.print("Satellites: "); Serial.println((int)i2cGPS.getSIV());
          Serial.print("HDOP: "); Serial.println(i2cGPS.getPDOP() * 1E-2);
        }
#endif
  
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
#ifdef DEBUG
        Serial.print("Battery(V): ");
        Serial.println(vbat, 2);
#endif
      
        // turn green LED on to indicate GNSS fix
        // or set NeoPixel to cyan
        if (i2cGPS.getFixType() == 3) {
#ifndef NoLED
#ifdef NeoPixel
          setLED(cyan); // Set NeoPixel to cyan
#else
          digitalWrite(GreenLED, HIGH);
#endif
#endif
          // increment valfix and cap at maxvalfix
          // don't do anything fancy in terms of decrementing valfix as we want to keep logging even if the fix is lost
          valfix += 1;
          if (valfix > maxvalfix) valfix = maxvalfix;
        }
        else {
#ifndef NoLED
#ifdef NeoPixel
          setLED(dim_cyan); // Set NeoPixel to dim cyan
#else
          digitalWrite(GreenLED, LOW); // Turn green LED off
#endif
#endif
        }
  
        if (valfix == maxvalfix) { // wait until we have enough valid fixes
          
          // Set and start the RTC
          alarmFlag = false; // Make sure alarm flag is clear
          rtc.begin(); // Start the RTC
          rtc.setTime(i2cGPS.getHour(), i2cGPS.getMinute(), i2cGPS.getSecond()); // Set the time
          rtc.setDate(i2cGPS.getDay(), i2cGPS.getMonth(), i2cGPS.getYear() - 2000); // Set the date
          set_Alarm(INTERVAL);
          // check if voltage is > LOWBAT(V), if not then don't try to log any data
          if (vbat < LOWBAT) {
            Serial.println("Low Battery!");
            break;
          }


          // Set the RAWX measurement rate
          //i2cGPS.sendCommand(setRATE_20Hz); // Set Navigation/Measurement Rate to 20 Hz
          //i2cGPS.sendCommand(setRATE_10Hz); // Set Navigation/Measurement Rate to 10 Hz
          i2cGPS.sendCommand(setRATE_8Hz); // Set Measurement Rate to 8 Hz, navigation to 2Hz
          //i2cGPS.sendCommand(setRATE_5Hz); // Set Navigation/Measurement Rate to 5 Hz
          //i2cGPS.sendCommand(setRATE_4Hz); // Set Navigation/Measurement Rate to 4 Hz
          //i2cGPS.sendCommand(setRATE_2Hz); // Set Navigation/Measurement Rate to 2 Hz
          //i2cGPS.sendCommand(setRATE_1Hz); // Set Navigation/Measurement Rate to 1 Hz
          
          delay(1100); // Wait

          // If we are in BASE mode, check the SURVEY_IN pin
          if (base_mode == true) {
            if (digitalRead(SurveyInPin) == LOW) {
              // We are in BASE mode and the SURVEY_IN pin is low so send the extra UBX messages:
              Serial.println("SURVEY_IN mode selected");
              survey_in_mode = true; // Set the survey_in_mode flag true
              i2cGPS.sendCommand(setRTCMon); // Enable the RTCM messages on UART2
              delay(1100);
              i2cGPS.sendCommand(setSurveyIn); // Enable SURVEY_IN mode
              delay(1100);
            }
          }
          
          while(Serial1.available()){Serial1.read();} // Flush RX buffer to clear UBX acknowledgements

          // Now that Serial1 should be idle and the buffer empty, start TC3 interrupts to copy all new data into SerialBuffer
          // Set the timer interval to 10 * 10 / 460800 = 0.000217 secs (10 bytes * 10 bits (1 start, 8 data, 1 stop) at 460800 baud)
          startTimerInterval(0.000217); 
          
          loop_step = start_rawx; // start rawx messages
        }
    }
    break;

    // (Re)Start RAWX messages
    case start_rawx: {
      i2cGPS.sendCommand(setRAWXon); // (Re)Start the UBX and NMEA messages

      bufferPointer = 0; // (Re)initialise bufferPointer
      maxSerialBufferAvailable = 0; // (Re)initialise maxSerialBufferAvailable
      loop_step = open_file; // start logging rawx data
    }
    break;

    // Open the log file
    case open_file: {
      
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
      
      // Do the divides to convert date and time to char
      char secT = RTCseconds/10 + '0';
      char secU = RTCseconds%10 + '0';
      char minT = RTCminutes/10 + '0';
      char minU = RTCminutes%10 + '0';
      char hourT = RTChours/10 + '0';
      char hourU = RTChours%10 + '0';
      char dayT = RTCday/10 +'0';
      char dayU = RTCday%10 +'0';
      char monT = RTCmonth/10 +'0';
      char monU = RTCmonth%10 +'0';
      char yearT = RTCyear/10 +'0';
      char yearU = RTCyear%10 +'0';
  
      // filename is limited to 8.3 characters so use format: YYYYMMDD/b_HHMMSS.ubx or YYYYMMDD/r_HHMMSS.ubx
      rawx_filename[2] = yearT;
      rawx_filename[3] = yearU;
      rawx_filename[4] = monT;
      rawx_filename[5] = monU;
      rawx_filename[6] = dayT;
      rawx_filename[7] = dayU;
      if (base_mode == false) rawx_filename[9] = 'r';
      rawx_filename[11] = hourT;
      rawx_filename[12] = hourU;
      rawx_filename[13] = minT;
      rawx_filename[14] = minU;
      rawx_filename[15] = secT;
      rawx_filename[16] = secU;
      
      dirname[2] = yearT;
      dirname[3] = yearU;
      dirname[4] = monT;
      dirname[5] = monU;
      dirname[6] = dayT;
      dirname[7] = dayU;


      // flash red LED to indicate SD write (leave on if an error occurs)
      // or flash NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(green); // Set the NeoPixel to green
#else
      setLED(black); // Turn NeoPixel off if NoLogLED
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off for NoLogLED
      digitalWrite(GreenLED, LOW); // Turn the green LED off for NoLogLED
#endif
#endif
#endif

      // try to create subdirectory (even if it exists already)
      sd.mkdir(dirname);
      
      // Open the rawx file for fast writing
      if (rawx_dataFile.open(rawx_filename, O_CREAT | O_WRITE | O_EXCL)) {
        Serial.print("Logging to ");
        Serial.println(rawx_filename);
      }
      // if the file isn't open, pop up an error:
      else {
        Serial.println("Panic!! Error opening RAWX file!");
        Serial.println("Waiting for reset...");
#ifndef NoLED
#ifdef NeoPixel
      setLED(red); // Set the NeoPixel to red to indicate a problem
#else
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate a problem
#endif
#endif
        // don't do anything more:
        while(1);
      }

#ifdef DEBUG
      // Set the log file creation time
      if (!rawx_dataFile.timestamp(T_CREATE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file create timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_CREATE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif

      // Now that SD write is complete
      // Turn the Red LED off or set NeoPixel to dim green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(dim_green);
#endif
#else
      digitalWrite(RedLED, LOW); // turn red LED off
#endif
#endif

      bytes_written = 0; // Clear bytes_written

      ubx_nmea_state = looking_for_B5_dollar; // set ubx_nmea_state to expect B5 or $
      ubx_length = 0; // set ubx_length to zero

      loop_step = write_file; // start logging rawx data
    }
    break;

    // Stuff bytes into serBuffer and write when we have reached SDpacket
    case write_file: {
      
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      // Check if NeoPixel should be flashed white
      if (this_color != white) { // Skip if the NeoPixel is already white
        // Check if the NeoPixel should be flashed white
        if (millis() < ExtIntTimer) {
          setLED(white); // Set the NeoPixel to white to indicate an ExtInt
        }
      }
      else { // NeoPixel must already be white so check if it should be turned off
        // Check if the timer has expired
        if (millis() > ExtIntTimer) {
          setLED(last_color); // Set the NeoPixel to the previous color
        }
      }
#endif
#endif
#endif

      int bufAvail = SerialBuffer.available();
      if (bufAvail > 0) {
#ifdef DEBUGserialBuffer
        if (bufAvail > maxSerialBufferAvailable) {
          maxSerialBufferAvailable = bufAvail;
          Serial.print(rtc.getHours()); Serial.print("H"); // Show timestamp
          Serial.print(rtc.getMinutes()); Serial.print("mn");
          Serial.print(rtc.getSeconds()); Serial.print("s : ");
          Serial.print("Max bufAvail: ");
          Serial.println(maxSerialBufferAvailable);
        }
#endif  
        uint8_t c = SerialBuffer.read_char();
        serBuffer[bufferPointer] = c;
        bufferPointer++;
        if (bufferPointer == SDpacket) {
          bufferPointer = 0;
          // Flash the red LED to indicate an SD write
          // or flash the NeoPixel green
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
          if (this_color != white) { // If the NeoPixel is not currently white
            setLED(write_color); // Set the NeoPixel
          }
          else { // If the NeoPixel is white, set last_color to write_color so it will revert to that when the white flash is complete
            last_color = write_color;
          }
#endif
#else
#ifndef NoLogLED
          digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
          numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
          //rawx_dataFile.sync(); // Sync the file system
          bytes_written += SDpacket;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
          if (this_color != white) { // If the NeoPixel is not currently white
            setLED(write_color + to_dim); // Set the NeoPixel
          }
          else { // If the NeoPixel is white, set last_color to dim_write_color so it will revert to that when the white flash is complete
            last_color = write_color + to_dim;
          }
#endif
#else
#ifndef NoLogLED
          digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#endif
#ifdef DEBUG
          if (numBytes != SDpacket) {
            Serial.print("SD write error! Write size was ");
            Serial.print(SDpacket);
            Serial.print(". ");
            Serial.print(numBytes);
            Serial.println(" were written.");
          }
#endif
        }
        // Process data bytes according to ubx_nmea_state
        // For UBX messages:
        // Sync Char 1: 0xB5
        // Sync Char 2: 0x62
        // Class byte
        // ID byte
        // Length: two bytes, little endian
        // Payload: length bytes
        // Checksum: two bytes
        // Only allow a new file to be opened when a complete packet has been processed and ubx_nmea_state has returned to "looking_for_B5_dollar"
        // Or when a data error is detected (sync_lost)
        switch (ubx_nmea_state) {
          Serial.println("Entering ubx_nmea_state case");
          case (looking_for_B5_dollar): {
            if (c == 0xB5) { // Have we found Sync Char 1 (0xB5) if we were expecting one?
              ubx_nmea_state = looking_for_62; // Now look for Sync Char 2 (0x62)
            }
            
            else {
              Serial.println("Panic!! Was expecting Sync Char 0xB5 but received: "); Serial.println(c, HEX);
              ubx_nmea_state = sync_lost;
            }
          }
          break;
          case (looking_for_62): {
            if (c == 0x62) { // Have we found Sync Char 2 (0x62) when we were expecting one?
              ubx_expected_checksum_A = 0; // Reset the expected checksum
              ubx_expected_checksum_B = 0;
              ubx_nmea_state = looking_for_class; // Now look for Class byte
            }
            else {
              Serial.println("Panic!! Was expecting Sync Char 0x62 but received: "); Serial.println(c, HEX);
              Serial.print("We received: "); Serial.println(c, HEX);
              ubx_nmea_state = sync_lost;
            }
          }
          break;
          // RXM_RAWX is class 0x02 ID 0x15
          // RXM_SFRBF is class 0x02 ID 0x13
          // TIM_TM2 is class 0x0d ID 0x03
          // NAV_POSLLH is class 0x01 ID 0x02
          // NAV_PVT is class 0x01 ID 0x07
          // NAV-STATUS is class 0x01 ID 0x03
          // NAV-POSLLH is class 0x01 ID 0x14
          // NAV-SAT is class 0x01 ID 0x35
          case (looking_for_class): {
            ubx_class = c;
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_nmea_state = looking_for_ID; // Now look for ID byte
#ifdef DEBUG
            // Class syntax checking
            if ((ubx_class != 0x02) and (ubx_class != 0x0d) and (ubx_class != 0x01)) {
              Serial.println("Panic!! Was expecting Class of 0x02 or 0x0d or 0x01 but received: "); Serial.println(ubx_class, HEX);
              ubx_nmea_state = sync_lost;
            }
#endif
          }
          break;
          case (looking_for_ID): {
            ubx_ID = c;
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_nmea_state = looking_for_length_LSB; // Now look for length LSB
#ifdef DEBUG
            // ID syntax checking
            if ((ubx_class == 0x02) and ((ubx_ID != 0x15) and (ubx_ID != 0x13))) {
              Serial.println("Panic!! Was expecting ID of 0x15 or 0x13 but received"); Serial.println(ubx_class, HEX);
              //ubx_nmea_state = sync_lost;
            }
            else if ((ubx_class == 0x0d) and (ubx_ID != 0x03)) {
              Serial.println("Panic!! Was expecting ID of 0x03 but received"); Serial.println(ubx_class, HEX);
              //ubx_nmea_state = sync_lost;
            }
            else if ((ubx_class == 0x01) and ((ubx_ID != 0x02) and (ubx_ID != 0x07) and (ubx_ID != 0x03) and (ubx_ID != 0x14) and (ubx_ID != 0x26) and (ubx_ID != 0x35))) {
              Serial.println("Panic!! Was expecting ID of 0x02 or 0x07 or 0x03 or 0x14 or 0x26 or 0x35 but received"); Serial.println(ubx_class, HEX);
              //ubx_nmea_state = sync_lost;
            }
            Serial.print("ubx_class = "); Serial.print(ubx_class, HEX);
            Serial.print("\t ubx_id = "); Serial.println(ubx_ID, HEX);
#endif
          }
          break;
          case (looking_for_length_LSB): {
            ubx_length = c; // Store the length LSB
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_nmea_state = looking_for_length_MSB; // Now look for length MSB
          }
          break;
          case (looking_for_length_MSB): {
            ubx_length = ubx_length + (c * 256); // Add the length MSB
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            ubx_nmea_state = processing_payload; // Now look for payload bytes (length: ubx_length)
          }
          break;
          case (processing_payload): {
            // If this is a NAV_PVT message, check the flags byte (byte offset 21) and report the carrSoln
            if ((ubx_class == 0x01) and (ubx_ID == 0x07)) { // Is this a NAV_PVT message (class 0x01 ID 0x07)?
              if (ubx_length == 71) { // Is this byte offset 21? (ubx_length will be 92 for byte offset 0, so will be 71 for byte offset 21)
#ifdef DEBUG
                Serial.print("NAV_PVT carrSoln: ");
                if ((c & 0xc0) == 0x00) {
                  Serial.println("none");
                }
                else if ((c & 0xc0) == 0x40) {
                  Serial.println("floating");
                }
                else if ((c & 0xc0) == 0x80) {
                  Serial.println("fixed");
                }
#endif
                if ((c & 0xc0) == 0x80) { // Have we got a fixed carrier solution?
#ifndef NoLED
#ifdef NeoPixel
                  if (write_color == green) { // Check that write_color is green before changing it to yellow, to give magenta priority
                    write_color = yellow; // Change the SD write color to yellow to indicate fixed carrSoln
                  }
#else
#ifndef NoLogLED
                  digitalWrite(GreenLED, !digitalRead(GreenLED)); // Toggle the green LED
#endif
#endif
#endif         
                }
                else { // carrSoln is not fixed
#ifndef NoLED
#ifdef NeoPixel
                  if (write_color == yellow) {
                    write_color = green; // Reset the SD write color to green only if it was yellow previously
                  }
#else
#ifndef NoLogLED
                  digitalWrite(GreenLED, HIGH); // If the fix is not TIME, leave the green LED on
#endif
#endif
#endif
                }
              }
            }
            // If this is a NAV_STATUS message, check the gpsFix byte (byte offset 4) and flash the green LED (or make the NeoPixel magenta) if the fix is TIME
            if ((ubx_class == 0x01) and (ubx_ID == 0x03)) { // Is this a NAV_STATUS message (class 0x01 ID 0x03)?
              if (ubx_length == 12) { // Is this byte offset 4? (ubx_length will be 16 for byte offset 0, so will be 12 for byte offset 4)
#ifdef DEBUG
                Serial.print("NAV_STATUS gpsFix: ");
                if (c == 0x00) {
                  Serial.println("no fix");
                }
                else if (c == 0x01) {
                  Serial.println("dead reckoning");
                }
                else if (c == 0x02) {
                  Serial.println("2D-fix");
                }
                else if (c == 0x03) {
                  Serial.println("3D-fix");
                }
                else if (c == 0x04) {
                  Serial.println("GPS + dead reckoning");
                }
                else if (c == 0x05) {
                  Serial.println("time");
                }
                else {
                  Serial.println("reserved");
                }
#endif
                if (c == 0x05) { // Have we got a TIME fix?
#ifndef NoLED
#ifdef NeoPixel
                  write_color = magenta; // Change the SD write color to magenta to indicate time fix (trumps yellow!)
#else
#ifndef NoLogLED
                  digitalWrite(GreenLED, !digitalRead(GreenLED)); // Toggle the green LED
#endif
#endif
#endif            
                }
                else {
#ifndef NoLED
#ifdef NeoPixel
                  if (write_color == magenta) {
                    write_color = green; // Reset the SD write color to green only if it was magenta previously (not yellow)
                  }
#else
#ifndef NoLogLED
                  digitalWrite(GreenLED, HIGH); // If the fix is not TIME, leave the green LED on
#endif
#endif
#endif
                }
              }
            }
            ubx_length = ubx_length - 1; // Decrement length by one
            ubx_expected_checksum_A = ubx_expected_checksum_A + c; // Update the expected checksum
            ubx_expected_checksum_B = ubx_expected_checksum_B + ubx_expected_checksum_A;
            if (ubx_length == 0) {
              ubx_expected_checksum_A = ubx_expected_checksum_A & 0xff; // Limit checksums to 8-bits
              ubx_expected_checksum_B = ubx_expected_checksum_B & 0xff;
              ubx_nmea_state = looking_for_checksum_A; // If we have received length payload bytes, look for checksum bytes
              }
          }
          break;
          case (looking_for_checksum_A): {
            ubx_checksum_A = c;
            ubx_nmea_state = looking_for_checksum_B;
            }
          break;
          case (looking_for_checksum_B): {
            ubx_checksum_B = c;
            ubx_nmea_state = looking_for_B5_dollar; // All bytes received so go back to looking for a new Sync Char 1 unless there is a checksum error
            if ((ubx_expected_checksum_A != ubx_checksum_A) or (ubx_expected_checksum_B != ubx_checksum_B)) {
              Serial.println("Panic!! UBX checksum error!");
              ubx_nmea_state = sync_lost;
              }
            }
          break;
        }
      }
      else {
        // read battery voltage
        vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
        }
      
      
      if (digitalRead(swPin) == LOW) stop_pressed = true;
      // Check if the stop button has been pressed or battery is low
      if ((stop_pressed == true) or (vbat < LOWBAT)) {
        loop_step = close_file; // now close the file
        break;
        }
      // Check if stop_ delay button has been pressed and stop_delayed is not already active
      if ((digitalRead(DelayedPin) == LOW) and (stop_delayed_active != true)) {
        stop_delayed_pressed = true;
        stop_delayed_active = true;
        set_Alarm(DELAYED_STOP);
        }
      // Check if stop_delayed ended
      else if ((alarmFlag == true) and (stop_delayed_active == true)) {
        stop_pressed = true; // set stop_pressed to true, or the logger would try to restart
        loop_step = close_file; // now close the file
        break;
        }
      // Check if there has been an RTC alarm and it is time to open a new file
      else if ((alarmFlag == true) and (ubx_nmea_state == looking_for_B5_dollar)) {
        loop_step = new_file; // now close the file and open a new one
        break;
        }
      else if (ubx_nmea_state == sync_lost) {
        loop_step = restart_file; // Sync has been lost so stop RAWX messages and open a new file before restarting RAWX
        }
    }
    break;

    // Close the current log file and open a new one without stopping RAWX messages
    case new_file: {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
#endif
        bufferPointer = 0; // reset bufferPointer
      }
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      // An RTC alarm was detected, so set the RTC alarm time to the next INTERVAL and loop back to open_file.
      // We only receive an RTC alarm on a minute mark, so it doesn't matter that the RTC seconds will have moved on at this point.
      alarmFlag = false; // Clear the RTC alarm flag
      set_Alarm(INTERVAL); // Set next alarm time

#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(cyan); // Set the NeoPixel to cyan
#else
      setLED(black); // Turn NeoPixel off if NoLogLED
#endif
#endif
#endif
      loop_step = open_file; // loop round again and open a new file
      bytes_written = 0; // Clear bytes_written
    }
    break;

    // Disable RAWX messages, save any residual data and close the file, possibly for the last time
    case close_file: {
      i2cGPS.sendCommand(setRAWXoff); // Disable RAWX messages
      int waitcount = 0;
      while (waitcount < dwell) { // Wait for residual data
        while (SerialBuffer.available()) {
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
            digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
            digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
        Serial.print(bytes_written);
        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      // Either the battery is low or the user pressed the stop button:
      if (stop_pressed == true) {
        // Stop switch was pressed so just wait for a reset
#ifndef NoLED
#ifdef NeoPixel
        setLED(red); // Set the NeoPixel to red
#else
        digitalWrite(RedLED, HIGH); // Turn the red LED on
#endif
#endif
        Serial.println("Waiting for reset...");
        while(1); // Wait for reset
      }
      else {
        // Low battery was detected so wait for the battery to recover
        Serial.println("Battery must be low - waiting for it to recover...");
#ifndef NoLED
#ifdef NeoPixel
        setLED(red); // Set the NeoPixel to red
#else
        digitalWrite(RedLED, HIGH); // Turn the red LED on
#endif
#endif
        // Check the battery voltage. Make sure it has been OK for at least 5 seconds before continuing
        int high_for = 0;
        while (high_for < 500) {
          // read battery voltage
          vbat = analogRead(A7) * (2.0 * 3.3 / 1023.0);
          if (vbat < LOWBAT) {
            high_for = 0; // If battery voltage is low, reset the count
          }
          else {
            high_for++; // Increase the count
          }
          delay(10); // Wait 10msec
        }
        // Now loop round again and restart rawx messages before opening a new file
#ifndef NoLED
#ifdef NeoPixel
        setLED(cyan); // Set the NeoPixel to cyan
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off
#endif
#endif
        loop_step = start_rawx;
      }
    }
    break;

    // RAWX data lost sync so disable RAWX messages, save any residual data, close the file, open another and restart RAWX messages
    // Don't update the next RTC alarm - leave it as it is
    case restart_file: {
      i2cGPS.sendCommand(setRAWXoff); // Disable RAWX messages
      int waitcount = 0;
      while (waitcount < dwell) { // Wait for residual data
        while (SerialBuffer.available()) {
          serBuffer[bufferPointer] = SerialBuffer.read_char(); // Put extra bytes into serBuffer
          bufferPointer++;
          if (bufferPointer == SDpacket) { // Write a full packet
            bufferPointer = 0;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
            digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
            numBytes = rawx_dataFile.write(&serBuffer, SDpacket);
            //rawx_dataFile.sync(); // Sync the file system
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
            setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
            digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
            bytes_written += SDpacket;
#ifdef DEBUG
            if (numBytes != SDpacket) {
              Serial.print("SD write error! Write size was ");
              Serial.print(SDpacket);
              Serial.print(". ");
              Serial.print(numBytes);
              Serial.println(" were written.");
            }
#endif
          }
        }
        waitcount++;
        delay(1);
      }
      // If there is any data left in serBuffer, write it to file
      if (bufferPointer > 0) {
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
        digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
        numBytes = rawx_dataFile.write(&serBuffer, bufferPointer); // Write remaining data
        rawx_dataFile.sync(); // Sync the file system
        bytes_written += bufferPointer;
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
        setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
        digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
        if (numBytes != bufferPointer) {
          Serial.print("SD write error! Write size was ");
          Serial.print(bufferPointer);
          Serial.print(". ");
          Serial.print(numBytes);
          Serial.println(" were written.");
        }
        Serial.print("Final SD Write: ");
        Serial.print(bufferPointer);
        Serial.println(" Bytes");
        Serial.print(bytes_written);
        Serial.println(" Bytes written");
#endif
        bufferPointer = 0; // reset bufferPointer
      }
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(green); // Set the NeoPixel to green
#endif
#else
#ifndef NoLogLED
      digitalWrite(RedLED, HIGH); // Turn the red LED on to indicate SD card write
#endif
#endif
#endif
      // Get the RTC time and date
      uint8_t RTCseconds = rtc.getSeconds();
      uint8_t RTCminutes = rtc.getMinutes();
      uint8_t RTChours = rtc.getHours();
      uint8_t RTCday = rtc.getDay();
      uint8_t RTCmonth = rtc.getMonth();
      uint8_t RTCyear = rtc.getYear();
#ifdef DEBUG
      // Set log file write time
      if (!rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file write timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_WRITE, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif
#ifdef DEBUG
      // Set log file access time
      if (!rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds)) {
        Serial.println("Warning! Could not set file access timestamp!");
      }
#else
      rawx_dataFile.timestamp(T_ACCESS, (RTCyear+2000), RTCmonth, RTCday, RTChours, RTCminutes, RTCseconds);
#endif      
      rawx_dataFile.close(); // close the file
#ifndef NoLED
#ifdef NeoPixel
#ifndef NoLogLED
      setLED(dim_green); // Set the NeoPixel to dim green
#endif
#else
      digitalWrite(RedLED, LOW); // Turn the red LED off to indicate SD card write is complete
#endif
#endif
#ifdef DEBUG
      uint32_t filesize = rawx_dataFile.fileSize(); // Get the file size
      Serial.print("File size is ");
      Serial.println(filesize);
      Serial.print("File size should be ");
      Serial.println(bytes_written);
#endif
      Serial.println("File closed!");
      loop_step = start_rawx; // loop round again and restart rawx messages before opening a new file
    }
    break;  
  }
}

void set_Alarm (int alarm_delay) {
          uint8_t nextAlarmSecond = i2cGPS.getSecond(); // Next alarm second
          uint8_t nextAlarmMin = (i2cGPS.getMinute()+alarm_delay); // Calculate next alarm minutes
          uint8_t addHour = 0;
          uint8_t nextAlarmHour = 0;
          while (nextAlarmMin - 60 > 0) {  // Calculate hours to add
            addHour += 1;
            nextAlarmMin = nextAlarmMin - 60;
            }
          nextAlarmMin = nextAlarmMin % 60; // Correct hour rollover
          nextAlarmHour = (i2cGPS.getHour() + addHour) % 24; // Correct day rollover
          rtc.setAlarmHours(nextAlarmHour); // Set RTC Alarm Hours
          rtc.setAlarmMinutes(nextAlarmMin); // Set RTC Alarm Minutes
          rtc.setAlarmSeconds(nextAlarmSecond); // Set RTC Alarm Seconds
          if (splitLog and !(stop_delayed_active)) Serial.print("Next new file set to: ");
          if (stop_delayed_active) Serial.print("Logging will stop at: ");

          if (splitLog or stop_delayed_active) { // One condition is active to set alarm
            Serial.print(nextAlarmHour); Serial.print("H"); Serial.print(nextAlarmMin);
            Serial.print("mn"); Serial.print(nextAlarmSecond); Serial.println("s");
            rtc.enableAlarm(rtc.MATCH_HHMMSS); // Alarm Match on hours, minutes and seconds
            rtc.attachInterrupt(alarmMatch); // Attach alarm interrupt
            }
  }
