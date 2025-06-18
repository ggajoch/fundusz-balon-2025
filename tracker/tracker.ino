// #define DEBUG

/* docs
    https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf
    https://origingps.com/wp-content/uploads/2018/08/Spider-and-Hornet-Low-Power-Operating-Mode-Application-Note-SiRFStarIV.pdf
    https://y1cj3stn5fbwhv73k0ipk1eg-wpengine.netdna-ssl.com/wp-content/uploads/2017/09/Telit_Using_SiRF_STAR_IV_with_an_external_Host_Application_Note_r0.pdf
    http://www.elgps.com/public_ftp/Documentos/SIRF_Protocol.pdf
*/

#include <CanSatKit.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <ArduinoLowPower.h>
#include <TinyGPSPlus.h>

using namespace CanSatKit;

BME280I2C bme;

constexpr auto LED = 13;

// constexpr auto gps_reset = A1; // VBACKUP on this pin
constexpr auto gps_force_on = A2;
constexpr auto gps_wakeup = A4;
constexpr auto gps_pps = 8;

Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            433.8,
            Bandwidth_125000_Hz,
            SpreadingFactor_11,
            CodingRate_4_8);

TinyGPSPlus gps;

// uint8_t calculate_checksum(const char * cmd) {
//   auto len = strlen(cmd);
//   len--; // without last byte (*)
//   uint8_t crc = 0;
//   for (uint8_t i = 1; i < len; ++i) {
//     crc ^= cmd[i];
//   }
//   return crc;
// }

// example: $PSRF100,0,115200,8,1,0*
// void send_nmea_command(const char * cmd) {
//   Serial.print(cmd);
//   SerialUSB.print(cmd);

//   uint8_t checksum = calculate_checksum(cmd);
//   char tmp[5];
//   sprintf(tmp, "%.2X\r\n", checksum);

//   Serial.print(tmp);
//   SerialUSB.print(tmp);
// }

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(gps_force_on, OUTPUT);
  pinMode(gps_wakeup, INPUT);
  pinMode(gps_pps, INPUT);

  Serial.begin(9600);
  Wire.begin();

#ifdef DEBUG
  SerialUSB.begin(9600);
  while (!SerialUSB);
  SerialUSB.println("start!");
#endif

  bool bme_ok = bme.begin();

#ifdef DEBUG
  if (bme_ok) {
    SerialUSB.println("BME280 OK!");
  } else {
    SerialUSB.println("Could not find a valid BME280 sensor, check wiring!");
  }
#endif
  
  digitalWrite(gps_force_on, HIGH);
#ifdef DEBUG
  delay(100);
  SerialUSB.print("GPS wakeup status: ");
  SerialUSB.println(digitalRead(gps_wakeup));
#endif
  delay(1000);

  radio.begin();
  radio.low_power(true);
  radio.disable_debug();

  digitalWrite(LED, LOW);

  // send_nmea_command("$PMTK225,9*"); // AlwaysLocate mode, not used
}

static uint32_t last_tx_time = 0;

bool timeout() {
  return millis() > last_tx_time + 5000u;
}

struct radio_frame {
  float latitude;
  float longitude;
  int16_t altitude_msl;
  int16_t temperature;
  float pressure;
  int8_t humidity;
  int8_t satellites;
} __attribute__((packed));

radio_frame frame;

static_assert(sizeof(radio_frame) == 18, "align?");

void loop()
{
#ifndef DEBUG
  LowPower.idle();
#endif

  if (timeout()) {
    last_tx_time = millis();

    float temperature, humidity, pressure;
    BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
    BME280::PresUnit presUnit(BME280::PresUnit_Pa);
    bme.read(pressure, temperature, humidity, tempUnit, presUnit);

    static bool state = false;
    state = !state;
#ifdef DEBUG
    digitalWrite(LED, state);
#endif

    frame.latitude = gps.location.isValid() ? gps.location.lat() : 0;
    frame.longitude = gps.location.isValid() ? gps.location.lng() : 0;
    frame.altitude_msl = gps.altitude.isValid() ? static_cast<int16_t>(gps.altitude.meters()) : 0;
    frame.temperature = static_cast<int16_t>(10*temperature);
    frame.pressure = pressure;
    frame.humidity = static_cast<int8_t>(humidity);
    frame.satellites = gps.satellites.isValid() ? gps.satellites.value() : -1;

#ifdef DEBUG
    printDateTime(gps.date, gps.time);

    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    printFloat(temperature, true, 5, 1);
    printFloat(pressure, true, 10, 1);
    printFloat(humidity, true, 5, 1);
    printInt(gps.satellites.value(), gps.satellites.isValid(), 5);

    SerialUSB.print(" | ");
    printInt(gps.charsProcessed(), true, 6);
    printInt(gps.sentencesWithFix(), true, 10);
    printInt(gps.failedChecksum(), true, 9);

    SerialUSB.println();
#endif

    radio.transmit(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
  }
  clear_nmea_queue();
}

static void clear_nmea_queue()
{
  while (Serial.available()) {
    char d = Serial.read();
    // SerialUSB.print(d);
    gps.encode(d);
  }
}

#ifdef DEBUG
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      SerialUSB.print('*');
    SerialUSB.print(' ');
  }
  else
  {
    SerialUSB.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      SerialUSB.print(' ');
  }
  clear_nmea_queue();
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  SerialUSB.print(sz);
  clear_nmea_queue();
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    SerialUSB.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    SerialUSB.print(sz);
  }

  if (!t.isValid())
  { 
    SerialUSB.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    SerialUSB.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  clear_nmea_queue();
}
#endif
