#include <CanSatKit.h>
#include <SPI.h>
#include <SD.h>


using namespace CanSatKit;

constexpr auto LED = 13;

Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            436.0,
            Bandwidth_125000_Hz,
            SpreadingFactor_11,
            CodingRate_4_8);



void setup() {
  pinMode(LED, OUTPUT);

  SerialUSB.begin(9600);
  while (!SerialUSB);

  SerialUSB.println("start!");
  
  radio.begin();
  radio.disable_debug();

  digitalWrite(LED, HIGH);

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
  static bool state = false;
  state = !state;
  digitalWrite(LED, state);

  frame.satellites = 4;
  frame.latitude = 50.447175;
  frame.longitude = 21.797497;
  frame.altitude_msl = 1758;
  frame.temperature = 405;
  frame.pressure = 82551;
  frame.humidity = 10;

  radio.transmit(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
  SerialUSB.println();

  delay(4000);
}
