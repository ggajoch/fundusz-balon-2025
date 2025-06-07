#include <CanSatKit.h>
#include <SPI.h>
#include <SD.h>

using namespace CanSatKit;


struct radio_frame {
  float latitude;
  float longitude;
  int16_t altitude_msl;
  int16_t temperature;
  float pressure;
  int8_t humidity;
  int8_t satellites;
} __attribute__((packed));


static_assert(sizeof(radio_frame) == 18, "align?");


constexpr int chipSelect = Pins::EM1::SD::ChipSelect;
File dataFile;


// set radio receiver parameters - see comments below
// remember to set the same radio parameters in
// transmitter and receiver boards!

Radio radio(Pins::EM1::Radio::ChipSelect,
            Pins::EM1::Radio::DIO0,
            435.0,                  // frequency in MHz
            Bandwidth_125000_Hz,    // bandwidth - check with CanSat regulations to set allowed value
            SpreadingFactor_11,      // see provided presentations to determine which setting is the best
            CodingRate_4_8);        // see provided presentations to determine which setting is the best

void setup() {
  SerialUSB.begin(115200);

  // start radio module  
  radio.begin();


  SerialUSB.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("Card failed, or not present");
    // don't do anything more:
  } else {
    SerialUSB.println("card initialized.");
    dataFile = SD.open("datalog.csv", FILE_WRITE);
  }

}

void loop() {
  uint8_t data[256];
  
  uint8_t length;
  radio.receive(data, length);
  
  if (length != sizeof(radio_frame)) {
    SerialUSB.println("Incorrect frame!");
    return;
  }
  radio_frame* f = reinterpret_cast<radio_frame*>(data);


  static uint32_t counter = 0;
  counter++;  


  String data_string = "";

  data_string += String(millis());
  data_string += ";";
  data_string += String(counter);
  data_string += ";";
  data_string += String(radio.get_rssi_last());
  data_string += ";";
  data_string += String(f->satellites);
  data_string += ";";
  data_string += String(f->longitude, 6);
  data_string += ";";
  data_string += String(f->latitude, 6);
  data_string += ";";
  data_string += String(f->altitude_msl);
  data_string += ";";
  data_string += String(f->temperature);
  data_string += ";";
  data_string += String(f->pressure);
  data_string += ";";
  data_string += String(f->humidity);

  SerialUSB.println(data_string);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(data_string);
    dataFile.flush();
  }
  // if the file isn't open, pop up an error:
  else {
    SerialUSB.println("error opening datalog.txt");
  }

}
