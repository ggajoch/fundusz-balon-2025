#include <CanSatKit.h>

using namespace CanSatKit;

// set radio receiver parameters - see comments below
// remember to set the same radio parameters in
// transmitter and receiver boards!
Radio radio(Pins::EM1::Radio::ChipSelect,
            Pins::EM1::Radio::DIO0,
            434.0,
            Bandwidth_500000_Hz,
            SpreadingFactor_9,
            CodingRate_4_8);
void setup() {
  SerialUSB.begin(115200);

  // start radio module  
  radio.begin();
}

void loop() {
  // prepare empty space for received frame
  // maximum length is maximum frame length + null termination
  // 255 + 1 byte = 256 bytes
  char data[256];

  // receive data and save it to string
  radio.receive(data);
  
  // get and print signal level (rssi)
  //SerialUSB.print("Received (RSSI = ");
  //SerialUSB.print(radio.get_rssi_last());
  //SerialUSB.print("): ");

  // print received message
  SerialUSB.print(data);
}
