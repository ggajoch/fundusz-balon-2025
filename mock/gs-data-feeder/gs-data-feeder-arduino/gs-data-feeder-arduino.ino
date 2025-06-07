#define NODEBUG

#ifdef DEBUG
#define PRINTLND SerialUSB.println
#else
#define PRINTLND(text) ((void)0)
#endif

#include <SPI.h>
#include <SD.h>

File myFile;

uint16_t rx_packet_number = 0;

void setup() {
  // Open serial communications and wait for port to open:
  SerialUSB.begin(9600);
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  PRINTLND("Initializing SD card...");

  if (!SD.begin(11)) {
    PRINTLND("initialization failed!");
    while (1);
  }
  PRINTLND("initialization done.");
}

void loop() {
  // open the file for reading:
  myFile = SD.open("test_data.csv");

  if (myFile) {
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      
      // print timestamp
      Serial.print(millis());
      Serial.print(";");

      // print rx packet number
      Serial.print(rx_packet_number++);
      Serial.print(";");
      
      Serial.println(myFile.readStringUntil('\n'));
      
      delay(3000);
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    PRINTLND("error opening test.txt");
  }
}
