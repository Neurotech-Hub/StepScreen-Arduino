#include "Adafruit_DAP.h"
#include <SPI.h>
#include "SdFat_Adafruit_Fork.h"

#define SD_CS 4
#define SWDIO 12
#define SWCLK 11
#define SWRST 9

#define FILENAME "fw.bin"

const int BUFSIZE = Adafruit_DAP_SAM::PAGESIZE;
uint8_t buf[BUFSIZE];

//create a DAP for programming Atmel SAM devices
Adafruit_DAP_SAM dap;

SdFat SD;

// Function called when there's an SWD error
void error(const char *text) {
  Serial.println(text);
  while (1);
}


void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  while(!Serial) {
    delay(1);         // will pause the chip until it opens serial console
  }

  dap.begin(SWCLK, SWDIO, SWRST, &error);

  // Default first (SdFat asks for 50 MHz). Some boards, including the
  // Feather M0 Adalogger, turn that into a clock the card will not
  // answer, so fall back to 12 MHz and then 4 MHz.
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("Default speed failed, retrying at 12 MHz");
    SD.end();
    if (!SD.begin(SD_CS, SD_SCK_MHZ(12))) {
      Serial.println("12 MHz init failed, retrying at 4 MHz");
      SD.end();
      if (!SD.begin(SD_CS, SD_SCK_MHZ(4))) {
        error("Card failed, or not present");
      }
    }
  }
  Serial.println("Card initialized");

  File32 dataFile = SD.open(FILENAME);

  if(!dataFile){
     error("Couldn't open file");
  }
  
  Serial.println("Connecting...");
  if ( !dap.targetConnect() ) {
    error(dap.error_message);
  }

  char debuggername[100];
  dap.dap_get_debugger_info(debuggername);
  Serial.print(debuggername); Serial.print("\n\r");

  uint32_t dsu_did;
  if (! dap.select(&dsu_did)) {
    Serial.print("Unknown device found 0x"); Serial.print(dsu_did, HEX);
    error("Unknown device found");
  }
  Serial.print("Found Target: ");
  Serial.print(dap.target_device.name);
  Serial.print("\tFlash size: ");
  Serial.print(dap.target_device.flash_size);
  Serial.print("\tFlash pages: ");
  Serial.println(dap.target_device.n_pages);
  //Serial.print("Page size: "); Serial.println(dap.target_device.flash_size / dap.target_device.n_pages);

  /* Example of how to read and set fuses
  Serial.print("Fuses... ");
  dap.fuseRead(); //MUST READ FUSES BEFORE SETTING OR WRITING ANY
  dap._USER_ROW.bit.WDT_Period = 0x0A;
  dap.fuseWrite();
  Serial.println(" done.");
  */

  Serial.println("Chip erase destroys all flash, including the bootloader.");
  Serial.println("The target enumerates over USB only if fw.bin contains a USB image.");
  Serial.print("Erasing... ");
  dap.erase();
  Serial.println(" done.");
  
  Serial.print("Programming... ");
  unsigned long t = millis();
  uint32_t addr = dap.program_start();

  while (dataFile.available()) {
      memset(buf, 0xFF, BUFSIZE);  // pad unused bytes in the last page
      dataFile.read(buf, BUFSIZE);
      dap.programBlock(addr, buf);
      addr += BUFSIZE;
  }
  Serial.println(millis() - t);

  // program_start() switches to the fastest SWD clock. Slow down so the
  // read-back is not a second source of mismatches.
  dap.dap_set_clock(50);
  if (!dataFile.seek(0)) {
    error("Couldn't rewind file");
  }

  uint8_t check[BUFSIZE];
  addr = 0;
  Serial.print("Verifying... ");
  while (dataFile.available()) {
    memset(buf, 0xFF, BUFSIZE);
    dataFile.read(buf, BUFSIZE);
    dap.readBlock(addr, check);
    if (memcmp(buf, check, BUFSIZE) != 0) {
      Serial.print("mismatch at 0x");
      Serial.println(addr, HEX);
      error("Verify failed");
    }
    addr += BUFSIZE;
  }
  dataFile.close();
  Serial.println("OK");

  // program_start() sets BOOTPROT to 7 (nothing protected) and leaves it
  // there. 0x02 protects the first 8 KB, the SAMD21 bootloader region.
  Serial.print("Protecting boot region... ");
  if (!dap.protectBoot()) {
    error(dap.error_message);
  }
  Serial.println("8 KB");

  dap.deselect();
  dap.dap_disconnect();

  // This board's USB port is the link to the PC, so it cannot see the
  // target enumerate. Pulse reset and let the target's own cable do that.
  pinMode(SWRST, OUTPUT);
  digitalWrite(SWRST, LOW);
  delay(10);
  digitalWrite(SWRST, HIGH);

  Serial.println("Target released.");
  Serial.println("Check the PC for the target's USB device.");
  Serial.println("\nDone!");
}

void loop() {
  //blink led on the host to show we're done
  digitalWrite(13, HIGH);
  delay(500); 
  digitalWrite(13, LOW);
  delay(500);  
}
