#include <SPI.h>
#include <RF24.h>
#include <SD.h>
#include <CAN.h>  

/* read location information from Jetson Nano and sends this to another node via nRF module*/

RF24 radio(7, 10); 
const byte address[6] = "PLT01"; 
const int SD_CS = 4;

// TDMA Settings for 3-node platoon
const uint8_t nodeID = 1;           
const unsigned long slotTime = 10;  
const unsigned long cycleTime = 30; 

const int numTrials = 100;

struct PlatoonPacket {
  uint8_t senderID;
  float position;
  float velocity;
  unsigned long timestamp;
};

File logFile;
static int trial = 1;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Init SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD init failed");
    while (1);
  }
  logFile = SD.open("tdma_log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("Log file creation failed");
    while (1);
  }
  logFile.println("Trial,FromID,PayloadSize,Latency,Success");
  logFile.flush();

  // Init nRF24
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(108);
  radio.openReadingPipe(1, address);
  radio.openWritingPipe(address);
  radio.startListening();

  if (!CAN.begin(500E3)) {
    Serial.println("CAN init failed");
    while (1);
  }

  delay(slotTime * nodeID); 
}

void loop() {
  unsigned long now = millis();
  unsigned long inCycle = now % cycleTime;
  unsigned long slotStart = (nodeID - 1) * slotTime;
  unsigned long slotEnd = nodeID * slotTime;

  //Listen for incoming nRF packets
  while (radio.available()) {
    PlatoonPacket rxPacket;
    unsigned long rxStart = micros();
    radio.read(&rxPacket, sizeof(rxPacket));
    unsigned long latency = rxStart - rxPacket.timestamp;

    logFile.print(trial);
    logFile.print(",");
    logFile.print(rxPacket.senderID);
    logFile.print(",");
    logFile.print(sizeof(rxPacket));
    logFile.print(",");
    logFile.print(latency);
    logFile.println(",1"); 
    logFile.flush();
  }

  //Transmit during assigned slot
  if (inCycle >= slotStart && inCycle < slotEnd && trial <= numTrials) {
    float pos = 0.0, vel = 0.0;
    bool canDataAvailable = false;

    if (CAN.parsePacket() && CAN.packetId() == 0x100 && CAN.available() >= 8) {
      byte buffer[8];
      for (int i = 0; i < 8; i++) {
        buffer[i] = CAN.read();
      }
      memcpy(&pos, &buffer[0], 4);
      memcpy(&vel, &buffer[4], 4);
      canDataAvailable = true;
    }

    if (canDataAvailable) {
      radio.stopListening();

      PlatoonPacket packet;
      packet.senderID = nodeID;
      packet.position = pos;
      packet.velocity = vel;
      packet.timestamp = micros();

      bool success = radio.write(&packet, sizeof(packet));
      delay(1); 
      radio.startListening();

      if (!success) {
        logFile.print(trial);
        logFile.print(",");
        logFile.print("NA");
        logFile.print(",");
        logFile.print(sizeof(packet));
        logFile.print(",");
        logFile.print("NA");
        logFile.println(",0"); 
        logFile.flush();
      }

      trial++;
    }

    delay(slotTime - 2); 
  }
}
