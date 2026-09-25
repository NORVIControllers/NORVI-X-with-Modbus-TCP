// NORVI X-CPU-ESPS3-X1 -V2-N16R2  -  Modbus TCP MASTER test (W5500 Ethernet)
// Auto-finds a Modbus TCP slave on the local subnet, reads 4 holding
// registers (FC03), then writes one register (FC06).

#include <SPI.h>
#include <Ethernet.h>

#define ETH_CS 1
#define MISO   13
#define MOSI   11
#define SCLK   12

// ---- Network settings ----
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress localIP(192, 168, 1, 50);        // used only if DHCP fails
IPAddress slaveIP(0, 0, 0, 0);             // found automatically by scan
const uint16_t slavePort = 502;
const uint8_t  unitId    = 1;

EthernetClient client;
uint16_t transId = 0;

// Scan the local /24 subnet for a host listening on Modbus TCP port 502
bool findSlave() {
  IPAddress me = Ethernet.localIP();
  Serial.println("Scanning subnet for Modbus TCP slave...");
  client.setConnectionTimeout(200);            // ms per host

  for (int i = 1; i < 255; i++) {
    if (i == me[3]) continue;                  // skip ourselves
    IPAddress ip(me[0], me[1], me[2], i);
    if (client.connect(ip, slavePort)) {
      slaveIP = ip;
      Serial.print("Slave found at: ");
      Serial.println(slaveIP);
      return true;                             // connection stays open
    }
    client.stop();
  }
  Serial.println("No slave found");
  return false;
}

// Send a request and read the response. Returns response length, 0 on error.
int modbusTransaction(uint8_t *req, int reqLen, uint8_t *resp, int respMax) {
  if (!client.connected()) {
    client.stop();
    if (!client.connect(slaveIP, slavePort)) {
      Serial.println("Connect to slave failed, re-scanning");
      findSlave();
      return 0;
    }
  }
  while (client.available()) client.read();   // flush old data

  client.write(req, reqLen);

  unsigned long t = millis();
  int n = 0, expected = 6;                     // MBAP header first, then PDU
  while (millis() - t < 2000) {
    while (client.available() && n < respMax) resp[n++] = client.read();
    if (n >= 6) expected = 6 + ((resp[4] << 8) | resp[5]);  // full frame length
    if (n >= expected) return n;
    delay(1);
  }
  Serial.println("Timeout");
  return 0;
}

// FC03 - Read Holding Registers
bool readHoldingRegs(uint16_t addr, uint16_t qty, uint16_t *out) {
  uint8_t req[12], resp[64];
  transId++;
  req[0] = transId >> 8;  req[1] = transId & 0xFF;   // Transaction ID
  req[2] = 0;             req[3] = 0;                // Protocol ID
  req[4] = 0;             req[5] = 6;                // Length
  req[6] = unitId;                                   // Unit ID
  req[7] = 0x03;                                     // Function code
  req[8] = addr >> 8;     req[9]  = addr & 0xFF;
  req[10] = qty >> 8;     req[11] = qty & 0xFF;

  int n = modbusTransaction(req, 12, resp, sizeof(resp));
  if (n < 9) return false;
  if (resp[7] & 0x80) {
    Serial.print("Modbus exception: "); Serial.println(resp[8]);
    return false;
  }
  for (int i = 0; i < qty; i++)
    out[i] = (resp[9 + i * 2] << 8) | resp[10 + i * 2];
  return true;
}

// FC06 - Write Single Register
bool writeSingleReg(uint16_t addr, uint16_t value) {
  uint8_t req[12], resp[64];
  transId++;
  req[0] = transId >> 8;  req[1] = transId & 0xFF;
  req[2] = 0;             req[3] = 0;
  req[4] = 0;             req[5] = 6;
  req[6] = unitId;
  req[7] = 0x06;
  req[8] = addr >> 8;     req[9]  = addr & 0xFF;
  req[10] = value >> 8;   req[11] = value & 0xFF;

  int n = modbusTransaction(req, 12, resp, sizeof(resp));
  if (n < 9) return false;
  if (resp[7] & 0x80) {
    Serial.print("Modbus exception: "); Serial.println(resp[8]);
    return false;
  }
  return n >= 12;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("NORVI X-CPU Modbus TCP Master test");

  pinMode(38, OUTPUT);            // PORT_RST - same as your board sketch
  digitalWrite(38, HIGH);
  delay(100);

  SPI.begin(SCLK, MISO, MOSI);
  Ethernet.init(ETH_CS);

  if (Ethernet.begin(mac) == 0) {              // try DHCP first
    Serial.println("DHCP failed, using static IP");
    Ethernet.begin(mac, localIP);
  }
  delay(1000);

  if (Ethernet.hardwareStatus() == EthernetNoHardware)
    Serial.println("W5500 not found");
  if (Ethernet.linkStatus() == LinkOFF)
    Serial.println("Ethernet cable not connected");

  Serial.print("My IP: ");
  Serial.println(Ethernet.localIP());

  while (!findSlave()) delay(3000);            // keep scanning until found
}

void loop() {
  static uint16_t counter = 0;
  uint16_t regs[4];

  // Read holding registers 0..3
  if (readHoldingRegs(0, 4, regs)) {
    Serial.print("Holding regs 0-3: ");
    for (int i = 0; i < 4; i++) { Serial.print(regs[i]); Serial.print("  "); }
    Serial.println();
  } else {
    Serial.println("Read failed");
  }

  // Write an incrementing value to register 10
  if (writeSingleReg(10, counter++)) {
    Serial.print("Wrote reg 10 = "); Serial.println(counter - 1);
  } else {
    Serial.println("Write failed");
  }

  delay(2000);
}
