// NORVI X-CPU-ESPS3-X1 -V2-N16R2  -  Modbus TCP SLAVE test (W5500 Ethernet)
// Supports: FC03 Read Holding, FC04 Read Input (same data),
//           FC06 Write Single Register, FC16 Write Multiple Registers

#include <SPI.h>
#include <Ethernet.h>

#define ETH_CS 1
#define MISO   13
#define MOSI   11
#define SCLK   12

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress localIP(192, 168, 1, 50);        // used only if DHCP fails

const uint8_t  unitId  = 1;
const uint16_t NUM_REGS = 100;
uint16_t regs[NUM_REGS];                   // Modbus register table

class ModbusServer : public EthernetServer {
public:
  ModbusServer(uint16_t port) : EthernetServer(port) {}
  void begin(uint16_t port = 0) { EthernetServer::begin(); }
};

ModbusServer server(502);

void sendException(EthernetClient &c, uint8_t *h, uint8_t fc, uint8_t code) {
  uint8_t r[9];
  r[0] = h[0]; r[1] = h[1];                // Transaction ID
  r[2] = 0;    r[3] = 0;                   // Protocol ID
  r[4] = 0;    r[5] = 3;                   // Length
  r[6] = h[6];                             // Unit ID
  r[7] = fc | 0x80;
  r[8] = code;
  c.write(r, 9);
}

void handleRequest(EthernetClient &c) {
  uint8_t h[7], pdu[260], r[260];

  if (c.available() < 7) return;
  for (int i = 0; i < 7; i++) h[i] = c.read();

  uint16_t len = (h[4] << 8) | h[5];       // bytes after this field (unit + PDU)
  int pduLen = len - 1;
  if (pduLen < 1 || pduLen > 253) {        // bad frame: throw data away
    while (c.available()) c.read();
    return;
  }

  unsigned long t = millis();              // wait for the rest of the frame
  while (c.available() < pduLen && millis() - t < 200) delay(1);
  if (c.available() < pduLen) {
    while (c.available()) c.read();
    return;
  }
  for (int i = 0; i < pduLen; i++) pdu[i] = c.read();

  if (h[6] != unitId && h[6] != 0xFF) return;   // not for us

  uint8_t  fc   = pdu[0];
  uint16_t addr = (pdu[1] << 8) | pdu[2];
  uint16_t qty  = (pdu[3] << 8) | pdu[4];

  // Common response header
  r[0] = h[0]; r[1] = h[1]; r[2] = 0; r[3] = 0; r[6] = h[6]; r[7] = fc;

  if (fc == 0x03 || fc == 0x04) {                       // ---- Read registers
    if (qty < 1 || qty > 125 || addr + qty > NUM_REGS) {
      sendException(c, h, fc, 2); return;
    }
    r[8] = qty * 2;
    for (int i = 0; i < qty; i++) {
      r[9 + i * 2]  = regs[addr + i] >> 8;
      r[10 + i * 2] = regs[addr + i] & 0xFF;
    }
    uint16_t l = 3 + qty * 2;
    r[4] = l >> 8; r[5] = l & 0xFF;
    c.write(r, 6 + l);

  } else if (fc == 0x06) {                              // ---- Write single
    if (addr >= NUM_REGS) { sendException(c, h, fc, 2); return; }
    regs[addr] = qty;                                   // 'qty' field holds the value
    r[4] = 0; r[5] = 6;
    memcpy(&r[8], &pdu[1], 4);                          // echo address + value
    c.write(r, 12);

  } else if (fc == 0x10) {                              // ---- Write multiple
    if (qty < 1 || qty > 123 || addr + qty > NUM_REGS) {
      sendException(c, h, fc, 2); return;
    }
    for (int i = 0; i < qty; i++)
      regs[addr + i] = (pdu[6 + i * 2] << 8) | pdu[7 + i * 2];
    r[4] = 0; r[5] = 6;
    memcpy(&r[8], &pdu[1], 4);                          // echo address + quantity
    c.write(r, 12);

  } else {
    sendException(c, h, fc, 1);                         // illegal function
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("NORVI X-CPU Modbus TCP Slave test");

  pinMode(38, OUTPUT);                     // PORT_RST - same as your board sketch
  digitalWrite(38, HIGH);
  delay(100);

  SPI.begin(SCLK, MISO, MOSI);
  Ethernet.init(ETH_CS);

  if (Ethernet.begin(mac) == 0) {          // DHCP first
    Serial.println("DHCP failed, using static IP");
    Ethernet.begin(mac, localIP);
  }
  delay(1000);

  if (Ethernet.hardwareStatus() == EthernetNoHardware)
    Serial.println("W5500 not found");
  if (Ethernet.linkStatus() == LinkOFF)
    Serial.println("Ethernet cable not connected");

  server.begin();
  Serial.print("Modbus TCP slave listening at ");
  Serial.print(Ethernet.localIP());
  Serial.println(":502  (Unit ID 1)");

  regs[1] = 1000;                          // some starting values
  regs[2] = 2000;
}

void loop() {
  Ethernet.maintain();                     // keep DHCP lease alive

  // Update some registers so a master can see live data
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 1000) {
    lastUpdate = millis();
    regs[0] = millis() / 1000;             // reg 0 = uptime in seconds
  }

  // Serve Modbus requests
  EthernetClient c = server.available();
  if (c) handleRequest(c);

  // Report writes from the master (register 10)
  static uint16_t lastReg10 = 0;
  if (regs[10] != lastReg10) {
    lastReg10 = regs[10];
    Serial.print("Master wrote reg 10 = ");
    Serial.println(lastReg10);
  }
}
