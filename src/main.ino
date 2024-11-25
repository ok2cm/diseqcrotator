//-----------------------------------------------------------------------------
// DiSEqC format:
// http://ucideas.org/projects/hard/diseqc/7.pdf
//
// command format:
// https://github.com/jkjuopperi/hamlib/blob/master/easycomm/easycomm.txt
//-----------------------------------------------------------------------------

#include <Arduino.h>

//-----------------------------------------------------------------------------

//#define USE_TONE
//#define DEBUG

const int azBusPin = 8;
const int elBusPin = 9;

//-----------------------------------------------------------------------------

// DiseqcBus class
class DiseqcBus {
  public:
    DiseqcBus(int pwmPin);
    void begin();
    void sendByte(uint8_t data);
  private:
    int _pin;
    void sendBit(bool bit);
};

// DiseqcRotator class
class DiseqcRotator {
  public:
    DiseqcRotator(DiseqcBus *bus, uint8_t address = 0x30);
    void begin();
    void command(uint8_t cmd);
    void halt();
    void driveEast(int8_t param = 0);
    void driveWest(int8_t param = 0);
    void gotoXX(uint16_t position);
  private:
    const uint8_t FRAMING = 0xE0;
    DiseqcBus *_bus;
    uint8_t _address;
};

// DiSEqC bus (assuming two identical rotators -> bus cannot be shared)
DiseqcBus azBus(azBusPin);
DiseqcBus elBus(elBusPin);
// rotator
DiseqcRotator azRot(&azBus);
DiseqcRotator elRot(&elBus);

//-----------------------------------------------------------------------------
// DiseqcBus
//-----------------------------------------------------------------------------

// Constructor
DiseqcBus::DiseqcBus(int pin) {
  _pin = pin;
}

// Initialization
void DiseqcBus::begin() {
  pinMode(_pin, OUTPUT);  
}

// Send single bit
void DiseqcBus::sendBit(bool bit) {
#ifdef USE_TONE
  // tone()/noTone() seems to be inaccurate and causes jitter
  const unsigned int FREQ = 22000;
  if (bit) {
    tone(_pin, FREQ);
    delayMicroseconds(500);
    noTone(_pin);
    delay(1);
  } else {
    tone(_pin, FREQ);
    delay(1);
    noTone(_pin);
    delayMicroseconds(500);
  }
#else
  // bitbang tuned for arduinoNano 16MHz main oscillator
  const unsigned int HALF_PERIOD = 20;
  if (bit) {
    for (int8_t i = 0; i < 11; i++) {
      digitalWrite(_pin, 1);
      delayMicroseconds(HALF_PERIOD);
      digitalWrite(_pin, 0);
      delayMicroseconds(HALF_PERIOD);
    }
    delay(1);
  } else {
    for (int8_t i = 0; i < 23; i++) {
      digitalWrite(_pin, 1);
      delayMicroseconds(HALF_PERIOD);
      digitalWrite(_pin, 0);
      delayMicroseconds(HALF_PERIOD);
    }
    delayMicroseconds(480);
  }
#endif
}

// Send single byte + parity bit
void DiseqcBus::sendByte(uint8_t data) {
  bool parity = true;
  for (uint8_t i = 0; i < 8; i++) {
    bool bit = (data & 0x80) != 0;
    parity ^= bit;
    data <<= 1;
    sendBit(bit);
  }
  sendBit(parity);
}

//-----------------------------------------------------------------------------
// DiseqcRotator
//-----------------------------------------------------------------------------

// Constructor
DiseqcRotator::DiseqcRotator(DiseqcBus *bus, uint8_t address) {
  _bus = bus;
  _address = address;
};

// Initialization
void DiseqcRotator::begin() {
}

// Send command
void DiseqcRotator::command(uint8_t cmd) {
  _bus->sendByte(FRAMING);
  _bus->sendByte(_address);
  _bus->sendByte(cmd);
}

// Stop positioner movement
void DiseqcRotator::halt() {
  command(0x60);
}

// Drive Motor East (with optional timeout/steps)
void DiseqcRotator::driveEast(int8_t param) {
  command(68);
  _bus->sendByte(param);
}

// Drive Motor West (with optional timeout/steps)
void DiseqcRotator::driveWest(int8_t param) {
  command(69);
  _bus->sendByte(param);
}

// Drive Motor to Angular Position
void DiseqcRotator::gotoXX(uint16_t position) {
  command(0x6E);
  _bus->sendByte(position >> 8);
  _bus->sendByte(position & 0xFF);
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// Initialization
void setup() {
  azBus.begin();
  elBus.begin();
  azRot.begin();
  elRot.begin();
  Serial.begin(9600);
}

// Convert string xxX.X to unsigned fixed point 13.4
boolean param2fxp(char *str, size_t len, uint16_t *fxp) {
  // integer part
  uint16_t num;
  switch (len) {
    // X.X
    case 3:
      // check format X.X
      if ((str[0] < '0') || (str[0] > '9') ||
          (str[1] != '.') ||
          (str[2] < '0') || (str[2] > '9')) return false;
      // convert X.X
      num = str[0] - '0';
      break;
    // XX.X
    case 4:
      // check format XX.X
      if ((str[0] < '0') || (str[0] > '9') ||
          (str[1] < '0') || (str[1] > '9') ||
          (str[2] != '.') ||
          (str[3] < '0') || (str[3] > '9')) return false;
      // convert XX.X
      num = 10*(str[0] - '0');
      num += str[1] - '0';
      break;
    // XXX.X
    case 5:
      // check format XXX.X
      if ((str[0] < '0') || (str[0] > '9') ||
          (str[1] < '0') || (str[1] > '9') ||
          (str[2] < '0') || (str[2] > '9') ||
          (str[3] != '.') ||
          (str[4] < '0') || (str[4] > '9')) return false;
      // convert XXX.X
      num = 100*(str[0] - '0');
      num += 10*(str[1] - '0');
      num += str[2] - '0';
      break;
    // invalid length
    default:
      return false;
  }

  // too large?
  if (num >= 512) return false;

  // integer + fractional part (last digit)
  const uint8_t frac[] = {
    0x0, // 0.0000°
    0x2, // 0.1250°
    0x3, // 0.1875°
    0x5, // 0.3125°
    0x6, // 0.3750°
    0x8, // 0.5000°
    0xA, // 0.6250°
    0xB, // 0.6875°
    0xD, // 0.8125°
    0xE  // 0.8750°
  };
  *fxp = (num << 4) | frac[str[len - 1] - '0'];
  return true;
}

// Command handler
void commandHandler(char *buf, size_t len) {
  // message too short?
  if (len < 2) return;
  // first letter
  switch (buf[0]) {
    // A
    case 'a':
    case 'A':
      switch (buf[1]) {
        // AZ
        case 'z':
        case 'Z': {
          uint16_t pos;
          if (!param2fxp(buf + 2, len - 2, &pos)) return;
          azRot.gotoXX(pos);
#ifdef DEBUG
          Serial.print("AZ ");
          Serial.print(pos/16.0, 5);
          Serial.println("");
#endif
          } return;
      }
      break;
    // E
    case 'e':
    case 'E':
      switch (buf[1]) {
        // EL
        case 'l':
        case 'L':
          uint16_t pos;
          if (!param2fxp(buf + 2, len - 2, &pos)) return;
          azRot.gotoXX(pos);
#ifdef DEBUG
          Serial.print("EL ");
          Serial.print(pos/16.0, 5);
          Serial.println("");
#endif
          return;
      }
      break;
    // M
    case 'm':
    case 'M':
      switch (buf[1]) {
        // ML
        case 'l':
        case 'L':
          azRot.driveWest();
#ifdef DEBUG
          Serial.println("AZ west");
#endif
          return;
        // MR
        case 'r':
        case 'R':
          azRot.driveEast();
#ifdef DEBUG
          Serial.println("AZ east");
#endif
          return;
        // MU
        case 'u':
        case 'U':
          elRot.driveWest();
#ifdef DEBUG
          Serial.println("EL west");
#endif
          return;
        // MD
        case 'd':
        case 'D':
          elRot.driveEast();
#ifdef DEBUG
          Serial.println("EL east");
#endif
          return;
      }
    break;
    // S
    case 's':
    case 'S':
      switch (buf[1]) {
        // SA
        case 'a':
        case 'A':
          azRot.halt();
#ifdef DEBUG
          Serial.println("AZ stop");
#endif
          return;
        // MR
        case 'e':
        case 'E':
          elRot.halt();
#ifdef DEBUG
          Serial.println("EL stop");
#endif
          return;
      }
    break;
  }
}

// Main loop
void loop() {
  while (Serial.available() > 0) {
    const size_t BUF_SIZE = 8;
    static char buf[BUF_SIZE];
    static size_t len = 0;
    char c = Serial.read();
    switch (c) {
      // end of message
      case ' ':
      case '\n':
      case '\r':
        commandHandler(buf, len);
        len = 0;
        break;
      // new character
      default:
        buf[len] = c;
        if (len < BUF_SIZE - 1) len++;
        break;
    }
  }  
}
