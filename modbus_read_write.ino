#define MODBUS_SLAVE_ID 0x01
#define MODBUS_FUNCTION_READ_HOLDING_REGISTERS 0x03

#define Control_Command 0x1000
#define Communication_Setting 0x4000

// Read Only Status Parameters Addresses
#define Output_Frequency 0x3000
#define Output_Voltage 0x3003
#define Output_Current 0x3004
#define Output_Power 0x3006

#define MODBUS_REGISTER_COUNT 10
#define MODBUS_BAUD_RATE 38400
#define RS485_TX_PIN 17
#define RS485_RX_PIN 16
#define RS485_DE_RE_PIN 4


HardwareSerial RS485Serial(1);
uint16_t registers[MODBUS_REGISTER_COUNT];

uint16_t calculateCRC16(uint8_t *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// void writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
//   digitalWrite(RS485_DE_RE_PIN, HIGH);  // Set DE/RE pin high for TX mode

//   uint8_t request[8];

//   request[0] = slaveId;         // Slave address
//   request[1] = 0x06;            // Function code for writing a single register
//   request[2] = address >> 8;    // High byte of address
//   request[3] = address & 0xFF;  // Low byte of address
//   request[4] = value >> 8;      // High byte of value
//   request[5] = value & 0xFF;    // Low byte of value

//   uint16_t crc = calculateCRC16(request, 6);
//   request[6] = crc & 0xFF;  // Low byte of CRC
//   request[7] = crc >> 8;    // High byte of CRC

//   Serial.print("Request Frame: ");
//   for (int i = 0; i < sizeof(request); i++) {
//     Serial.print("0x");
//     if (request[i] < 0x10) Serial.print("0");
//     Serial.print(request[i], HEX);
//     Serial.print(" ");
//   }
//   Serial.print("CRC: 0x");
//   Serial.print(crc, HEX);
//   Serial.println();

//   RS485Serial.write(request, sizeof(request));

//   Serial.println("Modbus request sent.");

//   digitalWrite(RS485_DE_RE_PIN, LOW);
// }

void writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
  // Set the DE/RE pin high to enable RX mode
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = 0x06;  // Function code for writing a single register
  request[2] = address >> 8;
  request[3] = address & 0xFF;
  request[4] = value >> 8;
  request[5] = value & 0xFF;
  uint16_t crc = calculateCRC16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;
  RS485Serial.write(request, sizeof(request));
  RS485Serial.flush();  // Ensure all data is sent
  delay(200);
  // Set the DE/RE pin low to enable TX mode
  digitalWrite(RS485_DE_RE_PIN, LOW);
}


void sendModbusRequest(uint8_t slaveId, uint8_t functionCode, uint16_t startAddress, uint16_t registerCount) {
  uint8_t request[8];
  request[0] = slaveId;
  request[1] = functionCode;
  request[2] = startAddress >> 8;
  request[3] = startAddress & 0xFF;
  request[4] = registerCount >> 8;
  request[5] = registerCount & 0xFF;
  uint16_t crc = calculateCRC16(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;

  Serial.print("Modbus Request: ");
  for (int i = 0; i < 8; i++) {
    Serial.print("0x");
    if (request[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(request[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  RS485Serial.write(request, sizeof(request));
}


bool readModbusResponse(uint8_t *response, uint16_t length) {
  uint32_t timeout = millis() + 5000;
  uint16_t index = 0;
  while (millis() < timeout) {
    if (RS485Serial.available()) {
      response[index++] = RS485Serial.read();
      if (index >= length) {
        return true;
      }
    }
  }
  return false;
}

bool readHoldingRegisters(uint8_t slaveId, uint16_t startAddress, uint16_t registerCount, uint16_t *buffer) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  sendModbusRequest(slaveId, MODBUS_FUNCTION_READ_HOLDING_REGISTERS, startAddress, registerCount);
  RS485Serial.flush();
  digitalWrite(RS485_DE_RE_PIN, LOW);
  uint8_t response[5 + 2 * registerCount];
  delay(250);  // Wait for response
  if (readModbusResponse(response, sizeof(response))) {
    uint16_t crc = (response[sizeof(response) - 1] << 8) | response[sizeof(response) - 2];
    if (calculateCRC16(response, sizeof(response) - 2) == crc) {
      for (int i = 0; i < registerCount; i++) {
        buffer[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
      }
      return true;
    } else {
      Serial.println("CRC error!");
    }
  } else {
    Serial.println("No response or timeout!");
  }
  return false;
}

int freq = 5;


void setup() {
  Serial.begin(115200);

  RS485Serial.begin(38400, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Start in receive mode

  Serial.println("Modbus RTU over RS485 initialized.");

  // writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, 5000);
  writeSingleRegister(MODBUS_SLAVE_ID, 0x1000, 0x05);  // Command to stop
  // writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, 5000); // Writing frequency to 2000 register address  5000 = 25 x 200 = 25Hz
  writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, freq * 200);  // Writing frequency to 2000 register address  5000 = 25 x 200 = 25Hz


  delay(1000);
}
bool vfd_on = false;
void loop() {
  delay(10);
  // writeSingleRegister(MODBUS_SLAVE_ID, 0x1000, 1);  // Command to run
  // delay(2000);

  if (vfd_on == false) {
    if (freq < 30) {
      freq += 5;
      writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, freq * 200);  // Writing frequency to 2000 register address  5000 = 25 x 200 = 25Hz
      Serial.println("Frequency Written--------------------------------");

      if (freq > 25) {
        // writeSingleRegister(MODBUS_SLAVE_ID, 0x1000, 1);  // Command to run
        // Serial.println("VFD Start-----------------------------------------------------");
        vfd_on = true;
        delay(1000);
      }
    } else {
      // freq = 5;
      // writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, freq*200); // Writing frequency to 2000 register address  5000 = 25 x 200 = 25Hz
      if (freq == 30) {
        // writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, 5);
        // writeSingleRegister(MODBUS_SLAVE_ID, 0x1000, 5);  // Command to stop
        // freq = 35;
      }
    }
  } else {
    if (freq >= 10 && freq != 0) {
      freq -= 5;
      writeSingleRegister(MODBUS_SLAVE_ID, 0x2000, freq * 200);  // Writing frequency to 2000 register address  5000 = 25 x 200 = 25Hz
      Serial.println("Frequency Written-------------------------------------");
      // if (freq <= 0) {
      //   writeSingleRegister(MODBUS_SLAVE_ID, 0x1000, 5);  // Command to stop
      //   vfd_on = false;
      //   // delay(8000);
      // }
    }
  }
  // Read the output frequency
  if (readHoldingRegisters(MODBUS_SLAVE_ID, 0x1000, 1, registers)) {
    for (int i = 0; i < MODBUS_REGISTER_COUNT; i++) {
      Serial.print("Register ");
      Serial.print(0x1000 + i);
      Serial.print(": ");
      Serial.println(registers[i]);
    }
  } else {
    Serial.println("Failed to read registers");
  }
  if (readHoldingRegisters(MODBUS_SLAVE_ID, 0x3000, 1, registers)) {
    for (int i = 0; i < MODBUS_REGISTER_COUNT; i++) {
      Serial.print("Register ===  ");
      Serial.print(0x3000 + i);
      Serial.print(": ");
      Serial.println(registers[i]);
    }
  } else {
    Serial.println("Failed to read registers");
  }
  // delay(2000);
  // writeSingleRegister(MODBUS_SLAVE_ID, 0x1000, 0x05);  // Command to stop
  delay(1000);
  // delay(2000);
}
