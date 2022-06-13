
/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI
    @param spi_cs the SPI CS pin to use along with the default SPI device
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs)
    : spi_dev(spi_cs, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE1) {}
	
	/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31865.h"
#ifdef __AVR
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif

#include <stdlib.h>

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI
*/
/**************************************************************************/
//
Adafruit_MAX31865::Adafruit_MAX31865() {

}


/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param spi_cs the SPI CS pin to use
    @param spi the SPIClass Instance to use CS pin to use
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool Adafruit_MAX31865::begin(int8_t spi_cs, SPIClass& spi, max31865_numwires_t wires) {
  spi_dev = spi;
  spi_dev.begin();

  settings._clock = 1000000;
  settings._bitOrder = MSBFIRST;
  settings._dataMode = SPI_MODE1;

  cs = spi_cs;
  setWires(wires);
  readFault(true); // MAX31865 data sheet (page 14) a manual or automatic fault detection cycle must be run at startup 
  enableBias(false);
  autoConvert(false);
  clearFault();

  // Serial.print("config: ");
  // Serial.println(readRegister8(MAX31865_CONFIG_REG), HEX);
  return true;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @param b If true a automatic fault-detection cycle is performed
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::readFault(boolean b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_FAULTDETCYCLE;  // trigger automatic fault-detection cycle
    writeRegister8(MAX31865_CONFIG_REG, t);
	delay(5);  // wait for 5ms
  }

  return readRegister8(MAX31865_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865::clearFault(void) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31865_CONFIG_FAULTSTAT;
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
  bias = b;
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void Adafruit_MAX31865::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable continuous conversion
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable continuous conversion
  }

  writeRegister8(MAX31865_CONFIG_REG, t);

  if (b && !continuous) {
    if (filter50Hz) {
      delay(70);
    } else {
      delay(60);
    } 
  }
  continuous = b;
}

/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/

void Adafruit_MAX31865::enable50Hz(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_FILT50HZ;
  } else {
    t &= ~MAX31865_CONFIG_FILT50HZ;
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
  filter50Hz = b;
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void Adafruit_MAX31865::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31865_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31865_CONFIG_3WIRE;
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float Adafruit_MAX31865::temperature(float RTDnominal, float refResistor, uint8_t biasOnDelayMS) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD(biasOnDelayMS);
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD(uint8_t biasOnDelayMS) {
  clearFault();
  if (!continuous) {
    if (!bias) {
      enableBias(true);
      delay(biasOnDelayMS);
    }
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, t);
    if (filter50Hz) {
      delay(70);
    } else {
      delay(60);
    }
  }

  uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);

  if (!bias) {
    enableBias(false); // Disable bias current again to reduce selfheating.
  }

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**********************************************/

uint8_t Adafruit_MAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void Adafruit_MAX31865::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  spi_dev.beginTransaction(settings);
  digitalWrite(cs, LOW);
  spi_dev.transfer(addr); //write register address

  // Serial.print(F("\tSPIDevice Wrote: "));
  // Serial.print(F("0x"));
  // Serial.print(addr, HEX);
  // Serial.println();

  for (size_t i = 0; i < n; i++) {
    buffer[i] = spi_dev.transfer(0x00);  // do the reading
  }
  
  digitalWrite(cs, HIGH);
  spi_dev.endTransaction();

  // Serial.print(F("\tSPIDevice Read: "));
  // for (uint16_t i = 0; i < n; i++) {
  //   Serial.print(F("0x"));
  //   Serial.print(buffer[i], HEX);
  //   Serial.print(F(", "));
  //   if (n % 32 == 31) {
  //     Serial.println();
  //   }
  // }
  // Serial.println();
}

void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  addr |= 0x80; // make sure top bit is set

  uint8_t buffer[2] = {addr, data};

  spi_dev.beginTransaction(settings);
  digitalWrite(cs, LOW);

  spi_dev.transfer(buffer[0]);
  spi_dev.transfer(buffer[1]);
  
  digitalWrite(cs, HIGH);
  spi_dev.endTransaction();

  // Serial.print(F("\tSPIDevice Wrote: "));
  // for (uint16_t i = 0; i < 2; i++) {
  //   Serial.print(F("0x"));
  //   Serial.print(buffer[i], HEX);
  //   Serial.print(F(", "));
  //   if (i % 32 == 31) {
  //     Serial.println();
  //   }
  // }
  // Serial.println();
}
