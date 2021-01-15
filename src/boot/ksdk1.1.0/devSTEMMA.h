#ifndef WARP_BUILD_ENABLE_DEVSTEMMA
#define WARP_BUILD_ENABLE_DEVSTEMMA
#endif


//Registers
#define STEMMA_REG_TEMP 0x04
#define STEMMA_REG_MOISURE 0x0F10


void    initSTEMMA(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer);
WarpStatus readFromSoilSensor(uint8_t addrHigh, uint8_t addrLow, uint16_t pullupValue);
WarpStatus writeToSoilSensor(uint8_t addrHigh, uint8_t addrLow, uint8_t *payloadBytes, uint8_t numberOfBytes);
uint16_t getSoilMoisture();
float getSoilMoistureGMC();