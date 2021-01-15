#ifndef WARP_BUILD_ENABLE_DEVAHT20
#define WARP_BUILD_ENABLE_DEVAHT20
#endif

//Commands
#define AHT20_SOFT_RESET 0xBA
#define AHT20_TRIGGER_MEASUREMENT 0xAC
#define AHT20_INITIALIZE 0xBE
#define AHT20_CALIB_0 0x08
#define AHT20_CALIB_1 0x00

#define AHT20_BUSY 0x80
#define AHT20_CAL_ENABLED 0x08

void    initAHT20(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer);
void    softResetAHT20();
WarpStatus writeToAHT20(uint8_t *payloadBytes, uint8_t numberOfBytes, uint16_t pullupValue);
WarpStatus readFromAHT20(uint8_t numberOfBytes);
void getMeasurementAHT20();
float getTempAHT20();
float getHumidityAHT20();
uint8_t getAHT20StatusByte();
