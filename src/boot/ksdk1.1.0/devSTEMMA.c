#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devSTEMMA.h"

extern volatile WarpI2CDeviceState  deviceSTEMMAState;
extern volatile uint8_t             gWarpI2cBaudRateKbps;
extern volatile uint8_t             gWarpI2cTimeoutMilliseconds;
extern volatile uint8_t             gWarpSupplySettlingDelayMilliseconds;

uint16_t pullup = 32768;

const float CALIB_MULTIPLIER = 3.1317;
const float CALIB_OFFSET = 454.6;

//Initialises soil sensor and performs a soft reset
void
initSTEMMA(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer)
{
    deviceStatePointer->i2cAddress = i2cAddress;
	OSA_TimeDelay(50); 


    //Reset
    uint8_t bytes[1];
    bytes[0] = 0xFF;
    writeToSoilSensor(0x00, 0x7F, bytes, 1);

    return;
}

//Performs a write operation to teh STEMMA Soil Sensor
WarpStatus
writeToSoilSensor(uint8_t addrHigh, uint8_t addrLow, uint8_t *payloadBytes, uint8_t numberOfBytes)
{
    uint8_t addressBytes[2];
    i2c_status_t status;
    i2c_device_t slave =
    {
        .address = deviceSTEMMAState.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    addressBytes[0] = addrHigh;
    addressBytes[1] = addrLow;
    //SEGGER_RTT_printf(0, "Write: Writing address- 0x%02x, 0x%02x\n", addressBytes[0], addressBytes[1]);
    status = I2C_DRV_MasterSendDataBlocking(
        0,
        &slave,
        NULL,
        0,
        addressBytes,
        2,
        gWarpI2cTimeoutMilliseconds
    );
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "Write: address failed 0x%02x \n", status);
        return kWarpStatusDeviceCommunicationFailed;
    }

    //SEGGER_RTT_printf(0, "Write: Writing data\n");
    status = I2C_DRV_MasterSendDataBlocking(
        0,
        &slave,
        NULL,
        0,
        payloadBytes,
        numberOfBytes,
        gWarpI2cTimeoutMilliseconds
    );
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "Write: address failed 0x%02x \n", status);
        return kWarpStatusDeviceCommunicationFailed;
    }
    return status;
}

//Performs a read operation on the STEMMA Soil Sensor. 
WarpStatus 
readFromSoilSensor(uint8_t addrHigh, uint8_t addrLow, uint16_t pullupValue)
{
    uint8_t payloadByte[2];

    //Send two read bytes, with high and low addressed
    i2c_status_t status;

    i2c_device_t slave =
    {
        .address = deviceSTEMMAState.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    payloadByte[0] = addrHigh;
    payloadByte[1] = addrLow;
    //SEGGER_RTT_printf(0, "Read: Writing - 0x%02x, 0x%02x\n", payloadByte[0], payloadByte[1]);
    status = I2C_DRV_MasterSendDataBlocking(
        0,
        &slave,
        NULL,
        0,
        payloadByte,
        2,
        gWarpI2cTimeoutMilliseconds
    );
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "Read: write failed 0x%02x \n", status);
        return kWarpStatusDeviceCommunicationFailed;
    }

    OSA_TimeDelay(1000);

    //Perform read
    status = I2C_DRV_MasterReceiveDataBlocking(
        0,
        &slave,
        NULL,
        0,
        (uint8_t *)deviceSTEMMAState.i2cBuffer,
        2,
        gWarpI2cTimeoutMilliseconds
    );


    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "Read: read failed 0x%02x \n", status);
        
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;

}

/**Converts raw sensor value to GMC using calibration parameters experimentally derived.
Sensor can be recalibrated by adjusing the CALIB_MULTIPLIER and CALIB_OFFSET constants */
float getSoilMoistureGMC()
{
    uint16_t raw = getSoilMoisture();
    float res = (float)(raw - CALIB_OFFSET) / CALIB_MULTIPLIER;
    if (res < 0) return 0;
    return res;
}

//Reads the current moisture level
uint16_t getSoilMoisture()
{
    uint16_t val = 65535;
    while (val == 65535)
    {
        readFromSoilSensor(0x0F, 0x10, pullup);
        uint8_t bit0 = deviceSTEMMAState.i2cBuffer[0];
        uint8_t bit1 = deviceSTEMMAState.i2cBuffer[1];
        val = (uint16_t)(bit0 << 8) | bit1;
    }
    return val;
}