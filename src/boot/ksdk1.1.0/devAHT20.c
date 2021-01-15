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

#include "devAHT20.h"

extern volatile WarpI2CDeviceState  deviceAHT20State;
extern volatile uint8_t             gWarpI2cBaudRateKbps;
extern volatile uint8_t             gWarpI2cTimeoutMilliseconds;
extern volatile uint8_t             gWarpSupplySettlingDelayMilliseconds;

uint16_t pullup = 32768;

uint32_t humidityData;
uint32_t tempData;

//Initialise sensor, send calibration commands
void
initAHT20(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer)
{
    WarpStatus status;
    deviceStatePointer->i2cAddress = i2cAddress;
	OSA_TimeDelay(40); //Wait 40ms

    uint8_t initPayload[1];
    initPayload[0] = AHT20_INITIALIZE;
    status = writeToAHT20(initPayload, 1, pullup);
    OSA_TimeDelay(20);

    //Send calibration command
    uint8_t resetPayload[1];
    resetPayload[0] = AHT20_SOFT_RESET;

    status = writeToAHT20(resetPayload, 1, pullup);
    
    OSA_TimeDelay(20);

    uint8_t calibPayload[2];
    calibPayload[0] = AHT20_CALIB_0;
    calibPayload[1] = AHT20_CALIB_1;
    status = writeToAHT20(calibPayload, 2, pullup);

    //wait for status to show as calibrated
    while (getAHT20StatusByte() & AHT20_BUSY)
    {
        OSA_TimeDelay(20);
        SEGGER_RTT_printf(0, "Waiting for calibration to finish... \n");
    }
        SEGGER_RTT_printf(0, "AHT20 init complete \n");

}

//Prompts the sensor to take and store new measurements. The measurements are not returned here, but can be retrieved using the appropriate function.
void getMeasurementAHT20()
{
    //Send measurement trigger
    uint8_t triggerPayload[3];
    triggerPayload[0] = AHT20_TRIGGER_MEASUREMENT;
    triggerPayload[1] = 0x33;
    triggerPayload[2] = 0x00;
    writeToAHT20(triggerPayload, 3, pullup);

    //Wait for measurement to complete
    OSA_TimeDelay(80);
    while (getAHT20StatusByte() & AHT20_BUSY)
    {
        // SEGGER_RTT_printf(0, "Waiting for measurement to finish... \n");
        OSA_TimeDelay(10);
    }
    // SEGGER_RTT_printf(0, "Measurement complete \n");


    //read measurement values
    readFromAHT20(7); 
    humidityData = ((uint32_t)(deviceAHT20State.i2cBuffer[1])<<12) | ((uint32_t)(deviceAHT20State.i2cBuffer[2]) << 4) | ((uint32_t)(deviceAHT20State.i2cBuffer[3]) >> 4);
    tempData = (((uint32_t)((deviceAHT20State.i2cBuffer[3]) & 0x0f) << 16) | ((uint32_t)(deviceAHT20State.i2cBuffer[4]) << 8) | ((uint32_t)(deviceAHT20State.i2cBuffer[5])));

    // SEGGER_RTT_printf(0, "Humidity raw %d \n", humidityData);
    // SEGGER_RTT_printf(0, "Temp raw %d \n", tempData);

    
}

//Return the temperature in degrees C. Must have called getMeasurementAHT20 prior to take a measurement
float getTempAHT20()
{
    float temp = (float)tempData;
    return (temp * 200 / 0x100000) - 50;
}


//Return the humidity as a percentage. Must have called getMeasurementAHT20 prior to take a measurement
float getHumidityAHT20()
{
    float humid = (float)humidityData;
    return (humid * 100 / 0x100000);
}

//Retrieves a status code from the sensor (for example to tell if the sensor is busy)
uint8_t getAHT20StatusByte()
{

    uint8_t statustriggerPayload[1];
    statustriggerPayload[0] = 0x71;

    WarpStatus status;
    status = writeToAHT20(statustriggerPayload, 1, pullup);
    if (status != kWarpStatusOK)
    {
        return 0xFF;
    }    
    status = readFromAHT20(1);
    // SEGGER_RTT_printf(0, "AHT20 Status 0x%02x \n", deviceAHT20State.i2cBuffer[0]);
    if (status == kWarpStatusOK)
    {
        return deviceAHT20State.i2cBuffer[0];
    }
    return 0xFF; //Ensures busy bit is set in event of error
}

//Reads the requested number of bytes from the sensor and stores them in the I2C buffer
WarpStatus readFromAHT20(uint8_t numberOfBytes)
{
    i2c_status_t status;
    USED(numberOfBytes);

    i2c_device_t slave =
    {
        .address = deviceAHT20State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    status = I2C_DRV_MasterReceiveDataBlocking(
        0,
        &slave,
        NULL,
        0,
        (uint8_t *)deviceAHT20State.i2cBuffer,
        numberOfBytes,
        gWarpI2cTimeoutMilliseconds
    );

    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "read failed 0x%02x \n", status);
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

//Writes one or more bytes to the AHT20
WarpStatus
writeToAHT20(uint8_t *payloadBytes, uint8_t numberOfBytes, uint16_t pullupValue)
{
    i2c_status_t status;

    i2c_device_t slave =
    {
        .address = deviceAHT20State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

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
        SEGGER_RTT_printf(0, "write failed 0x%02x \n", status);
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}