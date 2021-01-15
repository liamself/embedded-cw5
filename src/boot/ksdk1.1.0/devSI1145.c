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

#include "devSI1145.h"

extern volatile WarpI2CDeviceState  deviceSI1145State;
extern volatile uint8_t             gWarpI2cBaudRateKbps;
extern volatile uint8_t             gWarpI2cTimeoutMilliseconds;
extern volatile uint8_t             gWarpSupplySettlingDelayMilliseconds;

uint16_t pullup = 32768;

//Writes a byte to a parameter register
WarpStatus
writeSI1145Param(uint8_t paramAddr, uint8_t payload)
{
    WarpStatus status = writeSensorRegisterSI1145(SI1145_REG_PARAM_WR, payload, pullup);

    if (status != kWarpStatusOK)
    {
        return status;
    }

    status = writeSensorRegisterSI1145(SI1145_REG_COMMAND, paramAddr | SI1145_CMD_PARAM_SET, pullup);
    if (status != kWarpStatusOK)
    {
        return status;
    }
    return readSensorRegisterSI1145(SI1145_REG_PARAM_RD, 1);;
}

//Reads a parameter and stores result in I2C buffer
WarpStatus
readSI1145Param(uint8_t paramAddr)
{
    WarpStatus status = writeSensorRegisterSI1145(SI1145_REG_COMMAND, paramAddr | SI1145_CMD_PARAM_QUERY, pullup);

    if (status != kWarpStatusOK)
    {
        return status;
    }

    return readSensorRegisterSI1145(SI1145_REG_PARAM_RD, 1);
}

//Resets sensor, enables UV, IR and Visible light sensors, and switches to automatic measurement
void
initSI1145(const uint8_t i2cAddress, WarpI2CDeviceState volatile * deviceStatePointer)
{
    deviceStatePointer->i2cAddress = i2cAddress;
	OSA_TimeDelay(50); 

    //Initialize sensor
    resetSI1145();

    //Setup UV to default
    writeSensorRegisterSI1145(SI1145_REG_UCOEF0, 0x29, pullup);
    writeSensorRegisterSI1145(SI1145_REG_UCOEF1, 0x89, pullup);
    writeSensorRegisterSI1145(SI1145_REG_UCOEF2, 0x02, pullup);
    writeSensorRegisterSI1145(SI1145_REG_UCOEF3, 0x00, pullup);

    //Turn on Sensors
    writeSI1145Param(SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_EN_UV | SI1145_PARAM_CHLIST_EN_ALS_IR | SI1145_PARAM_CHLIST_EN_ALS_VIS);

    //Setup visible sensor
    writeSI1145Param(SI1145_PARAM_ALS_VIS_ADC_GAIN, 0);
    writeSI1145Param(SI1145_PARAM_ALS_VIS_ADC_COUNTER, 0x70);
    writeSI1145Param(SI1145_PARAM_ALS_VIS_ADC_MISC, 0x20);



    //Enable automatic measurement
    writeSensorRegisterSI1145(SI1145_REG_MEAS_RATE0, 0xff, pullup);
    writeSensorRegisterSI1145(SI1145_REG_COMMAND, 0x0f, pullup);


    return;



}

//Resets sensor registers. Required when sensor is started up
void
resetSI1145()
{
    writeSensorRegisterSI1145(SI1145_REG_MEAS_RATE0, 0, pullup);
    writeSensorRegisterSI1145(SI1145_REG_MEAS_RATE1, 0, pullup);
    writeSensorRegisterSI1145(SI1145_REG_IRQ_ENABLE, 0, pullup);
    writeSensorRegisterSI1145(SI1145_REG_INT_CFG, 0, pullup);
    writeSensorRegisterSI1145(SI1145_REG_IRQ_STATUS, 0, pullup);
    writeSensorRegisterSI1145(SI1145_REG_COMMAND, SI1145_CMD_RESET, pullup);

	OSA_TimeDelay(10); 
    writeSensorRegisterSI1145(SI1145_REG_HW_KEY, 0x17, pullup);
	OSA_TimeDelay(10); 
    return;
}


//Writes a byte to a given register
WarpStatus 
writeSensorRegisterSI1145(uint8_t deviceRegister, uint8_t payload, uint16_t pullupValue)
{
    uint8_t payloadByte[1], commandByte[1];
    i2c_status_t status;

    switch (deviceRegister)
    {
        case 0x03: case 0x04: case 0x07: case 0x08:
        case 0x09: case 0x0F: case 0x10: case 0x17:
        case 0x18: case 0x20: case 0x21: case 0x22:
        case 0x23: case 0x24: case 0x25: case 0x26:
        case 0x27: case 0x28: case 0x29: case 0x2A:
        case 0x2B: case 0x2C: case 0x2D: case 0x2E:
        case 0x3B: case 0x3C: case 0x3D: case 0x3E:
        {
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave = 
    {
        .address = deviceSI1145State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    commandByte[0] = deviceRegister;
    payloadByte[0] = payload;
    SEGGER_RTT_printf(0, "address 0x%02x, write 0x%02x to 0x%02x \n", slave.address, payload, commandByte[0]);
    status = I2C_DRV_MasterSendDataBlocking(
        0,
        &slave,
        commandByte,
        1,
        payloadByte,
        1,
        gWarpI2cTimeoutMilliseconds
    );
    if (status != kStatus_I2C_Success)
    {
        SEGGER_RTT_printf(0, "write failed 0x%02x \n", status);
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}


//Reads a given register
WarpStatus 
readSensorRegisterSI1145(uint8_t deviceRegister, uint8_t numberOfBytes)
{
    uint8_t cmdBuf[1] = {0xFF};
    i2c_status_t status;

    USED(numberOfBytes);
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02: case 0x03: 
        case 0x04: case 0x07: case 0x08: case 0x09: 
        case 0x0F: case 0x10: case 0x17: case 0x18: 
        case 0x20: case 0x21: case 0x22: case 0x23: 
        case 0x24: case 0x25: case 0x26: case 0x27: 
        case 0x28: case 0x29: case 0x2A: case 0x2B: 
        case 0x2C: case 0x2D: case 0x2E: case 0x30:
        case 0x3B: case 0x3C: case 0x3D: case 0x3E:
        {
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave = 
    {
        .address = deviceSI1145State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    cmdBuf[0] = deviceRegister;

   // SEGGER_RTT_printf(0, "address 0x%02x, read from 0x%02x \n", slave.address, cmdBuf[0]);
    //SEGGER_RTT_printf(0, "send read command \n");
    status = I2C_DRV_MasterReceiveDataBlocking(
        0,
        &slave,
        cmdBuf,
        1,
        (uint8_t *)deviceSI1145State.i2cBuffer,
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


//Returns the current UV light level
uint16_t 
si1145ReadUV()
{
    readSensorRegisterSI1145(SI1145_REG_AUX_DATA0_UVINDEX0, 2);
    uint16_t bit0 = (uint16_t)deviceSI1145State.i2cBuffer[0];
    uint16_t bit1 = (uint16_t)deviceSI1145State.i2cBuffer[1];
    //SEGGER_RTT_printf(0, "0x%02x \n", bit0);
    //SEGGER_RTT_printf(0, "0x%02x \n", bit1);
    return bit0 | (bit1<< 8);
}


//Returns the current Visible light level
uint16_t 
si1145ReadVisibleLight()
{
    readSensorRegisterSI1145(SI1145_REG_ALS_VIS_DATA0, 2);
    uint16_t bit0 = (uint16_t)deviceSI1145State.i2cBuffer[0];
    uint16_t bit1 = (uint16_t)deviceSI1145State.i2cBuffer[1];
    SEGGER_RTT_printf(0, "bit 0 0x%02x \n", bit0);
    SEGGER_RTT_printf(0, "bit 1 0x%02x \n", bit1);
    return bit0 | (bit1<< 8);
}

//Returns the current IR light level
uint16_t 
si1145ReadIR()
{
    readSensorRegisterSI1145(SI1145_REG_ALS_IR_DATA0, 2);
    uint16_t bit0 = (uint16_t)deviceSI1145State.i2cBuffer[0];
    uint16_t bit1 = (uint16_t)deviceSI1145State.i2cBuffer[1];
    //SEGGER_RTT_printf(0, "0x%02x \n", bit0);
    //SEGGER_RTT_printf(0, "0x%02x \n", bit1);
    return bit0 | (bit1<< 8);
}