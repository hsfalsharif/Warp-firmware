#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"
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
//#include "devINA219.h"

extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;

void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}


WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t * payload, int numberOfBytes)
{
	uint8_t		payloadOut[4], commandByte[1];
	i2c_status_t	status;

	// TODO: FILL THIS PART IN BASED ON THE SIMILAR IMPLEMENTATION SHOWN IN devMMA8451Q.c
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02:
        case 0x03: case 0x04: case 0x05:
        {
            /* OK */
			break;
        }
        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }
        i2c_device_t slave =
        {
            .address = deviceINA219State.i2cAddress,
            .baudRate_kbps = gWarpI2cBaudRateKbps
        };

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        commandByte[0] = deviceRegister;
        for (int i = 0 ; i < numberOfBytes ; i++)
            payloadOut[i] = payload[i];
        //warpPrint("Payload[0]: %02x\n", payloadOut[0]);
        //warpPrint("Payload[1]: %02x\n", payloadOut[1]);
        //warpPrint("Payload[2]: %02x", payloadOut[2]);
        //warpPrint("Payload[2]: %02x", payloadOut[3]);
        warpEnableI2Cpins();

        status = I2C_DRV_MasterSendDataBlocking(
                                0 /* I2C instance */,
                                &slave,
                                commandByte,
                                1,
                                payloadOut,
                                numberOfBytes,
                                gWarpI2cTimeoutMilliseconds);
        if (status != kStatus_I2C_Success)
        {
            return kWarpStatusDeviceCommunicationFailed;
        }

        return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(void)
{
    WarpStatus i2cWriteStatus;

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    uint8_t configBytes[2] = {0x21, 0x9F};
    uint8_t calibBytes[2] = {0x10, 0x00};
    i2cWriteStatus = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219Configuration, configBytes, 2);
    i2cWriteStatus = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219Calibration, calibBytes, 2);
    return i2cWriteStatus;
    //                        payloadConfigMSB);
    //i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219Configuration,
    //                        payloadConfigMSB);
    //i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219Configuration,
    //                        payloadConfigLSB);
    //return (i2cWriteStatus1 | i2cWriteStatus2);
}


WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t cmdBuf[1] = {0xFF};
    i2c_status_t    status;

    USED(numberOfBytes);
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02:
        case 0x03: case 0x04: case 0x05:
        {
            /* OK */
			break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
    {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    cmdBuf[0] = deviceRegister;
    warpEnableI2Cpins();

    status = I2C_DRV_MasterReceiveDataBlocking(
                            0 /* I2C peripheral instance*/,
                            &slave,
                            cmdBuf,
                            1,
                            (uint8_t *) deviceINA219State.i2cBuffer,
                            numberOfBytes,
                            gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;

}

// maybe this function doesn't work properly??
void
printSensorDataINA219(bool hexModeFlag)
{
    uint16_t    readSensorRegisterValueLSB;
    uint16_t    readSensorRegisterValueMSB;
    int16_t     readSensorRegisterValueCombined;
    WarpStatus  i2cReadStatus;
    // TODO: MAKE SURE YOU DONT HAVE ISSUES WITH THE SIGNS 
    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    
    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219SHUNT_VOLT, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d,", readSensorRegisterValueCombined);
        }
    }
    // TODO: NOW DO THE SAME THING BUT FOR THE OTHER TWO/THREE REGISTERS
    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219BUS_VOLT, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d,", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219POWER, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d,", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219CURRENT, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        warpPrint(" %d,", readSensorRegisterValueCombined);
        //if (hexModeFlag)
        //{
        //    warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        //}
        //else
        //{
        //    warpPrint(" %d,", readSensorRegisterValueCombined);
        //}
    }

}