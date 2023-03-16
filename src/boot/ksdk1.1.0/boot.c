/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

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
#include "fsl_lpuart_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"


#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"


#if (WARP_BUILD_ENABLE_DEVADXL362)
	#include "devADXL362.h"
	volatile WarpSPIDeviceState			deviceADXL362State;
#endif

#if (WARP_BUILD_ENABLE_DEVIS25xP)
	#include "devIS25xP.h"
	volatile WarpSPIDeviceState			deviceIS25xPState;
#endif

#if (WARP_BUILD_ENABLE_DEVISL23415)
	#include "devISL23415.h"
	volatile WarpSPIDeviceState			deviceISL23415State;
#endif

#if (WARP_BUILD_ENABLE_DEVAT45DB)
	#include "devAT45DB.h"
	volatile WarpSPIDeviceState			deviceAT45DBState;
#endif

#if (WARP_BUILD_ENABLE_DEVICE40)
	#include "devICE40.h"
	volatile WarpSPIDeviceState			deviceICE40State;
#endif

#if (WARP_BUILD_ENABLE_DEVBMX055)
	#include "devBMX055.h"
	volatile WarpI2CDeviceState			deviceBMX055accelState;
	volatile WarpI2CDeviceState			deviceBMX055gyroState;
	volatile WarpI2CDeviceState			deviceBMX055magState;
#endif

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	#include "devMMA8451Q.h"
	volatile WarpI2CDeviceState			deviceMMA8451QState;
#endif

#if (WARP_BUILD_ENABLE_DEVSSD1331)
	#include "devSSD1331.h"
#endif

#if (WARP_BUILD_ENABLE_DEVLPS25H)
	#include "devLPS25H.h"
	volatile WarpI2CDeviceState			deviceLPS25HState;
#endif

#if (WARP_BUILD_ENABLE_DEVHDC1000)
	#include "devHDC1000.h"
	volatile WarpI2CDeviceState			deviceHDC1000State;
#endif

#if (WARP_BUILD_ENABLE_DEVMAG3110)
	#include "devMAG3110.h"
	volatile WarpI2CDeviceState			deviceMAG3110State;
#endif

#if (WARP_BUILD_ENABLE_DEVSI7021)
	#include "devSI7021.h"
	volatile WarpI2CDeviceState			deviceSI7021State;
#endif

#if (WARP_BUILD_ENABLE_DEVL3GD20H)
	#include "devL3GD20H.h"
	volatile WarpI2CDeviceState			deviceL3GD20HState;
#endif

#if (WARP_BUILD_ENABLE_DEVBME680)
	#include "devBME680.h"
	volatile WarpI2CDeviceState			deviceBME680State;
	volatile uint8_t				deviceBME680CalibrationValues[kWarpSizesBME680CalibrationValuesCount];
#endif

#if (WARP_BUILD_ENABLE_DEVTCS34725)
	#include "devTCS34725.h"
	volatile WarpI2CDeviceState			deviceTCS34725State;
#endif

#if (WARP_BUILD_ENABLE_DEVSI4705)
	#include "devSI4705.h"
	volatile WarpI2CDeviceState			deviceSI4705State;
#endif

#if (WARP_BUILD_ENABLE_DEVCCS811)
	#include "devCCS811.h"
	olatile WarpI2CDeviceState			deviceCCS811State;
#endif

#if (WARP_BUILD_ENABLE_DEVAMG8834)
	#include "devAMG8834.h"
	volatile WarpI2CDeviceState			deviceAMG8834State;
#endif

#if (WARP_BUILD_ENABLE_DEVAS7262)
	#include "devAS7262.h"
	volatile WarpI2CDeviceState			deviceAS7262State;
#endif

#if (WARP_BUILD_ENABLE_DEVAS7263)
	#include "devAS7263.h"
	volatile WarpI2CDeviceState			deviceAS7263State;
#endif

#if (WARP_BUILD_ENABLE_DEVRV8803C7)
	#include "devRV8803C7.h"
	volatile WarpI2CDeviceState			deviceRV8803C7State;
#endif

#if (WARP_BUILD_ENABLE_DEVBGX)
	#include "devBGX.h"
	volatile WarpUARTDeviceState			deviceBGXState;
#endif

#if (WARP_BUILD_ENABLE_DEVINA219)
	#include "devINA219.h"
	volatile WarpI2CDeviceState			deviceINA219State;
#endif


volatile i2c_master_state_t				i2cMasterState;
volatile spi_master_state_t				spiMasterState;
volatile spi_master_user_config_t			spiUserConfig;
volatile lpuart_user_config_t				lpuartUserConfig;
volatile lpuart_state_t					lpuartState;


volatile bool						gWarpBooted				= false;
volatile uint32_t					gWarpI2cBaudRateKbps			= kWarpDefaultI2cBaudRateKbps;
volatile uint32_t					gWarpUartBaudRateBps			= kWarpDefaultUartBaudRateBps;
volatile uint32_t					gWarpSpiBaudRateKbps			= kWarpDefaultSpiBaudRateKbps;
volatile uint32_t					gWarpSleeptimeSeconds			= kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask					gWarpMode				= kWarpModeDisableAdcOnSleep;
volatile uint32_t					gWarpI2cTimeoutMilliseconds		= kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t					gWarpSpiTimeoutMicroseconds		= kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t					gWarpUartTimeoutMilliseconds		= kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t					gWarpMenuPrintDelayMilliseconds		= kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t					gWarpSupplySettlingDelayMilliseconds	= kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t					gWarpCurrentSupplyVoltage		= kWarpDefaultSupplyVoltageMillivolts;
char							gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

/*
 *	Since only one SPI transaction is ongoing at a time in our implementaion
 */
uint8_t							gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

//static void						sleepUntilReset(void);
static void						lowPowerPinStates(void);

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	static void					disableTPS62740(void);
	static void					enableTPS62740(uint16_t voltageMillivolts);
	static void					setTPS62740CommonControlLines(uint16_t voltageMillivolts);
#endif

static void						repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
								bool autoIncrement, int chunkReadsPerAddress, bool chatty,
								int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
								uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
static int						char2int(int character);
static void						activateAllLowPowerSensorModes(bool verbose);
static void						powerupAllSensors(void);
static uint8_t						readHexByte(void);
static int						read4digits(void);
static void						printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever);

int 							checkActivityPendSwingUpperXFirstHalf(int16_t acc_x, int idx);
int 							checkActivityPendSwingUpperXSecondHalf(int16_t acc_x, int idx);
int 							checkActivityPendSwingLowerXFirstHalf(int16_t acc_x, int idx);
int 							checkActivityPendSwingLowerXSecondHalf(int16_t acc_x, int idx);

int 							checkActivityPendSwingUpperYFirstHalf(int16_t acc_y, int idx);
int 							checkActivityPendSwingUpperYSecondHalf(int16_t acc_y, int idx);
int 							checkActivityPendSwingLowerYFirstHalf(int16_t acc_y, int idx);
int 							checkActivityPendSwingLowerYSecondHalf(int16_t acc_y, int idx);

int 							checkActivityPendSwingUpperZFirstHalf(int16_t acc_z, int idx);
int 							checkActivityPendSwingUpperZSecondHalf(int16_t acc_z, int idx);
int 							checkActivityPendSwingLowerZFirstHalf(int16_t acc_z, int idx);
int 							checkActivityPendSwingLowerZSecondHalf(int16_t acc_z, int idx);


int 							checkActivityHandWaveS2SUpperXFirstHalf(int16_t acc_x, int idx);
int 							checkActivityHandWaveS2SUpperXSecondHalf(int16_t acc_x, int idx);
int 							checkActivityHandWaveS2SLowerXFirstHalf(int16_t acc_x, int idx);
int 							checkActivityHandWaveS2SLowerXSecondHalf(int16_t acc_x, int idx);

int 							checkActivityHandWaveS2SUpperYFirstHalf(int16_t acc_y, int idx);
int 							checkActivityHandWaveS2SUpperYSecondHalf(int16_t acc_y, int idx);
int 							checkActivityHandWaveS2SLowerYFirstHalf(int16_t acc_y, int idx);
int 							checkActivityHandWaveS2SLowerYSecondHalf(int16_t acc_y, int idx);

int 							checkActivityHandWaveS2SUpperZFirstHalf(int16_t acc_z, int idx);
int 							checkActivityHandWaveS2SUpperZSecondHalf(int16_t acc_z, int idx);
int 							checkActivityHandWaveS2SLowerZFirstHalf(int16_t acc_z, int idx);
int 							checkActivityHandWaveS2SLowerZSecondHalf(int16_t acc_z, int idx);


int 							checkActivityHandWaveFBUpperXFirstHalf(int16_t acc_x, int idx);
int 							checkActivityHandWaveFBUpperXSecondHalf(int16_t acc_x, int idx);
int 							checkActivityHandWaveFBLowerXFirstHalf(int16_t acc_x, int idx);
int 							checkActivityHandWaveFBLowerXSecondHalf(int16_t acc_x, int idx);

int 							checkActivityHandWaveFBUpperYFirstHalf(int16_t acc_y, int idx);
int 							checkActivityHandWaveFBUpperYSecondHalf(int16_t acc_y, int idx);
int 							checkActivityHandWaveFBLowerYFirstHalf(int16_t acc_y, int idx);
int 							checkActivityHandWaveFBLowerYSecondHalf(int16_t acc_y, int idx);

int 							checkActivityHandWaveFBUpperZFirstHalf(int16_t acc_z, int idx);
int 							checkActivityHandWaveFBUpperZSecondHalf(int16_t acc_z, int idx);
int 							checkActivityHandWaveFBLowerZFirstHalf(int16_t acc_z, int idx);
int 							checkActivityHandWaveFBLowerZSecondHalf(int16_t acc_z, int idx);


/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);



/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */



//void
//sleepUntilReset(void)
//{
//	while (1)
//	{
//		#if (WARP_BUILD_ENABLE_DEVSI4705)
//			GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
//		#endif
//
//		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);
//
//		#if (WARP_BUILD_ENABLE_DEVSI4705)
//			GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
//		#endif
//
//		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
//	}
//}


void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *
	 *	TODO: we don't use hw flow control so don't need RTS/CTS
 	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
 	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

	//TODO: we don't use hw flow control so don't need RTS/CTS
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}


void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	/*
	 * We don't use the HW flow control and that messes with the SPI any way
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	 */
	CLOCK_SYS_DisableLpuartClock(0);
}



WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



void
warpEnableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	kWarpPinSPI_MISO_UART_RTS_UART_RTS --> PTA6 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	kWarpPinSPI_MOSI_UART_CTS --> PTA7 (ALT3)	*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAlt3);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(ALT3)		*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);
	#endif

	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
warpDisableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);

	/*	kWarpPinSPI_MISO_UART_RTS	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	kWarpPinSPI_MOSI_UART_CTS	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		/*	kWarpPinSPI_SCK	--> PTA9	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	#else
		/*	kWarpPinSPI_SCK	--> PTB0	(GPIO)			*/
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	#endif

	//TODO: we don't use HW flow control so can remove these since we don't use the RTS/CTS
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);

	CLOCK_SYS_DisableSpiClock(0);
}



void
warpDeasserAllSPIchipSelects(void)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Drive all chip selects high to disable them. Individual drivers call this routine before
	 *	appropriately asserting their respective chip selects.
	 *
	 *	Setup:
	 *		PTA12/kWarpPinISL23415_SPI_nCS	for GPIO
	 *		PTA9/kWarpPinAT45DB_SPI_nCS	for GPIO
	 *		PTA8/kWarpPinADXL362_SPI_nCS	for GPIO
	 *		PTB1/kWarpPinFPGA_nCS		for GPIO
	 *
	 *		On Glaux
	 		PTB2/kGlauxPinFlash_SPI_nCS for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortMuxAsGpio);
	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);
	#endif

	#if (WARP_BUILD_ENABLE_DEVISL23415)
		GPIO_DRV_SetPinOutput(kWarpPinISL23415_SPI_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_DEVAT45DB)
		GPIO_DRV_SetPinOutput(kWarpPinAT45DB_SPI_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_DEVADXL362)
		GPIO_DRV_SetPinOutput(kWarpPinADXL362_SPI_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_DEVICE40)
		GPIO_DRV_SetPinOutput(kWarpPinFPGA_nCS);
	#endif

	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
	#endif
}



void
debugPrintSPIsinkBuffer(void)
{
	for (int i = 0; i < kWarpMemoryCommonSpiBufferBytes; i++)
	{
		warpPrint("\tgWarpSpiCommonSinkBuffer[%d] = [0x%02X]\n", i, gWarpSpiCommonSinkBuffer[i]);
	}
	warpPrint("\n");
}



void
warpEnableI2Cpins(void)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
}



void
warpDisableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	CLOCK_SYS_DisableI2cClock(0);
}


#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
	void
	lowPowerPinStates(void)
	{
		/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state, except for the
		 *	sacrificial pins (WLCSP package, Glaux) where we set them to disabled. We choose
		 *	to set non-disabled pins to '0'.
		 *
		 *	NOTE: Pin state "disabled" means default functionality is active.
		 */

		/*
		 *			PORT A
		 */
		/*
		 *	Leave PTA0/1/2 SWD pins in their default state (i.e., as SWD / Alt3).
		 *
		 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

		/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
		 *
		 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

		/*
		 *	Disable PTA5
		 *
		 *	NOTE: Enabling this significantly increases current draw
		 *	(from ~180uA to ~4mA) and we don't need the RTC on Glaux.
		 *
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

		/*
		 *	PTA6, PTA7, PTA8, and PTA9 on Glaux are SPI and sacrificial SPI.
		 *
		 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
		 *
		 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
		 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

		/*
		 *	NOTE: The KL03 has no PTA10 or PTA11
		 */

		/*
		 *	In Glaux, PTA12 is a sacrificial pin for SWD_RESET, so careful not to drive it.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);



		/*
		 *			PORT B
		 *
		 *	PTB0 is LED on Glaux. PTB1 is unused, and PTB2 is FLASH_!CS
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

		/*
		 *	PTB3 and PTB4 (I2C pins) are true open-drain and we
		 *	purposefully leave them disabled since they have pull-ups.
		 *	PTB5 is sacrificial for I2C_SDA, so disable.
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);

		/*
		 *	NOTE:
		 *
		 *	The KL03 has no PTB8, PTB9, or PTB12.  Additionally, the WLCSP package
		 *	we in Glaux has no PTB6, PTB7, PTB10, or PTB11.
		 */

		/*
		 *	In Glaux, PTB13 is a sacrificial pin for SWD_RESET, so careful not to drive it.
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);

		GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
		GPIO_DRV_ClearPinOutput(kGlauxPinLED);

		return;
	}
#else
	void
	lowPowerPinStates(void)
	{
		/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state. We choose
		 *	to set them all to '0' since it happens that the devices we want to keep
		 *	deactivated (SI4705) also need '0'.
		 */

		/*
		 *			PORT A
		 */
		/*
		 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
		PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

		/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
		 *
		 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

		/*
		 *	Disable PTA5
		 *
		 *	NOTE: Enabling this significantly increases current draw
		 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
		 *
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

		/*
		 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
		 *
		 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
		 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

		/*
		 *	NOTE: The KL03 has no PTA10 or PTA11
		 */
		PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


		/*
		 *			PORT B
		 */
		PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
		PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
	}
#endif


#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
void
disableTPS62740(void)
{
		GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_REGCTRL);
}
#endif


#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
void
enableTPS62740(uint16_t voltageMillivolts)
{
	/*
	 *	By default, assusme pins are currently disabled (e.g., by a recent lowPowerPinStates())
	 *
	 *	Setup:
	 *		PTB5/kWarpPinTPS62740_REGCTRL for GPIO
	 *		PTB6/kWarpPinTPS62740_VSEL4 for GPIO
	 *		PTB7/kWarpPinTPS62740_VSEL3 for GPIO
	 *		PTB10/kWarpPinTPS62740_VSEL2 for GPIO
	 *		PTB11/kWarpPinTPS62740_VSEL1 for GPIO
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAsGpio);

	setTPS62740CommonControlLines(voltageMillivolts);
	GPIO_DRV_SetPinOutput(kWarpPinTPS62740_REGCTRL);
}
#endif


#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
void
setTPS62740CommonControlLines(uint16_t voltageMillivolts)
{
		switch(voltageMillivolts)
		{
			case 1800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 1900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2400:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2500:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2600:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2700:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2800:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 2900:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3000:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3100:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3200:
			{
				GPIO_DRV_ClearPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			case 3300:
			{
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL1);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL2);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL3);
				GPIO_DRV_SetPinOutput(kWarpPinTPS62740_VSEL4);

				break;
			}

			/*
			 *	Should never happen, due to previous check in warpScaleSupplyVoltage()
			 */
			default:
			{
				warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
			}
		}

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
}
#endif



void
warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == gWarpCurrentSupplyVoltage)
	{
		return;
	}

	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
		if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
		{
			enableTPS62740(voltageMillivolts);
			gWarpCurrentSupplyVoltage = voltageMillivolts;
		}
		else
		{
			warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
		}
	#endif
}



void
warpDisableSupplyVoltage(void)
{
	#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
		disableTPS62740();

		/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
		 */
		OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
	#endif
}


void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	WarpStatus	status = kWarpStatusOK;

	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
	if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}

	status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}
}


/*
void
printPinDirections(void)
{
	warpPrint("I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA_UART_RX));
	OSA_TimeDelay(100);
	warpPrint("I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL_UART_TX));
	OSA_TimeDelay(100);
	warpPrint("SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI_UART_CTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO_UART_RTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
	OSA_TimeDelay(100);
	warpPrint("ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
	OSA_TimeDelay(100);
}
*/




void
printBootSplash(uint16_t gWarpCurrentSupplyVoltage, uint8_t menuRegisterAddress, WarpPowerManagerCallbackStructure *  powerManagerCallbackStructure)
{
	/*
	 *	We break up the prints with small delays to allow us to use small RTT print
	 *	buffers without overrunning them when at max CPU speed.
	 */
	warpPrint("\r\n\n\n\n[ *\t\t\t\tWarp (HW revision C) / Glaux (HW revision B)\t\t\t* ]\n");
	warpPrint("\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
	//warpPrint("\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
	//		gWarpCurrentSupplyVoltage, menuRegisterAddress);
	//warpPrint("\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%db/s,\tI2C Pull-Up=%d\n\n",
	//		gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateBps);
	//warpPrint("\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
	//warpPrint("\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
	//warpPrint("\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
	//warpPrint("\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
	//warpPrint("\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
	//warpPrint("\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure->errorCount);
}

void
blinkLED(int pin)
{
	GPIO_DRV_SetPinOutput(pin);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(pin);
	OSA_TimeDelay(200);

	return;
}

void
warpPrint(const char *fmt, ...)
{
	int	fmtlen;
	va_list	arg;

	/*
	 *	We use an ifdef rather than a C if to allow us to compile-out
	 *	all references to SEGGER_RTT_*printf if we don't want them.
	 *
	 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
	 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
	 *	also takes our print buffer which we will eventually send over
	 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
	 *	2kB flash and removes the use of malloc so we can keep heap
	 *	allocation to zero.
	 */
	#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
		/*
		 *	We can't use SEGGER_RTT_vprintf to format into a buffer
		 *	since SEGGER_RTT_vprintf formats directly into the special
		 *	RTT memory region to be picked up by the RTT / SWD mechanism...
		 */
		va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
		va_end(arg);

		if (fmtlen < 0)
		{
			SEGGER_RTT_WriteString(0, gWarpEfmt);

			#if (WARP_BUILD_ENABLE_DEVBGX)
				if (gWarpBooted)
				{
					WarpStatus	status;

					enableLPUARTpins();
					initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
					status = sendBytesToUART((uint8_t *)gWarpEfmt, strlen(gWarpEfmt)+1);
					if (status != kWarpStatusOK)
					{
						SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
					}
					disableLPUARTpins();

					/*
					 *	We don't want to deInit() the BGX since that would drop
					 *	any remote terminal connected to it.
					 */
					//deinitBGX();
				}
			#endif

			return;
		}

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);

				status = sendBytesToUART((uint8_t *)gWarpPrintBuffer, max(fmtlen, kWarpDefaultPrintBufferSizeBytes));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#else
		/*
		 *	If we are not compiling in the SEGGER_RTT_printf,
		 *	we just send the format string of warpPrint()
		 */
		SEGGER_RTT_WriteString(0, fmt);

		/*
		 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
		 */
		#if (WARP_BUILD_ENABLE_DEVBGX)
			if (gWarpBooted)
			{
				WarpStatus	status;

				enableLPUARTpins();
				initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
				status = sendBytesToUART(fmt, strlen(fmt));
				if (status != kWarpStatusOK)
				{
					SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
				}
				disableLPUARTpins();

				/*
				 *	We don't want to deInit() the BGX since that would drop
				 *	any remote terminal connected to it.
				 */
				//deinitBGX();
			}
		#endif
	#endif


	/*
	 *	Throttle to enable SEGGER to grab output, otherwise "run" mode may miss lines.
	 */
	OSA_TimeDelay(5);

	return;
}

int
warpWaitKey(void)
{
	/*
	 *	SEGGER'S implementation assumes the result of result of
	 *	SEGGER_RTT_GetKey() is an int, so we play along.
	 */
	int		rttKey, bleChar = kWarpMiscMarkerForAbsentByte;

	/*
	 *	Set the UART buffer to 0xFF and then wait until either the
	 *	UART RX buffer changes or the RTT icoming key changes.
	 *
	 *	The check below on rttKey is exactly what SEGGER_RTT_WaitKey()
	 *	does in SEGGER_RTT.c.
	 */
	#if (WARP_BUILD_ENABLE_DEVBGX)
		deviceBGXState.uartRXBuffer[0] = kWarpMiscMarkerForAbsentByte;
		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
	#endif

	do
	{
		rttKey	= SEGGER_RTT_GetKey();

		#if (WARP_BUILD_ENABLE_DEVBGX)
			bleChar	= deviceBGXState.uartRXBuffer[0];
		#endif

		/*
		 *	NOTE: We ignore all chars on BLE except '0'-'9', 'a'-'z'/'A'-Z'
		 */
		if (!(bleChar > 'a' && bleChar < 'z') && !(bleChar > 'A' && bleChar < 'Z') && !(bleChar > '0' && bleChar < '9'))
		{
			bleChar = kWarpMiscMarkerForAbsentByte;
		}
	} while ((rttKey < 0) && (bleChar == kWarpMiscMarkerForAbsentByte));


	return rttKey;
}

int
main(void)
{
	WarpStatus				status;
	uint8_t					key;
	WarpSensorDevice			menuTargetSensor		= kWarpSensorMMA8451Q;
	volatile WarpI2CDeviceState *		menuI2cDevice			= NULL;
	uint8_t					menuRegisterAddress		= 0x00;
	rtc_datetime_t				warpBootDate;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: POWER_SYS_SetMode() depends on this order
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	When booting to CSV stream, we wait to be up and running as soon as possible after
	 *	a reset (e.g., a reset due to waking from VLLS0)
	 */
	if (!WARP_BUILD_BOOT_TO_CSVSTREAM)
	{
		warpPrint("\n\n\n\rBooting Warp, in 3... ");
		OSA_TimeDelay(1000);
		warpPrint("2... ");
		OSA_TimeDelay(1000);
		warpPrint("1...\n\n\n\r");
		OSA_TimeDelay(1000);
	}

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);


	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	//warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	//warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
	 *	Note that it is lowPowerPinStates() that sets the pin mux mode,
	 *	so until we call it pins are in their default state.
	 */
	//warpPrint("About to lowPowerPinStates()... ");
	//lowPowerPinStates();
	//warpPrint("done.\n");

	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST on Warp revB, kGlauxPinLED on Glaux)
	 */
	#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		blinkLED(kGlauxPinLED);
		blinkLED(kGlauxPinLED);
		blinkLED(kGlauxPinLED);
	#endif

	/*
	 *	Initialize all the sensors
	 */
	#if (WARP_BUILD_ENABLE_DEVBMX055)
		initBMX055accel(0x18	/* i2cAddress */,	&deviceBMX055accelState,	kWarpDefaultSupplyVoltageMillivoltsBMX055accel	);
		initBMX055gyro(	0x68	/* i2cAddress */,	&deviceBMX055gyroState,		kWarpDefaultSupplyVoltageMillivoltsBMX055gyro	);
		initBMX055mag(	0x10	/* i2cAddress */,	&deviceBMX055magState,		kWarpDefaultSupplyVoltageMillivoltsBMX055mag	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
		initMMA8451Q(	0x1D	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsMMA8451Q	);
		configureSensorMMA8451Q(0x00, 0x01);
	#endif


	#if (WARP_BUILD_ENABLE_DEVLPS25H)
		initLPS25H(	0x5C	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsLPS25H	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVHDC1000)
		initHDC1000(	0x43	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsHDC1000	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVMAG3110)
		initMAG3110(	0x0E	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsMAG3110	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVSI7021)
		initSI7021(	0x40	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsSI7021	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVL3GD20H)
		initL3GD20H(	0x6A	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsL3GD20H	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVBME680)
		initBME680(	0x77	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsBME680	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVTCS34725)
		initTCS34725(	0x29	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsTCS34725	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVSI4705)
		initSI4705(	0x11	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsSI4705	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVCCS811)
		initCCS811(	0x5A	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsCCS811	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVAMG8834)
		initAMG8834(	0x68	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsAMG8834	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVAS7262)
		initAS7262(	0x49	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsAS7262	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVAS7263)
		initAS7263(	0x49	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsAS7263	);
	#endif

	#if (WARP_BUILD_ENABLE_DEVINA219)
		initINA219(	0x40	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsINA219	);
		//configureSensorINA219();
		//warpPrint("INA219 Configured.\n");
	#endif



	/*
	 *	If WARP_BUILD_DISABLE_SUPPLIES_BY_DEFAULT, will turn of the supplies
	 *	below which also means that the console via BLE will be disabled as
	 *	the BLE module will be turned off by default.
	 */



	while (1)
	{
		/*
		 *	Do not, e.g., lowPowerPinStates() on each iteration, because we actually
		 *	want to use menu to progressiveley change the machine state with various
		 *	commands.
		 */

		#if (WARP_BUILD_ENABLE_ACTIVITY_CLASSIFICATION)
			warpPrint("\r- 'v': classify activity using accelerometer.\n");
		#endif

		key = warpWaitKey();

		switch (key)
		{
			#if (WARP_BUILD_ENABLE_ACTIVITY_CLASSIFICATION)
			case 'v':
			{	
				uint16_t x_invalid_pend = 0;
				uint16_t y_invalid_pend = 0;
				uint16_t z_invalid_pend = 0;

				uint16_t x_invalid_HW_S2S = 0;
				uint16_t y_invalid_HW_S2S = 0;
				uint16_t z_invalid_HW_S2S = 0;

				uint16_t x_invalid_HW_FB = 0;
				uint16_t y_invalid_HW_FB = 0;
				uint16_t z_invalid_HW_FB = 0;
				uint16_t confidenceLevel;
				warpScaleSupplyVoltage(1800);
				warpPrint("\n>>");
				warpWaitKey();
				warpPrint("\nAcc_X,Acc_Y,Acc_Z\n");
				for (int i = 0; i < 461; i++)
				{
					int16_t acc_x, acc_y, acc_z;
					uint16_t msb = 0;
					uint16_t lsb = 0;

					int x_greater_pend;
					int x_lesser_pend;

					int y_greater_pend;
					int y_lesser_pend;

					int z_greater_pend;
					int z_lesser_pend;

					int x_greater_HW_S2S;
					int x_lesser_HW_S2S;

					int y_greater_HW_S2S;
					int y_lesser_HW_S2S;

					int z_greater_HW_S2S;
					int z_lesser_HW_S2S;

					int x_greater_HW_FB;
					int x_lesser_HW_FB;

					int y_greater_HW_FB;
					int y_lesser_HW_FB;

					int z_greater_HW_FB;
					int z_lesser_HW_FB;

					// MSB X
					readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, 1);
					msb = deviceMMA8451QState.i2cBuffer[0];

					// LSB X
					readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_LSB, 1);
					lsb = deviceMMA8451QState.i2cBuffer[0];

					acc_x = ((msb & 0xFF) << 6) | (lsb >> 2);
					// sign extend the 14-bit value
					acc_x = (acc_x ^ (1 << 13)) - (1 << 13);
					// MSB Y
					readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, 1);
					msb = deviceMMA8451QState.i2cBuffer[0];

					// LSB Y
					readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_LSB, 1);
					lsb = deviceMMA8451QState.i2cBuffer[0];

					acc_y = ((msb & 0xFF) << 6) | (lsb >> 2);
					// sign extend the 14-bit value
					acc_y = (acc_y ^ (1 << 13)) - (1 << 13);
					// MSB Z
					readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, 1);
					msb = deviceMMA8451QState.i2cBuffer[0];

					// LSB Z
					readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_LSB, 1);
					lsb = deviceMMA8451QState.i2cBuffer[0];

					acc_z = ((msb & 0xFF) << 6) | (lsb >> 2);
					// sign extend the 14-bit value
					acc_z = (acc_z ^ (1 << 13)) - (1 << 13);
					
					if (i < 251)
					{
						x_greater_pend = checkActivityPendSwingUpperXFirstHalf(acc_x, i);
						x_lesser_pend = checkActivityPendSwingLowerXFirstHalf(acc_x, i);
						
						y_greater_pend = checkActivityPendSwingUpperYFirstHalf(acc_y, i);
						y_lesser_pend = checkActivityPendSwingLowerYFirstHalf(acc_y, i);

						z_greater_pend = checkActivityPendSwingUpperZFirstHalf(acc_z, i);
						z_lesser_pend = checkActivityPendSwingLowerZFirstHalf(acc_z, i);

						x_greater_HW_S2S = checkActivityHandWaveS2SUpperXFirstHalf(acc_x, i);
						x_lesser_HW_S2S = checkActivityHandWaveS2SLowerXFirstHalf(acc_x, i);
						
						y_greater_HW_S2S = checkActivityHandWaveS2SUpperYFirstHalf(acc_y, i);
						y_lesser_HW_S2S = checkActivityHandWaveS2SLowerYFirstHalf(acc_y, i);

						z_greater_HW_S2S = checkActivityHandWaveS2SUpperZFirstHalf(acc_z, i);
						z_lesser_HW_S2S = checkActivityHandWaveS2SLowerZFirstHalf(acc_z, i);
					
						x_greater_HW_FB = checkActivityHandWaveFBUpperXFirstHalf(acc_x, i);
						x_lesser_HW_FB = checkActivityHandWaveFBLowerXFirstHalf(acc_x, i);
						
						y_greater_HW_FB = checkActivityHandWaveFBUpperYFirstHalf(acc_y, i);
						y_lesser_HW_FB = checkActivityHandWaveFBLowerYFirstHalf(acc_y, i);

						z_greater_HW_FB = checkActivityHandWaveFBUpperZFirstHalf(acc_z, i);
						z_lesser_HW_FB = checkActivityHandWaveFBLowerZFirstHalf(acc_z, i);
					}
					else 
					{
						x_greater_pend = checkActivityPendSwingUpperXSecondHalf(acc_x, i - 251);
						x_lesser_pend = checkActivityPendSwingLowerXSecondHalf(acc_x, i - 251);
						
						y_greater_pend = checkActivityPendSwingUpperYSecondHalf(acc_y, i - 251);
						y_lesser_pend = checkActivityPendSwingLowerYSecondHalf(acc_y, i - 251);

						z_greater_pend = checkActivityPendSwingUpperZSecondHalf(acc_z, i - 251);
						z_lesser_pend = checkActivityPendSwingLowerZSecondHalf(acc_z, i - 251);

						x_greater_HW_S2S = checkActivityHandWaveS2SUpperXSecondHalf(acc_x, i - 251);
						x_lesser_HW_S2S = checkActivityHandWaveS2SLowerXSecondHalf(acc_x, i - 251);
						
						y_greater_HW_S2S = checkActivityHandWaveS2SUpperYSecondHalf(acc_y, i - 251);
						y_lesser_HW_S2S = checkActivityHandWaveS2SLowerYSecondHalf(acc_y, i - 251);

						z_greater_HW_S2S = checkActivityHandWaveS2SUpperZSecondHalf(acc_z, i - 251);
						z_lesser_HW_S2S = checkActivityHandWaveS2SLowerZSecondHalf(acc_z, i - 251);
					
						x_greater_HW_FB = checkActivityHandWaveFBUpperXSecondHalf(acc_x, i - 251);
						x_lesser_HW_FB = checkActivityHandWaveFBLowerXSecondHalf(acc_x, i - 251);
						
						y_greater_HW_FB = checkActivityHandWaveFBUpperYSecondHalf(acc_y, i - 251);
						y_lesser_HW_FB = checkActivityHandWaveFBLowerYSecondHalf(acc_y, i - 251);

						z_greater_HW_FB = checkActivityHandWaveFBUpperZSecondHalf(acc_z, i - 251);
						z_lesser_HW_FB = checkActivityHandWaveFBLowerZSecondHalf(acc_z, i - 251);
					}

					warpPrint("%d, %d, %d\n", acc_x, acc_y, acc_z);

					if (x_greater_pend + x_lesser_pend > 0)
					{
						x_invalid_pend++;
					}

					if (y_greater_pend + y_lesser_pend > 0)
					{
						y_invalid_pend++;
					}

					if (z_greater_pend + z_lesser_pend > 0)
					{
						z_invalid_pend++;
					}

					if (x_greater_HW_S2S + x_lesser_HW_S2S > 0)
					{
						x_invalid_HW_S2S++;
					}

					if (y_greater_HW_S2S + y_lesser_HW_S2S > 0)
					{
						y_invalid_HW_S2S++;
					}

					if (z_greater_HW_S2S + z_lesser_HW_S2S > 0)
					{
						z_invalid_HW_S2S++;
					}

					if (x_greater_HW_FB + x_lesser_HW_FB > 0)
					{
						x_invalid_HW_FB++;
					}

					if (y_greater_HW_FB + y_lesser_HW_FB > 0)
					{
						y_invalid_HW_FB++;
					}

					if (z_greater_HW_FB + z_lesser_HW_FB > 0)
					{
						z_invalid_HW_FB++;
					}
				}

				confidenceLevel = (100 * (1463 - x_invalid_pend - y_invalid_pend - z_invalid_pend)) / 1463;
				if (confidenceLevel > 70)
				{
					warpPrint("=>Pendulum swing from cable. Confidence: %d%%\n", confidenceLevel);
				}
				else
				{
					warpPrint("Not a pendulum swing. Confidence: %d%%\n", confidenceLevel);
				}
		
				confidenceLevel = (100 * (1463 - x_invalid_HW_S2S - y_invalid_HW_S2S - z_invalid_HW_S2S)) / 1463;
				if (confidenceLevel > 70)
				{
					warpPrint("=>Five side-to-side handwaves. Confidence: %d%%\n", confidenceLevel);
				}
				else
				{
					warpPrint("Not five side-to-side handwaves. Confidence: %d%%\n", confidenceLevel);
				}
				
				confidenceLevel = (100 * (1463 - x_invalid_HW_FB - y_invalid_HW_FB - z_invalid_HW_FB)) / 1463;
				if (confidenceLevel > 70)
				{
					warpPrint("=>Four forwards and backwards handwaves. Confidence: %d%%\n", confidenceLevel);
				}
				else
				{
					warpPrint("Not four forwards and backwards handwaves. Confidence: %d%%\n", confidenceLevel);
				}

				break;
			}
			#endif
#if (WARP_BUILD_ENABLE_DEVRV8803C7)
			case 'v':
			{
				warpPrint("\r\n\tSleeping for 3 seconds, then resetting\n");
				warpSetLowPowerMode(kWarpPowerModeVLLS0, 3 /* sleep seconds */);
				if (status != kWarpStatusOK)
				{
					warpPrint("warpSetLowPowerMode(kWarpPowerModeVLLS0, 3 /* sleep seconds : irrelevant here */)() failed...\n");
				}

				warpPrint("\r\n\tThis should never happen...\n");

				break;
			}
#endif
			
			/*
			 *	Write raw bytes read from console to Flash
			 */
			#if (WARP_BUILD_ENABLE_DEVIS25xP)
			case 'F':
			{
				warpPrint(
					"\r\n\tDevice: IS25xP"
					"\r\n\t'1' - Info and status registers"
					"\r\n\t'2' - Dump JEDEC Table"
					"\r\n\t'3' - Read"
				);
				warpPrint(
					"\r\n\t'4' - Write Enable"
					"\r\n\t'5' - Write"
					"\r\n\t'6' - Chip Erase"
				);
				warpPrint("\r\n\tEnter selection> ");
				key = warpWaitKey();
				warpPrint("\n");

				switch(key)
				{
					
				/*
				 *	Read informational and status registers
				 */
				case '1':
				{
					uint8_t	ops1[] = {	/* Read JEDEC ID Command */
						0x9F,	/* Instruction Code */
						0x00,	/* Dummy Receive Byte */
						0x00,	/* Dummy Receive Byte */
						0x00,	/* Dummy Receive Byte */
					};
					status = spiTransactionIS25xP(ops1, sizeof(ops1)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read JEDEC ID failed...\n");
					}
					else
					{
						warpPrint("JEDEC ID = [0x%X] [0x%X] [0x%X]\n", deviceIS25xPState.spiSinkBuffer[1], deviceIS25xPState.spiSinkBuffer[2], deviceIS25xPState.spiSinkBuffer[3]);
					}
					

					uint8_t	ops2[] = {	/* Read Manufacturer & Device ID */
						0x90,	/* Instruction Code */
						0x00,	/* Dummy Byte 1	    */ 
						0x00,	/* Dummy Byte 2     */ 
						0x00,	/* Control. 00h: First MFID then ID. 01h: First ID then MFID. */
						0x00,	/* Dummy Receive Byte */
						0x00,	/* Dummy Receive Byte */
					};
					status = spiTransactionIS25xP(ops2, sizeof(ops2)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Manufacturer ID failed...\n");
					}
					else
					{
						warpPrint("Manufacturer & Device ID = [0x%X] [0x%X]\n", deviceIS25xPState.spiSinkBuffer[4], deviceIS25xPState.spiSinkBuffer[5]);
					}

					uint8_t	ops3[] = {	/* Read ID / Release Power Down */
						0xAB,	/* Instruction Code */  
						0x00,	/* Dummy Byte */  
						0x00,	/* Dummy Byte */ 
						0x00,	/* Dummy Byte */ 
						0x00,	/* Dummy Receive Byte */
					};
					status = spiTransactionIS25xP(ops3, sizeof(ops3)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("Flash ID = [0x%x]\n", deviceIS25xPState.spiSinkBuffer[4]);
					}

					uint8_t	ops4[] = {	/* Read Status Register */
						0x05,	/* Byte0 */  
						0x00,	/* Dummy Byte1 */
					};
					status = spiTransactionIS25xP(ops4, sizeof(ops4)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("Status = ["BYTE_TO_BINARY_PATTERN"]\n", BYTE_TO_BINARY(deviceIS25xPState.spiSinkBuffer[1]));
					}

					uint8_t	ops5[] = {	/* Read Function Register */
						0x48,	/* RDFR */  
						0x00,	/* Dummy Byte1 */
					};
					status = spiTransactionIS25xP(ops5, sizeof(ops5)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("RDFR = ["BYTE_TO_BINARY_PATTERN"]\n", BYTE_TO_BINARY(deviceIS25xPState.spiSinkBuffer[1]));
					}

					uint8_t	ops6[] = {	/* Read Read Parameters */
						0x61,	/* RDRP */  
						0x00,	/* Dummy Byte1 */
					};
					status = spiTransactionIS25xP(ops6, sizeof(ops6)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("ReadParam = ["BYTE_TO_BINARY_PATTERN"]\n", BYTE_TO_BINARY(deviceIS25xPState.spiSinkBuffer[1]));
					}

					uint8_t	ops7[] = {	/* Read Extended Read Parameters */
						0x81,	/* RDERP */  
						0x00,	/* Dummy Byte1 */
					};
					status = spiTransactionIS25xP(ops7, sizeof(ops7)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("ExtReadParam = ["BYTE_TO_BINARY_PATTERN"]\n", BYTE_TO_BINARY(deviceIS25xPState.spiSinkBuffer[1]));
					}

					uint8_t	ops8[] = {	/* Read Unique ID */
						0x4B,	/* RDUID */  
						0x00,	/* Dummy Byte */
						0x00,	/* Dummy Byte */
						0x00,	/* Dummy Byte */
						0x00,	/* Dummy Byte */
						0x00,	/* Receive */
					};
					status = spiTransactionIS25xP(ops8, sizeof(ops8)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("UID = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5]);
					}

					break;
				}

				
				/*
				 *	Dump first 0xF addresses from JEDEC table
				 */
				case '2':
				{
					uint8_t	ops[] = {	/* Read JEDEC Discoverable Params */
						0x5A,	/* RDSFDP */  
						0x00,	/* Address Byte */
						0x00,	/* Address Byte */
						0x00,	/* Address Byte */
						0x00,	/* Dummy Byte */
						0x00,	/* Receive 0x00 */
						0x00,	/* Receive 0x01 */
						0x00,	/* Receive 0x02 */
						0x00,	/* Receive 0x03 */
						0x00,	/* Receive 0x04 */
						0x00,	/* Receive 0x05 */
						0x00,	/* Receive 0x06 */
						0x00,	/* Receive 0x07 */
						0x00,	/* Receive 0x08 */
						0x00,	/* Receive 0x09 */
						0x00,	/* Receive 0x0A */
						0x00,	/* Receive 0x0B */
						0x00,	/* Receive 0x0C */
						0x00,	/* Receive 0x0D */
						0x00,	/* Receive 0x0E */
						0x00,	/* Receive 0x0F */
					};
					status = spiTransactionIS25xP(ops, sizeof(ops)/sizeof(uint8_t) /* opCount */);
					if (status != kWarpStatusOK)
					{
						warpPrint("SPI transaction to read Flash ID failed...\n");
					}
					else
					{
						warpPrint("SFDP[0x00] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x00]);
						warpPrint("SFDP[0x01] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x01]);
						warpPrint("SFDP[0x02] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x02]);
						warpPrint("SFDP[0x03] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x03]);
						warpPrint("SFDP[0x04] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x04]);
						warpPrint("SFDP[0x05] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x05]);
						warpPrint("SFDP[0x06] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x06]);
						warpPrint("SFDP[0x07] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x07]);
						warpPrint("SFDP[0x08] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x08]);
						warpPrint("SFDP[0x09] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x09]);
						warpPrint("SFDP[0x0A] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x0A]);
						warpPrint("SFDP[0x0B] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x0B]);
						warpPrint("SFDP[0x0C] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x0C]);
						warpPrint("SFDP[0x0D] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x0D]);
						warpPrint("SFDP[0x0E] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x0E]);
						warpPrint("SFDP[0x0F] = [0x%X]\n", deviceIS25xPState.spiSinkBuffer[5 + 0x0F]);
					}
					break;
				}

				/*
				 *	Perform a read
				 */
				case '3':
				{
					WarpStatus 	status;
					uint8_t		buf[32] = {0};

					status = readMemoryIS25xP(0, 32, buf);
					if (status != kWarpStatusOK)
					{
						warpPrint("\r\n\tCommunication failed: %d", status);
					}
					else
					{
						warpPrint("\n");
						for (size_t i = 0; i < 32; i++)
						{
							warpPrint("0x%08x:\t%x\n", i, buf[i]);
							OSA_TimeDelay(5);
							
						}
					}

					
					break;
				}

				
				/*
				 *	Write Enable
				 */
				case '4':
				{
					WarpStatus 	status;
					uint8_t		ops[] = {
						0x06,	/* WREN */
					};					

					status = spiTransactionIS25xP(ops, 1);
					if (status != kWarpStatusOK)
					{
						warpPrint("\r\n\tCommunication failed: %d", status);
					}
					else
					{
						warpPrint("OK.\n");
					}
					
					break;
				}


				/*
				 *	Perform a write
				 */
				case '5':
				{
					WarpStatus 	status;
					uint8_t		buf[32] = {0};
					
					for (size_t i = 0; i < 32; i++)
					{
						buf[i] = i;
					}
					

					status = programPageIS25xP(0, 32, buf);
					if (status != kWarpStatusOK)
					{
						warpPrint("\r\n\tCommunication failed: %d", status);
					}
					else
					{
						warpPrint("OK.\n");
					}

					
					break;
				}


				/*
				 *	Erase chip (reset to 0xFF)
				 */
				case '6':
				{
					WarpStatus 	status;			

					status = chipEraseIS25xP();
					if (status != kWarpStatusOK)
					{
						warpPrint("\r\n\tCommunication failed: %d", status);
					}
					else
					{
						warpPrint("OK.\n");
					}

					break;
				}
				default:
				{
					warpPrint("\r\n\tInvalid selection.");
					break;

				}
				}
				// warpPrint("\r\n\tStart address (e.g., '0000')> ");
				// //xx = read4digits();

				// warpPrint("\r\n\tNumber of bytes to read from console (e.g., '0000')> ");
				// //xx = read4digits();

				// warpPrint("\r\n\tEnter [%d] raw bytes > ");

				break;
			}
			#endif

			/*
			 *	Use data from Flash to program FPGA
			 */
			/*
			case 'P':
			{
				warpPrint("\r\n\tStart address (e.g., '0000')> ");
				//xx = read4digits();

				warpPrint("\r\n\tNumber of bytes to use (e.g., '0000')> ");
				//xx = read4digits();

				break;
			}
			*/

			/*
			 *	Ignore naked returns.
			 */
			/*
			case '\n':
			{
				warpPrint("\r\tPayloads make rockets more than just fireworks.");
				break;
			}
			*/
			default:
			{
				warpPrint("\r\tInvalid\n");
			}
		}
	}

	return 0;
}



void
loopForSensor(	const char *  tagString,
		WarpStatus  (* readSensorRegisterFunction)(uint8_t deviceRegister, int numberOfBytes),
		volatile WarpI2CDeviceState *  i2cDeviceState,
		volatile WarpSPIDeviceState *  spiDeviceState,
		uint8_t  baseAddress,
		uint8_t  minAddress,
		uint8_t  maxAddress,
		int  repetitionsPerAddress,
		int  chunkReadsPerAddress,
		int  spinDelay,
		bool  autoIncrement,
		uint16_t  sssupplyMillivolts,
		uint8_t  referenceByte,
		uint16_t adaptiveSssupplyMaxMillivolts,
		bool  chatty
		)
{
	WarpStatus		status;
	uint8_t			address = min(minAddress, baseAddress);
	int			readCount = repetitionsPerAddress + 1;
	int			nSuccesses = 0;
	int			nFailures = 0;
	int			nCorrects = 0;
	int			nBadCommands = 0;
	uint16_t		actualSssupplyMillivolts = sssupplyMillivolts;


	if (	(!spiDeviceState && !i2cDeviceState) ||
		(spiDeviceState && i2cDeviceState) )
	{
		warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
	}

	warpScaleSupplyVoltage(actualSssupplyMillivolts);
	warpPrint(tagString);

	/*
	 *	Keep on repeating until we are above the maxAddress, or just once if not autoIncrement-ing
	 *	This is checked for at the tail end of the loop.
	 */
	while (true)
	{
		for (int i = 0; i < readCount; i++) for (int j = 0; j < chunkReadsPerAddress; j++)
		{
			status = readSensorRegisterFunction(address+j, 1 /* numberOfBytes */);
			if (status == kWarpStatusOK)
			{
				nSuccesses++;
				if (actualSssupplyMillivolts > sssupplyMillivolts)
				{
					actualSssupplyMillivolts -= 100;
					warpScaleSupplyVoltage(actualSssupplyMillivolts);
				}

				if (spiDeviceState)
				{
					if (referenceByte == spiDeviceState->spiSinkBuffer[2])
					{
						nCorrects++;
					}

					if (chatty)
					{
						warpPrint("\r\t0x%02x --> [0x%02x 0x%02x 0x%02x]\n",
							address+j,
							spiDeviceState->spiSinkBuffer[0],
							spiDeviceState->spiSinkBuffer[1],
							spiDeviceState->spiSinkBuffer[2]);
					}
				}
				else
				{
					if (referenceByte == i2cDeviceState->i2cBuffer[0])
					{
						nCorrects++;
					}

					if (chatty)
					{
						warpPrint("\r\t0x%02x --> 0x%02x\n",
							address+j,
							i2cDeviceState->i2cBuffer[0]);
					}
				}
			}
			else if (status == kWarpStatusDeviceCommunicationFailed)
			{
				warpPrint("\r\t0x%02x --> ----\n",
					address+j);

				nFailures++;
				if (actualSssupplyMillivolts < adaptiveSssupplyMaxMillivolts)
				{
					actualSssupplyMillivolts += 100;
					warpScaleSupplyVoltage(actualSssupplyMillivolts);
				}
			}
			else if (status == kWarpStatusBadDeviceCommand)
			{
				nBadCommands++;
			}

			if (spinDelay > 0)
			{
				OSA_TimeDelay(spinDelay);
			}
		}

		if (autoIncrement)
		{
			address++;
		}

		if (address > maxAddress || !autoIncrement)
		{
			/*
			 *	We either iterated over all possible addresses, or were asked to do only
			 *	one address anyway (i.e. don't increment), so we're done.
			 */
			break;
		}
	}

	/*
	 *	We intersperse RTT_printfs with forced delays to allow us to use small
	 *	print buffers even in RUN mode.
	 */
	warpPrint("\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
	OSA_TimeDelay(50);
	warpPrint("\r\t%d/%d successes matched ref. value of 0x%02x.\n", nCorrects, nSuccesses, referenceByte);
	OSA_TimeDelay(50);
	warpPrint("\r\t%d bad commands.\n\n", nBadCommands);
	OSA_TimeDelay(50);


	return;
}



void
repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress, bool autoIncrement, int chunkReadsPerAddress, bool chatty, int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts, uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte)
{
	switch (warpSensorDevice)
	{
		//case kWarpSensorADXL362:
		//{
		//	/*
		//	 *	ADXL362: VDD 1.6--3.5
		//	 */
		//	#if (WARP_BUILD_ENABLE_DEVADXL362)
		//		loopForSensor(	"\r\nADXL362:\n\r",		/*	tagString			*/
		//				&readSensorRegisterADXL362,	/*	readSensorRegisterFunction	*/
		//				NULL,				/*	i2cDeviceState			*/
		//				&deviceADXL362State,		/*	spiDeviceState			*/
		//				baseAddress,			/*	baseAddress			*/
		//				0x00,				/*	minAddress			*/
		//				0x2E,				/*	maxAddress			*/
		//				repetitionsPerAddress,		/*	repetitionsPerAddress		*/
		//				chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
		//				spinDelay,			/*	spinDelay			*/
		//				autoIncrement,			/*	autoIncrement			*/
		//				sssupplyMillivolts,		/*	sssupplyMillivolts		*/
		//				referenceByte,			/*	referenceByte			*/
		//				adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
		//				chatty				/*	chatty				*/
		//				);
		//	//#else
		//	//	warpPrint("\r\n\tADXL362 Read Aborted. Device Disabled :(");
		//	#endif
//
		//	break;
		//}

		case kWarpSensorMMA8451Q:
		{
			/*
			 *	MMA8451Q: VDD 1.95--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVMMA8451Q) // HERE IS WHERE WE NEED TO OPERATE
			//	loopForSensor(	"\r\nMMA8451Q:\n\r",		/*	tagString			*/
			//			&readSensorRegisterMMA8451Q,	/*	readSensorRegisterFunction	*/
			//			&deviceMMA8451QState,		/*	i2cDeviceState			*/
			//			NULL,				/*	spiDeviceState			*/
			//			baseAddress,			/*	baseAddress			*/
			//			0x00,				/*	minAddress			*/
			//			0x31,				/*	maxAddress			*/
			//			repetitionsPerAddress,		/*	repetitionsPerAddress		*/
			//			chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
			//			spinDelay,			/*	spinDelay			*/
			//			autoIncrement,			/*	autoIncrement			*/
			//			sssupplyMillivolts,		/*	sssupplyMillivolts		*/
			//			referenceByte,			/*	referenceByte			*/
			//			adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
			//			chatty				/*	chatty				*/
			//			);
			int	key;
			WarpStatus status;
			//uint8_t			address = baseAddress;
			uint16_t		actualSssupplyMillivolts = sssupplyMillivolts;
			//if (!&deviceMMA8451QState)
			//{
			//	warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
			//}
			warpScaleSupplyVoltage(actualSssupplyMillivolts);

			warpPrint("Read a single register? (y/n) > ");
			key = warpWaitKey();
			switch (key) {
				case 'y':
				{
					warpPrint("\r\nMMA8451Q:\n");
					warpPrint("\r\n:\n");

					for (int i = 0; i < 501; i++)
					{
						uint16_t regVal = 0;
						status = readSensorRegisterMMA8451Q(baseAddress, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							regVal = deviceMMA8451QState.i2cBuffer[0];
							warpPrint("\r\t0x%02x --> 0x%02x\n", baseAddress, regVal);
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							baseAddress);
						}
					}
				}

				case 'n':
				{
					//warpPrint("\r\nMMA8451Q:\n");
					//warpPrint("\r\n:\n");
					warpPrint("Acc_X,Acc_Y,Acc_Z\n");
					for (int i = 0; i < 501; i++)
					{
						int16_t acc_x, acc_y, acc_z;
						uint16_t msb = 0;
						uint16_t lsb = 0;
						// MSB X
						status = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_MSB, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							msb = deviceMMA8451QState.i2cBuffer[0];
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							kWarpSensorOutputRegisterMMA8451QOUT_X_MSB);
						}
						// LSB X
						status = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_X_LSB, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							lsb = deviceMMA8451QState.i2cBuffer[0];
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							kWarpSensorOutputRegisterMMA8451QOUT_X_LSB);
						}

						acc_x = ((msb & 0xFF) << 6) | (lsb >> 2);
						// sign extend the 14-bit value
						acc_x = (acc_x ^ (1 << 13)) - (1 << 13);

						// MSB Y
						status = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							msb = deviceMMA8451QState.i2cBuffer[0];
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							kWarpSensorOutputRegisterMMA8451QOUT_Y_MSB);
						}
						// LSB Y
						status = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Y_LSB, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							lsb = deviceMMA8451QState.i2cBuffer[0];
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							kWarpSensorOutputRegisterMMA8451QOUT_Y_LSB);
						}

						acc_y = ((msb & 0xFF) << 6) | (lsb >> 2);
						// sign extend the 14-bit value
						acc_y = (acc_y ^ (1 << 13)) - (1 << 13);


						// MSB Z
						status = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							msb = deviceMMA8451QState.i2cBuffer[0];
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							kWarpSensorOutputRegisterMMA8451QOUT_Z_MSB);
						};
						// LSB Y
						status = readSensorRegisterMMA8451Q(kWarpSensorOutputRegisterMMA8451QOUT_Z_LSB, chunkReadsPerAddress);
						if (status == kWarpStatusOK)
						{
							lsb = deviceMMA8451QState.i2cBuffer[0];
						}
						else if (status == kWarpStatusDeviceCommunicationFailed)
						{
							warpPrint("\r\t0x%02x --> ----\n",
							kWarpSensorOutputRegisterMMA8451QOUT_Z_LSB);
						}

						acc_z = ((msb & 0xFF) << 6) | (lsb >> 2);
						// sign extend the 14-bit value
						acc_z = (acc_z ^ (1 << 13)) - (1 << 13);

						// NOW THAT YOU'VE GOT THE VALUES, YOU NEED TO DO THE PRINTING
						warpPrint("%d, %d, %d\n", acc_x, acc_y, acc_z);
					}
				}	
			}	

			// TODO: MAKE IT SO THAT YOU CAN READ THE VALUES OF DIFFERENT REGISTERS OR READ ACCEL DATA
			
			#else
				warpPrint("\r\n\tMMA8451Q Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorBME680:
		{
			/*
			 *	BME680: VDD 1.7--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVBME680)
				loopForSensor(	"\r\nBME680:\n\r",		/*	tagString			*/
						&readSensorRegisterBME680,	/*	readSensorRegisterFunction	*/
						&deviceBME680State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x1D,				/*	minAddress			*/
						0x75,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\nBME680 Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		case kWarpSensorBMX055accel:
		{
			/*
			 *	BMX055accel: VDD 2.4V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVBMX055)
				loopForSensor(	"\r\nBMX055accel:\n\r",		/*	tagString			*/
						&readSensorRegisterBMX055accel,	/*	readSensorRegisterFunction	*/
						&deviceBMX055accelState,	/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x39,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tBMX055accel Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorBMX055gyro:
		{
			/*
			 *	BMX055gyro: VDD 2.4V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVBMX055)
				loopForSensor(	"\r\nBMX055gyro:\n\r",		/*	tagString			*/
						&readSensorRegisterBMX055gyro,	/*	readSensorRegisterFunction	*/
						&deviceBMX055gyroState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x39,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tBMX055gyro Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorBMX055mag:
		{
			/*
			 *	BMX055mag: VDD 2.4V -- 3.6V
			 */
			#if WARP_BUILD_ENABLE_DEVBMX055
				loopForSensor(	"\r\nBMX055mag:\n\r",		/*	tagString			*/
						&readSensorRegisterBMX055mag,	/*	readSensorRegisterFunction	*/
						&deviceBMX055magState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x40,				/*	minAddress			*/
						0x52,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\t BMX055mag Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorMAG3110:
		{
			/*
			 *	MAG3110: VDD 1.95 -- 3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVMAG3110)
				loopForSensor(	"\r\nMAG3110:\n\r",		/*	tagString			*/
						&readSensorRegisterMAG3110,	/*	readSensorRegisterFunction	*/
						&deviceMAG3110State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x11,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tMAG3110 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorL3GD20H:
		{
			/*
			 *	L3GD20H: VDD 2.2V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVL3GD20H)
				loopForSensor(	"\r\nL3GD20H:\n\r",		/*	tagString			*/
						&readSensorRegisterL3GD20H,	/*	readSensorRegisterFunction	*/
						&deviceL3GD20HState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x0F,				/*	minAddress			*/
						0x39,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tL3GD20H Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorLPS25H:
		{
			/*
			 *	LPS25H: VDD 1.7V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVLPS25H)
				loopForSensor(	"\r\nLPS25H:\n\r",		/*	tagString			*/
						&readSensorRegisterLPS25H,	/*	readSensorRegisterFunction	*/
						&deviceLPS25HState,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x08,				/*	minAddress			*/
						0x24,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tLPS25H Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorTCS34725:
		{
			/*
			 *	TCS34725: VDD 2.7V -- 3.3V
			 */
			#if WARP_BUILD_ENABLE_DEVTCS34725
				loopForSensor(	"\r\nTCS34725:\n\r",		/*	tagString			*/
						&readSensorRegisterTCS34725,	/*	readSensorRegisterFunction	*/
						&deviceTCS34725State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x1D,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tTCS34725 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorSI4705:
		{
			/*
			 *	SI4705: VDD 2.7V -- 5.5V
			 */
			#if (WARP_BUILD_ENABLE_DEVSI4705)
				loopForSensor(	"\r\nSI4705:\n\r",		/*	tagString			*/
						&readSensorRegisterSI4705,	/*	readSensorRegisterFunction	*/
						&deviceSI4705State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x09,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tSI4705 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorHDC1000:
		{
			/*
			 *	HDC1000: VDD 3V--5V
			 */
			#if (WARP_BUILD_ENABLE_DEVHDC1000)
				loopForSensor(	"\r\nHDC1000:\n\r",		/*	tagString			*/
						&readSensorRegisterHDC1000,	/*	readSensorRegisterFunction	*/
						&deviceHDC1000State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x1F,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tHDC1000 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorSI7021:
		{
			/*
			 *	SI7021: VDD 1.9V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVSI7021)
				loopForSensor(	"\r\nSI7021:\n\r",		/*	tagString			*/
						&readSensorRegisterSI7021,	/*	readSensorRegisterFunction	*/
						&deviceSI7021State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x09,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tSI7021 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorCCS811:
		{
			/*
			 *	CCS811: VDD 1.8V -- 3.6V
			 */
			#if (WARP_BUILD_ENABLE_DEVCCS811)
				loopForSensor(	"\r\nCCS811:\n\r",		/*	tagString			*/
						&readSensorRegisterCCS811,	/*	readSensorRegisterFunction	*/
						&deviceCCS811State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0xFF,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tCCS811 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorAMG8834:
		{
			/*
			 *	AMG8834: VDD 3.3V -- 3.3V
			 */
			#if WARP_BUILD_ENABLE_DEVAMG8834
				loopForSensor(	"\r\nAMG8834:\n\r",		/*	tagString			*/
						&readSensorRegisterAMG8834,	/*	readSensorRegisterFunction	*/
						&deviceAMG8834State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0xFF,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tAMG8834 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorAS7262:
		{
			/*
			 *	AS7262: VDD 2.7--3.6
			 */
			#if (WARP_BUILD_ENABLE_DEVAS7262)
				loopForSensor(	"\r\nAS7262:\n\r",		/*	tagString			*/
						&readSensorRegisterAS7262,	/*	readSensorRegisterFunction	*/
						&deviceAS7262State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x2B,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tAS7262 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorAS7263:
		{
			/*
			 *	AS7263: VDD 2.7--3.6
			 */
			#if WARP_BUILD_ENABLE_DEVAS7263
				loopForSensor(	"\r\nAS7263:\n\r",		/*	tagString			*/
						&readSensorRegisterAS7263,	/*	readSensorRegisterFunction	*/
						&deviceAS7263State,		/*	i2cDeviceState			*/
						NULL,				/*	spiDeviceState			*/
						baseAddress,			/*	baseAddress			*/
						0x00,				/*	minAddress			*/
						0x2B,				/*	maxAddress			*/
						repetitionsPerAddress,		/*	repetitionsPerAddress		*/
						chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
						spinDelay,			/*	spinDelay			*/
						autoIncrement,			/*	autoIncrement			*/
						sssupplyMillivolts,		/*	sssupplyMillivolts		*/
						referenceByte,			/*	referenceByte			*/
						adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
						chatty				/*	chatty				*/
						);
			#else
				warpPrint("\r\n\tAS7263 Read Aborted. Device Disabled :( ");
			#endif

			break;
		}

		case kWarpSensorINA219:
		{
			/*
			 *	INA219: VDD 3.00--5.5
			 */
			#if (WARP_BUILD_ENABLE_DEVINA219)
				WarpStatus		status;
				uint8_t			address = baseAddress;
				uint16_t		actualSssupplyMillivolts = sssupplyMillivolts;
				int			readCount = repetitionsPerAddress + 1;
				int			nSuccesses = 0;
				int			nFailures = 0;
				int			nBadCommands = 0;
				//if (!&deviceINA219State)
				//{
				//	warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_YELLOW RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorSanity RTT_CTRL_RESET "\n");
				//}
				warpScaleSupplyVoltage(actualSssupplyMillivolts);
				warpPrint("\r\nINA219:\n");
				for (int i = 0; i < readCount; i++)
				{
					status = readSensorRegisterINA219(address, 2 /* numberOfBytes */);
					if (status == kWarpStatusOK) {
						nSuccesses++;
						if (address == 0)
						{
							warpPrint("\r\t0x%02x --> 0x%02x%02x\n",
							address,
							deviceINA219State.i2cBuffer[0],
							deviceINA219State.i2cBuffer[1]);
						}
						else
						{
							warpPrint("\r\t0x%02x --> %d\n",
							address,
							((deviceINA219State.i2cBuffer[0] & 0xFF) << 8)  |
							(deviceINA219State.i2cBuffer[1] & 0xFF));
						}
					}
					else if (status == kWarpStatusDeviceCommunicationFailed)
					{
						warpPrint("\r\t0x%02x --> ----\n",
						address);
						nFailures++;
					}
					else if (status == kWarpStatusBadDeviceCommand)
					{
						nBadCommands++;
					}
				}
				warpPrint("\r\n\t%d/%d success rate.\n", nSuccesses, (nSuccesses + nFailures));
				OSA_TimeDelay(50);
				warpPrint("\r\t%d bad commands.\n\n", nBadCommands);
				OSA_TimeDelay(50);

				//loopForSensor(	"\r\nINA219:\n\r",		/*	tagString			*/
				//		&readSensorRegisterINA219,	/*	readSensorRegisterFunction	*/
				//		&deviceINA219State,		/*	i2cDeviceState			*/
				//		NULL,				/*	spiDeviceState			*/
				//		baseAddress,			/*	baseAddress			*/
				//		0x00,				/*	minAddress			*/
				//		0x05,				/*	maxAddress			*/
				//		repetitionsPerAddress,		/*	repetitionsPerAddress		*/
				//		chunkReadsPerAddress,		/*	chunkReadsPerAddress		*/
				//		spinDelay,			/*	spinDelay			*/
				//		autoIncrement,			/*	autoIncrement			*/
				//		sssupplyMillivolts,		/*	sssupplyMillivolts		*/
				//		referenceByte,			/*	referenceByte			*/
				//		adaptiveSssupplyMaxMillivolts,	/*	adaptiveSssupplyMaxMillivolts	*/
				//		chatty				/*	chatty				*/
				//		);
			#else
				warpPrint("\r\n\tINA219 Read Aborted. Device Disabled :(");
			#endif

			break;
		}

		default:
		{
			warpPrint("\r\tInvalid warpSensorDevice [%d] passed to repeatRegisterReadForDeviceAndAddress.\n", warpSensorDevice);
		}
	}

	if (warpSensorDevice != kWarpSensorADXL362)
	{
		warpDisableI2Cpins();
	}
}



int
char2int(int character)
{
	if (character >= '0' && character <= '9')
	{
		return character - '0';
	}

	if (character >= 'a' && character <= 'f')
	{
		return character - 'a' + 10;
	}

	if (character >= 'A' && character <= 'F')
	{
		return character - 'A' + 10;
	}

	return 0;
}



uint8_t
readHexByte(void)
{
	uint8_t		topNybble, bottomNybble;

	topNybble = warpWaitKey();
	bottomNybble = warpWaitKey();

	return (char2int(topNybble) << 4) + char2int(bottomNybble);
}



int
read4digits(void)
{
	uint8_t		digit1, digit2, digit3, digit4;

	digit1 = warpWaitKey();
	digit2 = warpWaitKey();
	digit3 = warpWaitKey();
	digit4 = warpWaitKey();

	return (digit1 - '0')*1000 + (digit2 - '0')*100 + (digit3 - '0')*10 + (digit4 - '0');
}



WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
						gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}



WarpStatus
writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength)
{
	uint8_t		inBuffer[payloadLength];
	spi_status_t	status;

	warpEnableSPIpins();
	status = SPI_DRV_MasterTransferBlocking(0					/* master instance */,
						NULL					/* spi_master_user_config_t */,
						payloadBytes,
						inBuffer,
						payloadLength				/* transfer size */,
						gWarpSpiTimeoutMicroseconds		/* timeout in microseconds (unlike I2C which is ms) */);
	warpDisableSPIpins();

	return (status == kStatus_SPI_Success ? kWarpStatusOK : kWarpStatusCommsError);
}



void
powerupAllSensors(void)
{
	/*
	 *	BMX055mag
	 *
	 *	Write '1' to power control bit of register 0x4B. See page 134.
	 */
	#if (WARP_BUILD_ENABLE_DEVBMX055)
		WarpStatus	status = writeByteToI2cDeviceRegister(	deviceBMX055magState.i2cAddress		/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x4B					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 0)				/*	payloadByte		*/);
		if (status != kWarpStatusOK)
		{
			warpPrint("\r\tPowerup command failed, code=%d, for BMX055mag @ 0x%02x.\n", status, deviceBMX055magState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerup command failed. BMX055 disabled \n");
	#endif
}



void
activateAllLowPowerSensorModes(bool verbose)
{
	/*
	 *	ADXL362:	See Power Control Register (Address: 0x2D, Reset: 0x00).
	 *
	 *	POR values are OK.
	 */

	/*
	 *	IS25XP:	Put in powerdown momde
	 */
	#if (WARP_BUILD_ENABLE_DEVIS25xP)
		/*
		 *	Put the Flash in deep power-down
		 */
		//TODO: move 0xB9 into a named constant
		//spiTransactionIS25xP({0xB9 /* op0 */,  0x00 /* op1 */,  0x00 /* op2 */, 0x00 /* op3 */, 0x00 /* op4 */, 0x00 /* op5 */, 0x00 /* op6 */}, 1 /* opCount */);
	#endif

	/*
	 *	BMX055accel: At POR, device is in Normal mode. Move it to Deep Suspend mode.
	 *
	 *	Write '1' to deep suspend bit of register 0x11, and write '0' to suspend bit of register 0x11. See page 23.
	 */
	#if WARP_BUILD_ENABLE_DEVBMX055
		WarpStatus	status = writeByteToI2cDeviceRegister(	deviceBMX055accelState.i2cAddress	/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x11					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 5)				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for BMX055accel @ 0x%02x.\n", status, deviceBMX055accelState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. BMX055 disabled\n");
	#endif

	/*
	 *	BMX055gyro: At POR, device is in Normal mode. Move it to Deep Suspend mode.
	 *
	 *	Write '1' to deep suspend bit of register 0x11. See page 81.
	 */
	#if (WARP_BUILD_ENABLE_DEVBMX055)
		status = writeByteToI2cDeviceRegister(	deviceBMX055gyroState.i2cAddress	/*	i2cAddress		*/,
							true					/*	sendCommandByte		*/,
							0x11					/*	commandByte		*/,
							true					/*	sendPayloadByte		*/,
							(1 << 5)				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for BMX055gyro @ 0x%02x.\n", status, deviceBMX055gyroState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. BMX055 disabled\n");
	#endif



	/*
	 *	BMX055mag: At POR, device is in Suspend mode. See page 121.
	 *
	 *	POR state seems to be powered down.
	 */



	/*
	 *	MMA8451Q: See 0x2B: CTRL_REG2 System Control 2 Register (page 43).
	 *
	 *	POR state seems to be not too bad.
	 */



	/*
	 *	LPS25H: See Register CTRL_REG1, at address 0x20 (page 26).
	 *
	 *	POR state seems to be powered down.
	 */



	/*
	 *	MAG3110: See Register CTRL_REG1 at 0x10. (page 19).
	 *
	 *	POR state seems to be powered down.
	 */



	/*
	 *	HDC1000: currently can't turn it on (3V)
	 */



	/*
	 *	SI7021: Can't talk to it correctly yet.
	 */



	/*
	 *	L3GD20H: See CTRL1 at 0x20 (page 36).
	 *
	 *	POR state seems to be powered down.
	 */
	#if (WARP_BUILD_ENABLE_DEVL3GD20H)
		status = writeByteToI2cDeviceRegister(	deviceL3GD20HState.i2cAddress	/*	i2cAddress		*/,
							true				/*	sendCommandByte		*/,
							0x20				/*	commandByte		*/,
							true				/*	sendPayloadByte		*/,
							0x00				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for L3GD20H @ 0x%02x.\n", status, deviceL3GD20HState.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. L3GD20H disabled\n");
	#endif



	/*
	 *	BME680: TODO
	 */



	/*
	 *	TCS34725: By default, is in the "start" state (see page 9).
	 *
	 *	Make it go to sleep state. See page 17, 18, and 19.
	 */
	#if (WARP_BUILD_ENABLE_DEVTCS34725)
		status = writeByteToI2cDeviceRegister(	deviceTCS34725State.i2cAddress	/*	i2cAddress		*/,
							true				/*	sendCommandByte		*/,
							0x00				/*	commandByte		*/,
							true				/*	sendPayloadByte		*/,
							0x00				/*	payloadByte		*/);
		if ((status != kWarpStatusOK) && verbose)
		{
			warpPrint("\r\tPowerdown command failed, code=%d, for TCS34725 @ 0x%02x.\n", status, deviceTCS34725State.i2cAddress);
		}
	#else
		warpPrint("\r\tPowerdown command abandoned. TCS34725 disabled\n");
	#endif



	/*
	 *	SI4705: Send a POWER_DOWN command (byte 0x17). See AN332 page 124 and page 132.
	 *
	 *	For now, simply hold its reset line low.
	 */
	#if (WARP_BUILD_ENABLE_DEVSI4705)
		GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	#endif
}

int checkActivityPendSwingUpperXFirstHalf(int16_t acc_x, int idx) {
	const int16_t boundarySegment [] =
	{
		2124 , 2898 , 3076 , 5163 , 1695 , 1868 , 1903 , 2108 , 335 , 457 , 1464 , 
		309 , 465 , 355 , 1053 , 456 , 746 , 711 , 842 , 947 , 892 , 
		674 , 602 , 559 , 508 , 564 , 548 , 540 , 534 , 403 , 433 , 
		443 , 456 , 469 , 444 , 443 , 442 , 410 , 364 , 322 , 284 , 
		293 , 300 , 321 , 333 , 367 , 382 , 410 , 426 , 427 , 445 , 
		429 , 430 , 431 , 439 , 437 , 437 , 4700 , 2259 , 2082 , 1366 , 
		734 , 504 , 582 , 603 , 561 , 472 , 438 , 4681 , 2799 , 1215 , 
		1006 , 745 , 4079 , 2330 , 960 , 539 , 444 , 716 , 594 , 548 , 
		633 , 611 , 602 , 608 , 634 , 617 , 621 , 618 , 604 , 590 , 
		582 , 584 , 594 , 603 , 601 , 629 , 632 , 634 , 628 , 612 , 
		609 , 606 , 591 , 591 , 588 , 608 , 619 , 628 , 636 , 633 , 
		642 , 636 , 640 , 618 , 614 , 587 , 593 , 571 , 582 , 571 ,
		555 , 551 , 544 , 539 , 531 , 544 , 543 , 539 , 522 , 544 , 
		527 , 521 , 530 , 524 , 512 , 509 , 502 , 489 , 474 , 469 , 
		474 , 465 , 474 , 476 , 491 , 499 , 505 , 523 , 536 , 548 , 
		572 , 581 , 604 , 602 , 619 , 637 , 622 , 638 , 657 , 659 , 
		670 , 675 , 673 , 671 , 678 , 674 , 677 , 672 , 682 , 673 , 
		674 , 680 , 677 , 680 , 679 , 683 , 674 , 685 , 685 , 684 , 
		690 , 680 , 688 , 682 , 683 , 679 , 672 , 660 , 663 , 646 , 
		636 , 638 , 628 , 616 , 614 , 605 , 586 , 590 , 580 , 565 , 
		565 , 553 , 545 , 552 , 538 , 532 , 514 , 508 , 504 , 498 , 
		506 , 510 , 521 , 505 , 503 , 520 , 523 , 530 , 545 , 556 , 
		571 , 575 , 587 , 594 , 605 , 610 , 624 , 621 , 637 , 651 , 
		649 , 672 , 669 , 672 , 677 , 682 , 692 , 694 , 694 , 700 , 
		685 , 697 , 689 , 695 , 679 , 685 , 686 , 680 , 673 , 669
	};

	if (acc_x >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingUpperXSecondHalf(int16_t acc_x, int idx) {
	const int16_t boundarySegment [] =
	{
		667 , 661 , 665 , 651 , 643 , 637 , 638 , 629 , 636 , 635 , 
		606 , 614 , 611 , 607 , 594 , 584 , 587 , 593 , 579 , 587 , 
		576 , 567 , 562 , 574 , 575 , 575 , 584 , 585 , 597 , 612 , 
		614 , 626 , 625 , 638 , 642 , 647 , 651 , 661 , 667 , 689 , 
		674 , 666 , 684 , 672 , 685 , 695 , 692 , 696 , 688 , 695 , 
		707 , 698 , 708 , 698 , 714 , 715 , 712 , 714 , 693 , 703 , 
		691 , 694 , 688 , 690 , 689 , 683 , 682 , 681 , 660 , 676 , 
		665 , 663 , 644 , 655 , 639 , 640 , 633 , 614 , 622 , 612 , 
		600 , 607 , 608 , 580 , 587 , 575 , 567 , 559 , 561 , 547 , 
		545 , 553 , 552 , 541 , 539 , 544 , 543 , 552 , 543 , 548 , 
		560 , 554 , 554 , 582 , 586 , 581 , 589 , 593 , 606 , 613 , 
		632 , 626 , 624 , 638 , 626 , 647 , 650 , 650 , 651 , 651 , 
		662 , 667 , 655 , 666 , 664 , 664 , 669 , 670 , 656 , 664 , 
		669 , 673 , 666 , 669 , 666 , 661 , 655 , 671 , 658 , 649 , 
		657 , 659 , 650 , 643 , 629 , 638 , 640 , 632 , 626 , 620 , 
		634 , 633 , 644 , 630 , 636 , 636 , 638 , 642 , 649 , 657 , 
		663 , 667 , 652 , 660 , 675 , 682 , 681 , 679 , 682 , 679 , 
		681 , 686 , 685 , 674 , 675 , 677 , 694 , 689 , 693 , 701 , 
		693 , 693 , 696 , 683 , 684 , 694 , 678 , 680 , 682 , 690 , 
		680 , 671 , 663 , 666 , 668 , 657 , 656 , 666 , 659 , 656 , 
		645 , 645 , 642 , 638 , 631 , 635 , 633 , 624 , 618 , 620 , 
	//	609 , 610 , 601 , 601 , 598 , 589 , 599 , 586 , 586 , 587 , 
	//	570 , 578 , 578 , 574 , 586 , 565 , 561 , 569 , 571 , 577 , 
	//	574 , 584 , 595 , 582 , 591 , 592 , 600 , 609 , 617 , 619 , 
	//	622 , 623 , 625 , 637 , 632 , 628 , 654 , 646 , 638 , 644
	};

	if (acc_x >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingLowerXFirstHalf(int16_t acc_x, int idx) {
	const int16_t boundarySegment[] = 
	{
		-4468 , -4486 , -4412 , -5521 , -3197 , -3136 , -3193 , -3100 , -1049 , -903 , -1480 ,
		-651 , -627 , -381 , -851 , -464 , -758 , -501 , -542 , -769 , -704 , 
		-518 , -490 , -569 , -616 , -808 , -848 , -932 , -1018 , -1005 , -1023 , 
		-1045 , -1088 , -1091 , -1044 , -993 , -930 , -838 , -724 , -618 , -532 , 
		-487 , -464 , -435 , -419 , -405 , -382 , -374 , -326 , -317 , -315 , 
		-267 , -254 , -249 , -249 , -219 , -195 , -3372 , -3001 , -2758 , -1654 , 
		-666 , -344 , -482 , -493 , -443 , -296 , -254 , -3415 , -3457 , -1405 , 
		-1158 , -835 , -3037 , -2954 , -1148 , -389 , -444 , -792 , -586 , -468 , 
		-535 , -473 , -398 , -368 , -330 , -259 , -219 , -158 , -88 , -26 , 
		30 , 72 , 78 , 95 , 113 , 101 , 100 , 102 , 96 , 84 , 
		65 , 62 , 63 , 59 , 56 , 36 , 35 , 28 , 20 , 17 , 
		-6 , 0 , -16 , -18 , -22 , -13 , -23 , -17 , -30 , -25
		-17 , -29 , -24 , -21 , -13 , -28 , -29 , -25 , -22 , -36 , 
		-33 , -23 , -30 , -28 , -16 , -23 , -14 , -7 , 14 , 13 , 
		14 , 25 , 26 , 32 , 39 , 47 , 53 , 55 , 48 , 60 , 
		52 , 73 , 64 , 98 , 107 , 113 , 130 , 146 , 141 , 159 , 
		166 , 175 , 189 , 195 , 206 , 222 , 221 , 232 , 230 , 245 , 
		238 , 252 , 245 , 240 , 255 , 247 , 258 , 257 , 253 , 256 , 
		250 , 268 , 260 , 258 , 255 , 259 , 260 , 268 , 255 , 266 , 
		256 , 262 , 248 , 244 , 230 , 233 , 230 , 226 , 236 , 221 , 
		217 , 213 , 213 , 200 , 198 , 184 , 202 , 188 , 192 , 178 , 
		182 , 170 , 161 , 169 , 163 , 160 , 171 , 170 , 165 , 164 , 
		163 , 175 , 183 , 178 , 185 , 198 , 200 , 205 , 209 , 203 , 
		217 , 204 , 209 , 204 , 197 , 198 , 188 , 166 , 178 , 160 , 
		165 , 161 , 157 , 143 , 143 , 133 , 130 , 124 , 121 , 113
	};

	if (acc_x <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingLowerXSecondHalf(int16_t acc_x, int idx) {
	const int16_t boundarySegment[] = 
	{
		103 , 105 , 93 , 99 , 91 , 101 , 86 , 89 , 80 , 79 , 
		94 , 86 , 87 , 95 , 94 , 96 , 95 , 101 , 111 , 111 , 
		120 , 127 , 130 , 150 , 143 , 159 , 156 , 161 , 157 , 160 , 
		170 , 166 , 181 , 174 , 194 , 203 , 207 , 213 , 223 , 225 , 
		250 , 254 , 260 , 272 , 273 , 275 , 288 , 288 , 304 , 311 , 
		303 , 314 , 316 , 318 , 322 , 315 , 324 , 326 , 337 , 335 , 
		343 , 334 , 340 , 338 , 337 , 343 , 334 , 329 , 344 , 328 , 
		325 , 335 , 336 , 315 , 319 , 308 , 313 , 310 , 302 , 304 , 
		296 , 295 , 272 , 280 , 287 , 267 , 279 , 267 , 261 , 255 , 
		261 , 245 , 244 , 245 , 231 , 240 , 239 , 236 , 243 , 228 , 
		224 , 242 , 230 , 214 , 210 , 209 , 217 , 217 , 210 , 213 , 
		204 , 210 , 216 , 210 , 222 , 207 , 202 , 210 , 211 , 207 , 
		198 , 203 , 203 , 202 , 200 , 196 , 189 , 182 , 188 , 180 , 
		181 , 173 , 178 , 173 , 174 , 177 , 179 , 171 , 170 , 185 , 
		181 , 167 , 174 , 183 , 197 , 190 , 196 , 204 , 206 , 220 , 
		206 , 213 , 216 , 230 , 240 , 240 , 246 , 246 , 245 , 257 , 
		259 , 255 , 276 , 280 , 287 , 282 , 289 , 295 , 306 , 311 , 
		313 , 310 , 325 , 338 , 343 , 349 , 334 , 337 , 329 , 329 , 
		341 , 337 , 344 , 351 , 352 , 342 , 354 , 356 , 346 , 342 , 
		348 , 355 , 355 , 354 , 344 , 353 , 344 , 338 , 343 , 336 , 
		337 , 333 , 322 , 314 , 327 , 311 , 297 , 308 , 310 , 300 , 
	//	301 , 294 , 301 , 281 , 282 , 285 , 275 , 270 , 270 , 259 , 
	//	266 , 254 , 266 , 254 , 250 , 245 , 261 , 245 , 247 , 241 , 
	//	238 , 232 , 227 , 238 , 231 , 232 , 228 , 221 , 217 , 223 , 
	//	230 , 231 , 225 , 221 , 220 , 228 , 226 , 226 , 238 , 228
	};

	if (acc_x <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingUpperYFirstHalf(int16_t acc_y, int idx) {
	const int16_t boundarySegment [] =
	{
		8119 , 4168 , 2773 , 2929 , 3103 , 3876 , 5803 , 5668 , 3228 , 2128 , 2372 , 
		2880 , 4822 , 4689 , 4753 , 5220 , 5857 , 6906 , 8380 , 9813 , 10880 , 
		11866 , 11781 , 11617 , 11662 , 11812 , 11553 , 11072 , 10429 , 10044 , 9773 , 
		9356 , 8942 , 8801 , 8683 , 8487 , 8390 , 8517 , 8506 , 8683 , 8815 , 
		8791 , 8614 , 8461 , 8518 , 8594 , 8656 , 8720 , 8836 , 8944 , 8986 , 
		9069 , 9137 , 9228 , 9322 , 9317 , 9243 , 9394 , 10527 , 8871 , 9168 , 
		8522 , 8371 , 8002 , 7678 , 7273 , 7031 , 6700 , 7012 , 5300 , 5564 , 
		5044 , 5025 , 4649 , 4145 , 3793 , 3721 , 3558 , 3619 , 3755 , 4006 , 
		4314 , 4740 , 5152 , 5638 , 6136 , 6636 , 7140 , 7637 , 8087 , 8459 , 
		8817 , 8995 , 9101 , 9104 , 8988 , 8823 , 8626 , 8562 , 8627 , 8776 , 
		8745 , 8706 , 8580 , 8382 , 8241 , 7992 , 7941 , 7933 , 7848 , 7885 , 
		7952 , 8025 , 8137 , 8274 , 8360 , 8460 , 8483 , 8444 , 8378 , 8238 , 
		8078 , 7910 , 7717 , 7540 , 7338 , 7190 , 7005 , 6802 , 6604 , 6400 , 
		6194 , 5956 , 5734 , 5475 , 5253 , 4996 , 4736 , 4474 , 4260 , 3971 , 
		3788 , 3586 , 3424 , 3302 , 3267 , 3312 , 3431 , 3586 , 3756 , 3966 , 
		4122 , 4333 , 4526 , 4731 , 4917 , 5111 , 5287 , 5489 , 5649 , 5833 , 
		5963 , 6078 , 6189 , 6269 , 6310 , 6354 , 6396 , 6400 , 6390 , 6372 , 
		6362 , 6317 , 6294 , 6264 , 6243 , 6213 , 6234 , 6240 , 6282 , 6325 , 
		6367 , 6397 , 6468 , 6529 , 6572 , 6573 , 6591 , 6602 , 6539 , 6520 , 
		6458 , 6399 , 6286 , 6172 , 6084 , 5969 , 5841 , 5708 , 5545 , 5382 , 
		5244 , 5091 , 4935 , 4767 , 4600 , 4441 , 4323 , 4190 , 4097 , 3991 , 
		3963 , 3965 , 4007 , 4078 , 4155 , 4270 , 4374 , 4483 , 4613 , 4737 , 
		4847 , 4947 , 5036 , 5142 , 5233 , 5317 , 5382 , 5439 , 5492 , 5521 , 
		5549 , 5558 , 5571 , 5549 , 5528 , 5521 , 5511 , 5520 , 5476 , 5471 , 
		5472 , 5491 , 5526 , 5558 , 5600 , 5626 , 5664 , 5713 , 5733 , 5754
	};

	if (acc_y >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingUpperYSecondHalf(int16_t acc_y, int idx) {
	const int16_t boundarySegment [] =
	{
		5753 , 5759 , 5749 , 5737 , 5706 , 5687 , 5616 , 5580 , 5519 , 5452 , 
		5371 , 5283 , 5183 , 5099 , 5008 , 4888 , 4777 , 4645 , 4537 , 4406 , 
		4304 , 4208 , 4094 , 3993 , 3890 , 3807 , 3763 , 3729 , 3736 , 3754 , 
		3815 , 3878 , 3951 , 4007 , 4107 , 4193 , 4251 , 4343 , 4397 , 4497 , 
		4560 , 4624 , 4689 , 4758 , 4782 , 4833 , 4857 , 4880 , 4902 , 4925 , 
		4935 , 4930 , 4958 , 4926 , 4908 , 4921 , 4880 , 4883 , 4867 , 4861 , 
		4900 , 4909 , 4936 , 4974 , 4982 , 5021 , 5037 , 5050 , 5051 , 5056 , 
		5052 , 5041 , 5008 , 4983 , 4958 , 4923 , 4855 , 4836 , 4764 , 4711 , 
		4656 , 4584 , 4498 , 4444 , 4366 , 4298 , 4208 , 4162 , 4090 , 4040 , 
		4020 , 3995 , 3991 , 4011 , 4029 , 4057 , 4104 , 4145 , 4190 , 4225 , 
		4269 , 4311 , 4354 , 4392 , 4436 , 4479 , 4488 , 4526 , 4543 , 4586 , 
		4585 , 4593 , 4597 , 4605 , 4602 , 4591 , 4587 , 4577 , 4557 , 4568 , 
		4542 , 4538 , 4548 , 4566 , 4568 , 4609 , 4632 , 4660 , 4685 , 4698 , 
		4713 , 4740 , 4727 , 4729 , 4717 , 4730 , 4706 , 4692 , 4667 , 4630 , 
		4609 , 4561 , 4530 , 4485 , 4444 , 4382 , 4333 , 4267 , 4221 , 4153 , 
		4100 , 4058 , 4016 , 3971 , 3931 , 3893 , 3872 , 3846 , 3838 , 3845 , 
		3864 , 3898 , 3908 , 3943 , 3982 , 4002 , 4033 , 4061 , 4095 , 4133 , 
		4156 , 4181 , 4227 , 4228 , 4260 , 4286 , 4299 , 4303 , 4333 , 4335 , 
		4351 , 4350 , 4364 , 4348 , 4337 , 4342 , 4345 , 4327 , 4329 , 4335 , 
		4323 , 4329 , 4342 , 4363 , 4368 , 4383 , 4390 , 4392 , 4398 , 4386 , 
		4398 , 4380 , 4383 , 4367 , 4346 , 4321 , 4298 , 4284 , 4247 , 4206 , 
	//	4177 , 4154 , 4128 , 4093 , 4067 , 4039 , 4026 , 3995 , 3978 , 3972 , 
	//	3959 , 3950 , 3958 , 3973 , 3969 , 3990 , 4013 , 4028 , 4045 , 4057 , 
	//	4087 , 4096 , 4109 , 4121 , 4118 , 4147 , 4158 , 4164 , 4173 , 4176 , 
	//	4182 , 4172 , 4189 , 4183 , 4178 , 4179 , 4162 , 4173 , 4154 , 4167
	};

	if (acc_y >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}


int checkActivityPendSwingLowerYFirstHalf(int16_t acc_y, int idx) {
	const int16_t boundarySegment[] = 
	{
		-4433 , -2476 , -2611 , -4855 , -4261 , -4504 , -4953 , -4864 , -3692 , -3516 , -4320 , 
		-3952 , -4782 , -4467 , -4347 , -4300 , -3923 , -3674 , -3620 , -3439 , -2896 , 
		-2358 , -1343 , -295 , 694 , 1612 , 2425 , 3384 , 4365 , 5112 , 5609 , 
		5992 , 6494 , 6933 , 7235 , 7663 , 7834 , 7661 , 7646 , 7479 , 7371 , 
		7375 , 7598 , 7789 , 7762 , 7662 , 7552 , 7440 , 7220 , 6956 , 6718 , 
		6389 , 6081 , 5756 , 5422 , 5069 , 4783 , 4446 , 2051 , 3663 , 2436 , 
		2798 , 2555 , 2310 , 1746 , 1509 , 1111 , 996 , 372 , 824 , 584 , 
		592 , 341 , 365 , 429 , 441 , 145 , 374 , 367 , 203 , 130 , 
		-30 , -228 , -388 , -502 , -580 , -588 , -564 , -435 , -225 , 87 , 
		469 , 987 , 1549 , 2216 , 2992 , 3767 , 4482 , 5054 , 5439 , 5652 , 
		5885 , 6026 , 6180 , 6234 , 6157 , 6292 , 6153 , 6049 , 5932 , 5821 , 
		5760 , 5621 , 5453 , 5342 , 5172 , 4964 , 4767 , 4548 , 4334 , 4146 , 
		3954 , 3762 , 3581 , 3416 , 3226 , 3050 , 2873 , 2730 , 2592 , 2464 , 
		2314 , 2208 , 2110 , 2043 , 1969 , 1920 , 1900 , 1918 , 1920 , 1967 , 
		2004 , 2050 , 2108 , 2146 , 2159 , 2100 , 2019 , 1918 , 1812 , 1718 , 
		1686 , 1637 , 1646 , 1691 , 1777 , 1871 , 1991 , 2097 , 2229 , 2381 , 
		2555 , 2766 , 2969 , 3197 , 3430 , 3662 , 3900 , 4124 , 4350 , 4576 , 
		4750 , 4945 , 5078 , 5220 , 5307 , 5381 , 5398 , 5396 , 5370 , 5325 , 
		5243 , 5157 , 5036 , 4905 , 4792 , 4649 , 4543 , 4402 , 4283 , 4128 , 
		3990 , 3859 , 3758 , 3648 , 3524 , 3409 , 3305 , 3220 , 3153 , 3090 , 
		3028 , 2983 , 2947 , 2919 , 2920 , 2917 , 2911 , 2902 , 2897 , 2911 , 
		2883 , 2841 , 2783 , 2726 , 2671 , 2594 , 2550 , 2519 , 2517 , 2513 , 
		2559 , 2619 , 2716 , 2790 , 2877 , 2985 , 3110 , 3223 , 3360 , 3517 , 
		3681 , 3822 , 3967 , 4113 , 4252 , 4401 , 4523 , 4624 , 4700 , 4775 , 
		4820 , 4827 , 4794 , 4762 , 4696 , 4630 , 4556 , 4453 , 4361 , 4250
	};

	if (acc_y <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingLowerYSecondHalf(int16_t acc_y, int idx) {
	const int16_t boundarySegment[] = 
	{
		4149 , 4059 , 3965 , 3857 , 3778 , 3667 , 3592 , 3488 , 3411 , 3324 , 
		3263 , 3195 , 3163 , 3107 , 3056 , 3044 , 3017 , 3017 , 3009 , 3022 , 
		3024 , 3028 , 3074 , 3117 , 3150 , 3191 , 3211 , 3217 , 3208 , 3178 , 
		3131 , 3094 , 3043 , 3043 , 3015 , 3021 , 3027 , 3047 , 3077 , 3101 , 
		3120 , 3164 , 3213 , 3266 , 3322 , 3389 , 3485 , 3564 , 3634 , 3737 , 
		3839 , 3918 , 3994 , 4098 , 4208 , 4249 , 4332 , 4387 , 4431 , 4445 , 
		4440 , 4437 , 4404 , 4366 , 4322 , 4253 , 4201 , 4146 , 4103 , 4048 , 
		3980 , 3921 , 3872 , 3819 , 3762 , 3711 , 3691 , 3620 , 3616 , 3575 , 
		3552 , 3524 , 3522 , 3504 , 3502 , 3506 , 3516 , 3514 , 3522 , 3532 , 
		3528 , 3499 , 3503 , 3463 , 3437 , 3405 , 3380 , 3361 , 3350 , 3349 , 
		3337 , 3339 , 3346 , 3356 , 3376 , 3395 , 3448 , 3494 , 3543 , 3562 , 
		3625 , 3689 , 3749 , 3805 , 3866 , 3931 , 4007 , 4053 , 4101 , 4148 , 
		4186 , 4226 , 4240 , 4214 , 4204 , 4173 , 4136 , 4100 , 4057 , 4014 , 
		3969 , 3916 , 3871 , 3833 , 3793 , 3734 , 3694 , 3660 , 3623 , 3606 , 
		3569 , 3541 , 3534 , 3493 , 3488 , 3490 , 3485 , 3487 , 3485 , 3497 , 
		3508 , 3518 , 3528 , 3547 , 3563 , 3577 , 3600 , 3598 , 3594 , 3601 , 
		3592 , 3574 , 3564 , 3563 , 3538 , 3542 , 3545 , 3561 , 3551 , 3565 , 
		3564 , 3593 , 3591 , 3628 , 3640 , 3670 , 3695 , 3727 , 3749 , 3791 , 
		3823 , 3858 , 3884 , 3932 , 3961 , 3990 , 4009 , 4051 , 4069 , 4083 , 
		4087 , 4085 , 4066 , 4043 , 4028 , 4011 , 3978 , 3964 , 3922 , 3918 , 
		3886 , 3864 , 3823 , 3819 , 3794 , 3781 , 3758 , 3752 , 3747 , 3738 , 
	//	3729 , 3710 , 3720 , 3725 , 3715 , 3715 , 3714 , 3731 , 3726 , 3728 , 
	//	3707 , 3722 , 3698 , 3693 , 3689 , 3682 , 3657 , 3648 , 3641 , 3657 , 
	//	3639 , 3652 , 3657 , 3661 , 3686 , 3691 , 3706 , 3708 , 3725 , 3756 , 
	//	3778 , 3792 , 3817 , 3847 , 3874 , 3895 , 3926 , 3933 , 3954 , 3971
	};

	if (acc_y <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingUpperZFirstHalf(int16_t acc_z, int idx) {
	const int16_t boundarySegment [] =
	{
		3981 , 3180 , 3404 , 4485 , 4054 , 3834 , 3525 , 1710 , 719 , 995 , 1740 , 
		1033 , 3027 , 1452 , 1994 , 1387 , 1564 , 2693 , 1152 , 1674 , 1492 , 
		1630 , 2016 , 2100 , 2038 , 2332 , 2807 , 3163 , 3490 , 3782 , 3761 , 
		3649 , 3573 , 3446 , 3277 , 3040 , 2785 , 2534 , 2328 , 2284 , 2330 , 
		2340 , 2266 , 2152 , 2058 , 1881 , 1808 , 1701 , 1643 , 1585 , 1525 , 
		1492 , 1462 , 1455 , 1459 , 1464 , 1463 , 1494 , 6659 , 3658 , 4140 , 
		4157 , 5414 , 2761 , 2257 , 1561 , 1629 , 1784 , 5105 , 5101 , 5093 , 
		1612 , 1412 , 1542 , 5025 , 5096 , 5089 , 3405 , 1724 , 3931 , 1716 , 
		1995 , 1577 , 1426 , 1479 , 1449 , 1272 , 1314 , 1273 , 1223 , 1217 , 
		1139 , 1077 , 1005 , 948 , 912 , 926 , 962 , 1013 , 1025 , 1014 , 
		938 , 804 , 660 , 568 , 336 , 277 , 284 , 305 , 321 , 309 , 
		309 , 320 , 305 , 282 , 275 , 278 , 273 , 280 , 263 , 265 , 
		255 , 219 , 216 , 206 , 168 , 160 , 141 , 123 , 107 , 82 , 
		77 , 24 , 32 , 20 , 19 , -16 , 0 , 7 , 45 , 66 , 
		91 , 113 , 130 , 178 , 230 , 269 , 327 , 384 , 436 , 486 , 
		523 , 592 , 628 , 682 , 728 , 772 , 794 , 855 , 869 , 874 , 
		916 , 921 , 925 , 922 , 941 , 930 , 942 , 937 , 926 , 923 , 
		950 , 946 , 965 , 976 , 970 , 981 , 1002 , 982 , 986 , 974 , 
		988 , 964 , 958 , 949 , 931 , 951 , 943 , 945 , 928 , 950 , 
		936 , 927 , 945 , 924 , 936 , 963 , 958 , 957 , 960 , 945 , 
		943 , 929 , 938 , 929 , 913 , 923 , 935 , 925 , 937 , 945 , 
		943 , 952 , 948 , 946 , 977 , 964 , 977 , 962 , 960 , 951 , 
		914 , 886 , 830 , 792 , 745 , 683 , 673 , 613 , 577 , 545 , 
		500 , 468 , 432 , 398 , 387 , 386 , 336 , 331 , 289 , 296 , 
		270 , 251 , 246 , 248 , 209 , 205 , 196 , 181 , 174 , 162
	};

	if (acc_z >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingUpperZSecondHalf(int16_t acc_z, int idx) {
	const int16_t boundarySegment [] =
	{
		167 , 154 , 151 , 152 , 141 , 155 , 136 , 137 , 139 , 135 , 
		123 , 125 , 118 , 120 , 107 , 125 , 145 , 123 , 150 , 152 , 
		171 , 166 , 167 , 196 , 210 , 244 , 242 , 289 , 322 , 332 , 
		354 , 386 , 419 , 450 , 459 , 480 , 518 , 545 , 549 , 582 , 
		598 , 614 , 603 , 630 , 652 , 641 , 653 , 642 , 653 , 642 , 
		667 , 655 , 654 , 677 , 664 , 683 , 696 , 689 , 704 , 718 , 
		722 , 727 , 704 , 702 , 719 , 728 , 728 , 750 , 749 , 740 , 
		740 , 747 , 754 , 756 , 752 , 764 , 755 , 776 , 775 , 777 , 
		771 , 783 , 787 , 766 , 792 , 769 , 781 , 785 , 797 , 779 , 
		814 , 790 , 795 , 812 , 818 , 818 , 794 , 816 , 789 , 802 , 
		774 , 764 , 747 , 716 , 723 , 668 , 658 , 597 , 555 , 528 , 
		501 , 485 , 448 , 422 , 391 , 381 , 370 , 350 , 334 , 319 , 
		320 , 305 , 319 , 310 , 304 , 291 , 273 , 278 , 273 , 281 , 
		274 , 254 , 266 , 237 , 226 , 220 , 225 , 224 , 198 , 204 , 
		213 , 194 , 183 , 188 , 186 , 197 , 197 , 183 , 209 , 203 , 
		222 , 220 , 220 , 239 , 250 , 256 , 265 , 286 , 318 , 313 , 
		346 , 366 , 385 , 403 , 402 , 447 , 437 , 475 , 487 , 480 , 
		474 , 509 , 511 , 529 , 538 , 509 , 531 , 530 , 534 , 546 , 
		533 , 562 , 558 , 550 , 563 , 575 , 590 , 592 , 589 , 592 , 
		591 , 606 , 611 , 606 , 621 , 605 , 610 , 604 , 617 , 649 , 
		620 , 620 , 640 , 642 , 644 , 633 , 633 , 655 , 630 , 650 , 
	//	645 , 677 , 660 , 656 , 653 , 658 , 669 , 658 , 665 , 681 , 
	//	686 , 674 , 678 , 692 , 678 , 670 , 681 , 662 , 671 , 658 , 
	//	645 , 649 , 630 , 596 , 589 , 576 , 555 , 517 , 518 , 488 , 
	//	448 , 440 , 426 , 410 , 382 , 379 , 361 , 346 , 345 , 341
	};

	if (acc_z >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}


int checkActivityPendSwingLowerZFirstHalf(int16_t acc_z, int idx) {
	int16_t boundarySegment[] = 
	{
		-1807 , -1608 , -1960 , -4551 , -4610 , -4406 , -3395 , -2650 , -2413 , -2741 , -3204 , 
		-3283 , -5401 , -2444 , -2726 , -2929 , -1952 , -2519 , -1492 , -1862 , -1044 , 
		-998 , -1368 , -1168 , -666 , -604 , -669 , -517 , -310 , -238 , 57 , 
		361 , 477 , 522 , 557 , 452 , 449 , 490 , 568 , 464 , 374 , 
		340 , 346 , 388 , 490 , 485 , 496 , 489 , 503 , 541 , 625 , 
		676 , 786 , 843 , 887 , 912 , 923 , 850 , -4477 , -494 , -1296 , 
		-863 , -1822 , 13 , 317 , 761 , 293 , 40 , -2295 , -2315 , -2231 , 
		428 , 544 , 378 , -4195 , -2072 , -2115 , -2159 , -16 , -1445 , 108 , 
		-645 , -119 , 66 , -117 , -131 , 80 , -54 , -115 , -137 , -219 , 
		-237 , -271 , -303 , -320 , -408 , -486 , -634 , -747 , -851 , -898 , 
		-906 , -880 , -808 , -744 , -656 , -667 , -712 , -727 , -727 , -667 , 
		-567 , -492 , -403 , -274 , -221 , -178 , -131 , -144 , -145 , -171 , 
		-193 , -193 , -232 , -242 , -236 , -272 , -271 , -277 , -273 , -282 , 
		-291 , -276 , -280 , -292 , -289 , -264 , -268 , -265 , -275 , -286 , 
		-313 , -339 , -330 , -362 , -378 , -403 , -409 , -432 , -436 , -450 , 
		-441 , -460 , -420 , -390 , -344 , -280 , -226 , -169 , -87 , -30 , 
		40 , 129 , 197 , 302 , 369 , 446 , 486 , 561 , 606 , 647 , 
		666 , 690 , 693 , 688 , 706 , 705 , 706 , 702 , 710 , 714 , 
		696 , 704 , 714 , 721 , 735 , 723 , 731 , 745 , 768 , 746 , 
		760 , 759 , 753 , 772 , 764 , 727 , 726 , 745 , 728 , 725 , 
		707 , 725 , 698 , 693 , 681 , 659 , 627 , 617 , 573 , 537 , 
		519 , 456 , 428 , 390 , 337 , 292 , 253 , 186 , 148 , 103 , 
		86 , 78 , 62 , 36 , 41 , 23 , 1 , -3 , -7 , -23 , 
		-20 , -20 , -20 , -18 , -21 , -42 , -28 , -45 , -15 , -24 , 
		-34 , -25 , -18 , -16 , -23 , -23 , -40 , -31 , -38 , -30
	};
	
	
	if (acc_z <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityPendSwingLowerZSecondHalf(int16_t acc_z, int idx) {
	int16_t boundarySegment[] = 
	{
		-45 , -46 , -29 , -40 , -39 , -49 , -48 , -47 , -81 , -69 , 
		-73 , -67 , -70 , -64 , -61 , -79 , -79 , -85 , -98 , -100 , 
		-113 , -106 , -105 , -108 , -122 , -140 , -118 , -147 , -142 , -144 , 
		-146 , -146 , -149 , -138 , -121 , -96 , -86 , -63 , -27 , 2 , 
		30 , 54 , 99 , 126 , 164 , 213 , 245 , 302 , 313 , 370 , 
		379 , 423 , 450 , 461 , 492 , 511 , 512 , 525 , 532 , 542 , 
		558 , 547 , 576 , 598 , 595 , 604 , 596 , 610 , 613 , 620 , 
		624 , 635 , 646 , 620 , 644 , 628 , 639 , 608 , 607 , 601 , 
		615 , 591 , 583 , 598 , 568 , 577 , 549 , 529 , 493 , 499 , 
		442 , 442 , 411 , 384 , 346 , 314 , 290 , 248 , 245 , 190 , 
		178 , 160 , 147 , 136 , 103 , 112 , 94 , 97 , 103 , 100 , 
		97 , 101 , 104 , 110 , 107 , 113 , 98 , 106 , 122 , 103 , 
		100 , 97 , 79 , 62 , 64 , 59 , 49 , 54 , 37 , 45 , 
		30 , 46 , 26 , 29 , 30 , 28 , 25 , 16 , 26 , 16 , 
		17 , 18 , 11 , 20 , 30 , 9 , 21 , 23 , 9 , 23 , 
		14 , 12 , 12 , -1 , -2 , 0 , 1 , 6 , 2 , -3 , 
		-14 , -10 , 1 , 11 , 18 , 11 , 49 , 39 , 71 , 100 , 
		118 , 133 , 159 , 169 , 178 , 229 , 235 , 262 , 302 , 298 , 
		341 , 330 , 366 , 390 , 415 , 403 , 414 , 428 , 429 , 452 , 
		463 , 454 , 467 , 466 , 477 , 505 , 486 , 508 , 505 , 489 , 
		516 , 532 , 516 , 526 , 508 , 517 , 517 , 507 , 522 , 498 , 
	//	489 , 473 , 496 , 500 , 481 , 462 , 457 , 430 , 429 , 405 , 
	//	386 , 378 , 350 , 324 , 310 , 290 , 277 , 242 , 239 , 214 , 
	//	209 , 193 , 182 , 196 , 157 , 148 , 155 , 161 , 134 , 148 , 
	//	152 , 152 , 150 , 146 , 150 , 139 , 137 , 158 , 153 , 137
	};
	
	
	if (acc_z <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

// Hand wave side-to-side

int checkActivityHandWaveS2SUpperXFirstHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		2328 , 2704 , 2895 , 3196 , 3445 , 4008 , 4614 , 5134 , 5650 , 6027 , 6459 , 
		6744 , 7094 , 7323 , 7348 , 7391 , 7439 , 7437 , 7608 , 7722 , 7828 , 
		7978 , 7988 , 7830 , 7739 , 7642 , 7353 , 6906 , 6332 , 5645 , 4722 , 
		3634 , 2505 , 1468 , 376 , -630 , -1265 , -1337 , -1434 , -1720 , -2157 , 
		-2497 , -2924 , -3077 , -3302 , -3652 , -3996 , -4243 , -4588 , -4876 , -4895 , 
		-4717 , -4991 , -5406 , -5688 , -5962 , -6362 , -6600 , -6567 , -6359 , -6163 , 
		-5643 , -5010 , -4330 , -3526 , -3041 , -2570 , -2132 , -1820 , -1575 , -1316 , 
		-1002 , -379 , 231 , 862 , 1608 , 2310 , 3216 , 3835 , 4533 , 5218 , 
		5867 , 6355 , 7030 , 7699 , 8306 , 8752 , 9149 , 9409 , 9469 , 9376 , 
		9321 , 9351 , 9356 , 9476 , 9390 , 9491 , 9428 , 9531 , 9550 , 9469 , 
		9421 , 9223 , 9065 , 8970 , 9058 , 9160 , 9358 , 9421 , 9338 , 9172 , 
		9125 , 9203 , 9467 , 9768 , 10071 , 10408 , 10702 , 10798 , 10744 , 10547 , 
		10349 , 10102 , 9974 , 9732 , 9418 , 9019 , 8746 , 8413 , 7968 , 7610 , 
		7404 , 6965 , 6312 , 5437 , 4433 , 3252 , 2473 , 1665 , 1027 , 147 , 
		-623 , -1503 , -2105 , -2576 , -2966 , -3098 , -3202 , -3521 , -4007 , -4514 , 
		-4935 , -5331 , -5407 , -5569 , -5924 , -6124 , -6481 , -6537 , -6445 , -6253 , 
		-6031 , -5713 , -5545 , -5279 , -4948 , -4634 , -4271 , -3847 , -3159 , -2518 , 
		-1982 , -1465 , -1127 , -688 , -204 , 432 , 975 , 1437 , 1806 , 2260 , 
		2739 , 3182 , 3615 , 4211 , 4822 , 5509 , 6462 , 7339 , 8146 , 8760 , 
		9108 , 9581 , 9843 , 10111 , 10197 , 10252 , 10127 , 9853 , 9445 , 9101 , 
		8834 , 8620 , 8474 , 8501 , 8682 , 8912 , 8973 , 9015 , 9015 , 9045 , 
		9106 , 9203 , 9305 , 9307 , 9415 , 9583 , 9821 , 9953 , 10009 , 9991 , 
		10056 , 10200 , 10366 , 10584 , 10745 , 11011 , 11353 , 11548 , 11504 , 11190 , 
		10738 , 10146 , 9254 , 8502 , 7868 , 7253 , 6854 , 6371 , 5932 , 5229 , 
		4374 , 3484 , 2454 , 1220 , -139 , -1362 , -2296 , -2957 , -3426 , -3865
	};
	
	
	if (acc_x >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SUpperXSecondHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		-3998 , -3848 , -4102 , -4434 , -4892 , -5247 , -5669 , -5748 , -5822 , -5723 , 
		-5571 , -5485 , -5412 , -5225 , -5054 , -4819 , -4475 , -4148 , -3765 , -3380 , 
		-2880 , -2525 , -2103 , -1761 , -1421 , -1028 , -552 , -36 , 358 , 900 , 
		1583 , 2457 , 3125 , 3786 , 4595 , 5592 , 6559 , 7156 , 7712 , 8024 , 
		8456 , 8787 , 9130 , 9395 , 9559 , 9667 , 9767 , 10009 , 9886 , 9691 , 
		9571 , 9217 , 8814 , 8471 , 8337 , 8163 , 8164 , 8224 , 8414 , 8504 , 
		8593 , 8601 , 8536 , 8513 , 8437 , 8495 , 8487 , 8660 , 8741 , 8756 , 
		8922 , 9222 , 9292 , 9445 , 9689 , 9894 , 10040 , 10318 , 10611 , 10912 , 
		11084 , 11029 , 10869 , 10473 , 10451 , 10377 , 10030 , 9819 , 9424 , 9054 , 
		8491 , 7336 , 6200 , 5187 , 4272 , 3617 , 3026 , 2254 , 1501 , 680 , 
		-364 , -1262 , -2039 , -2632 , -3067 , -3642 , -4474 , -5223 , -5632 , -5574 , 
		-5276 , -5028 , -4919 , -4933 , -4864 , -4653 , -4569 , -4427 , -4088 , -3730 , 
		-3310 , -2820 , -2342 , -1816 , -1347 , -759 , -143 , 559 , 1027 , 1572 , 
		2362 , 3067 , 4048 , 5007 , 5525 , 6357 , 6932 , 7344 , 7771 , 8146 , 
		8552 , 8729 , 8851 , 8997 , 9298 , 9695 , 10085 , 10636 , 11097 , 11185 , 
		11178 , 10937 , 10583 , 10156 , 9735 , 9249 , 8867 , 8777 , 8534 , 8277 , 
		8076 , 8124 , 8299 , 8521 , 8647 , 8698 , 8462 , 8162 , 7895 , 7679 , 
		7677 , 7940 , 8254 , 8616 , 9001 , 9579 , 10195 , 10988 , 11577 , 12155 , 
		12613 , 12626 , 12641 , 12633 , 12850 , 12391 , 11918 , 11419 , 10990 , 10549 , 
		9970 , 9491 , 9070 , 8642 , 8514 , 8299 , 7944 , 7318 , 6852 , 6211 , 
		5516 , 4833 , 3864 , 2945 , 2098 , 1256 , 448 , -254 , -1050 , -1731 , 
	//	-2547 , -3264 , -3732 , -4151 , -4502 , -4806 , -5045 , -5358 , -5458 , -5429 , 
	//	-5273 , -4894 , -4380 , -3814 , -3198 , -2543 , -1498 , -313 , 1073 , 2388 , 
	//	3820 , 5417 , 6972 , 8254 , 8809 , 8900 , 8940 , 9155 , 9676 , 9762 , 
	//	9723 , 9506 , 9376 , 9311 , 9523 , 9849 , 10106 , 10407 , 10694 , 10936
	};
	
	
	if (acc_x >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SLowerXFirstHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		-976 , -1104 , -1089 , -1064 , -943 , -1036 , -1022 , -1010 , -834 , -673 , -469 , 
		-96 , 302 , 523 , 852 , 1171 , 1199 , 1161 , 884 , 642 , 176 , 
		-526 , -1128 , -1914 , -2709 , -3634 , -4503 , -5362 , -6124 , -6787 , -7362 , 
		-7966 , -8307 , -8504 , -8608 , -8546 , -8637 , -8805 , -9074 , -9144 , -9077 , 
		-9217 , -9336 , -9641 , -9766 , -9708 , -9600 , -9583 , -9508 , -9260 , -9235 , 
		-9465 , -9403 , -9314 , -9308 , -9314 , -9134 , -8944 , -8955 , -9067 , -9183 , 
		-9603 , -10098 , -10470 , -10842 , -10921 , -11026 , -11104 , -11128 , -11131 , -10980 , 
		-10802 , -10779 , -10709 , -10690 , -10516 , -10270 , -10228 , -10097 , -9971 , -9810 , 
		-9569 , -9125 , -8674 , -8053 , -7418 , -6612 , -5867 , -5175 , -4239 , -3124 , 
		-2003 , -905 , 212 , 944 , 1838 , 2411 , 3064 , 3411 , 3618 , 3889 , 
		4141 , 4455 , 4661 , 4710 , 4574 , 4368 , 4122 , 3953 , 3810 , 3760 , 
		3537 , 3179 , 2695 , 2276 , 1831 , 1392 , 878 , 454 , 44 , -309 , 
		-799 , -1222 , -1654 , -2040 , -2586 , -3201 , -3878 , -4451 , -5048 , -5746 , 
		-6356 , -6955 , -7512 , -7963 , -8199 , -8360 , -8895 , -9339 , -9933 , -10269 , 
		-10407 , -10431 , -10425 , -10520 , -10446 , -10474 , -10106 , -9705 , -9475 , -9418 , 
		-9323 , -9227 , -9299 , -9301 , -9188 , -9168 , -9089 , -9121 , -9153 , -9329 , 
		-9491 , -9709 , -9853 , -10027 , -10188 , -10346 , -10539 , -10763 , -11143 , -11470 , 
		-11742 , -11901 , -11947 , -11992 , -11864 , -11772 , -11673 , -11303 , -11030 , -10772 , 
		-10393 , -9930 , -9281 , -8845 , -8518 , -8147 , -7730 , -7233 , -6634 , -5988 , 
		-5040 , -4307 , -3513 , -2813 , -1883 , -944 , -69 , 673 , 1453 , 2109 , 
		2638 , 3144 , 3610 , 3881 , 3994 , 3996 , 4045 , 4079 , 4167 , 4217 , 
		4202 , 4075 , 3845 , 3567 , 3043 , 2479 , 1885 , 1509 , 1145 , 927 , 
		648 , 400 , 174 , -160 , -579 , -1345 , -2391 , -3376 , -4116 , -4642 , 
		-5002 , -5326 , -5574 , -6154 , -6668 , -7227 , -7982 , -8893 , -9704 , -10507 , 
		-11074 , -11012 , -10834 , -10804 , -10579 , -10474 , -10208 , -10185 , -10158 , -9797
	};
	
	
	if (acc_x <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SLowerXSecondHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		-9666 , -9708 , -9454 , -9270 , -9056 , -9047 , -9129 , -9280 , -9458 , -9691 , 
		-9903 , -9977 , -10124 , -10337 , -10482 , -10619 , -10803 , -10960 , -11157 , -11308 , 
		-11488 , -11537 , -11599 , -11573 , -11549 , -11572 , -11572 , -11476 , -11326 , -11236 , 
		-11165 , -11079 , -10875 , -10678 , -10505 , -10412 , -10185 , -9796 , -9280 , -8704 , 
		-8020 , -7429 , -6642 , -5881 , -5205 , -4233 , -3201 , -2455 , -1470 , -465 , 
		207 , 1161 , 2002 , 2667 , 3197 , 3779 , 4140 , 4372 , 4414 , 4332 , 
		4237 , 4233 , 4364 , 4505 , 4625 , 4623 , 4475 , 4168 , 3817 , 3480 , 
		2962 , 2314 , 1808 , 1313 , 705 , 154 , -528 , -1342 , -2233 , -3236 , 
		-4176 , -4911 , -5639 , -6235 , -7205 , -8155 , -8690 , -9321 , -9908 , -10190 , 
		-10373 , -10156 , -10244 , -10277 , -10548 , -11199 , -11938 , -12162 , -12187 , -12084 , 
		-11536 , -11058 , -10427 , -9848 , -9683 , -9418 , -9102 , -8935 , -9064 , -9378 , 
		-9680 , -9752 , -9739 , -9669 , -9744 , -9897 , -9961 , -10163 , -10368 , -10550 , 
		-10714 , -10844 , -11026 , -11284 , -11363 , -11483 , -11687 , -11989 , -12149 , -12308 , 
		-12546 , -12729 , -13004 , -13209 , -12807 , -12599 , -12324 , -12064 , -11853 , -11506 , 
		-11092 , -10411 , -9585 , -8835 , -8206 , -7541 , -6983 , -6580 , -6067 , -5319 , 
		-4494 , -3599 , -2725 , -1900 , -981 , -91 , 875 , 1433 , 2138 , 2829 , 
		3476 , 3856 , 4091 , 4121 , 4059 , 3958 , 4030 , 4094 , 4155 , 4127 , 
		3889 , 3356 , 2834 , 2272 , 1657 , 955 , 267 , -836 , -1971 , -3133 , 
		-4267 , -4850 , -5631 , -6611 , -7698 , -7741 , -7674 , -7637 , -7686 , -7891 , 
		-8030 , -8389 , -9014 , -9790 , -10966 , -12025 , -12660 , -12810 , -12860 , -12781 , 
		-12688 , -12355 , -11776 , -11491 , -11326 , -11300 , -11464 , -11606 , -11366 , -11099 , 
	//	-10655 , -10196 , -10008 , -9811 , -9698 , -9726 , -9685 , -9498 , -9346 , -9289 , 
	//	-9425 , -9690 , -9932 , -10258 , -10642 , -11051 , -11630 , -12221 , -12919 , -13608 , 
	//	-14288 , -14967 , -15556 , -16106 , -16263 , -16176 , -15952 , -15645 , -15564 , -15238 , 
	//	-14817 , -14318 , -13764 , -13285 , -12805 , -12547 , -12242 , -11985 , -11650 , -11248
	};
	
	
	if (acc_x <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SUpperYFirstHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-3540 , -3591 , -3540 , -3415 , -3317 , -3161 , -3035 , -2935 , -2816 , -2706 , -2535 , 
		-2393 , -2143 , -1939 , -1688 , -1476 , -1326 , -1004 , -855 , -795 , -769 , 
		-523 , -396 , -232 , -59 , 17 , 154 , 351 , 156 , 175 , 168 , 
		100 , 106 , -47 , -93 , -376 , -557 , -880 , -884 , -823 , -811 , 
		-954 , -952 , -1031 , -1101 , -1063 , -1135 , -959 , -935 , -1074 , -934 , 
		-978 , -933 , -939 , -1030 , -1040 , -965 , -944 , -810 , -764 , -760 , 
		-577 , -530 , -511 , -436 , -393 , -438 , -387 , -349 , -311 , -294 , 
		-235 , -174 , -139 , -126 , -106 , -21 , 72 , 186 , 365 , 408 , 
		321 , 302 , 430 , 440 , 435 , 486 , 519 , 658 , 678 , 734 , 
		638 , 587 , 371 , 164 , 93 , 71 , 70 , -143 , -102 , -132 , 
		-198 , -190 , -255 , -291 , -409 , -519 , -609 , -612 , -690 , -687 , 
		-633 , -658 , -679 , -611 , -558 , -507 , -456 , -344 , -206 , -56 , 
		153 , 346 , 554 , 744 , 829 , 1038 , 1321 , 1545 , 1530 , 1626 , 
		1863 , 1994 , 2127 , 2306 , 2491 , 2598 , 2536 , 2376 , 2073 , 1592 , 
		1185 , 843 , 551 , 469 , 199 , -46 , -170 , -112 , -256 , -313 , 
		-310 , -248 , -379 , -522 , -590 , -630 , -672 , -679 , -679 , -638 , 
		-615 , -637 , -714 , -729 , -747 , -783 , -818 , -828 , -828 , -757 , 
		-576 , -383 , -343 , -312 , -165 , -48 , 149 , 369 , 505 , 616 , 
		751 , 895 , 1017 , 1106 , 1141 , 1229 , 1218 , 1196 , 1100 , 1043 , 
		909 , 753 , 624 , 588 , 540 , 402 , 240 , 77 , 17 , -15 , 
		-96 , -110 , -83 , -179 , -296 , -408 , -419 , -405 , -464 , -499 , 
		-538 , -610 , -672 , -690 , -703 , -710 , -765 , -800 , -829 , -816 , 
		-718 , -476 , -280 , -36 , 129 , 335 , 615 , 817 , 1062 , 1307 , 
		1661 , 1993 , 2371 , 2636 , 2802 , 2912 , 2960 , 2787 , 2631 , 2690 , 
		2625 , 2635 , 2472 , 2251 , 1916 , 1462 , 1164 , 831 , 328 , -57
	};
	
	
	if (acc_y >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SUpperYSecondHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-346 , -418 , -638 , -759 , -801 , -674 , -711 , -899 , -1114 , -1256 , 
		-1338 , -1238 , -1199 , -1185 , -1202 , -1159 , -1207 , -1105 , -1016 , -924 , 
		-752 , -566 , -444 , -303 , -128 , -32 , 34 , 171 , 331 , 481 , 
		575 , 596 , 517 , 448 , 475 , 500 , 509 , 532 , 435 , 578 , 
		628 , 606 , 580 , 564 , 538 , 415 , 376 , 357 , 228 , 133 , 
		122 , 58 , -37 , -91 , -235 , -315 , -383 , -377 , -391 , -496 , 
		-482 , -497 , -540 , -589 , -618 , -674 , -659 , -557 , -396 , -253 , 
		-35 , 390 , 751 , 726 , 863 , 1024 , 1219 , 1256 , 1360 , 1541 , 
		1852 , 2127 , 2220 , 2172 , 1984 , 1651 , 1537 , 1410 , 1489 , 1736 , 
		2205 , 2452 , 2712 , 3135 , 3050 , 2570 , 2042 , 1678 , 1407 , 1207 , 
		1005 , 872 , 877 , 663 , 396 , 157 , -20 , -239 , -389 , -481 , 
		-636 , -724 , -747 , -738 , -660 , -601 , -614 , -564 , -414 , -286 , 
		-115 , -38 , -22 , 62 , 114 , 152 , 96 , 131 , 141 , 146 , 
		131 , 104 , 23 , -97 , -102 , -79 , -60 , 40 , 187 , 300 , 
		457 , 587 , 624 , 534 , 548 , 492 , 409 , 415 , 376 , 385 , 
		475 , 424 , 351 , 318 , 247 , 186 , 118 , -77 , -194 , -283 , 
		-398 , -390 , -381 , -391 , -352 , -311 , -298 , -230 , -126 , 72 , 
		239 , 464 , 535 , 703 , 866 , 978 , 1123 , 1336 , 1530 , 1633 , 
		1730 , 1860 , 1854 , 1771 , 1634 , 1291 , 1242 , 1246 , 1731 , 2335 , 
		2886 , 3260 , 3338 , 3403 , 3287 , 2852 , 2360 , 1812 , 1536 , 1459 , 
		1516 , 1701 , 1726 , 1766 , 1715 , 1603 , 1494 , 1454 , 1092 , 823 , 
	//	714 , 615 , 437 , 61 , -110 , -182 , -170 , -149 , -46 , 115 , 
	//	321 , 532 , 799 , 1127 , 1390 , 1653 , 1865 , 1942 , 1994 , 1877 , 
	//	1737 , 1470 , 1032 , 389 , 186 , -64 , -43 , 41 , 139 , 253 , 
	//	342 , 436 , 513 , 735 , 921 , 984 , 1225 , 1487 , 1757 , 1793
	};
	
	if (acc_y >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SLowerYFirstHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-4340 , -4211 , -4248 , -4315 , -4301 , -4377 , -4355 , -4351 , -4372 , -4298 , -4287 , 
		-4249 , -4287 , -4215 , -4196 , -4140 , -4070 , -4048 , -3983 , -3791 , -3661 , 
		-3583 , -3432 , -3336 , -3171 , -2987 , -2882 , -2921 , -2692 , -2621 , -2684 , 
		-2676 , -2966 , -2903 , -2757 , -2608 , -2633 , -2436 , -2616 , -2703 , -2715 , 
		-2654 , -2696 , -2671 , -2645 , -2607 , -2611 , -2587 , -2927 , -2898 , -2990 , 
		-2882 , -2829 , -2823 , -2782 , -2776 , -2837 , -2928 , -3014 , -3036 , -3060 , 
		-3105 , -3118 , -3159 , -3144 , -3089 , -2954 , -2835 , -2785 , -2735 , -2618 , 
		-2535 , -2434 , -2399 , -2218 , -2094 , -2057 , -2076 , -2126 , -2167 , -2132 , 
		-2039 , -1966 , -1930 , -1916 , -1981 , -2090 , -2149 , -2262 , -2254 , -2290 , 
		-2326 , -2385 , -2341 , -2284 , -2327 , -2365 , -2374 , -2323 , -2402 , -2424 , 
		-2398 , -2434 , -2415 , -2419 , -2413 , -2419 , -2377 , -2416 , -2386 , -2395 , 
		-2393 , -2390 , -2335 , -2367 , -2418 , -2447 , -2444 , -2468 , -2502 , -2512 , 
		-2479 , -2502 , -2446 , -2472 , -2447 , -2402 , -2451 , -2411 , -2374 , -2338 , 
		-2345 , -2334 , -2473 , -2634 , -2621 , -2710 , -2908 , -2852 , -2711 , -2612 , 
		-2639 , -2673 , -2725 , -2835 , -2805 , -2706 , -2698 , -2712 , -2816 , -2897 , 
		-2810 , -2740 , -2627 , -2618 , -2634 , -2622 , -2672 , -2683 , -2643 , -2662 , 
		-2623 , -2533 , -2470 , -2369 , -2319 , -2267 , -2222 , -2164 , -2064 , -2045 , 
		-2120 , -2159 , -2127 , -2068 , -2021 , -1988 , -1983 , -2019 , -2027 , -2056 , 
		-2069 , -2097 , -2103 , -2150 , -2175 , -2199 , -2226 , -2292 , -2280 , -2289 , 
		-2195 , -2111 , -2096 , -2164 , -2256 , -2250 , -2264 , -2251 , -2251 , -2247 , 
		-2252 , -2306 , -2371 , -2359 , -2316 , -2268 , -2247 , -2253 , -2256 , -2235 , 
		-2222 , -2202 , -2144 , -2142 , -2187 , -2186 , -2165 , -2120 , -2093 , -2104 , 
		-2162 , -2272 , -2316 , -2372 , -2355 , -2385 , -2401 , -2375 , -2426 , -2433 , 
		-2603 , -2739 , -2837 , -2700 , -2698 , -2736 , -2736 , -2565 , -2469 , -2558 , 
		-2691 , -2909 , -3072 , -3073 , -3040 , -2970 , -2984 , -2897 , -2784 , -2705
	};
	
	
	if (acc_y <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SLowerYSecondHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-2598 , -2630 , -2566 , -2559 , -2541 , -2650 , -2619 , -2475 , -2326 , -2252 , 
		-2226 , -2322 , -2359 , -2297 , -2234 , -2187 , -2107 , -2085 , -2128 , -2144 , 
		-2248 , -2414 , -2408 , -2375 , -2636 , -2548 , -2458 , -2397 , -2421 , -2475 , 
		-2425 , -2320 , -2215 , -2136 , -2033 , -2020 , -2039 , -2100 , -2053 , -2114 , 
		-2172 , -2206 , -2228 , -2284 , -2322 , -2285 , -2256 , -2291 , -2312 , -2347 , 
		-2454 , -2498 , -2417 , -2335 , -2291 , -2323 , -2347 , -2385 , -2387 , -2336 , 
		-2314 , -2333 , -2276 , -2265 , -2278 , -2206 , -2203 , -2249 , -2324 , -2417 , 
		-2523 , -2798 , -3045 , -3046 , -3097 , -3168 , -3229 , -3200 , -3156 , -3147 , 
		-3292 , -3413 , -3352 , -3180 , -2964 , -2685 , -2579 , -2522 , -2627 , -2796 , 
		-3091 , -3216 , -3352 , -3689 , -3426 , -3098 , -2846 , -2758 , -2757 , -2893 , 
		-3167 , -3412 , -3387 , -3345 , -3228 , -3071 , -2808 , -2611 , -2517 , -2525 , 
		-2504 , -2580 , -2675 , -2682 , -2708 , -2629 , -2558 , -2580 , -2678 , -2686 , 
		-2787 , -2854 , -2858 , -2926 , -2930 , -2908 , -2916 , -2917 , -2815 , -2746 , 
		-2601 , -2504 , -2409 , -2265 , -2174 , -2135 , -2092 , -2140 , -2213 , -2244 , 
		-2295 , -2341 , -2368 , -2338 , -2316 , -2344 , -2331 , -2353 , -2380 , -2387 , 
		-2389 , -2420 , -2429 , -2450 , -2409 , -2446 , -2430 , -2349 , -2358 , -2363 , 
		-2342 , -2422 , -2517 , -2527 , -2524 , -2499 , -2486 , -2566 , -2618 , -2740 , 
		-2837 , -2988 , -3045 , -3121 , -3206 , -3262 , -3365 , -3440 , -3490 , -3435 , 
		-3514 , -3524 , -3502 , -3413 , -3298 , -2993 , -2934 , -2946 , -3293 , -3617 , 
		-3970 , -4020 , -4170 , -4053 , -3821 , -3496 , -3200 , -2832 , -2724 , -2957 , 
		-3212 , -3435 , -3358 , -3374 , -3261 , -3001 , -2918 , -3078 , -3008 , -2933 , 
	//	-2998 , -2937 , -2935 , -2643 , -2458 , -2234 , -2190 , -2293 , -2378 , -2473 , 
	//	-2627 , -2780 , -2929 , -3049 , -3134 , -3247 , -3331 , -3350 , -3346 , -3339 , 
	//	-3215 , -3098 , -2808 , -2443 , -2354 , -2216 , -2175 , -2147 , -2161 , -2199 , 
	//	-2222 , -2192 , -2167 , -2233 , -2295 , -2368 , -2575 , -2769 , -2991 , -2975
	};
	
	
	if (acc_y <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}


int checkActivityHandWaveS2SUpperZFirstHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		288 , 256 , 306 , 458 , 556 , 550 , 592 , 456 , 657 , 639 , 697 , 
		711 , 714 , 850 , 1063 , 1152 , 1276 , 1207 , 959 , 922 , 1193 , 
		1202 , 1030 , 920 , 1067 , 1266 , 1431 , 1541 , 1602 , 2078 , 2586 , 
		2844 , 2954 , 3013 , 3217 , 3285 , 3324 , 3332 , 3389 , 3445 , 3843 , 
		3489 , 3976 , 3831 , 4136 , 4251 , 4144 , 4042 , 3689 , 3665 , 3580 , 
		3734 , 3883 , 3824 , 3823 , 3696 , 3557 , 3573 , 3563 , 3598 , 3561 , 
		3439 , 3402 , 3314 , 3372 , 3426 , 3406 , 3222 , 2992 , 2861 , 2807 , 
		2823 , 2876 , 2811 , 2820 , 2922 , 3088 , 3190 , 3273 , 3341 , 3453 , 
		3569 , 3502 , 3285 , 3059 , 2984 , 2750 , 2518 , 2359 , 2258 , 2177 , 
		2086 , 2071 , 2029 , 2034 , 1912 , 1790 , 1523 , 1483 , 1588 , 1472 , 
		1246 , 953 , 586 , 509 , 400 , 224 , 70 , 40 , -51 , -154 , 
		-232 , -210 , -254 , -177 , -19 , 321 , 501 , 583 , 852 , 1038 , 
		1201 , 1285 , 1211 , 1208 , 1389 , 1713 , 2150 , 2785 , 3350 , 3477 , 
		3484 , 3577 , 3739 , 3764 , 3711 , 3808 , 3920 , 3974 , 4036 , 4112 , 
		4162 , 4230 , 4684 , 5053 , 5021 , 4925 , 4799 , 4931 , 4809 , 4494 , 
		4245 , 4179 , 4438 , 4774 , 5063 , 5088 , 4926 , 4823 , 4737 , 4693 , 
		4726 , 4623 , 4622 , 4526 , 4420 , 4318 , 4209 , 4104 , 3861 , 3632 , 
		3492 , 3400 , 3477 , 3392 , 3500 , 3518 , 3455 , 3435 , 3402 , 3399 , 
		3328 , 3262 , 3252 , 3318 , 3393 , 3338 , 3288 , 3271 , 3155 , 3147 , 
		3008 , 2733 , 2484 , 2177 , 2151 , 2134 , 2192 , 2266 , 2184 , 1970 , 
		1809 , 1746 , 1567 , 1403 , 1245 , 1059 , 931 , 808 , 632 , 410 , 
		66 , -135 , -233 , -167 , -41 , 30 , 156 , 268 , 392 , 576 , 
		789 , 821 , 802 , 804 , 943 , 1138 , 1555 , 2132 , 2475 , 2902 , 
		3034 , 3184 , 3345 , 3599 , 3732 , 3774 , 3699 , 3925 , 4089 , 4292 , 
		4627 , 4831 , 4941 , 4871 , 4839 , 4945 , 4940 , 4964 , 5066 , 5126
	};
	
	
	if (acc_z >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SUpperZSecondHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		5095 , 5256 , 5224 , 5348 , 5388 , 5290 , 5127 , 5060 , 5123 , 5302 , 
		5408 , 5660 , 5751 , 5511 , 5116 , 4698 , 4445 , 4246 , 3941 , 3841 , 
		3789 , 3751 , 3668 , 3644 , 3693 , 3441 , 3314 , 3281 , 3240 , 3177 , 
		3161 , 3136 , 3151 , 3157 , 3097 , 3072 , 3100 , 3124 , 3266 , 3073 , 
		2944 , 2932 , 2910 , 2769 , 2598 , 2481 , 2297 , 1921 , 1824 , 1894 , 
		1987 , 1891 , 1625 , 1380 , 1166 , 952 , 860 , 752 , 823 , 862 , 
		901 , 846 , 703 , 623 , 359 , 225 , 118 , -23 , -196 , -293 , 
		-323 , -192 , 248 , 1261 , 1991 , 2403 , 2511 , 2472 , 2523 , 2472 , 
		2329 , 2307 , 2401 , 2713 , 2978 , 3258 , 3438 , 3946 , 4480 , 4981 , 
		5500 , 5881 , 5427 , 4938 , 4463 , 4383 , 4544 , 4827 , 5089 , 5415 , 
		5699 , 5681 , 5444 , 5252 , 4940 , 4524 , 4286 , 4315 , 4453 , 4651 , 
		4887 , 5157 , 5338 , 5342 , 5165 , 4998 , 5066 , 5186 , 5359 , 5413 , 
		5282 , 5018 , 4653 , 4485 , 4314 , 4123 , 4053 , 3977 , 3948 , 3922 , 
		3728 , 3596 , 3466 , 3440 , 3297 , 3366 , 3352 , 3358 , 3360 , 3258 , 
		3221 , 3231 , 3172 , 3193 , 2996 , 2965 , 2840 , 2714 , 2501 , 2286 , 
		2107 , 2033 , 2034 , 1844 , 1781 , 1739 , 1614 , 1579 , 1461 , 1240 , 
		1110 , 1020 , 885 , 769 , 535 , 397 , 353 , 379 , 351 , 317 , 
		350 , 451 , 848 , 1129 , 1400 , 1507 , 1620 , 1785 , 1886 , 2031 , 
		2031 , 2045 , 2027 , 2012 , 2132 , 2497 , 2792 , 3241 , 3613 , 4045 , 
		4324 , 4638 , 4552 , 4477 , 4309 , 4444 , 4415 , 4503 , 5031 , 5654 , 
		6373 , 6199 , 6072 , 5604 , 5164 , 4986 , 4884 , 4735 , 4843 , 5026 , 
	//	5300 , 5516 , 5380 , 5048 , 4563 , 4172 , 4269 , 4500 , 4788 , 4885 , 
	//	5030 , 5226 , 5239 , 5187 , 5209 , 5185 , 5037 , 4781 , 4654 , 4826 , 
	//	4908 , 4965 , 4902 , 5017 , 4981 , 5015 , 4990 , 4633 , 4148 , 3754 , 
	//	3589 , 3704 , 3824 , 3996 , 3969 , 4265 , 4359 , 4371 , 4483 , 4469
	};
	
	if (acc_z >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SLowerZFirstHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		-784 , -776 , -770 , -786 , -752 , -690 , -664 , -480 , -643 , -649 , -779 , 
		-961 , -1102 , -1330 , -1489 , -1472 , -1412 , -1165 , -921 , -838 , -1059 , 
		-1142 , -998 , -852 , -1077 , -1338 , -1421 , -1207 , -802 , -730 , -626 , 
		-532 , -506 , -523 , -487 , -499 , -324 , -108 , 105 , 301 , 91 , 
		313 , 176 , 307 , 292 , 215 , 324 , 650 , 925 , 1149 , 1320 , 
		1158 , 979 , 1028 , 1027 , 1064 , 1141 , 1109 , 1055 , 970 , 953 , 
		919 , 882 , 902 , 740 , 546 , 402 , 406 , 488 , 529 , 547 , 
		515 , 512 , 475 , 372 , 194 , -52 , -278 , -479 , -607 , -727 , 
		-807 , -842 , -867 , -945 , -1040 , -1074 , -1158 , -1241 , -1386 , -1523 , 
		-1506 , -1549 , -1607 , -1818 , -2000 , -2114 , -2209 , -2481 , -2712 , -2836 , 
		-2934 , -2859 , -2610 , -2611 , -2568 , -2448 , -2402 , -2392 , -2247 , -2174 , 
		-2056 , -2030 , -1978 , -1989 , -1971 , -2031 , -2079 , -2121 , -2204 , -2258 , 
		-2263 , -2227 , -2005 , -1768 , -1531 , -1515 , -1654 , -1875 , -1898 , -1735 , 
		-1520 , -1111 , -733 , -364 , -17 , -4 , -44 , 82 , 196 , 488 , 
		714 , 958 , 876 , 673 , 929 , 1253 , 1535 , 1603 , 1333 , 1702 , 
		1525 , 1419 , 1322 , 1122 , 1095 , 1080 , 1190 , 1299 , 1165 , 937 , 
		630 , 395 , 222 , 122 , 108 , 170 , 269 , 204 , 333 , 448 , 
		512 , 464 , 557 , 524 , 416 , 342 , 323 , 287 , 270 , 279 , 
		240 , 122 , -40 , -198 , -307 , -406 , -460 , -525 , -745 , -1053 , 
		-1284 , -1351 , -1388 , -1271 , -1309 , -1422 , -1632 , -1934 , -2084 , -2118 , 
		-2251 , -2406 , -2529 , -2633 , -2791 , -2893 , -3141 , -3320 , -3348 , -3174 , 
		-2898 , -2723 , -2493 , -2367 , -2301 , -2202 , -2220 , -2308 , -2388 , -2548 , 
		-2771 , -2875 , -2902 , -2944 , -2949 , -2894 , -3029 , -3276 , -3261 , -3178 , 
		-2798 , -2456 , -2139 , -1969 , -1776 , -1454 , -1085 , -959 , -823 , -812 , 
		-713 , -601 , -515 , -173 , 163 , 345 , 540 , 716 , 898 , 1182
	};
	
	
	if (acc_z <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveS2SLowerZSecondHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		1371 , 1380 , 1240 , 1012 , 904 , 990 , 1211 , 1320 , 1251 , 1138 , 
		976 , 640 , 347 , 335 , 400 , 458 , 449 , 470 , 489 , 477 , 
		373 , 375 , 432 , 260 , 413 , 493 , 498 , 361 , 272 , 289 , 
		237 , 192 , 187 , 209 , 49 , -28 , -240 , -336 , -578 , -675 , 
		-708 , -924 , -1110 , -1171 , -1166 , -1239 , -1439 , -1451 , -1612 , -1906 , 
		-2117 , -2269 , -2311 , -2320 , -2266 , -2156 , -2212 , -2328 , -2645 , -2914 , 
		-3119 , -3222 , -3257 , -3261 , -3097 , -3023 , -2954 , -2815 , -2620 , -2457 , 
		-2315 , -2352 , -2484 , -3019 , -3501 , -3733 , -3689 , -3536 , -3593 , -3492 , 
		-3179 , -2989 , -2883 , -2851 , -2738 , -2586 , -2422 , -2382 , -2220 , -2239 , 
		-2236 , -2051 , -1545 , -1226 , -873 , -593 , -520 , -673 , -755 , -621 , 
		-337 , 85 , 404 , 636 , 868 , 1116 , 1166 , 979 , 853 , 803 , 
		823 , 725 , 686 , 754 , 781 , 678 , 414 , 138 , -161 , -355 , 
		-374 , -278 , -59 , -67 , -118 , -137 , -135 , -203 , -348 , -378 , 
		-296 , -152 , -86 , -208 , -111 , -362 , -544 , -626 , -684 , -694 , 
		-755 , -869 , -868 , -931 , -1056 , -1363 , -1564 , -1830 , -1979 , -2026 , 
		-2189 , -2307 , -2310 , -2200 , -2171 , -2069 , -2078 , -2157 , -2135 , -2052 , 
		-2118 , -2292 , -2399 , -2463 , -2529 , -2539 , -2547 , -2489 , -2405 , -2335 , 
		-2270 , -2301 , -2596 , -2859 , -3056 , -3177 , -3316 , -3459 , -3582 , -3749 , 
		-3681 , -3635 , -3509 , -3244 , -2988 , -3015 , -2980 , -3043 , -3115 , -3123 , 
		-2928 , -2838 , -2608 , -2471 , -2399 , -2312 , -2053 , -1909 , -2029 , -2042 , 
		-2167 , -1929 , -1800 , -1436 , -1108 , -926 , -804 , -473 , -65 , 210 , 
	//	232 , 192 , 228 , 552 , 823 , 936 , 733 , 632 , 500 , 409 , 
	//	450 , 310 , 315 , 295 , 265 , 129 , 25 , -83 , -330 , -778 , 
	//	-1196 , -1415 , -1582 , -1799 , -1755 , -1913 , -1994 , -1543 , -884 , -226 , 
	//	189 , 92 , -180 , -560 , -955 , -1523 , -1737 , -1973 , -2249 , -2443
	};
	
	
	if (acc_z <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}


// Hand wave backwards + forwards

int checkActivityHandWaveFBUpperXFirstHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		218 , 164 , 94 , 42 , -31 , -181 , -212 , -216 , -96 , -170 , -221 , 
		-230 , -295 , -218 , 1 , -6 , 9 , -41 , -2 , 137 , 555 , 
		1018 , 1378 , 1312 , 1308 , 1112 , 1065 , 1217 , 1323 , 1313 , 996 , 
		739 , 863 , 1171 , 1659 , 1940 , 1823 , 1454 , 1233 , 1437 , 1373 , 
		1444 , 1733 , 1916 , 1963 , 1919 , 2043 , 1962 , 1896 , 1678 , 1573 , 
		1669 , 2015 , 2370 , 2649 , 2775 , 2586 , 2171 , 1839 , 1608 , 1514 , 
		1480 , 1550 , 1770 , 1988 , 2072 , 2128 , 1790 , 1621 , 1309 , 1045 , 
		723 , 623 , 604 , 777 , 814 , 964 , 1196 , 1260 , 1018 , 819 , 
		615 , 337 , 266 , 452 , 681 , 815 , 727 , 639 , 441 , 277 , 
		267 , 181 , 83 , 76 , 34 , -17 , -35 , -22 , 25 , 37 , 
		52 , 38 , -31 , -124 , -168 , -188 , -142 , -111 , 31 , 85 , 
		100 , 94 , 32 , 56 , 82 , 76 , 84 , 108 , 133 , 120 , 
		152 , 116 , 154 , 149 , 89 , 30 , 29 , 20 , 48 , 24 , 
		35 , -22 , -116 , -199 , -265 , -390 , -488 , -438 , -414 , -334 , 
		-356 , -339 , -322 , -291 , -202 , -116 , 5 , 255 , 398 , 642 , 
		809 , 900 , 1044 , 953 , 926 , 905 , 886 , 1007 , 1212 , 1439 , 
		1811 , 1759 , 1774 , 1822 , 1842 , 1897 , 1912 , 1716 , 1597 , 1736 , 
		1711 , 1671 , 1783 , 2029 , 2126 , 2094 , 2210 , 2359 , 2460 , 2192 , 
		2546 , 2465 , 2110 , 1801 , 1647 , 1617 , 1582 , 1618 , 1677 , 1447 , 
		1308 , 1271 , 1442 , 1572 , 1582 , 1518 , 1651 , 1611 , 1552 , 1543 , 
		1529 , 1443 , 1146 , 986 , 982 , 1012 , 1083 , 1004 , 825 , 775 , 
		635 , 393 , 212 , 95 , 48 , 253 , 365 , 438 , 356 , 354 , 
		309 , 233 , 226 , 154 , 24 , -34 , -55 , 1 , 87 , 100 , 
		28 , -19 , 10 , 62 , 172 , 231 , 182 , 68 , -63 , -113 , 
		-48 , 54 , 116 , 45 , -42 , -102 , -100 , -22 , 80 , 182
	};
	
	
	if (acc_x >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBUpperXSecondHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		209 , 227 , 210 , 256 , 342 , 393 , 431 , 405 , 342 , 335 , 
		266 , 168 , 55 , -51 , -53 , 19 , 128 , 193 , 179 , 33 , 
		-44 , -8 , 73 , 206 , 326 , 288 , 155 , 206 , 249 , 306 , 
		373 , 301 , 344 , 325 , 379 , 738 , 1178 , 1393 , 1625 , 1960 , 
		1980 , 1982 , 1723 , 1704 , 1607 , 1484 , 1445 , 1473 , 1521 , 1466 , 
		1250 , 1600 , 1878 , 1937 , 1945 , 2070 , 2178 , 2157 , 2123 , 2024 , 
		2010 , 1810 , 1661 , 1773 , 1826 , 2027 , 2205 , 2274 , 2325 , 2303 , 
		2413 , 2226 , 2005 , 1772 , 1739 , 1742 , 1867 , 1880 , 2064 , 2113 , 
		1974 , 1753 , 1444 , 1125 , 935 , 927 , 995 , 1002 , 1013 , 950 , 
		878 , 750 , 555 , 372 , 339 , 216 , 122 , 166 , 210 , 363 , 
		342 , 515 , 498 , 377 , 365 , 180 , 209 , 311 , 456 , 463 , 
		499 , 392 , 294 , 262 , 191 , 242 , 268 , 202 , 187 , 157 , 
		71 , 124 , 225 , 318 , 374 , 425 , 431 , 374 , 336 , 303 , 
		288 , 190 , 207 , 265 , 414 , 543 , 696 , 703 , 658 , 588 , 
		540 , 520 , 477 , 384 , 291 , 337 , 506 , 625 , 664 , 626 , 
		538 , 358 , 216 , 24 , -193 , -290 , -295 , -200 , -120 , 24 , 
		220 , 335 , 432 , 619 , 746 , 715 , 653 , 343 , 131 , 114 , 
		267 , 411 , 776 , 1137 , 1292 , 1212 , 1163 , 1389 , 1538 , 1559 , 
		1480 , 1154 , 749 , 423 , 585 , 1278 , 1760 , 2070 , 2055 , 1904 , 
		1391 , 1615 , 1848 , 1494 , 1188 , 1190 , 1491 , 1773 , 2062 , 2229 , 
		2314 , 2319 , 2179 , 1954 , 1734 , 1536 , 1299 , 1412 , 1662 , 1786 , 
	//	1769 , 1617 , 1424 , 1381 , 1325 , 1290 , 1242 , 1260 , 1285 , 1314 , 
	//	1335 , 1244 , 1012 , 658 , 413 , 376 , 394 , 551 , 669 , 712 , 
	//	593 , 498 , 332 , 336 , 251 , 97 , 195 , 396 , 681 , 729 , 
	//	714 , 608 , 449 , 280 , 228 , 105 , -67 , -165 , -159 , 24
	};
	
	
	if (acc_x >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBLowerXFirstHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		-882 , -864 , -930 , -1038 , -1131 , -1125 , -1256 , -1388 , -1540 , -1590 , -1577 , 
		-1574 , -1563 , -1590 , -1603 , -1542 , -1411 , -1249 , -1158 , -1059 , -1333 , 
		-1570 , -1686 , -1568 , -1312 , -1088 , -1111 , -1363 , -1561 , -1475 , -1180 , 
		-1013 , -1245 , -1497 , -1993 , -2248 , -1961 , -1382 , -1103 , -1163 , -1123 , 
		-1108 , -1227 , -1248 , -1045 , -1085 , -1269 , -1182 , -1036 , -774 , -623 , 
		-555 , -737 , -1034 , -1239 , -1393 , -1302 , -985 , -657 , -360 , -298 , 
		-344 , -446 , -674 , -1016 , -1228 , -1420 , -1330 , -1371 , -1267 , -1179 , 
		-1045 , -1085 , -1116 , -1295 , -1394 , -1560 , -1880 , -2228 , -2258 , -2097 , 
		-1861 , -1567 , -1394 , -1528 , -1683 , -1729 , -1685 , -1557 , -1327 , -1195 , 
		-1193 , -1239 , -1297 , -1440 , -1482 , -1537 , -1519 , -1478 , -1439 , -1387 , 
		-1340 , -1330 , -1279 , -1256 , -1256 , -1272 , -1390 , -1459 , -1617 , -1731 , 
		-1740 , -1670 , -1572 , -1536 , -1438 , -1348 , -1316 , -1272 , -1231 , -1148 , 
		-1072 , -980 , -970 , -951 , -927 , -978 , -1035 , -1148 , -1236 , -1344 , 
		-1493 , -1602 , -1688 , -1739 , -1825 , -1854 , -1844 , -1926 , -2006 , -2110 , 
		-2104 , -2167 , -2182 , -2207 , -2358 , -2424 , -2503 , -2705 , -2758 , -2830 , 
		-2767 , -2740 , -2692 , -2491 , -2254 , -2087 , -1874 , -1801 , -1792 , -1645 , 
		-1737 , -1693 , -1694 , -1722 , -1758 , -2007 , -1920 , -1724 , -1459 , -1264 , 
		-1173 , -1041 , -1169 , -1191 , -1250 , -1242 , -1306 , -1577 , -1676 , -1396 , 
		-1594 , -1423 , -1078 , -827 , -697 , -739 , -762 , -842 , -1007 , -973 , 
		-980 , -929 , -990 , -1064 , -1030 , -1014 , -1153 , -1205 , -1340 , -1581 , 
		-1639 , -1721 , -1586 , -1418 , -1418 , -1476 , -1517 , -1436 , -1315 , -1301 , 
		-1277 , -1143 , -1096 , -1185 , -1192 , -1343 , -1383 , -1378 , -1240 , -1030 , 
		-923 , -907 , -1010 , -1006 , -1024 , -1142 , -1163 , -1207 , -1337 , -1460 , 
		-1464 , -1483 , -1510 , -1550 , -1732 , -1877 , -2006 , -1964 , -1915 , -1957
	};
	
	
	if (acc_x <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBLowerXSecondHalf(int16_t acc_x, int idx)
{
	int16_t boundarySegment[] = 
	{
		-1715 , -1665 , -1634 , -1604 , -1602 , -1543 , -1501 , -1355 , -1274 , -1285 , 
		-1234 , -1204 , -1141 , -1175 , -1341 , -1521 , -1768 , -1927 , -2069 , -2091 , 
		-2216 , -2412 , -2639 , -2786 , -2866 , -2860 , -2773 , -2730 , -2795 , -2798 , 
		-2867 , -2855 , -2952 , -2947 , -2921 , -2958 , -3002 , -2831 , -2687 , -2556 , 
		-2312 , -2198 , -2033 , -1976 , -1737 , -1472 , -1215 , -1099 , -1167 , -1178 , 
		-1278 , -1544 , -1678 , -1755 , -1779 , -1694 , -1514 , -1283 , -1129 , -1052 , 
		-1094 , -1014 , -955 , -971 , -1038 , -1061 , -1059 , -1102 , -1147 , -1189 , 
		-1347 , -1286 , -1243 , -1156 , -1209 , -1238 , -1349 , -1436 , -1664 , -1719 , 
		-1634 , -1471 , -1240 , -1027 , -905 , -961 , -1097 , -1206 , -1303 , -1398 , 
		-1390 , -1390 , -1361 , -1376 , -1485 , -1456 , -1466 , -1462 , -1426 , -1449 , 
		-1402 , -1441 , -1322 , -1155 , -1123 , -1068 , -1255 , -1529 , -1752 , -1917 , 
		-2093 , -2076 , -1958 , -1926 , -1889 , -1862 , -1836 , -1746 , -1689 , -1663 , 
		-1593 , -1700 , -1823 , -1926 , -2006 , -2107 , -2149 , -2082 , -2028 , -1873 , 
		-1680 , -1462 , -1293 , -1191 , -1206 , -1209 , -1292 , -1337 , -1342 , -1348 , 
		-1376 , -1368 , -1271 , -1192 , -1137 , -1319 , -1534 , -1707 , -1856 , -1902 , 
		-1934 , -1914 , -1892 , -1824 , -1765 , -1806 , -1839 , -1924 , -2040 , -2232 , 
		-2348 , -2365 , -2460 , -2653 , -2846 , -2905 , -2839 , -2685 , -2625 , -2654 , 
		-2693 , -2697 , -2732 , -2771 , -2692 , -2648 , -2601 , -2711 , -2746 , -2661 , 
		-2436 , -2134 , -1739 , -1373 , -1359 , -1770 , -2008 , -1982 , -1697 , -1328 , 
		-909 , -1117 , -1436 , -1458 , -1360 , -1426 , -1661 , -1887 , -2038 , -1963 , 
		-1910 , -1797 , -1645 , -1486 , -1342 , -1168 , -1113 , -1372 , -1686 , -1758 , 
	//	-1687 , -1595 , -1504 , -1499 , -1435 , -1350 , -1318 , -1300 , -1395 , -1638 , 
	//	-1785 , -1856 , -1692 , -1414 , -1183 , -1120 , -1230 , -1449 , -1587 , -1668 , 
	//	-1539 , -1482 , -1444 , -1452 , -1361 , -1267 , -1261 , -1376 , -1475 , -1447 , 
	//	-1422 , -1304 , -1271 , -1252 , -1368 , -1427 , -1311 , -1253 , -1307 , -1480
	};
	
	
	if (acc_x <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBUpperYFirstHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-2856 , -2702 , -2507 , -2298 , -2054 , -1799 , -1554 , -1369 , -1112 , -882 , -638 , 
		-325 , -28 , 258 , 507 , 851 , 1094 , 1313 , 1565 , 1830 , 2037 , 
		2120 , 2162 , 2272 , 2336 , 2329 , 2411 , 2246 , 2214 , 2329 , 2225 , 
		2058 , 2087 , 2230 , 2113 , 1950 , 2220 , 2124 , 1897 , 1726 , 1469 , 
		1405 , 1430 , 1260 , 1507 , 1563 , 1489 , 1380 , 1368 , 1387 , 1408 , 
		1265 , 1211 , 1107 , 1130 , 1008 , 979 , 1088 , 1142 , 1236 , 1228 , 
		1275 , 1389 , 1540 , 1629 , 1738 , 1909 , 2046 , 2180 , 2283 , 2455 , 
		2608 , 2791 , 2895 , 2864 , 2941 , 2973 , 3036 , 3324 , 3336 , 3214 , 
		3091 , 2879 , 2643 , 2377 , 2155 , 2039 , 1924 , 1752 , 1582 , 1484 , 
		1367 , 1173 , 965 , 888 , 734 , 577 , 306 , 15 , -188 , -366 , 
		-545 , -667 , -796 , -851 , -1007 , -1212 , -1368 , -1521 , -1622 , -1684 , 
		-1727 , -1816 , -1917 , -2009 , -2110 , -2251 , -2393 , -2462 , -2547 , -2684 , 
		-2744 , -2762 , -2768 , -2788 , -2761 , -2708 , -2636 , -2538 , -2395 , -2258 , 
		-2134 , -1971 , -1773 , -1518 , -1223 , -943 , -628 , -321 , -34 , 205 , 
		429 , 639 , 912 , 1248 , 1581 , 2159 , 2613 , 2806 , 2927 , 3040 , 
		3179 , 3325 , 3351 , 3523 , 3707 , 3660 , 3579 , 3725 , 3769 , 3739 , 
		3549 , 3396 , 3210 , 3207 , 2870 , 2940 , 2954 , 3126 , 3298 , 3580 , 
		3495 , 3047 , 2701 , 2193 , 1771 , 1630 , 1630 , 1783 , 1872 , 1710 , 
		1693 , 1672 , 1514 , 1555 , 1637 , 1654 , 1640 , 1603 , 1634 , 1523 , 
		1581 , 1632 , 1763 , 1944 , 2055 , 2089 , 2126 , 2082 , 2023 , 1989 , 
		2073 , 2196 , 2333 , 2407 , 2473 , 2564 , 2573 , 2617 , 2771 , 2845 , 
		2986 , 3189 , 3145 , 3102 , 3094 , 2976 , 2834 , 2661 , 2434 , 2295 , 
		2347 , 2235 , 1959 , 1754 , 1566 , 1351 , 1052 , 744 , 532 , 306 , 
		168 , 36 , -31 , -160 , -379 , -588 , -733 , -906 , -1067 , -1172 , 
		-1253 , -1347 , -1472 , -1629 , -1680 , -1805 , -1895 , -1911 , -1942 , -1995
	};
	
	
	if (acc_y >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBUpperYSecondHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-1964 , -1925 , -1931 , -1868 , -1780 , -1644 , -1585 , -1313 , -1147 , -866 , 
		-624 , -386 , -173 , 52 , 306 , 435 , 662 , 806 , 1030 , 1337 , 
		1678 , 1812 , 1881 , 2069 , 2187 , 2338 , 2513 , 2745 , 2938 , 2968 , 
		2885 , 3071 , 3197 , 3409 , 3545 , 3632 , 3731 , 3483 , 3498 , 3507 , 
		3312 , 3438 , 3672 , 3878 , 3880 , 3690 , 3585 , 3535 , 3681 , 3301 , 
		3417 , 3502 , 3590 , 3900 , 3341 , 3027 , 2790 , 2452 , 2482 , 2638 , 
		2839 , 2811 , 2844 , 2652 , 2557 , 2616 , 2510 , 2317 , 2206 , 2090 , 
		1793 , 1694 , 1723 , 1574 , 1706 , 1559 , 1548 , 1623 , 1611 , 1644 , 
		1741 , 1965 , 2097 , 2244 , 2492 , 2551 , 2517 , 2794 , 3042 , 3167 , 
		3388 , 3541 , 3505 , 3421 , 3398 , 3359 , 3366 , 3309 , 3076 , 2798 , 
		2474 , 2240 , 2104 , 1782 , 1668 , 1652 , 1597 , 1474 , 1396 , 1294 , 
		1043 , 688 , 482 , 210 , -68 , -263 , -390 , -545 , -703 , -721 , 
		-919 , -1061 , -1160 , -1248 , -1286 , -1277 , -1136 , -1050 , -952 , -806 , 
		-663 , -531 , -310 , -176 , -28 , 180 , 267 , 409 , 595 , 684 , 
		733 , 762 , 604 , 303 , 478 , 361 , 545 , 772 , 941 , 1007 , 
		1122 , 1282 , 1424 , 1563 , 1697 , 2010 , 2220 , 2298 , 2424 , 2578 , 
		2619 , 2739 , 2928 , 3158 , 3125 , 3124 , 3274 , 3512 , 3702 , 4112 , 
		4443 , 4185 , 3848 , 3817 , 3741 , 3792 , 3950 , 3804 , 4045 , 4059 , 
		4106 , 3943 , 3952 , 4275 , 4373 , 4558 , 4622 , 4625 , 4566 , 4260 , 
		4058 , 3930 , 3575 , 3353 , 3620 , 3802 , 3970 , 4101 , 4115 , 4002 , 
		3720 , 3582 , 3496 , 3264 , 3035 , 2810 , 2714 , 2634 , 2494 , 2425 , 
	//	2419 , 2539 , 2626 , 2773 , 2897 , 3037 , 3219 , 3303 , 3516 , 3708 , 
	//	3926 , 4031 , 4241 , 4481 , 4673 , 4810 , 5079 , 5185 , 5284 , 5209 , 
	//	4982 , 4586 , 4370 , 4311 , 4109 , 4007 , 3863 , 3647 , 3314 , 3100 , 
	//	2802 , 2468 , 2199 , 2097 , 1766 , 1500 , 1257 , 1027 , 775 , 452
	};
	
	if (acc_y >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBLowerYFirstHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-4336 , -4234 , -4227 , -4218 , -4138 , -4047 , -3950 , -3769 , -3592 , -3426 , -3266 , 
		-3165 , -3012 , -2866 , -2705 , -2637 , -2518 , -2363 , -2183 , -2086 , -1915 , 
		-1892 , -1854 , -1836 , -1796 , -1723 , -1753 , -1598 , -1526 , -1567 , -1523 , 
		-1414 , -1365 , -1234 , -1275 , -1458 , -1760 , -1984 , -2139 , -2198 , -1811 , 
		-1795 , -1862 , -1856 , -1977 , -2061 , -2123 , -2140 , -2148 , -2245 , -2380 , 
		-2399 , -2437 , -2413 , -2358 , -2272 , -2217 , -2164 , -2030 , -2032 , -2012 , 
		-1969 , -1911 , -1864 , -1835 , -1762 , -1743 , -1754 , -1780 , -1705 , -1677 , 
		-1660 , -1613 , -1573 , -1540 , -1739 , -1655 , -1748 , -2004 , -2108 , -2074 , 
		-2061 , -2081 , -2053 , -2007 , -1953 , -2129 , -2288 , -2412 , -2478 , -2640 , 
		-2813 , -2907 , -2999 , -3128 , -3206 , -3303 , -3314 , -3269 , -3272 , -3334 , 
		-3457 , -3551 , -3636 , -3775 , -3851 , -3868 , -3920 , -3957 , -4086 , -4216 , 
		-4335 , -4380 , -4433 , -4453 , -4430 , -4387 , -4317 , -4290 , -4239 , -4160 , 
		-4104 , -4066 , -4032 , -3992 , -3989 , -3988 , -3980 , -4022 , -4051 , -4070 , 
		-4122 , -4155 , -4177 , -4202 , -4239 , -4247 , -4280 , -4233 , -4146 , -4043 , 
		-3955 , -3753 , -3552 , -3368 , -3175 , -3161 , -3003 , -2654 , -2277 , -1928 , 
		-1581 , -1363 , -1173 , -1141 , -1101 , -836 , -657 , -531 , -455 , -421 , 
		-335 , -352 , -450 , -409 , -482 , -464 , -462 , -474 , -726 , -1276 , 
		-1497 , -1373 , -1239 , -951 , -893 , -1130 , -1406 , -1621 , -1692 , -1618 , 
		-1723 , -1840 , -1818 , -1861 , -1943 , -1982 , -1992 , -2005 , -2018 , -1909 , 
		-1819 , -1796 , -1765 , -1836 , -1865 , -1763 , -1698 , -1574 , -1437 , -1303 , 
		-1263 , -1264 , -1223 , -1197 , -1171 , -1264 , -1319 , -1371 , -1513 , -1603 , 
		-1790 , -1995 , -2067 , -2150 , -2242 , -2332 , -2494 , -2591 , -2714 , -2809 , 
		-3033 , -3233 , -3237 , -3286 , -3290 , -3369 , -3292 , -3288 , -3320 , -3350 , 
		-3452 , -3516 , -3619 , -3684 , -3715 , -3756 , -3785 , -3766 , -3791 , -3876 , 
		-3957 , -4003 , -4056 , -4089 , -4160 , -4181 , -4159 , -4239 , -4314 , -4343
	};
	
	
	if (acc_y <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBLowerYSecondHalf(int16_t acc_y, int idx)
{
	int16_t boundarySegment[] = 
	{
		-4428 , -4497 , -4531 , -4612 , -4712 , -4792 , -4853 , -5033 , -5119 , -5282 , 
		-5400 , -5522 , -5561 , -5588 , -5622 , -5545 , -5514 , -5438 , -5402 , -5359 , 
		-5318 , -5132 , -4879 , -4671 , -4429 , -4186 , -3947 , -3679 , -3450 , -3084 , 
		-2695 , -2373 , -2167 , -1987 , -1719 , -1532 , -1205 , -869 , -794 , -613 , 
		-380 , -394 , -548 , -530 , -592 , -690 , -859 , -1049 , -1451 , -1511 , 
		-1711 , -1586 , -1494 , -1660 , -1431 , -1269 , -1198 , -1176 , -1266 , -1466 , 
		-1685 , -1769 , -1848 , -1772 , -1779 , -1792 , -1746 , -1659 , -1586 , -1518 , 
		-1375 , -1338 , -1341 , -1266 , -1258 , -1245 , -1180 , -1165 , -1169 , -1088 , 
		-1079 , -1155 , -1287 , -1440 , -1596 , -1653 , -1651 , -1758 , -1930 , -2037 , 
		-2164 , -2307 , -2443 , -2443 , -2550 , -2629 , -2770 , -2927 , -3008 , -3066 , 
		-3066 , -3124 , -3252 , -3302 , -3424 , -3600 , -3807 , -3862 , -3908 , -4038 , 
		-4033 , -3976 , -3950 , -3914 , -3872 , -3831 , -3910 , -3885 , -3835 , -3893 , 
		-3875 , -3921 , -4012 , -4080 , -4226 , -4405 , -4656 , -4858 , -5052 , -5302 , 
		-5527 , -5683 , -5906 , -6080 , -6248 , -6368 , -6501 , -6587 , -6657 , -6728 , 
		-6727 , -6694 , -6496 , -6169 , -6182 , -5999 , -5975 , -5996 , -5987 , -5909 , 
		-5874 , -5802 , -5720 , -5657 , -5607 , -5642 , -5608 , -5470 , -5352 , -5222 , 
		-4957 , -4769 , -4648 , -4414 , -4131 , -3836 , -3658 , -3468 , -3170 , -2976 , 
		-2829 , -2327 , -1884 , -1523 , -1351 , -1244 , -886 , -588 , -551 , -425 , 
		-394 , -297 , -480 , -401 , -375 , -526 , -558 , -631 , -806 , -864 , 
		-702 , -602 , -501 , -627 , -1024 , -1210 , -1422 , -1583 , -1601 , -1506 , 
		-1396 , -1546 , -1556 , -1540 , -1529 , -1490 , -1502 , -1610 , -1534 , -1559 , 
	//	-1477 , -1513 , -1510 , -1579 , -1579 , -1667 , -1825 , -1949 , -2108 , -2204 , 
	//	-2310 , -2389 , -2539 , -2623 , -2747 , -2850 , -3033 , -3179 , -3312 , -3435 , 
	//	-3478 , -3482 , -3506 , -3605 , -3731 , -3837 , -4049 , -4177 , -4258 , -4276 , 
	//	-4334 , -4320 , -4317 , -4435 , -4438 , -4416 , -4435 , -4453 , -4497 , -4432
	};
	
	
	if (acc_y <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}


int checkActivityHandWaveFBUpperZFirstHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		534 , 249 , -38 , -456 , -1238 , -1887 , -2601 , -3138 , -3466 , -3524 , -3501 , 
		-3354 , -3155 , -2632 , -1971 , -1163 , -349 , 622 , 1712 , 2710 , 3701 , 
		4793 , 5735 , 7026 , 7738 , 8331 , 8976 , 9492 , 9668 , 9547 , 9638 , 
		10001 , 9686 , 9509 , 9696 , 9971 , 10221 , 9942 , 9468 , 9573 , 9489 , 
		9315 , 9231 , 9417 , 9459 , 9563 , 9468 , 9347 , 9268 , 9187 , 9266 , 
		9231 , 9043 , 8785 , 8669 , 8537 , 8478 , 8591 , 8805 , 8993 , 9199 , 
		9312 , 9611 , 9839 , 10143 , 10588 , 10845 , 10939 , 11100 , 11279 , 11286 , 
		11155 , 10973 , 10730 , 10352 , 9935 , 9754 , 9415 , 8676 , 8074 , 7792 , 
		7365 , 6855 , 6441 , 5914 , 5370 , 4972 , 4763 , 4542 , 3796 , 2963 , 
		2086 , 1403 , 930 , 349 , -100 , -574 , -1010 , -1291 , -1534 , -1870 , 
		-2111 , -2271 , -2522 , -2607 , -2479 , -2216 , -1858 , -1406 , -980 , -588 , 
		-288 , 14 , 298 , 540 , 539 , 385 , 124 , 25 , -74 , -210 , 
		-407 , -508 , -672 , -775 , -793 , -837 , -778 , -684 , -579 , -543 , 
		-628 , -682 , -784 , -975 , -1239 , -1570 , -1919 , -2137 , -2366 , -2452 , 
		-2550 , -2566 , -2584 , -2420 , -2168 , -1731 , -825 , 456 , 1766 , 3030 , 
		4254 , 5544 , 6480 , 7338 , 8202 , 9142 , 10187 , 10885 , 11227 , 11225 , 
		11286 , 11239 , 11138 , 11083 , 10812 , 10633 , 10604 , 10682 , 10987 , 11216 , 
		11236 , 10666 , 9998 , 9554 , 9361 , 9282 , 9440 , 9391 , 9274 , 9080 , 
		9379 , 9539 , 9798 , 9996 , 9677 , 8942 , 8426 , 8935 , 9550 , 10031 , 
		10471 , 10741 , 10887 , 10939 , 11036 , 11172 , 11425 , 11678 , 11739 , 11900 , 
		12013 , 12245 , 12347 , 12172 , 12190 , 12289 , 11966 , 11701 , 11496 , 11129 , 
		10264 , 9423 , 8753 , 8150 , 7447 , 6833 , 6483 , 6210 , 5820 , 5055 , 
		4176 , 3431 , 2668 , 1819 , 1299 , 996 , 652 , 261 , -152 , -253 , 
		-458 , -695 , -1166 , -1472 , -1692 , -1893 , -2005 , -2131 , -2309 , -2320 , 
		-2354 , -2156 , -1888 , -1662 , -1405 , -1008 , -633 , -71 , 348 , 605
	};
	
	
	if (acc_z >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBUpperZSecondHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		797 , 822 , 637 , 484 , 477 , 434 , 542 , 522 , 401 , 240 , 
		104 , -41 , -176 , -174 , -113 , 17 , 249 , 447 , 381 , 428 , 
		780 , 1370 , 2117 , 2687 , 3314 , 3481 , 3557 , 3874 , 4648 , 5619 , 
		6572 , 6874 , 7349 , 7974 , 8553 , 8894 , 9526 , 10199 , 10825 , 11158 , 
		11697 , 11779 , 11654 , 11775 , 12199 , 12434 , 12614 , 12575 , 12160 , 11556 , 
		11008 , 10671 , 10726 , 10557 , 10119 , 9398 , 8782 , 8573 , 8973 , 9349 , 
		9091 , 8882 , 9152 , 9490 , 9711 , 10246 , 10737 , 11220 , 11749 , 12208 , 
		12575 , 13037 , 13327 , 13348 , 13422 , 13651 , 14002 , 14294 , 14472 , 14532 , 
		14519 , 14528 , 14575 , 14513 , 14407 , 14321 , 14221 , 14036 , 13476 , 12498 , 
		11496 , 10687 , 9682 , 9175 , 8598 , 7667 , 6584 , 5697 , 5052 , 4671 , 
		4385 , 4208 , 3807 , 3650 , 3313 , 3014 , 2227 , 1437 , 528 , 123 , 
		-260 , -382 , -377 , -372 , -496 , -966 , -1377 , -1776 , -2084 , -1973 , 
		-1946 , -1694 , -1421 , -1048 , -636 , -214 , 200 , 534 , 918 , 1142 , 
		1052 , 936 , 927 , 937 , 1165 , 1332 , 1504 , 1595 , 1790 , 2264 , 
		2864 , 3472 , 4202 , 5016 , 4830 , 4595 , 4390 , 4194 , 4080 , 4089 , 
		4113 , 4130 , 4165 , 4250 , 4466 , 4664 , 4998 , 5261 , 5550 , 5875 , 
		6370 , 6876 , 7064 , 7498 , 8313 , 8368 , 8470 , 8692 , 9018 , 9336 , 
		9802 , 10725 , 11651 , 12005 , 12281 , 12560 , 12955 , 13295 , 13070 , 13355 , 
		13495 , 13127 , 12891 , 12716 , 12288 , 11965 , 11749 , 11695 , 11138 , 10849 , 
		11115 , 11277 , 11776 , 11950 , 11886 , 11928 , 11896 , 11982 , 12430 , 12763 , 
		12898 , 13126 , 13329 , 13591 , 13792 , 13942 , 13954 , 13960 , 13961 , 14065 , 
	//	14222 , 14317 , 14323 , 14373 , 14479 , 14582 , 14699 , 14623 , 14582 , 14568 , 
	//	14458 , 14380 , 14279 , 14057 , 13582 , 13209 , 12279 , 11103 , 9844 , 9086 , 
	//	8658 , 8590 , 8354 , 7596 , 6899 , 5718 , 4692 , 3717 , 3029 , 2377 , 
	//	2141 , 1952 , 1707 , 1608 , 1752 , 1519 , 998 , 364 , -25 , -244
	};
	if (acc_z >= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBLowerZFirstHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		-6310 , -6999 , -7578 , -7996 , -8174 , -8515 , -8605 , -8798 , -8970 , -9180 , -9281 , 
		-9274 , -9115 , -9032 , -8995 , -8835 , -8417 , -8118 , -7864 , -7598 , -7235 , 
		-6627 , -5761 , -5258 , -4534 , -3805 , -3184 , -2452 , -1796 , -757 , 342 , 
		829 , 1422 , 2025 , 2528 , 2919 , 3141 , 4130 , 5024 , 5037 , 5069 , 
		5411 , 5587 , 5589 , 5591 , 5727 , 6060 , 6339 , 6456 , 6671 , 6666 , 
		6755 , 7067 , 7413 , 7565 , 7737 , 7790 , 7595 , 7293 , 6929 , 6599 , 
		6268 , 5703 , 5319 , 4891 , 4356 , 3993 , 3747 , 3424 , 2971 , 2350 , 
		1615 , 821 , 10 , -560 , -1033 , -1830 , -2273 , -2728 , -3166 , -3832 , 
		-4391 , -4825 , -5279 , -5750 , -6314 , -6752 , -7241 , -7746 , -7772 , -7681 , 
		-7462 , -7353 , -7450 , -7451 , -7624 , -7670 , -7614 , -7691 , -7742 , -7634 , 
		-7519 , -7327 , -7050 , -6891 , -6787 , -6792 , -6794 , -6818 , -6780 , -6700 , 
		-6744 , -6726 , -6722 , -6676 , -6485 , -6147 , -5636 , -5319 , -5098 , -4886 , 
		-4723 , -4652 , -4540 , -4531 , -4649 , -4725 , -4954 , -5308 , -5779 , -6255 , 
		-6544 , -6946 , -7416 , -7843 , -8167 , -8346 , -8475 , -8509 , -8594 , -8684 , 
		-8842 , -9142 , -9440 , -9844 , -10048 , -10215 , -10677 , -11148 , -11502 , -11662 , 
		-11658 , -11400 , -10812 , -10086 , -9290 , -8586 , -8245 , -7691 , -6737 , -5295 , 
		-3722 , -2157 , -722 , 379 , 1244 , 2125 , 2224 , 2186 , 2127 , 2240 , 
		2824 , 3914 , 5174 , 6186 , 6549 , 6578 , 6372 , 6435 , 6602 , 6796 , 
		6427 , 6219 , 5982 , 5828 , 6245 , 7206 , 7882 , 7215 , 6410 , 5783 , 
		5207 , 4853 , 4663 , 4535 , 4200 , 3844 , 3373 , 2894 , 2415 , 1640 , 
		761 , -87 , -933 , -1564 , -2166 , -2843 , -3310 , -3803 , -4276 , -4511 , 
		-4512 , -4749 , -5127 , -5582 , -5945 , -6323 , -6713 , -7082 , -7352 , -7433 , 
		-7328 , -7185 , -7120 , -6953 , -6989 , -7164 , -7244 , -7191 , -7104 , -7289 , 
		-7486 , -7747 , -7714 , -7708 , -7628 , -7477 , -7397 , -7263 , -6989 , -6884 , 
		-6822 , -6832 , -6868 , -6858 , -6849 , -6868 , -6885 , -7075 , -7112 , -7043
	};
	
	
	if (acc_z <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}

int checkActivityHandWaveFBLowerZSecondHalf(int16_t acc_z, int idx)
{
	int16_t boundarySegment[] = 
	{
		-6903 , -6814 , -6563 , -6376 , -6395 , -6382 , -6510 , -6606 , -6491 , -6216 , 
		-6016 , -5921 , -5868 , -6002 , -6305 , -6839 , -7583 , -8317 , -8803 , -9420 , 
		-10124 , -10998 , -11855 , -12437 , -13070 , -13435 , -13543 , -13566 , -13580 , -13733 , 
		-13852 , -13430 , -12991 , -12602 , -12235 , -11382 , -10526 , -9789 , -9231 , -8466 , 
		-7699 , -6717 , -5650 , -4377 , -3189 , -1890 , -602 , 515 , 1700 , 2672 , 
		3252 , 3439 , 3022 , 3341 , 4387 , 5922 , 7170 , 7665 , 7145 , 6465 , 
		6731 , 7126 , 6800 , 6306 , 5975 , 5210 , 4473 , 3776 , 2985 , 2308 , 
		1747 , 1045 , 583 , 356 , 102 , -385 , -974 , -1498 , -1900 , -2328 , 
		-2805 , -3316 , -3713 , -3963 , -4197 , -4407 , -4707 , -4924 , -5096 , -5190 , 
		-5440 , -5773 , -5934 , -6161 , -6322 , -6213 , -6096 , -5983 , -6012 , -6157 , 
		-6483 , -6888 , -7221 , -7538 , -7803 , -8042 , -7809 , -7699 , -7480 , -7617 , 
		-7760 , -8002 , -8245 , -8384 , -8548 , -8302 , -8021 , -7844 , -7828 , -8153 , 
		-8302 , -8398 , -8461 , -8488 , -8396 , -8178 , -7936 , -7718 , -7518 , -7210 , 
		-6676 , -6148 , -5913 , -5687 , -5671 , -5596 , -5576 , -5653 , -5830 , -6096 , 
		-6620 , -7204 , -8006 , -8852 , -8898 , -8897 , -8870 , -8934 , -8976 , -9251 , 
		-9575 , -9862 , -10079 , -10226 , -10498 , -10804 , -11298 , -11751 , -12130 , -12489 , 
		-12954 , -13364 , -13636 , -13854 , -14267 , -14308 , -14378 , -14520 , -14774 , -14712 , 
		-14230 , -13819 , -13469 , -12711 , -11971 , -11360 , -10797 , -10025 , -8850 , -8021 , 
		-7045 , -5977 , -4929 , -3968 , -2840 , -1599 , -375 , 735 , 2226 , 3049 , 
		3043 , 2765 , 2500 , 2354 , 2350 , 2296 , 2276 , 2138 , 1850 , 1363 , 
		1130 , 762 , 481 , -137 , -592 , -934 , -970 , -968 , -1015 , -1259 , 
	//	-1586 , -1823 , -1889 , -2171 , -2621 , -2990 , -3497 , -3865 , -4126 , -4416 , 
	//	-4706 , -5004 , -5249 , -5423 , -5450 , -5583 , -5701 , -5889 , -6124 , -6486 , 
	//	-6882 , -7286 , -7706 , -7840 , -7909 , -7538 , -7184 , -6875 , -6615 , -6423 , 
	//	-6543 , -6756 , -6849 , -7084 , -7400 , -7473 , -7230 , -7052 , -7033 , -7160
	};
	
	
	if (acc_z <= boundarySegment[idx])
	{
		return 1;
	}
	return 0;
}