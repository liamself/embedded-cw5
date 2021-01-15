#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"
#include "devSSD1331_FONT.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 3),
};


static int writeData(uint8_t *bytes, uint8_t numberOfBytes)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinDC);

	for (int i = 0; i < numberOfBytes; i++)
	{
		payloadBytes[i] = bytes[i];
	}
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					numberOfBytes	/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;

}

int
writeCommands(uint8_t* commands, uint8_t numberOfCommands)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)commands,
					(uint8_t * restrict)&inBuffer[0],
					numberOfCommands	/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;

	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1	/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

void ssd1331ClearScreen()
{
	
	
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	

}

void
ssd1331DrawPixel(uint8_t x, uint8_t y, uint16_t colour)
{
	//Check pixel is in range
	if (x < 0 || x >= SSD1331_WIDTH || y < 0 || y >= SSD1331_HEIGHT)
	{
		return;
	}

	uint8_t commands[6];

	commands[0] = (kSSD1331CommandSETCOLUMN);
	commands[1] = (x);
	commands[2] = (x);
	commands[3] = (kSSD1331CommandSETROW);
	commands[4] = (y);
	commands[5] = (y);
	
	writeCommands(commands, 6);

	uint8_t colourBytes[2];
	colourBytes[0] = colour >> 8;
	colourBytes[1] = colour;
	writeData(colourBytes, 2);	
}

void ssd1331DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t colour)
{
	//Cap bounds
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;
	if (x0 >= SSD1331_WIDTH) x0 = SSD1331_WIDTH - 1;
	if (x1 >= SSD1331_WIDTH) x1 = SSD1331_WIDTH - 1;
	if (y0 >= SSD1331_HEIGHT) y0 = SSD1331_HEIGHT - 1;
	if (y1 >= SSD1331_HEIGHT) y1 = SSD1331_HEIGHT - 1;

	uint8_t commands[8];

	commands[0] = (kSSD1331CommandDRAWLINE);
	commands[1] = (x0);
	commands[2] = (y0);
	commands[3] = (x1);
	commands[4] = (y1);
	commands[5] = (R(colour));
	commands[6] = (G(colour));
	commands[7] = (B(colour));

	writeCommands(commands, 8);
}


void ssd1331DrawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t lineColour, uint16_t fillColour)
{
	//Cap bounds
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;
	if (x0 >= SSD1331_WIDTH) x0 = SSD1331_WIDTH - 1;
	if (x1 >= SSD1331_WIDTH) x1 = SSD1331_WIDTH - 1;
	if (y0 >= SSD1331_HEIGHT) y0 = SSD1331_HEIGHT - 1;
	if (y1 >= SSD1331_HEIGHT) y1 = SSD1331_HEIGHT - 1;

	writeCommand(kSSD1331CommandDRAWRECT);
	writeCommand(x0);
	writeCommand(y0);
	writeCommand(x1);
	writeCommand(y1);	
	writeCommand(R(lineColour));
	writeCommand(G(lineColour));
	writeCommand(B(lineColour));
	writeCommand(R(fillColour));
	writeCommand(G(fillColour));
	writeCommand(B(fillColour));
}


void 
ssd1331DrawChar(uint8_t x, uint8_t y, char character, uint16_t colour)
{
	//Check char will fit on screen
	if (x < 0 || x > (SSD1331_WIDTH - 6) || y < 0 || y > (SSD1331_HEIGHT - 8))
	{
		return;
	}


	//Start position in font array
	uint16_t start = (character - 32) * 6; 
	uint8_t xpos = x;
	//Print character
	for (int i = start ; i < (start + 6); i++)
	{
		uint8_t col = ssd1331font[i];
		if (col == 0) 
		{
			xpos++;
			continue; //Skip blank lines
		}
		//SEGGER_RTT_printf(0, "0x%02x \n", col);
		int line = 0;
		for (int j = y; j < (y + 8); j++)
		{
			if (col & (1 << (j - y))) //If pixel is set, draw
			{
				//SEGGER_RTT_printf(0, "*", col);
				line++;
				//ssd1331DrawPixel(xpos, ypos, colour);
			}
			else if (line > 0)
			{
				//Draw line
				ssd1331DrawLine(xpos, j - line, xpos, j - 1, colour);
				line = 0;
			}
		}
		if (line > 0)
		{
			//Draw line
			ssd1331DrawLine(xpos, (y + 8) - line, xpos, y + 7, colour);
			line = 0;
		}
		xpos++;
	}

}

void
ssd1331WriteLine(uint8_t x, uint8_t y, char* str, uint16_t colour)
{
	//Check bounds
	if (x < 0 || y < 0) return;

	//SEGGER_RTT_printf(0, str);
	//Write until end of line, then wrap to next line
	int charIdx = 0;
	int xpos = x;
	int ypos = y;
	
	char character = str[charIdx++];
	while (character != '\0') //Read until NULL character
	{		
		//SEGGER_RTT_printf(0, character);
		if (xpos > (SSD1331_WIDTH - 6)) //Move cursor
		{
			xpos = 0;
			ypos += 8;
		}
		if (ypos > (SSD1331_HEIGHT - 8))
		{
			return; //Out of room
		}
		
		//SEGGER_RTT_printf(0, character);
		//SEGGER_RTT_printf(0, "%d: Draw to %d, %d\n", character, xpos, ypos);
		ssd1331DrawChar(xpos, ypos, character, colour);

		xpos += 7;
		character = str[charIdx++];

	}
	//SEGGER_RTT_printf(0, "Stop writing");
		
}




int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 3u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with initialization sequence...\n");

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
//	SEGGER_RTT_WriteString(0, "\r\n\tDone with enabling fill...\n");

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	SEGGER_RTT_WriteString(0, "\r\n\tDone with screen clear...\n");


	//Set current to high
	writeCommand(kSSD1331CommandMASTERCURRENT);
	writeCommand(0x0F);

	//Set constrast to high for green component
	writeCommand(kSSD1331CommandCONTRASTB);
	writeCommand(0xFF);

	//Draw rectangle
	//ssd1331DrawRectangle(0,0, 90, 37, RGB(255,0,0), RGB(0,255,0));

	//Draw Line
	ssd1331DrawLine(0,0,95, 63, RGB(0,0,255));
	ssd1331DrawPixel(30,30, RGB(255,0,0));
	
	ssd1331DrawPixel(30,37, RGB(255,0,0));
	ssd1331DrawPixel(32,30, RGB(255,255,0));
	ssd1331DrawPixel(12,30, RGB(255,32,76));
	ssd1331WriteLine(0, 0, "d!", RGB(255,255,255));
	writeCommand(0x2E);

	/*
	 *	Read the manual for the SSD1331 (SSD1331_1.2.pdf) to figure
	 *	out how to fill the entire screen with the brightest shade
	 *	of green.
	 */

	


	//SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle...\n");





	return 0;
}

