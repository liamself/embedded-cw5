/*
 *	See https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino for the Arduino driver.
 */

#define SSD1331_WIDTH 96
#define SSD1331_HEIGHT 64
#define RGB(R,G,B) (((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3)) // Avoids needing to specify separate component colours in commands
#define R(COLOUR) ((COLOUR >> 11) & 0x1F) //Recovers R component from RGB encoding
#define G(COLOUR) ((COLOUR >> 5) & 0x3F) //Recovers G component from RGB encoding
#define B(COLOUR) (COLOUR & 0x1F) //Recovers B component from RGB encoding

typedef enum
{
	kSSD1331ColororderRGB		= 1,
	kSSD1331DelaysHWFILL		= 3,
	kSSD1331DelaysHWLINE		= 1,
} SSD1331Constants;


typedef enum
{
	kSSD1331CommandDRAWLINE			= 0x21,
	kSSD1331CommandDRAWRECT			= 0x22,
	kSSD1331CommandCLEAR			= 0x25,
	kSSD1331CommandFILL				= 0x26,
	kSSD1331CommandSETCOLUMN		= 0x15,
	kSSD1331CommandSETROW			= 0x75,
	kSSD1331CommandCONTRASTA		= 0x81,
	kSSD1331CommandCONTRASTB		= 0x82,
	kSSD1331CommandCONTRASTC		= 0x83,
	kSSD1331CommandMASTERCURRENT	= 0x87,
	kSSD1331CommandSETREMAP			= 0xA0,
	kSSD1331CommandSTARTLINE		= 0xA1,
	kSSD1331CommandDISPLAYOFFSET	= 0xA2,
	kSSD1331CommandNORMALDISPLAY	= 0xA4,
	kSSD1331CommandDISPLAYALLON		= 0xA5,
	kSSD1331CommandDISPLAYALLOFF	= 0xA6,
	kSSD1331CommandINVERTDISPLAY	= 0xA7,
	kSSD1331CommandSETMULTIPLEX		= 0xA8,
	kSSD1331CommandSETMASTER		= 0xAD,
	kSSD1331CommandDISPLAYOFF		= 0xAE,
	kSSD1331CommandDISPLAYON		= 0xAF,
	kSSD1331CommandPOWERMODE		= 0xB0,
	kSSD1331CommandPRECHARGE		= 0xB1,
	kSSD1331CommandCLOCKDIV			= 0xB3,
	kSSD1331CommandPRECHARGEA		= 0x8A,
	kSSD1331CommandPRECHARGEB		= 0x8B,
	kSSD1331CommandPRECHARGEC		= 0x8C,
	kSSD1331CommandPRECHARGELEVEL	= 0xBB,
	kSSD1331CommandVCOMH			= 0xBE,
} SSD1331Commands;

int	devSSD1331init(void);

void ssd1331DrawPixel(uint8_t x, uint8_t y, uint16_t colour);
void ssd1331DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t colour);
void ssd1331DrawRectangle(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t lineColour, uint16_t fillColour);
void ssd1331DrawChar(uint8_t x, uint8_t y, char character, uint16_t colour);
void ssd1331WriteLine(uint8_t x, uint8_t y, char* str, uint16_t colour);
void ssd1331Clear();