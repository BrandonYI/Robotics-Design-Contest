#include "stm32f10x.h"

// ==========================================================================
// structure definition
// ==========================================================================

// This structure describes a single character's display information
typedef struct {
	const u8 widthBits;					// width, in bits (or pixels), of the character
	const u16 offset;					// offset of the character's bitmap, in bytes, into the the FONT_INFO's data array
} FONT_CHAR_INFO;	

// Describes a single font
typedef struct {
	const u8 heightPages;	// height, in pages (8 pixels), of the font's characters
	const u8 startChar;		// the first character in the font (e.g. in charInfo and data)
	const u8 endChar;		// the last character in the font
	//const u8 spacePixels;	// number of pixels that a space character takes up
	const FONT_CHAR_INFO *charInfo;		// pointer to array of char information
	const u8 *data;			// pointer to generated array of character visual representation
} FONT_INFO;

typedef struct {
	const u16 *data;
	u16 width;
	u16 height;
} tImage;

/* Font data for Microsoft Sans Serif 8pt */
extern const u8 microsoftSansSerif_8ptBitmaps[];
extern const FONT_INFO microsoftSansSerif_8ptFontInfo;
extern const FONT_CHAR_INFO microsoftSansSerif_8ptDescriptors[];

/* Font data for Serif Gothic 16pt */
extern const u8 serifGothic_16ptBitmaps[];
extern const FONT_INFO serifGothic_16ptFontInfo;
extern const FONT_CHAR_INFO serifGothic_16ptDescriptors[];

/* Bitmap info for logo */
extern const tImage Logo;
