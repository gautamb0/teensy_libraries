/***************************************************
  This is our library for the Adafruit HX8357D Breakout
  ----> http://www.adafruit.com/products/2050

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_HX8357.h"
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

// Constructor when using software SPI.  All output pins are configurable.
Adafruit_HX8357::Adafruit_HX8357(int8_t cs, int8_t dc, int8_t mosi,
				   int8_t sclk, int8_t rst, int8_t miso) : Adafruit_GFX(HX8357_TFTWIDTH, HX8357_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_HX8357::Adafruit_HX8357(int8_t cs, int8_t dc, int8_t rst) : Adafruit_GFX(HX8357_TFTWIDTH, HX8357_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
}

void Adafruit_HX8357::spiwrite(uint8_t c) {

  Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
#if defined (__AVR__)
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
    SPCR = backupSPCR;
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
   /* for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
	*mosiport |=  mosipinmask;
      } else {
	//digitalWrite(_mosi, LOW); 
	*mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }   */
   Serial.println("in teensy spiwrite"); 
   	if (hwSPI) {
		SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
	}else{ 	
   for (uint8_t bit = 0x80; bit; bit >>= 1) {
		*mosiport = ((c & bit) ? 1 : 0);
		*clkport = 1;
		*clkport = 0;
	}}
	Serial.println("finished teensy spiwrite");
#elif defined (__arm__)
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.transfer(c);
#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
	*mosiport |=  mosipinmask;
      } else {
	//digitalWrite(_mosi, LOW); 
	*mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
  }
}


void Adafruit_HX8357::writecommand(uint8_t c) {
Serial.println("in writecommand");
#if defined(__MK20DX128__) || defined(__MK20DX256__)
  	Serial.println("in teensy portion");
	if (hwSPI) {
		Serial.println("in hwSPI portion");
		SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
		return;
	}
#endif
  *dcport &=  ~dcpinmask;
  //digitalWrite(_dc, LOW);
  *clkport &= ~clkpinmask;
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  spiwrite(c);
  Serial.print("Command 0x"); Serial.println(c, HEX);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void Adafruit_HX8357::writedata(uint8_t c) {
Serial.println("in writedata");
#if defined(__MK20DX128__) || defined(__MK20DX256__)
Serial.println("in teensy portion");	
if (hwSPI) {
Serial.println("in good");
		SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
		while (((SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full
		return;
	}
#endif
  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *clkport &= ~clkpinmask;
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);
  
  spiwrite(c);
  Serial.print("Data 0x"); Serial.println(c, HEX);

  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
}

#if defined(__MK20DX128__) || defined(__MK20DX256__)
static bool spi_pin_is_cs(uint8_t pin)
{
	if (pin == 2 || pin == 6 || pin == 9) return true;
	if (pin == 10 || pin == 15) return true;
	if (pin >= 20 && pin <= 23) return true;
	return false;
}

static uint8_t spi_configure_cs_pin(uint8_t pin)
{
        switch (pin) {
                case 10: CORE_PIN10_CONFIG = PORT_PCR_MUX(2); return 0x01; // PTC4
                case 2:  CORE_PIN2_CONFIG  = PORT_PCR_MUX(2); return 0x01; // PTD0
                case 9:  CORE_PIN9_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTC3
                case 6:  CORE_PIN6_CONFIG  = PORT_PCR_MUX(2); return 0x02; // PTD4
                case 20: CORE_PIN20_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTD5
                case 23: CORE_PIN23_CONFIG = PORT_PCR_MUX(2); return 0x04; // PTC2
                case 21: CORE_PIN21_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTD6
                case 22: CORE_PIN22_CONFIG = PORT_PCR_MUX(2); return 0x08; // PTC1
                case 15: CORE_PIN15_CONFIG = PORT_PCR_MUX(2); return 0x10; // PTC0
        }
        return 0;
}

#define CTAR_24MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_16MHz   (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0) | SPI_CTAR_DBR)
#define CTAR_12MHz   (SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_8MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0))
#define CTAR_6MHz    (SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))
#define CTAR_4MHz    (SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1))

#endif

void Adafruit_HX8357::begin(uint8_t type) {
  Serial.println("Begin.");
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);

  
#if defined (__AVR__)
  if(hwSPI) { // Using hardware SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR;
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
    	Serial.println("In teensy code");
    	if (_mosi == 0) _mosi = 11;
	if (_sclk == 0) _sclk = 13;
	if ( spi_pin_is_cs(_cs) && spi_pin_is_cs(_dc)
	 && (_mosi == 7 || _mosi == 11)
	 && (_sclk == 13 || _sclk == 14)
	 && !(_cs ==  2 && _dc == 10) && !(_dc ==  2 && _cs == 10)
	 && !(_cs ==  6 && _dc ==  9) && !(_dc ==  6 && _cs ==  9)
	 && !(_cs == 20 && _dc == 23) && !(_dc == 20 && _cs == 23)
	 && !(_cs == 21 && _dc == 22) && !(_dc == 21 && _cs == 22) ) {
		hwSPI = true;
		if (_sclk == 13) {
			CORE_PIN13_CONFIG = PORT_PCR_MUX(2) | PORT_PCR_DSE;
			SPCR.setSCK(13);
		} else {
			CORE_PIN14_CONFIG = PORT_PCR_MUX(2);
			SPCR.setSCK(14);
		}
		if (_mosi == 11) {
			CORE_PIN11_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(11);
		} else {
			CORE_PIN7_CONFIG = PORT_PCR_MUX(2);
			SPCR.setMOSI(7);
		}
		ctar = CTAR_12MHz;
		pcs_data = spi_configure_cs_pin(_cs);
		pcs_command = pcs_data | spi_configure_cs_pin(_dc);
		SIM_SCGC6 |= SIM_SCGC6_SPI0;
		SPI0.MCR = SPI_MCR_MDIS | SPI_MCR_HALT;
		SPI0.CTAR0 = ctar | SPI_CTAR_FMSZ(7);
		SPI0.CTAR1 = ctar | SPI_CTAR_FMSZ(15);
		SPI0.MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
#elif defined (__arm__)
 if(hwSPI) { // Using hardware SPI
    SPI.begin();
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
  }

  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(100);
    digitalWrite(_rst, LOW);
    delay(100);
    digitalWrite(_rst, HIGH);
    delay(150);
  }

  
  if (type == HX8357B) {
    Serial.println("linux HX8357B"); 
    // seqpower
    writecommand(HX8357B_SETPOWER);
    writedata(0x44);
    writedata(0x41); 
    writedata(0x06);
    // seq_vcom
    writecommand(HX8357B_SETVCOM);
    writedata(0x40); 
    writedata(0x10);
    // seq_power_normal
    writecommand(HX8357B_SETPWRNORMAL);
    writedata(0x05); 
    writedata(0x12);
    // seq_panel_driving
    writecommand(HX8357B_SET_PANEL_DRIVING);
    writedata(0x14); 
    writedata(0x3b);
    writedata(0x00);
    writedata(0x02);
    writedata(0x11);
    // seq_display_frame
    writecommand(HX8357B_SETDISPLAYFRAME);
    writedata(0x0c);  // 6.8mhz
    // seq_panel_related
    writecommand(HX8357B_SETPANELRELATED);
    writedata(0x01);  // BGR
    // seq_undefined1
    writecommand(0xEA);
    writedata(0x03);
    writedata(0x00);
    writedata(0x00); 
    // undef2
    writecommand(0xEB);
    writedata(0x40);
    writedata(0x54);
    writedata(0x26); 
    writedata(0xdb);
    // seq_gamma
    writecommand(HX8357B_SETGAMMA); // 0xC8
    writedata(0x00);
    writedata(0x15);
    writedata(0x00); 
    writedata(0x22);
    writedata(0x00);
    writedata(0x08);
    writedata(0x77); 
    writedata(0x26);
    writedata(0x66);
    writedata(0x22);
    writedata(0x04); 
    writedata(0x00);

    // seq_addr mode
    writecommand(HX8357_MADCTL);
    writedata(0xC0);
    // pixel format
    writecommand(HX8357_COLMOD);
    writedata(0x55);
    
    // set up whole address box
    // paddr
    writecommand(HX8357_PASET);
    writedata(0x00);
    writedata(0x00);
    writedata(0x01); 
    writedata(0xDF);
    // caddr
    writecommand(HX8357_CASET);
    writedata(0x00);
    writedata(0x00);
    writedata(0x01); 
    writedata(0x3F);

    // display mode
    writecommand(HX8357B_SETDISPMODE);
    writedata(0x00); // CPU (DBI) and internal oscillation ??
    // exit sleep
    writecommand(HX8357_SLPOUT);

    delay(120);
    // main screen turn on
    writecommand(HX8357_DISPON);
    delay(10);
  } else if (type == HX8357D) {
    Serial.println("initializing display");
	  writecommand(HX8357_SWRESET);

    // setextc
    writecommand(HX8357D_SETC);
    writedata(0xFF);
    writedata(0x83);
    writedata(0x57);
    delay(300);
    // setRGB which also enables SDO
    writecommand(HX8357_SETRGB); 
    writedata(0x80);  //enable SDO pin!
//    writedata(0x00);  //disable SDO pin!
    writedata(0x0);
    writedata(0x06);
    writedata(0x06);

    writecommand(HX8357D_SETCOM);
    writedata(0x25);  // -1.52V
    
    writecommand(HX8357_SETOSC);
    writedata(0x68);  // Normal mode 70Hz, Idle mode 55 Hz
    
    writecommand(HX8357_SETPANEL); //Set Panel
    writedata(0x05);  // BGR, Gate direction swapped
    
    writecommand(HX8357_SETPWR1);
    writedata(0x00);  // Not deep standby
    writedata(0x15);  //BT
    writedata(0x1C);  //VSPR
    writedata(0x1C);  //VSNR
    writedata(0x83);  //AP
    writedata(0xAA);  //FS
    
    writecommand(HX8357D_SETSTBA);  
    writedata(0x50);  //OPON normal
    writedata(0x50);  //OPON idle
    writedata(0x01);  //STBA
    writedata(0x3C);  //STBA
    writedata(0x1E);  //STBA
    writedata(0x08);  //GEN
    
    writecommand(HX8357D_SETCYC);  
    writedata(0x02);  //NW 0x02
    writedata(0x40);  //RTN
    writedata(0x00);  //DIV
    writedata(0x2A);  //DUM
    writedata(0x2A);  //DUM
    writedata(0x0D);  //GDON
    writedata(0x78);  //GDOFF
    
    writecommand(HX8357D_SETGAMMA); 
    writedata(0x02);
    writedata(0x0A);
    writedata(0x11);
    writedata(0x1d);
    writedata(0x23);
    writedata(0x35);
    writedata(0x41);
    writedata(0x4b);
    writedata(0x4b);
    writedata(0x42);
    writedata(0x3A);
    writedata(0x27);
    writedata(0x1B);
    writedata(0x08);
    writedata(0x09);
    writedata(0x03);
    writedata(0x02);
    writedata(0x0A);
    writedata(0x11);
    writedata(0x1d);
    writedata(0x23);
    writedata(0x35);
    writedata(0x41);
    writedata(0x4b);
    writedata(0x4b);
    writedata(0x42);
    writedata(0x3A);
    writedata(0x27);
    writedata(0x1B);
    writedata(0x08);
    writedata(0x09);
    writedata(0x03);
    writedata(0x00);
    writedata(0x01);
    
    writecommand(HX8357_COLMOD);
    writedata(0x55);  // 16 bit
    
    writecommand(HX8357_MADCTL);  
    writedata(0xC0); 
    
    writecommand(HX8357_TEON);  // TE off
    writedata(0x00); 
    
    writecommand(HX8357_TEARLINE);  // tear line
    writedata(0x00); 
    writedata(0x02);
    
    writecommand(HX8357_SLPOUT); //Exit Sleep
    delay(150);
    
    writecommand(HX8357_DISPON);  // display on
    delay(50);
    Serial.println("Done initializing.");
  } else {
    Serial.println("unknown type");
  }
}


void Adafruit_HX8357::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(HX8357_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(HX8357_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(HX8357_RAMWR); // write to RAM
}


void Adafruit_HX8357::pushColor(uint16_t color) {
  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}

void Adafruit_HX8357::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  /* 18 bit hack for testing */
  /*
  uint8_t r = (color >> 10) & 0x1F, g = (color >> 5) & 0x3F, b = color & 0x1F;
  r <<= 3;
  g <<= 2;
  b <<= 3;

  spiwrite(r);
  spiwrite(g);
  spiwrite(b);
  */

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void Adafruit_HX8357::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void Adafruit_HX8357::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  //digitalWrite(_dc, HIGH);
  //digitalWrite(_cs, LOW);
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}

void Adafruit_HX8357::fillScreen(uint16_t color) {
  fillRect(0, 0, _width, _height, color);
}

// fill a rectangle
void Adafruit_HX8357::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  /* 18 bit hack for testing */
  /*
  uint8_t r = (color >> 10) & 0x1F, g = (color >> 5) & 0x3F, b = color & 0x1F;
  r <<= 3;
  g <<= 2;
  b <<= 3;
  */

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_HX8357::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_HX8357::setRotation(uint8_t m) {
Serial.println("in setrotation");
  writecommand(HX8357_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
     _width  = HX8357_TFTWIDTH;
     _height = HX8357_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MV | MADCTL_MY | MADCTL_RGB);
     _width  = HX8357_TFTHEIGHT;
     _height = HX8357_TFTWIDTH;
     break;
  case 2:
    writedata( MADCTL_RGB);
     _width  = HX8357_TFTWIDTH;
     _height = HX8357_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
     _width  = HX8357_TFTHEIGHT;
     _height = HX8357_TFTWIDTH;
     break;
  }
}


void Adafruit_HX8357::invertDisplay(boolean i) {
  writecommand(i ? HX8357_INVON : HX8357_INVOFF);
}


////////// stuff not actively being used, but kept for posterity


uint8_t Adafruit_HX8357::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#if defined (__AVR__)
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
    SPCR = backupSPCR;
#elif defined (__arm__)
    SPI.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    r = SPI.transfer(0x00);
#endif
  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

 uint8_t Adafruit_HX8357::readdata(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   
   return r;
}
 

uint8_t Adafruit_HX8357::readcommand8(uint8_t c, uint8_t index) {
   digitalWrite(_dc, LOW);
   digitalWrite(_sclk, LOW);
   digitalWrite(_cs, LOW);
   spiwrite(c);
 
   digitalWrite(_dc, HIGH);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   return r;
}


 
/*

 uint16_t Adafruit_HX8357::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_mosi, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_mosi, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_HX8357::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_mosi, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_mosi, OUTPUT); // back to output
 return r;
 }
 
 */
