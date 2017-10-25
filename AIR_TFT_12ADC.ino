#include <TFT.h>    // This further includes Adafruit_GFX and Adafruit_ST7735 from C:/Program Files/Arduino/libraries/tft/utility
#include <SPI.h>
#include <Wire.h>
#include "TC_coeff.h"

// Using the hardware SPI interface only these additional pins need to be defined
#define TFT_CS     9
#define TFT_RST    7  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to -1!
#define TFT_DC     8
#define LED1_PIN 	10     // aka PB2,SS Wired to RED LED, Also used as slave select pin for the SD card 
#define LED2_PIN 	17     // aka PC3,A3, Wired to Blue LED, also used for the Servo pulse pin

#define VBUS_ADC 7			// ADC7
#define VBUS_ADC_BW  (5.0*(14+6.8)/(1024*6.8))		//adc bit weight for voltage divider 14.0K and 6.8k to gnd.

// defines for the bar graph
#define BAR_gap 2     // pixels between two bars

#define BAR_BOT_LEGEND (disp_height -8)
#define BAR_TOP_LEGEND (0)
#define BAR_bottom (disp_height-10)  // location of bottom most pixel
#define BAR_top 8                       // top most pixel
#define BAR_height (BAR_bottom - BAR_top)
#define BAR_leftMargin 8
#define BAR_rightMargin  8 

#define N_BARS 6		// 1 through 6
#define BAR_width (( disp_width - BAR_leftMargin - BAR_rightMargin - ((N_BARS-1) * BAR_gap) ) /N_BARS)// pixels of width of bar
#define MAX_CHANNEL N_BARS

unsigned short disp_height, disp_width;

// The instance of the TFT class controlling the LCD -- using the HW SPI interface from the cpu
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// The LTC_2495 has a strange ADC output format. It has 16 bits of ADC resolution plus a seperate sign and overflow bit. 
// The data arrives in a 3 byte stream with the 6lsb all indicating 0's.  So when shifted down 6 positions the bits 0 through 15 
// hold a unsigned 16 bit int

#define LTC_2495_ADDR 	(0x45) 		// 7 bit I2C address for ADC -- All address pins left floating
#define ADC_REFERENCE_V (2.048)		// Reference voltage on VREF+, v.s. VREF-
#define ADC_SPEED_GAIN (0x83) 		// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
#define ADC_Bit_Value (ADC_REFERENCE_V/2/16/65536)  // Bit value of ADC to correspond to gain and reference 
union adc
{
	uint8_t bytes[4];	
	int32_t adc_val_24;

} ADC_result = {0,0,0,0};

float Temps[ MAX_CHANNEL+1];		// index 0  is cold junction, then ch1, ch2
#define COLD_JCT_NDX 	0

#define EGT_MAX  900
#define EGT_ZOOM 650

/* draws a bar at column pos as a fraction of max height*/
	
void DrawBar(  uint8_t pos, float height,  uint16_t color = ST7735_MAGENTA) 
{
  // draw a rectangle bar from the bottom up, spaced every n pixels
  unsigned char x ;
  unsigned char y ;
  
  if (pos == 0 ) // This is not a bar -- internal mesurment 
  {
	  
	  return;
  }
  else 
	  pos--;		// since bars nuymbers are 1 based
  
  x = pos*(BAR_width+BAR_gap)+BAR_leftMargin;
  
  // 
  if (height > 1.0) 
  {
	  height = 1.0;
	  color = ST7735_RED;		// over range 
  }
  if (height < 0.001)			// under range
  {
	height = 0.01;
	color = ST7735_BLUE;
  }

  y = height * BAR_height;

  // draw a heading -- 
  // TODO: this should only be executed once -- redundant to redraw the legends
  // TODO: draw a side scale 
  tft.drawChar(x+(BAR_width/3),BAR_BOT_LEGEND,'1'+pos,ST7735_WHITE,ST7735_BLACK,1); 
  
  tft.fillRect( x,BAR_bottom - y, BAR_width,y,color);        // draw the bar 
  tft.fillRect( x,BAR_top, BAR_width, BAR_height-y, ST7735_BLACK);  // erase everytihng above the top 
}

void setup(void) {
	
	uint8_t ret_val;
	uint8_t num;
	
	Serial.begin(115200);
	Serial.print("Hello! AIR_TFT_ADC");

	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab, try different tab settings if the display is not correct
	tft.fillScreen(ST7735_BLACK);	// clear the screen 
	tft.setRotation(3);
	tft.setCursor(0, 0);
	tft.println("EGT monitor V0.0 ");

	pinMode(LED1_PIN, OUTPUT);    // RED
	digitalWrite( LED1_PIN, 0);

	delay(500);
	disp_height = tft.height();
	disp_width = tft.width();
/*
	Serial.print("tft width = ");
	Serial.println(disp_width);
	Serial.print("tft height = ");
	Serial.println( disp_height);
*/
	//Wire.setClock( 10000);	 // Slow down the I2C clock for longer wire
	Wire.begin();								// initialize the i2C interface as master
	// setting a lower i2c clock frequency,  F_SCL = F_CPU/ 16 + (2*TWBR*TW_Prescale)
	TWBR =99;	// for 20Khz clock rate 
	TWSR |=  1; // == prescale /4
	Wire.beginTransmission(LTC_2495_ADDR);		// 0x45 when all 3 address pins are floating
	Wire.write(0xa0);		// Enable bit only
	Wire.write(0xC0);		// meassure internal temp
	ret_val = Wire.endTransmission();
	// 0 == success
	// 1 == NACK upon transmit of address
	// 2 == NACK upon transmit of DATA
	// 3 == Other error
	if (ret_val !=0 )
	{
		Serial.print("ADC not responding -- stop!");
		digitalWrite( LED1_PIN, 1);
		
		tft.println("ADC not responding -- stop!");
		while (1);
	}

	delay( 200); // conversion takes about 160ms 
	num = Wire.requestFrom(LTC_2495_ADDR,3);		// this starts a new conversion so that the main loop can read the int temp on the first call
	if (num != 3 )
	{
		Serial.print("ADC not ready with result, STOP! ");
		digitalWrite( LED1_PIN, 1);
		tft.println("ADC not ready with result, STOP!");
		while (1) ;
	}
	
	// MSB to LSB
	ADC_result.bytes[2] = Wire.read();
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();

	uint16_t adc_val16 = ADC_result.adc_val_24 >> 6;	// lowest 6 bits are 0's 
	Temps[COLD_JCT_NDX] = ( adc_val16 * ADC_REFERENCE_V /12.25) -273.0;				// temperature formula from datasheet in Kelvin

	tft.print("Internal temp: ");
	tft.println(Temps[COLD_JCT_NDX]); 
	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
	tft.print("Battery Voltage: ");
	tft.println(Vbus_Volt);
		
	delay (2000);	// after the read above a conversion is started again -- can't talk to the chip again until it's done 
	tft.fillScreen(ST7735_BLACK);	// clear the screen 
	Serial.println("exit setup");
}






void loop() 
{ 
	bool zoom;
	int32_t ADC_cnt; //  32 bit integer so that we can convey 16bits plus a sign 
	static int8_t next_ch = 0; // channel 0 is internal temp reading for cold junction temp -- not displayed
	int8_t curr_ch;			   // the current channel data that is beeing processed lags one behind the next channel	
	int num;
	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
		
	if (Vbus_Volt < 6.6 )
	{
		tft.fillScreen(ST7735_BLACK);
		tft.setCursor(10, 10);
		tft.print(Vbus_Volt);
		tft.print(" V");
		tft.setCursor(10, 30);
		tft.print(" Voltage too low ! ");
		return;
	}
	
	// set the channel for the next reading 
	if ( ++next_ch > MAX_CHANNEL )
	{
		next_ch = COLD_JCT_NDX;
		Wire.beginTransmission(LTC_2495_ADDR);	
		Wire.write(0xa0);		// Enable bit only
		Wire.write(0xC0);		// meassure internal temp
		Wire.endTransmission( 0 ); // ending with ReStart instead of STOP so that we still can read the result from the last conversion
	}
	else
	{	
		// select next input channels -- differential mode
	 	Wire.beginTransmission(LTC_2495_ADDR);	
		Wire.write(0xA8 + next_ch -1);			// meassure differential input channel, first channel is In+=0, In-=1 
	//	Wire.write (0xB0+next_ch-1);// +next_ch-1);// single 
		Wire.write(ADC_SPEED_GAIN);				// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
		Wire.endTransmission( 0 ); // ending with Re-Start instead of STOP so that we still can read the result from the last conversion
	}
	
	curr_ch = next_ch - 1;			// the channel for which we just read the data in this loop
	if( curr_ch < COLD_JCT_NDX )
		curr_ch = MAX_CHANNEL;
	
	num = Wire.requestFrom(LTC_2495_ADDR,3);	// reads the last conversion and restarts new conversion immediatly
// TODO:		
// if not enough is read then alarm	
//	Serial.print(" i2c returns bytes (3): ");Serial.println(num);
	ADC_result.bytes[2] = Wire.read();	// MSB  
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();	// LSB
		
	// shift out the lower 6 bits that are all 0's and use the 16 bits as positive integer
	ADC_cnt = (uint16_t) (ADC_result.adc_val_24 >> 6); // it's a positive 16 bit number
	
	if (curr_ch == 0 )	// the 'last' conversion was on internal temp 
	{
		Temps[COLD_JCT_NDX] = 	 (ADC_cnt * ADC_REFERENCE_V /12.25) -273.0;// temperature formula from datasheet in Kelvin
		// add a delay here so that the execution time for this internal measurment is the same as for the TC calcs and display
		// so that we can optimize the loop delay for the general case	
		delay(24);
		//Serial.print(" cold junction:");
		//Serial.println(Temps[COLD_JCT_NDX] ); 
	}
	else // it's a thermocouple
	{
		float * tab;		// pointer to the coeff table 
			
		// sign bit true ==  positive voltages in lower 16 bits /*
		
		if ( ! (ADC_result.bytes[2] & 0x80) )	// it's a negative 16 bit number 
		{
			ADC_cnt = 0;			// upside down thermocouples read 0 
		} 

		float TC_mv = ADC_cnt * ADC_Bit_Value * 1000; // in millivolts for coeff table 
		//Serial.print("TC_mv");
		//Serial.println(TC_mv); 
		
		// depending on the voltage, choose appropriate table as per NIST
		if (TC_mv < 20.644)		// threshold of table 1 to 2
			tab = K_TC_coeff0_500;
		else
			tab = K_TC_coeff500_plus; 

		// calculate the thermocouple t90 from thermal EMF per NIST polynomial
		float TC_Temp;
		for (int n = 0; n <= N_poly; n++ )
		{
			double EE;// the n'th power of TC EMF
			if ( n == 0 )
			{
				EE = TC_mv;
				TC_Temp = tab[0];
			}
			else
			{
				TC_Temp += EE * tab[n];
				EE *= TC_mv;	// next power 
			}

		}
		// this is not quite correct, but close enough -- should convert cold temp to EMF since poly is based from 0deg C
		// requires logarithm functions -- too expensive in mem foot print
		Temps[curr_ch] = TC_Temp += Temps[COLD_JCT_NDX];
	}

	// switch to zoomed mode when all channels are above threshold, 
	// maybe add a hysteresis if this is jumpy
	zoom = true;
	for (int i =1; i<= MAX_CHANNEL; i++ )
	{
		if (Temps[i] < EGT_ZOOM)
		{
			zoom = false;
			break;
		}
		
	}
	
	if (Temps[curr_ch] <= 0 )
		DrawBar(curr_ch, 0 );
	else
	{ 
		if (zoom)
			DrawBar(curr_ch, (Temps[curr_ch] - EGT_ZOOM)/(EGT_MAX - EGT_ZOOM), ST7735_GREEN);
		else
			DrawBar(curr_ch, Temps[curr_ch]/ EGT_MAX , ST7735_BLUE);
	}
			
	
	// according to the datasheet of LTC2495/page5 the conversion time in 50/60hz 1x speed mode is 149ms 
	// communication,display and computation takes 24ms per loop, so we wait 130ms for a total delay of 154 ms
	delay(130);		// this is the fastest the chip can convert in the chosen mode and clock
	


}
	
  /*
  // display them the other way.
  for (;i>=0;i--)
    DrawBar(11-i,(i+1)*8+5,i*2+7);
  
  delay(5000);  

  tft.fillScreen(ST7735_BLACK);
  testfillcircles(10, ST7735_BLUE);
  testdrawcircles(10, ST7735_WHITE);
  delay(500);

  testroundrects();
  delay(500);

  testtriangles();
  delay(500);

  mediabuttons();
  delay(500);

  Serial.println("done");
  delay(1000);
*/
  

/*
void testlines(uint16_t color) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color);
  }

  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, 0, 0, y, color);
  }

  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
  }

  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t y=0; y < tft.height(); y+=5) {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x=0; x < tft.width(); x+=5) {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

void testdrawrects(uint16_t color) {
  tft.fillScreen(ST7735_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
  }
}


void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=radius; x < tft.width(); x+=radius*2) {
    for (int16_t y=radius; y < tft.height(); y+=radius*2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
    for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles() {
  tft.fillScreen(ST7735_BLACK);
  int color = 0xF800;
  int t;
  int w = tft.width()/2;
  int x = tft.height()-1;
  int y = 0;
  int z = tft.width();
  for(t = 0 ; t <= 15; t++) {
    tft.drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
  }
}

void testroundrects() {
  tft.fillScreen(ST7735_BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = tft.width()-2;
    int h = tft.height()-2;
    for(i = 0 ; i <= 16; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST7735_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST7735_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST7735_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST7735_BLACK);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST7735_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST7735_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST7735_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST7735_WHITE);
  tft.print(" seconds.");
}

void mediabuttons() {
  // play
  tft.fillScreen(ST7735_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, ST7735_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, ST7735_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, ST7735_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST7735_GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, ST7735_RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST7735_RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST7735_GREEN);
}
*/
