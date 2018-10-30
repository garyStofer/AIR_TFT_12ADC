#include <TFT.h>    // This further includes Adafruit_GFX and Adafruit_ST7735 from C:/Program Files/Arduino/libraries/tft/utility
#include <SPI.h>
#include <Wire.h>
#include "TC_coeff.h"

// Using the HW SPI interface of ATMEL 328P
// Connect signal SCL on TFT board to SCK of 328P , pin17, PB5 (SCK) 
// Connect signal SDA on TFT board to MOSI of 328P, pin15 , PB3(MOSI)
// only these additional pins need to be defined
#define TFT_CS     9 // Chip select -- Could also use SlaveSelect from HW SPI interface 
#define TFT_RST    7 // you can also connect this to the Arduino reset, in which case, set this #define pin to -1!
#define TFT_DC     8 //  This is Data/Command of the TFT, often labeled as A0 on TFT board

// The two LEDS 
#define LED1_PIN 	10     // aka PB2,SS Wired to RED LED, Also used as slave select pin for the SD card 
#define LED2_PIN 	17     // aka PC3,A3, Wired to Blue LED, also used for the Servo pulse pin



#define VBUS_ADC 7			// ADC7
#define VBUS_ADC_BW  (5.0*(14+6.8)/(1024*6.8))	//Bit weight of ADC7 using a voltage divider 14.0K and 6.8k to gnd.

// defines for the bar graph
#define CHAR_HEIGHT 7 	// a character at scale 1 is 7 pixels tall , plus 1 for gap
#define CHAR_WIDTH 5    // plus one for gap  
#define BAR_gap 1       // horizontal gap between two bars

#define BAR_BOT_LEGEND (disp_height -8)
#define BAR_TOP_LEGEND (8)
#define BAR_bottom (disp_height-10)  // location of bottom most pixel
#define BAR_top BAR_TOP_LEGEND       // top most pixel
#define BAR_height (BAR_bottom - BAR_top)
#define BAR_leftMargin 28
#define BAR_rightMargin 0
#define BAR_DISP_width (disp_width - BAR_leftMargin - BAR_rightMargin)

#define N_BARS 6		// 1 through 6
#define BAR_width (( disp_width - BAR_leftMargin - BAR_rightMargin - ((N_BARS-1) * BAR_gap) ) /N_BARS)// pixels of width of bar
#define MAX_CHANNEL N_BARS

unsigned short disp_height, disp_width;

// The instance of the TFT class controlling the LCD -- using the HW SPI interface from the cpu
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// The LTC_2495 has a strange ADC output format. It has 16 bits of ADC resolution plus a seperate sign and overflow bit. 
// The data arrives in a 3 byte stream with 6 lsb indicating all 0's. When shifted down 6 positions the bits 0 through 15 
// hold a unsigned 16 bit int

#define LTC_2495_ADDR 	(0x45) 		// 7 bit I2C address for ADC -- All address pins left floating
#define ADC_REFERENCE_V (2.048)		// Reference voltage on VREF+, v.s. VREF-
#define ADC_SPEED_GAIN (0x83) 		// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
#define ADC_Bit_Value (ADC_REFERENCE_V/2/16/65536)  // Bit value of ADC to correspond to gain and reference 

// Union to read the LTC_2495 ADC
union adc
{
	uint8_t bytes[4];	
	int32_t adc_val_24;

} ADC_result = {0,0,0,0};

float Temps[ MAX_CHANNEL+1];		// index 0  is cold junction, then ch1, ch2
#define COLD_JCT_NDX 	0

#define TREND_HYST 1		// half of the hystery in deg C -- 
	
#define CtoF( tC ) ( tC / 0.5555555555 +32)
#define FtoC(xF ) ( (xF - 32.0) * 0.55555555555)

// in deg F  used for display only 
#define F_EGT_MAX  1650		  
#define F_EGT_ZOOM 1200		
// in deg C -- used in code
#define C_EGT_MAX  (FtoC(F_EGT_MAX))
#define C_EGT_ZOOM (FtoC(F_EGT_ZOOM))

/* 
 *  	Draws the scale grid and legend.  Arguments indicate low and high endpoints
 *
*/
	
void DrawScale( int low, int high ) 
{

	unsigned char y ;
	int m;
  
  // draw scale and grid 
	tft.fillRect( 0,0, BAR_leftMargin,disp_height,ST7735_BLACK);        // clear the text area on the left
	y = BAR_top;
	tft.drawFastHLine(BAR_leftMargin -BAR_gap, y , BAR_DISP_width, ST7735_WHITE);
	tft.setCursor(0, y-4);
	tft.print(high);
	
	y = BAR_top + BAR_height/4;
	m = (high-low)/4*3+low;
	tft.drawFastHLine(BAR_leftMargin -BAR_gap, y, BAR_DISP_width, ST7735_WHITE);
	tft.setCursor(0, y-4);
	tft.print(m);
	
	y = BAR_top + BAR_height/2;
	m = (high-low)/4*2+low;
	tft.drawFastHLine(BAR_leftMargin -BAR_gap, y, BAR_DISP_width, ST7735_WHITE);
	tft.setCursor(0, y-4);
	tft.print(m);
	
	y = BAR_top + BAR_height/4*3;
	m = (high-low)/4*1+low;
	tft.drawFastHLine(BAR_leftMargin -BAR_gap, y, BAR_DISP_width, ST7735_WHITE);
	tft.setCursor(0, y-4);
	tft.print(m);
	
	tft.drawFastHLine(BAR_leftMargin -BAR_gap, BAR_bottom-1, BAR_DISP_width, ST7735_WHITE);
	tft.setCursor(0, BAR_bottom-4);
	tft.print(low);

	// the bar numbers 
	for ( int pos = 0; pos<N_BARS; pos++)
	{
		int x = pos*(BAR_width+BAR_gap)+BAR_leftMargin;
		tft.drawChar(x+(BAR_width/3),BAR_BOT_LEGEND,'1'+pos,ST7735_WHITE,ST7735_BLACK,1); 
	}
	  
	return;
}
	
	
/* Draws a single bar at column "pos" as a fraction of max height*/
	
void DrawBar( int8_t pos, float height, uint16_t color = ST7735_MAGENTA, signed char trend = 0) 
{
  // draw a rectangle bar from the bottom up, spaced every n pixels
  unsigned char x,y;
  
  if (pos == 0 ) // This is not a bar -- internal mesurment 
  {
	// The delay is here so that the execution time for this internal measurment is the same as for the TC calcs and display
	// so that we can optimize the loop delay for the general case.
	delay(24);
	//Serial.print(" cold junction:");
	//Serial.println(Temps[COLD_JCT_NDX] ); 
	  return;
  }
  else 
	  pos--;		// since bars nuymbers are 1 based
  
  if (height > 1.0) 
  {
	  height = 1.0;
	  color = ST7735_RED;		// over range 
  }
  
  if (height < 0.01)			// under range
  {
	height = 0.01;				// keep a very thin line 
  }
  

  
  x = pos*(BAR_width+BAR_gap)+BAR_leftMargin;
  y = height * BAR_height;
 

  tft.fillRect( x,BAR_bottom - y, BAR_width,y,color);        // draw the bar 
  tft.fillRect( x,0, BAR_width, BAR_height-y+BAR_TOP_LEGEND, ST7735_BLACK);  // erase everytihng above the top 
 
	switch (trend)
	{
		case 1: // going up
			tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8 ,0x18,ST7735_RED,ST7735_BLACK,1); 
			break;
			
		case 2:// going down
			tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,0x19,ST7735_YELLOW,ST7735_BLACK,1); 
			break;
	//	default:  // don't need this -- bar top is already cleared
	//		tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,' ',ST7735_YELLOW,ST7735_BLACK,1); 
			
			
	}
}

void BarDisplay( int8_t curr_ch , float *Temps, unsigned char trend)
{
	bool zoom = true;
	static bool prev_zoom = true;
	
	// switch to zoomed mode when all channels are above threshold 
	for (int i =1; i<= MAX_CHANNEL; i++ )
	{
		if (Temps[i] < C_EGT_ZOOM)
			zoom = false;
	}
	// draw the scale and grid when the display range changes 
	if (zoom != prev_zoom)
	{
		if (zoom )
			DrawScale(F_EGT_ZOOM, F_EGT_MAX);	
		else
			DrawScale(0, F_EGT_MAX);	
			
		prev_zoom  = zoom;	
	}
	

	if (zoom)
		DrawBar(curr_ch, (Temps[curr_ch] - C_EGT_ZOOM)/(C_EGT_MAX - C_EGT_ZOOM), ST7735_GREEN, trend);
	else
		DrawBar(curr_ch, Temps[curr_ch]/ C_EGT_MAX , ST7735_BLUE, trend);	
	
}


// This is ARDUINO setup function -- called once upon reset
void setup(void) {
	
	//uint8_t ret_val;
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

	Wire.begin();								// initialize the i2C interface as master
	//Wire.setClock( 10000); 			//This call seems to do nothing -- so I write the HW registers diectly
	// setting a lower i2c clock frequency,  F_SCL = F_CPU/ 16 + (2*TWBR*TW_Prescale)
	TWBR = 99;	// for 20Khz clock rate 
	TWSR |=  1; // == prescale /4
	
#ifdef notneeded 
// TODO: can I delete this initial internal reading and let the loop take care of it ??	
	Wire.beginTransmission(LTC_2495_ADDR);		
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

	delay( 200 ); // conversion takes about 160ms 
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
// TODO : can I delete the above cold temp reading ?	
	
#endif 
	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
	tft.print("Battery Voltage: ");
	tft.println(Vbus_Volt);

	// Delay to read the above messages 	
	delay (2000);	// after the read above a conversion is started again -- can't talk to the chip again until it's done 
	tft.fillScreen(ST7735_BLACK);	// clear the screen 

	Serial.println("exit setup");
}





// This is the ARDUINO loop execute function, called contineously 
void loop() 
{ 
	static int8_t next_ch = MAX_CHANNEL; 
	static bool prev_zoom = true;
	bool zoom = true;
	int32_t ADC_cnt; //  32 bit integer so that we can convey 16bits plus a sign 
	int8_t curr_ch;			   // the current channel data that is beeing processed lags one behind the next channel	
	int num;
	unsigned char trend =0;

	
	// Begin of acquiring TC readings  -- one reading per loop 	
	// set the channel for the next reading 
	if ( ++next_ch > MAX_CHANNEL )
	{
		next_ch = COLD_JCT_NDX;
		Wire.beginTransmission(LTC_2495_ADDR);	
		Wire.write(0xa0);		// Enable bit only
		Wire.write(0xC0);		// meassure internal temp
		uint8_t ret_val = Wire.endTransmission( 0 ); // ending with ReStart instead of STOP so that we still can read the result from the last conversion
		// 0 == success
		// 1 == NACK upon transmit of address
		// 2 == NACK upon transmit of DATA
		// 3 == Other error
		if (ret_val )
		{
			digitalWrite( LED1_PIN, 1);
			tft.fillScreen(ST7735_BLACK);
			tft.setCursor(10, 10);
			tft.println("ADC not responding -- STOP!");
			
		}
		
		// do the Vbus reading at the same time as the cold junction reading
		float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
		// Check the power bus voltage
		if (Vbus_Volt < 6.6 )
		{
			tft.fillScreen(ST7735_BLACK);
			tft.setCursor(10, 10);
			tft.print(Vbus_Volt);
			tft.print(" V");
			tft.setCursor(10, 30);
			tft.print(" Voltage too low! STOP ");
			while (1);
		}
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
	
	if (Wire.requestFrom(LTC_2495_ADDR,3) != 3 )	// reads the last conversion and restarts new conversion immediately
	{
		digitalWrite( LED1_PIN, 1);
		tft.fillScreen(ST7735_BLACK);
		tft.setCursor(10, 10);
		tft.println("ADC not ready with result, STOP!");
		while (1) ;
	}

	ADC_result.bytes[2] = Wire.read();	// MSB  
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();	// LSB
		
	// shift out the lower 6 bits that are all 0's and use the 16 bits as positive integer
	ADC_cnt = (uint16_t) (ADC_result.adc_val_24 >> 6); // it's a positive 16 bit number
	
	if (curr_ch == 0 )	// the 'last' conversion was the internal temp (cold junction)
	{
		Temps[COLD_JCT_NDX] = 	 (ADC_cnt * ADC_REFERENCE_V /12.25) -273.0;// temperature formula from datasheet in Kelvin
	}
	else // it's a thermocouple
	{
		float * tab;		// pointer to the coeff table 
			
		// if sign bit true ==  positive voltages in lower 16 bits /*
		if ( ! (ADC_result.bytes[2] & 0x80) )	// it's a negative 16 bit number 
		{
			ADC_cnt = 0;			// upside down connected thermocouple:  read 0 
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
		float TC_Temp_C;
		for (int n = 0; n <= N_poly; n++ )
		{
			double EE;// the n'th power of TC EMF
			if ( n == 0 )
			{
				EE = TC_mv;
				TC_Temp_C = tab[0];
			}
			else
			{
				TC_Temp_C += EE * tab[n];
				EE *= TC_mv;	// next power 
			}

		}

		// This math is not quite correct, but close enough -- should convert cold temp to EMF since poly is based from 0 deg C,
		// however that requires logarithm functions -- too expensive in mem foot print
		if (TC_Temp_C > 0.0 )
		{	
			TC_Temp_C += Temps[COLD_JCT_NDX]; // add in the cold junction temp
			
			if ( (TC_Temp_C - TREND_HYST) > Temps[curr_ch] )
				trend = 1;		// Rising 
			else if ((TC_Temp_C + TREND_HYST) < Temps[curr_ch] )
				trend = 2;		// Falling
			Temps[curr_ch] = TC_Temp_C;
		}
		else
			Temps[curr_ch] = -10e5;		// If the probe is connected upside down, display a zero bar
		
			

	}
// end of acquiring TC readings  

	BarDisplay( curr_ch, Temps, trend);

	// according to the datasheet of LTC2495/page5 the conversion time in 50/60hz 1x speed mode is 149ms 
	// communication,display and computation takes 24ms per loop, so we wait 130ms for a total delay of 154 ms
// TODO: Tune delay with scope
	
	
	delay(140);		// this is the fastest the chip can convert in the chosen mode and clock
	


}
