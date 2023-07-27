#include "BoardPins.h"
#include <TFT.h>    // This further includes Adafruit_GFX and Adafruit_ST7735 from C:/Program Files/Arduino/libraries/tft/utility
#include <SPI.h>	// SPI for the SD  card 
#include <Wire.h>	// I2C for the TFT 
#include "TC_coeff.h"
#include "build_opts.h"
#include "Encoder.h"

//#define SIMUL
#define WITH_SD_CARD
#ifdef WITH_SD_CARD

// NOTE: Make sure SD card is Formatted with FAT16 as discussed in the readme file of the lib. Will not work on Fat32, Exfat,Fat12

#include <Fat16.h>
#define SD_RECORD_PERiod 25	// every n times trough a complete loop of 7 adc readings , 1.4 second per loop,  22 ~ 30 seconds

// change the filename according to which Aircraft this is installed, in file build_opts.h
#define DataFile N_NUMBER".csv"
#endif // with SD_Card


//#define VBUS_ADC 7			// ADC7 on atmega 328P
#define VBUS_ADC_BW  (5.0*(14+6.8)/(1024*6.8))	//Bit weight of ADC7 using a voltage divider 14.0K and 6.8k to gnd.

// defines for the bar graph
#define CHAR_HEIGHT 7 	// a character at scale 1 is 7 pixels tall , plus 1 for gap
#define CHAR_WIDTH  5   // plus one for gap  
#define BAR_gap     1   // horizontal gap between two bars


// Strangely the defines of witdh and height are backwards in the adafruit source 
#define disp_height ST7735_TFTWIDTH
#define disp_width ST7735_TFTHEIGHT
// defines for arrow icons
#define ARROW_WIDTH 7
#define ARROW_HEIGHT 8
#define BAR_BOT_LEGEND (disp_height -8)
#define BAR_TOP_LEGEND (8)
#define BAR_bottom (disp_height-10)  // location of bottom most pixel
#define BAR_top BAR_TOP_LEGEND       // top most pixel
#define BAR_height (BAR_bottom - BAR_top)
#define BAR_leftMargin 28
#define BAR_rightMargin 0
#define BAR_DISP_width (disp_width - BAR_leftMargin - BAR_rightMargin)

#define TEXT_leftMargin 0
#define TEXT_lineHeight 19
#define TEXT_colVAL 	60
#define TEXT_trendCOL 	140

#define N_BARS 6			// 1 through 6
#define MAX_CHANNEL N_BARS 
#define COLD_JCT_NDX 	0  // Cyl1 =1, Cyl2 =2 ,etc

#define BAR_width (( disp_width - BAR_leftMargin - BAR_rightMargin - ((N_BARS-1) * BAR_gap) ) /N_BARS)// pixels of width of bar

#define LTC_2495_ADDR_EGT 	(0x45) 		// 7 bit I2C address for EGT ADC -- All address pins left floating
#define LTC_2495_ADDR_CHT 	(0x14) 		// CHT address  
#define LTC_2495_GLOBAL_ADDR (0x77)		// addresses all LTC devices on the bus at once

// global data elements :
#ifdef WITH_SD_CARD
SdCard card;        // SD card class element
Fat16 file;         // FAT class element
#endif 

// The instance of the TFT class controlling the LCD -- using the HW SPI interface from the cpu
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// arrow icon bitmaps for TFT display which must be stored in flash memory
const uint8_t arrowUp[] PROGMEM =   {0x10,0x10,0x38,0x38,0x7C,0x7C,0xFE,0xFE};
const uint8_t arrowDown[] PROGMEM = {0xFE,0xFE,0x7C,0x7C,0x38,0x38,0x10,0x10};

typedef struct {
	const short F_MAX;				// max of range
	const short F_WARN;				// warning led comes on
	const short F_ZOOM;				// all ch above this and the display goes into expanded scale F_ZOOM to F_MAX
	const short F_ALARM;			// Both Warn and alarm leds come on.
	const int 	ADCaddress;   // todo : should be uint8_t
	float  Temps[MAX_CHANNEL+1]; // all temps in deg F except index 0 for cold junction temp
	int8_t Trend[MAX_CHANNEL+1]	;
	int8_t min_ndx;
	int8_t max_ndx;
	float delta;
}Params_t ;

Params_t EGT = {1650,1525,1200,1575,LTC_2495_ADDR_EGT};		// all temps in deg F except Cold Junction
Params_t CHT = {420,365,200,390,LTC_2495_ADDR_CHT};				// ""




// The LTC_2495 has a strange ADC output format. It has 16 bits of ADC resolution plus a seperate sign and overflow bit. 
// The data arrives in a 3 byte stream with 6 lsb indicating all 0's. When shifted down 6 positions the bits 0 through 15 
// hold a unsigned 16 bit int
union adc_bytes		// Union to read the LTC_2495 ADC
{
	uint8_t bytes[4];	
	int32_t adc_val_24;

} ADC_result = {0,0,0,0};

union  {
	short all;
	struct  
	{	
		short EGT:1;
		short CHT:1;
		short EGT_cht :1;			// if true == EGT_display if false == CHT display
		short BAR_text :1;
	}bits;
} Options ={0};


#define ADC_REFERENCE_V (2.048)		// Reference voltage on VREF+, v.s. VREF-
#define ADC_SPEED_GAIN (0x83) 		// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
#define ADC_Bit_Value (ADC_REFERENCE_V/2/16/65536)  // Bit value of ADC to correspond to gain and reference 


#define CtoF( tC ) ( tC / 0.5555555555 +32)
//#define FtoC(xF ) ( (xF - 32.0) * 0.55555555555)


#define TREND_HYST_F 0.9		// half of the hystery in deg f -- 



/* 
 *  	Function to draw the scale, grid and legend.  Arguments indicate low and high endpoints
 *
*/
	
void DrawScale( int low, int high ) 
{

	unsigned char y ;
	int m;
	tft.setTextColor( ST7735_YELLOW,ST7735_BLACK );
	
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
	
	
/* Function to draw a single bar at column "pos" as a fraction of max height*/
// position 1 is bar column #1	
	
void DrawBar( int8_t pos, float height, uint16_t color = ST7735_MAGENTA, signed char trend = 0) 
{
  // draw a rectangle bar from the bottom up, spaced every n pixels
  unsigned char x,y;
  
  if (pos == 0 ) // This is not a bar -- internal mesurment 
  {
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

  if (y>ARROW_HEIGHT) // only show trend if bar is tall enough
  {
  	switch (trend)
  	{
  		case 1: // going up
  		  tft.drawBitmap(x+((BAR_width-ARROW_WIDTH)/2),BAR_bottom-y+1,arrowUp,ARROW_WIDTH,ARROW_HEIGHT,ST7735_BLACK);
  			//tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,0x18,ST7735_RED,ST7735_BLACK,1); 
  			break;
  			
  		case 2:// going down -- Note BLUE on the blow non zoomed bar will not be visible 
  			tft.drawBitmap(x+((BAR_width-ARROW_WIDTH)/2),BAR_bottom-y+1,arrowDown,ARROW_WIDTH,ARROW_HEIGHT,ST7735_BLACK);
  			//tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,0x19,ST7735_YELLOW,ST7735_BLACK,1); 
  			break;
  	//	default:  // don't need this -- bar top is already cleared
  	//		tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,' ',ST7735_YELLOW,ST7735_BLACK,1); 
  			
  			
  	}
  }
}


/*
	force grid to be drawn
*/ 
void
DispGrid( Params_t *p )
{
	bool zoom = true;
	
	for (int i =1; i<= MAX_CHANNEL; i++ )
	{
		if (p->Temps[i] <=  p->F_ZOOM)
			zoom = false;
	}	
	
	if (zoom )
		DrawScale(p->F_ZOOM, p->F_MAX); 	
	else
		DrawScale(0, p->F_MAX);
	
}
	

/*
	Function to draw the bargraph, one channel at a time.
*/	
void BarDisplay( int8_t curr_ch , Params_t *p)
{
	bool zoom = true;
	static bool prev_zoom = true;
	int f_zoom = p->F_ZOOM;
	int f_range = p->F_MAX - p->F_ZOOM;
	unsigned char trend = p->Trend[curr_ch];
	
	// switch to zoomed mode when all channels are above threshold 
	for (int i =1; i<= MAX_CHANNEL; i++ )
	{
		if (p->Temps[i] < f_zoom)
			zoom = false;
	}
	// draw the scale and grid when the display range changes 
	if (zoom != prev_zoom)
	{
		if (zoom )
			DrawScale(p->F_ZOOM, p->F_MAX);	
		else
			DrawScale(0, p->F_MAX);	
			
		prev_zoom  = zoom;	
	}
	

	if (zoom)
	{	
		uint16_t color;

		if (p->Temps[curr_ch] > p->F_ALARM )
			color = ST7735_RED;
		else if ( p->Temps[curr_ch] > p->F_WARN )
			color = ST7735_YELLOW;
		else color = ST7735_GREEN;
		
		DrawBar(curr_ch, (p->Temps[curr_ch] - f_zoom)/f_range, color, trend);
	}
	else
		DrawBar(curr_ch, p->Temps[curr_ch]/ p->F_MAX , ST7735_BLUE, trend);	
	
}
bool init_ADC(Params_t *p )
{
	p->Temps[COLD_JCT_NDX] = -275.0;	// invalid temp if no ADC is present on this address
	
	Wire.beginTransmission( p->ADCaddress);		
	Wire.write(0xA0);		// Enable bit only
	Wire.write(0xC0);		// meassure internal temp
	int8_t ret_val = Wire.endTransmission(1);	// true will send a stop message, releasing the bus after transmission. false will send a restart, keeping the connection active.
	// 0 == success, 1 == NACK upon transmit of address, 2 == NACK upon transmit of DATA, 3 == Other error

	if (ret_val !=0 )
	{
//tft.println("ADC not responding -- stop!");
		digitalWrite( LED1_PIN, 1);
		return false;
	}
	delay( 160 ); // conversion takes about 150ms 
	uint8_t num = Wire.requestFrom(p->ADCaddress,3);		// this starts a new conversion so that the main loop can read the int temp on the first call
	if (num != 3 )
	{
//tft.println("ADC not ready with result, STOP! ");
		return false;
	}
	
	// MSB to LSB
	ADC_result.bytes[2] = Wire.read();
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();

	uint16_t adc_val16 = ADC_result.adc_val_24 >> 6;	// lowest 6 bits are 0's 
	p->Temps[COLD_JCT_NDX] = ( adc_val16 * ADC_REFERENCE_V /12.25) -273.0;				// temperature formula from datasheet in Kelvin  THIS is IN C !!
	
	return true;
}

// todo: change type of return value to what Wire.endtransmission returns
int8_t ADC_InitNextConversion (Params_t *p, uint8_t next_ch )
{
#ifdef SIMUL
	return 0;
#endif
	if ( p == NULL )
		Wire.beginTransmission(LTC_2495_GLOBAL_ADDR);	 // global address of All ADCs to set next channel in parallel
	else	
		Wire.beginTransmission(p->ADCaddress);	
	
	if (next_ch == COLD_JCT_NDX)
	{
		Wire.write(0xA0);		// Enable bit only
		Wire.write(0xC0);		// meassure internal temp
	}
	else
	{
		Wire.write(0xA8 + next_ch -1);			// meassure differential input channel, first channel is In+=0, In-=1 -- minus 1 for 0 based selection
		Wire.write(ADC_SPEED_GAIN);				// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
	}
	
	// The I2C bus must be held in the "Restart" mode otherwise it triggers a new conversion cycle we don't want 
	return Wire.endTransmission( 0 ); // ending with Re-Start instead of STOP so that we still can read the result from the last conversion
	//return codes:  0 == success, 1 == NACK upon transmit of address, 2 == NACK upon transmit of DATA, 3 == Other error
		
}
int8_t  ADC_AquireConversion( Params_t *p, int8_t this_ch ) 
{
	int32_t ADC_cnt; //  32 bit integer so that we can convey 16bits plus a sign 
		
	if (this_ch > MAX_CHANNEL )
		return 0;
#ifdef SIMUL
	p->Temps[this_ch] = 220;	
	return 0;
#endif
	
	// reading the 3 bytes from the chip triggers a new conversion cycle -- Must keep the bus in "Restart" mode otyherwise the
	// end of the first read with a STOP condition will trigger an unwanted conversion cycle on the second ADC.
	// So we read all ADC's by leaving the bus in the "RESTART" state -- 3rd argument below == false
 	if (Wire.requestFrom(p->ADCaddress,3,false) != 3 )	// latches the last conversion data and restarts new conversion immediately
	{
		digitalWrite( LED1_PIN, 1);
		tft.fillScreen(ST7735_BLACK);
		tft.setCursor(0, 0);
		
		if (p->ADCaddress == 0x14 )
			tft.print( F("CHT "));
		else
  		tft.print( F("EGT "));
		
		tft.println(F("ADC not ready"));

		return 1;
	}
	// this gets the 3 bytes from the "wire" buffer 
	ADC_result.bytes[2] = Wire.read();	// MSB  
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();	// LSB
 

	// shift out the lower 6 bits that are all 0's and use the 16 bits as positive integer
	ADC_cnt = (uint16_t) (ADC_result.adc_val_24 >> 6); // it's a positive 16 bit number
	
	if (this_ch == COLD_JCT_NDX )	// the 'last' conversion was the internal temp (cold junction)
	{
		p->Temps[COLD_JCT_NDX] = 	 (ADC_cnt * ADC_REFERENCE_V /12.25) -273.0;// temperature formula from datasheet in Kelvin THIS IS in C !!
	}
	else // it's a thermocouple
	{
		// if sign bit true ==  positive voltages in lower 16 bits /*
		if ( ! (ADC_result.bytes[2] & 0x80) )	// it's a negative 16 bit number 
		{
			ADC_cnt = 0;			// upside down connected thermocouple:  read 0 
			// I could just flip it, but offset errors might be different, so better to get the probes on right	
		} 

		float TC_mv = ADC_cnt * ADC_Bit_Value * 1000; // in millivolts for coeff table 
		//Serial.print("TC_mv: ");
		//Serial.println(TC_mv); 
			
	
		float * table;		// pointer to the coeff table 
		// note: Both CHT and EGT probes are K-type because of availability 	
		//Serial.println("K-Type"); 
		// depending on the voltage, choose appropriate table as per NIST
		if (TC_mv < 20.644)		// threshold of table 1 to 2
			table = K_TC_coeff0_500;
		else
			table = K_TC_coeff500_plus;

		// calculate the thermo couple t90 from thermal EMF per NIST polynomial
		float TC_Temp_C;
		for (int n = 0; n <= N_poly; n++ )
		{
			double EE;// the n'th power of TC EMF
			if ( n == 0 )
			{
				EE = TC_mv;
				TC_Temp_C = table[0];
			}
			else
			{
				TC_Temp_C += EE * table[n];
				EE *= TC_mv;	// next power 
			}

		}
		
			
		// This math is not quite correct, but close enough -- should convert cold temp to EMF since poly is based from 0 deg C,
		// however that requires logarithm functions -- too expensive in mem foot print
		TC_Temp_C += p->Temps[COLD_JCT_NDX]; // add in the cold junction temp  -- kept in deg C

		float TC_Temp_F = CtoF(TC_Temp_C);	
		// this trend and peak detection is affected by noisy ADC readings since there is no averaging of the readings
		// for speed reasons.
			
		if ( (TC_Temp_F - TREND_HYST_F) > p->Temps[this_ch] )
		{
			p->Trend[this_ch] = 1;		// Rising 
		}
		else if ((TC_Temp_F + TREND_HYST_F) < p->Temps[this_ch] )
		{
			p->Trend[this_ch] = 2;		// Falling 
		}
		else 
			p->Trend[this_ch] = 0;		// neither 
			
		p->Temps[this_ch] = TC_Temp_F;		// store the current measurment 
	}

// end of acquiring TC readings  and analyzing trends
	return 0;
}
void Calc_min_max_delta_alarms ( Params_t *p , int8_t this_ch, unsigned int loopcnt)
{
	float min,max,temp;
 	if ( this_ch ==  0 && loopcnt >1)	// update min, max and alarm once around the loop
  {

		p->min_ndx = p->max_ndx =0;
		min=9999.9;
		max =0.0;

		for (int n = 1; n <= MAX_CHANNEL; n++ )
		{
			temp = p->Temps[n];

			if (temp > max ) 
			{
				max = temp;
				p->max_ndx = n;
			}
				
			if (temp < min ) 
			{
				min = temp;
				p->min_ndx = n;
			}
		}
	   
		if ( max > p->F_ALARM)	
				digitalWrite( LED1_PIN, 1);
		
		if (max > p->F_WARN )
			digitalWrite( LED2_PIN, 1);
		
		p->delta = p->Temps[p->max_ndx] - p->Temps[p->min_ndx];
	}
	
	if (this_ch == 4 )  // make the LEDS blink
	{
		digitalWrite( LED1_PIN, 0);
		digitalWrite( LED2_PIN, 0);
	}
		
}

void UpdateDisplay ( Params_t *p, int8_t this_ch, unsigned int loop_cnt)
{
	// display bars or text
	if (Options.bits.BAR_text )
	{
	 // display graph
		BarDisplay( this_ch, p); 
	}
	else	// text display
	{
		if ( this_ch == 0 ) // print the channel indicator and delta
		{	 // update delta and min max indicatioin once around all 6 CH 
			int n;
			for ( n = 1; n <= MAX_CHANNEL; n++ )
			{
				tft.setCursor(TEXT_leftMargin,(n-1)*TEXT_lineHeight);
			
				if (n == p->max_ndx)
					tft.setTextColor( ST7735_RED,ST7735_BLACK );
				else if (n == p->min_ndx)
					tft.setTextColor( ST7735_BLUE,ST7735_BLACK );
				else
					tft.setTextColor( ST7735_WHITE,ST7735_BLACK );
				
				if (Options.bits.EGT_cht)
					tft.print( F("EGT"));
				else
					tft.print( F("CHT"));
					
				tft.print(n);
				tft.print(":");
			}

			// the delta in F
			if (loop_cnt > 0)
			{
				tft.setTextColor( ST7735_YELLOW,ST7735_BLACK );
				tft.setCursor(TEXT_leftMargin,(n-1)*TEXT_lineHeight);
				tft.print( F("Delta: "));
				tft.print( p->delta,0);
				tft.print( F("  ")); // erase tail
			}
		}
		else // print the channel values 
		{
			int line = (this_ch-1) * TEXT_lineHeight;
			// using setTextColor with BG set to black erases 
			tft.setTextColor( ST7735_WHITE,ST7735_BLACK );
			tft.setCursor(TEXT_colVAL,line);
			tft.print(p->Temps[this_ch],0);
			tft.print( F("   ")); // erasing tail if it goes from 3 to 2 digits
				
			if (loop_cnt > 1)
			{
				tft.fillRect(TEXT_trendCOL,line,ARROW_WIDTH,TEXT_lineHeight,ST7735_BLACK);
				if ( p->Trend[this_ch] == 1 )
					tft.drawBitmap(TEXT_trendCOL,line,arrowUp,ARROW_WIDTH,ARROW_HEIGHT,ST7735_RED);
				else if (p->Trend[this_ch] == 2 )
					tft.drawBitmap(TEXT_trendCOL,line,arrowDown,ARROW_WIDTH,ARROW_HEIGHT,ST7735_BLUE);

			}
	
		}

	}
} 
#ifdef WITH_SD_CARD	
void storeDataToFile ( Params_t *p , bool ADC_present )
{	
	if (ADC_present )
	{
		for (int i =1; i<= MAX_CHANNEL; i++ )
		{
			file.print(p->Temps[i],0);file.write_P(PSTR(","));
		}
		file.print(p->delta,0);file.write_P(PSTR(","));
	}
	else
		file.write_P(PSTR(" , , , , , , 0, "));
}	
#endif


// This is the ARDUINO setup function -- called once upon reset
void setup(void) {

	uint8_t ret_val;
	uint8_t num;
	Options.all = 0;
	Options.bits.BAR_text =1; // start with bar display

	
	pinMode(LED1_PIN, OUTPUT);    // PB2,SS Wired to RED LED -- must be output if SD SPI bus is used -- Hardwired logic in chip.
	pinMode(LED2_PIN, OUTPUT);    //  	
	digitalWrite( LED1_PIN, 0);
	digitalWrite( LED2_PIN, 0);
 
  EncoderInit( Enc_A_PIN, Enc_B_PIN, Enc_PRESS_PIN );

	
	//Serial.begin(56700);
	//Serial.print("Hello! AIR_TFT_ADC");
	digitalWrite( LED2_PIN, 1);	 // blue led on during init
	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab, try different tab settings if the display is not correct
re_do:
	
	tft.setRotation(3);   // 3 for red pcb
	tft.setCursor(0, 0);
	tft.setTextSize(2);
	tft.fillScreen(ST7735_BLACK);	// clear the screen 
	digitalWrite( LED2_PIN, 0);
 
	tft.println(F("EGT/CHT\nMonitor"));		
	tft.println(F("Version 1.2"));
 digitalWrite( LED1_PIN, 1);
 
 
	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
	tft.print(F("VBatt: "));
	tft.print(Vbus_Volt);
	tft.println(F("V"));
	
	if (Vbus_Volt < 6.6)	// 2 LI-ion cells just about empty
	{
		tft.println(F("Voltage too low."));
		delay( 500 ); 
		goto re_do;	  // do not start SD card writes 
	}

 

	
	Wire.begin();						// initialize the i2C interface as master
	//Wire.setClock( 10000); 			//This call seems to do nothing -- so I write the HW registers diectly
	// setting a lower i2c clock frequency,  F_SCL = F_CPU/ 16 + (2*TWBR*TW_Prescale)
	TWBR = 99;	// for 20Khz clock rate 
	TWSR |=  1; // == prescale /4
	
#ifdef SIMUL
	Options.bits.CHT = true;
	Options.bits.EGT = true;
	CHT.Temps[COLD_JCT_NDX] = 21; 
	EGT.Temps[COLD_JCT_NDX] = 21; 
#else
	Options.bits.CHT = init_ADC( &CHT);
	Options.bits.EGT = init_ADC( &EGT);
#endif
	
	digitalWrite( LED2_PIN, 0);
	
	if ( Options.bits.CHT == false  )
		tft.println( F("No CHT ADC!"));
	if ( Options.bits.EGT == false  )
		tft.println( F("No EGT ADC!"));
	
	if (Options.bits.EGT == false && Options.bits.CHT == false  )
	{
	//		Serial.print("No ADC is responding -- stop!");
		digitalWrite( LED1_PIN, 1);
		tft.println( F("No ADCs found"));
		delay( 1500 ); 
		goto re_do;
	}
	
 #ifdef WITH_SD_CARD
 if (!file.isOpen()   )
 {
  if (  card.begin(SD_CARD_CS) ) 
  {
    if (Fat16::init(&card))
    {
      if ( file.open(DataFile, O_CREAT | O_WRITE | O_APPEND))
      {
	      file.println( F("CHT/EGT Monitor in deg F")); // header indicates new run
        file.print( F("Battery Voltage: "));
				file.println( Vbus_Volt );
        file.println( F("Time,#E1,#E2,#E3,#E4,#E5,#E6,E_Delta,#C1,#C2,#C3,#C4,#C5,#C6,C_Delta"));  
        file.sync();
      }
      else
      {
        tft.println( F("No File Open"));
      }
    }
    else
    {
      tft.println( F("No FAT init"));
    }

  }
  else
  {
      tft.println( F("No SD init"));
  }
 }
#endif 
	
// there is at least one ADC present or we don't get here
	if ( Options.bits.CHT )
	{
		tft.print( F("CHT: "));
		tft.print(CtoF( CHT.Temps[COLD_JCT_NDX])); 
		tft.println( F("F"));
		Options.bits.EGT_cht = 0;
	}
	
	if ( Options.bits.EGT )
	{
		tft.print( F("EGT: "));
		tft.print(CtoF( EGT.Temps[COLD_JCT_NDX])); 
		tft.println( F("F"));
		Options.bits.EGT_cht = 1;		// EGT mode wins on startup
	}

	// Delay to read the above messages on the screen and wait until the ADCs are ready again for an other reading to be started
	delay (2500);	
	
	tft.fillScreen(ST7735_BLACK);	// clear the screen 
	if (Options.bits.BAR_text)		// display the grid
	{
		tft.setTextSize(1);
		DispGrid((Options.bits.EGT_cht) ? &EGT : &CHT );
			
	}
	else
		tft.setTextSize(2);				// display in text mode 
	
		digitalWrite( LED1_PIN, 0);
		digitalWrite( LED1_PIN, 0);
 // Serial.println("exit setup");
}

// This is the ARDUINO loop execute function, called contineously 
void loop() 
{ 
	int8_t adc_error = 0;
	static unsigned long time_ms_next = 0;
	static unsigned char prev_short_press = 0;
	static unsigned char prev_long_press = 0;
	static char prev_enc_cnt  = 1;
	
	// The curr_ch is the channel for which we can read the data in this loop instance for which the conversion was started in the prior loop
	// The next_ch is the channel for which the ADC conversion gets started in this loop intance. 
 
	static int8_t curr_ch = COLD_JCT_NDX;
	int8_t next_ch;

	static unsigned int  loopcnt = 0;	// 


	// do the Vbus reading and disconnect the SD card when battery is too low
	// requires supercap on Vbattery 	
	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
		
	if (Vbus_Volt < 6.4  )
	{
		digitalWrite( LED1_PIN, 0);		// reduce current draw on the way out, so that the file.close can happen on the supercap
		digitalWrite( LED2_PIN, 0);
	
#ifdef WITH_SD_CARD
		if ( file.isOpen() )			// Close the SD card cleanly
		{
			file.write_P(PSTR("End"));
			file.close();
		}
#endif
		tft.fillScreen(ST7735_BLACK);
		tft.setCursor(10, 10);
		tft.setTextSize(2);
		tft.setTextColor( ST7735_YELLOW,ST7735_BLACK );
		tft.print(Vbus_Volt);
		tft.print( F(" V"));
		tft.setCursor(10, 30);
		tft.print( F("Voltage low!\n STOP!"));
		// of course being able to turn off the device would be better solution alltogether
		while (1);
	}
	
	// change of display mode -- text or bars / cht/egt /time
	if ( LongPressCnt != prev_long_press || ShortPressCnt != prev_short_press ||  EncoderCnt != prev_enc_cnt)
	{	
		tft.fillScreen(ST7735_BLACK);					// clear the screen 
		if (	EncoderCnt != prev_enc_cnt)
		{
				tft.setCursor(10, 40);
				tft.setTextSize(2);
				tft.setTextColor( ST7735_YELLOW,ST7735_BLACK );
	
			if ( EncoderCnt > prev_enc_cnt )
			{
				// display the time on the display and delay a few seconds 
				tft.print(F("\nTime:"));
				tft.print(time_ms_next/60000);
				tft.println(F(" min"));

			}
			
			else
			{
				
				tft.print(F("\nVolt: "));
				tft.print(Vbus_Volt);
				tft.println(F("V"));
			}
			
			prev_enc_cnt = EncoderCnt;
			delay(2500);
			tft.fillScreen(ST7735_BLACK);					// clear the screen 
		}
			
		if ( LongPressCnt != prev_long_press)
		{
			prev_long_press = LongPressCnt;
			Options.bits.BAR_text = !Options.bits.BAR_text; // toggle display bars or text
		}
		else
		{
			prev_short_press = ShortPressCnt;
			if (Options.bits.EGT && Options.bits.CHT )		// If both ADC's are connected switch the display to the other -- else no action on long press
			{
				Options.bits.EGT_cht = !Options.bits.EGT_cht; // switch to the other one
			//	curr_ch = MAX_CHANNEL;  // so it starts from the cold_junction reading again
			}	
		}
	
		if (Options.bits.BAR_text)						// redisplay the grid
		{
			tft.setTextSize(1);
			DispGrid((Options.bits.EGT_cht) ? &EGT :&CHT );
		}
		else
			tft.setTextSize(2);
	}
	
	
#ifdef kjjkuuhjlkjkjlkjlkjlkj	
	if (digitalRead(Enc_PRESS_PIN ) == 0 )
	{
		unsigned long t = millis();
		
		tft.fillScreen(ST7735_BLACK);					// clear the screen 
	
		while (digitalRead(Enc_PRESS_PIN ) == 0 )	// wait until button is released
		{
		}
		// button press released
		
		if ( millis() -t  < 400 ) // short press action -- switch EGT/CHT display 
		{
			if (Options.bits.EGT && Options.bits.CHT )		// If both ADC's are connected switch the display to the other -- else no action on long press
			{
				Options.bits.EGT_cht = !Options.bits.EGT_cht; // switch to the other one
			//	curr_ch = MAX_CHANNEL;  // so it starts from the cold_junction reading again
			}	
		}
		else // short press action -- switch between text and bar display 
		{
			Options.bits.BAR_text = !Options.bits.BAR_text; // toggle display 
		}
		
		if (Options.bits.BAR_text)						// redisplay the grid
		{
			tft.setTextSize(1);
			DispGrid((Options.bits.EGT_cht) ? &EGT :&CHT );
		}
		else
			tft.setTextSize(2);
	
		delay(100) ; // in case of a bounce
	}
#endif 	
	// according to the datasheet of LTC2495/page5 the conversion time in 50/60hz 1x speed mode is 149ms -- we wait ~200ms 
	if (millis() < time_ms_next  )	// wait for next measure interval 
		return; 
	
	time_ms_next = millis() +155; // next measurement cycle 
 
	// Begin of acquiring TC readings  -- one reading per loop 	
	// latch the previous reading and setup the channel for the next conversion 

	if ( curr_ch >= MAX_CHANNEL )
	{	// we are once around -- start again with the cold jucntion
		loopcnt++;
		next_ch = COLD_JCT_NDX;
	}
	else
	{	
		next_ch = curr_ch+1;
	}

	adc_error += ADC_InitNextConversion (NULL,  next_ch );		// using global addressing of the ADC since both devices are setup the same
	
	if (adc_error)
	{
			digitalWrite( LED1_PIN, 1);
			tft.fillScreen(ST7735_BLACK);
			tft.setCursor(0, 10);
			tft.println( F("ADC not responding to Init"));
			return;		// loop until problem corrected 
	}

 
	// now get the ADC readings and start the next conversion cycle immediatly 
	if (Options.bits.EGT )
		adc_error += ADC_AquireConversion (&EGT,  curr_ch );
 
	if (Options.bits.CHT )
		adc_error += ADC_AquireConversion (&CHT,  curr_ch );
 	
		
	if (adc_error)
	{
			digitalWrite( LED1_PIN, 1);
			//tft.fillScreen(ST7735_BLACK);
			tft.setCursor(0, 10);
			tft.println( F("ADC not ready with data"));
	
			return;		// loop until problem corrected 
	}

	if (Options.bits.EGT )
		Calc_min_max_delta_alarms ( &EGT,curr_ch, loopcnt);
	
	if (Options.bits.CHT )
		Calc_min_max_delta_alarms ( &CHT, curr_ch, loopcnt);
	
	UpdateDisplay( (Options.bits.EGT_cht) ? &EGT :&CHT, curr_ch, loopcnt);


	
#ifdef WITH_SD_CARD	

	if (file.isOpen() && loopcnt % SD_RECORD_PERiod == 0 && curr_ch ==0 ) 
	{

		unsigned short secs = time_ms_next/1000;
		

		
		if (secs/60 <10 )
		  file.write_P(PSTR("0"));
		
		file.print(secs/60 );file.write_P(PSTR(":"));		// minutes and seconds since start
		
		if (secs%60 <10 )
			file.write_P(PSTR("0"));
		
		file.print(secs%60 );file.write_P(PSTR(","));	
		
		storeDataToFile ( &EGT , Options.bits.EGT );
		storeDataToFile ( &CHT , Options.bits.CHT );
		file.write_P(PSTR("\n"));			
		file.sync();

	}
#endif	
			
	curr_ch = next_ch;  // for next time around


}
