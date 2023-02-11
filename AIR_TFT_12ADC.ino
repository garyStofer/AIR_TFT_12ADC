#include <TFT.h>    // This further includes Adafruit_GFX and Adafruit_ST7735 from C:/Program Files/Arduino/libraries/tft/utility
#include <SPI.h>	// SPI for the SD  card 
#include <Wire.h>	// I2C for the TFT 
#include "TC_coeff.h"

// Using the HW SPI interface of ATMEL 328P -- Note Atmega pin "SS" (Wiring pin 10)  must be configured as OUTPUT even though it might not be used as SlaveSelect. 
// SPI interface will not function otherwise
// Connect signal SCL on TFT board to SCK of 328P , pin17, PB5 (SCK) 
// Connect signal SDA on TFT board to MOSI of 328P, pin15 , PB3(MOSI)
// only these additional pins need to be defined
#define TFT_CS     9 // Chip select -- Could also use SlaveSelect from HW SPI interface 
#define TFT_RST    7 // you can also connect this to the Arduino reset, in which case, set this #define pin to -1!
#define TFT_DC     6 //  This is Data/Command of the TFT, often labeled as A0 on TFT board


// The two LEDS 
#define LED1_PIN 	10     // aka PB2,SS Wired to RED LED -- must be output if SD SPI bus is used -- Hardwired logic in chip.
#define LED2_PIN 	17     // aka PC3,A3, Wired to Blue LED

#define RotaryKnob_A 2	//aka D2 PD2, Int0
#define RotaryKnob_B	14	// aka A0,D14,PC0
#define RotaryKnob_Push 3	// aka D3,PD3,Int1


#define WITH_SD_CARD
#ifdef WITH_SD_CARD

// NOTE: Make sure SD card is Formatted with FAT16 as discussed in the readme file of the lib. Will not work on Fat32, Exfat,Fat12

#include <Fat16.h>
// #include <Fat16util.h> // use functions to print strings from flash memory

#define SD_CARD_CS    5   // aka D5,PD5   SD chip select pin.
SdCard card;        // SD card class element
Fat16 file;         // FAT class element

#define SD_RECORD_PERiod 5	// every n times trough a complete loop of 7 adc readings , 1.4 second per loop,  5 = 7 seconds

// change the filename according to which Aircraft this is installed
#define DataFile "N9169K.csv"
#endif // with SD_Card


#define VBUS_ADC 7			// ADC7
#define VBUS_ADC_BW  (5.0*(14+6.8)/(1024*6.8))	//Bit weight of ADC7 using a voltage divider 14.0K and 6.8k to gnd.

// defines for the bar graph
#define CHAR_HEIGHT 7 	// a character at scale 1 is 7 pixels tall , plus 1 for gap
#define CHAR_WIDTH 5    // plus one for gap  
#define BAR_gap 1       // horizontal gap between two bars

// arrow icon bitmaps which must be stored in flash
// new lines must start on new bytes
const uint8_t arrowUp[] PROGMEM = {0x10,0x10,0x38,0x38,0x7C,0x7C,0xFE,0xFE};
const uint8_t arrowDown[] PROGMEM = {0xFE,0xFE,0x7C,0x7C,0x38,0x38,0x10,0x10};
// Starngly enough the defines of witdh and height are backwards in the adafruit source 
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


// The instance of the TFT class controlling the LCD -- using the HW SPI interface from the cpu
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Global data elements
static float Temps[ MAX_CHANNEL+1] ;		// index 0  is cold junction, then ch1, ch2
static int8_t Trend[MAX_CHANNEL+1] ;
static float delta;
static unsigned long time_ms =0;
static unsigned long last_rec =1;

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
		short EGT_cht:1;
		short BAR_text :1;
	}bits;
} Options ={0};

#define LTC_2495_ADDR 	(0x45) 		// 7 bit I2C address for ADC -- All address pins left floating
#define ADC_REFERENCE_V (2.048)		// Reference voltage on VREF+, v.s. VREF-
#define ADC_SPEED_GAIN (0x83) 		// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
#define ADC_Bit_Value (ADC_REFERENCE_V/2/16/65536)  // Bit value of ADC to correspond to gain and reference 


#define CtoF( tC ) ( tC / 0.5555555555 +32)
#define FtoC(xF ) ( (xF - 32.0) * 0.55555555555)

// #define EGT_MODE // when using as EGT instrument, otherwise CHT scaling

#ifdef EGT_MODE
// in deg F  used for display only 
	#define F_MAX  1650	
	#define F_WARN 1500
	#define F_ZOOM 1200	
	#define F_ALARM 1550
#else // All Franklin engines CHT max in Probe hole is 390F
	#define F_MAX  420	
	#define F_WARN 365
	#define F_ZOOM 150
	#define F_ALARM 390
#endif

	
// in deg C -- used in code
#define C_MAX  (FtoC(F_MAX)) 
#define C_WARN (FtoC(F_WARN))
#define C_ZOOM (FtoC(F_ZOOM)) 
#define C_ALARM ( FtoC(F_ALARM) )


#define TREND_HYST 0.5		// half of the hystery in deg C -- 



/* 
 *  	Function to draw the scale, grid and legend.  Arguments indicate low and high endpoints
 *
*/
	
void DrawScale( int low, int high ) 
{

	unsigned char y ;
	int m;

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
  		  tft.drawBitmap(x+((BAR_width-ARROW_WIDTH)/2),BAR_bottom-y+1,arrowUp,ARROW_WIDTH,ARROW_HEIGHT,ST7735_RED);
  			//tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,0x18,ST7735_RED,ST7735_BLACK,1); 
  			break;
  			
  		case 2:// going down -- Note BLUE on the blow non zoomed bar will not be visible 
  			tft.drawBitmap(x+((BAR_width-ARROW_WIDTH)/2),BAR_bottom-y+1,arrowDown,ARROW_WIDTH,ARROW_HEIGHT,ST7735_BLUE);
  			//tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,0x19,ST7735_YELLOW,ST7735_BLACK,1); 
  			break;
  	//	default:  // don't need this -- bar top is already cleared
  	//		tft.drawChar(x+(BAR_width/2)-3,BAR_bottom - y -8,' ',ST7735_YELLOW,ST7735_BLACK,1); 
  			
  			
  	}
  }
}
// force grid to be drawn
void
DispGrid( void )
{
	bool zoom = true;
	for (int i =1; i<= MAX_CHANNEL; i++ )
	{
		if (Temps[i] <=  C_ZOOM)
			zoom = false;
	}	
	if (zoom )
		DrawScale(F_ZOOM, F_MAX);	
	else
		DrawScale(0, F_MAX);
	
}
	

// Function to draw the bargraph, one channel at a time.
void BarDisplay( int8_t curr_ch , float *Temps, unsigned char trend)
{
	bool zoom = true;
	static bool prev_zoom = true;
	
	// switch to zoomed mode when all channels are above threshold 
	for (int i =1; i<= MAX_CHANNEL; i++ )
	{
		if (Temps[i] < C_ZOOM)
			zoom = false;
	}
	// draw the scale and grid when the display range changes 
	if (zoom != prev_zoom)
	{
		if (zoom )
			DrawScale(F_ZOOM, F_MAX);	
		else
			DrawScale(0, F_MAX);	
			
		prev_zoom  = zoom;	
	}
	

	if (zoom)
	{	
		uint16_t color = ST7735_GREEN; ;

		if (Temps[curr_ch] > C_ALARM )
			color = ST7735_RED;
		else if ( Temps[curr_ch] > C_WARN )
			color = ST7735_YELLOW;
		
				
		DrawBar(curr_ch, (Temps[curr_ch] - C_ZOOM)/(C_MAX - C_ZOOM), color, trend);
	}
	else
		DrawBar(curr_ch, Temps[curr_ch]/ C_MAX , ST7735_BLUE, trend);	
	
}


// This is the ARDUINO setup function -- called once upon reset
void setup(void) {
	
	uint8_t ret_val;
	uint8_t num;
	Options.all = 0;
	Options.bits.BAR_text =1;
	
	pinMode(LED1_PIN, OUTPUT);    // PB2,SS Wired to RED LED -- must be output if SD SPI bus is used -- Hardwired logic in chip.
	digitalWrite( LED1_PIN, 0);
	
	//Serial.begin(56700);
	//Serial.print("Hello! AIR_TFT_ADC");

	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab, try different tab settings if the display is not correct
re_do:	
	tft.setRotation(3);   // 3 for red pcb
	tft.setCursor(0, 0);
	tft.setTextSize(2);
	tft.fillScreen(ST7735_BLACK);	// clear the screen 

#ifdef EGT_MODE	
	Options.bits.EGT_cht = 1;
	tft.println("EGT Monitor");
#else
	Options.bits.EGT_cht = 0;
	tft.println("CHT Monitor");
#endif
		
	tft.println("Version 1.0\n");


 


	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
	tft.print("VBatt: ");
	tft.print(Vbus_Volt);
	tft.println("V\n");
	
	if (Vbus_Volt < 6.6)	// 2 LI-ion cells just about empty
	{
		tft.println("Voltage too low.");
		delay( 500 ); 
		goto re_do;	  // do not start SD card writes 
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
        if (Options.bits.EGT_cht )
          file.print("EGT");
        else 
          file.print("CHT");
          
        file.println(" Monitor in deg F"); // header indicates new run
        file.print("Battery Voltage: ");file.println( Vbus_Volt );
        file.println("Time,#1,#2,#3,#4,#5,#6,Delta"); // header indicates new run
        file.sync();
      }
      else
      {
        tft.println("NoFileOpen");
      }
    }
    else
    {
      tft.println("NoFATinit");
    }

  }
  else
  {
      tft.println("NoSDinit");
  }
 }
#endif 
	
	Wire.begin();						// initialize the i2C interface as master
	//Wire.setClock( 10000); 			//This call seems to do nothing -- so I write the HW registers diectly
	// setting a lower i2c clock frequency,  F_SCL = F_CPU/ 16 + (2*TWBR*TW_Prescale)
	TWBR = 99;	// for 20Khz clock rate 
	TWSR |=  1; // == prescale /4
 
	Wire.beginTransmission(LTC_2495_ADDR);		
	Wire.write(0xa0);		// Enable bit only
	Wire.write(0xC0);		// meassure internal temp
	ret_val = Wire.endTransmission();	// 0 == success, 1 == NACK upon transmit of address, 2 == NACK upon transmit of DATA, 3 == Other error
	if (ret_val !=0 )
	{
//		Serial.print("ADC not responding -- stop!");
		digitalWrite( LED1_PIN, 1);
		tft.println("No ADC found!");
		delay( 500 ); 
		goto re_do;
	}
	delay( 200 ); // conversion takes about 160ms 
	num = Wire.requestFrom(LTC_2495_ADDR,3);		// this starts a new conversion so that the main loop can read the int temp on the first call
	if (num != 3 )
	{
//		Serial.print("ADC not ready with result, STOP! ");
		digitalWrite( LED1_PIN, 1);
		tft.println("ADC not ready!");
		delay( 500 ); 
		goto re_do;
	}
	
	// MSB to LSB
	ADC_result.bytes[2] = Wire.read();
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();

	uint16_t adc_val16 = ADC_result.adc_val_24 >> 6;	// lowest 6 bits are 0's 
	Temps[COLD_JCT_NDX] = ( adc_val16 * ADC_REFERENCE_V /12.25) -273.0;				// temperature formula from datasheet in Kelvin

	tft.print("Temp: ");
	tft.print(CtoF( Temps[COLD_JCT_NDX])); 
	tft.println("F\n");


	


  
 

	// Delay to read the above messages on the screen.
	delay (2500);	// after the read above a conversion is started again -- can't talk to the chip again until it's done 
	
	tft.fillScreen(ST7735_BLACK);	// clear the screen 
	if (Options.bits.BAR_text)		// redisplay the grid
	{
		tft.setTextSize(1);
		DispGrid();
	}
	else
		tft.setTextSize(2);
	

 // Serial.println("exit setup");
  }


// This is the ARDUINO loop execute function, called contineously 
void loop() 
{ 

	// The curr_ch is the channel for which we can read the data in this loop instance for which the conversion was started in the prior loop
	// The next_ch is the channel for which the ADC conversion gets started in this loop intance. 
 
	static int8_t curr_ch = COLD_JCT_NDX;
	int8_t next_ch;
	int32_t ADC_cnt; //  32 bit integer so that we can convey 16bits plus a sign 
	static int  loopcnt =0;
	static float MAX_temp=0;	// long term max temperature to be recorded to SD card file upon shut down

	
	// according to the datasheet of LTC2495/page5 the conversion time in 50/60hz 1x speed mode is 149ms -- we wait ~200ms 
	if (millis() < time_ms  )	// wait for next measure interval 
		return; 
	
	time_ms = millis(); 
	time_ms += 170;			// next measurement cycle 

	
	// do the Vbus reading and disconnect the SD card when battery is too low
	// requires supercap on Vbattery 	
	float Vbus_Volt = analogRead(VBUS_ADC) * VBUS_ADC_BW;	// read the battery voltage 
		
	if (Vbus_Volt < 6.4  )
	{
		digitalWrite( LED1_PIN, 0);		// reduce current draw on the way out, so that the file.close can happen on the supercap
		digitalWrite( LED2_PIN, 0);
	
		if ( file.isOpen() )			// Close the SD card cleanly
		{
			file.print("End  MAX-temp: ");
			file.println (CtoF(MAX_temp));
			file.close();
		}
		
		tft.fillScreen(ST7735_BLACK);
		tft.setCursor(10, 10);
		tft.print(Vbus_Volt);
		tft.print(" V");
		tft.setCursor(10, 30);
		tft.print("Voltage low!\n STOP!");
		// of course being able to turn off the device would be better solution alltogether
		while (1);
	}
	
	// Begin of acquiring TC readings  -- one reading per loop 	
	// latch the previous reading and set the channel for the next  
	if ( curr_ch >= MAX_CHANNEL )
	{	// we are once around -- start again with the cold jucntion
		loopcnt++;
		next_ch = COLD_JCT_NDX;

		Wire.beginTransmission(LTC_2495_ADDR);	
		Wire.write(0xa0);		// Enable bit only
		Wire.write(0xC0);		// meassure internal temp
		uint8_t ret_val = Wire.endTransmission( 0 ); // ending with ReStart instead of STOP so that we still can read the result from the last conversion
		// 0 == success, 1 == NACK upon transmit of address, 2 == NACK upon transmit of DATA, 3 == Other error

		if (ret_val )
		{
			digitalWrite( LED1_PIN, 1);
			tft.fillScreen(ST7735_BLACK);
			tft.setCursor(0, 10);
			tft.println("ADC not responding");
			return;		// loop until problem corrected 
		}
		


	}
	else
	{	
		next_ch = curr_ch+1;
		// select next input channels -- differential mode
		
	 	Wire.beginTransmission(LTC_2495_ADDR);	
		Wire.write(0xA8 + next_ch -1);			// meassure differential input channel, first channel is In+=0, In-=1 -- minus 1 for 0 based selection
	//	Wire.write (0xB0+next_ch-1);// +next_ch-1);// single 
		Wire.write(ADC_SPEED_GAIN);				// 1x speed , gain 16, input range = +- 64mv, (Vref/2/16= 0.064), 
		Wire.endTransmission( 0 ); // ending with Re-Start instead of STOP so that we still can read the result from the last conversion

	}


	if (Wire.requestFrom(LTC_2495_ADDR,3) != 3 )	// latches the last conversion data and restarts new conversion immediately
	{
		digitalWrite( LED1_PIN, 1);
		tft.fillScreen(ST7735_BLACK);
		tft.setCursor(0, 10);
		tft.println("ADC not ready");
		return;
	}

	ADC_result.bytes[2] = Wire.read();	// MSB  
	ADC_result.bytes[1] = Wire.read();
	ADC_result.bytes[0] = Wire.read();	// LSB
		


	// shift out the lower 6 bits that are all 0's and use the 16 bits as positive integer
	ADC_cnt = (uint16_t) (ADC_result.adc_val_24 >> 6); // it's a positive 16 bit number
	
	if (curr_ch == COLD_JCT_NDX )	// the 'last' conversion was the internal temp (cold junction)
	{
		Temps[COLD_JCT_NDX] = 	 (ADC_cnt * ADC_REFERENCE_V /12.25) -273.0;// temperature formula from datasheet in Kelvin
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
		if (Options.bits.EGT_cht )
		{	
			//Serial.println("K-Type"); 
			// depending on the voltage, choose appropriate table as per NIST
			if (TC_mv < 20.644)		// threshold of table 1 to 2
				table = K_TC_coeff0_500;
			else
				table = K_TC_coeff500_plus; 
		}
		else
		{
			//Serial.println("J-Type");
			//tab =J_TC_coeff0_760; 
			table = K_TC_coeff0_500;	// using K-type CHT probes since  J Types are not available 
		}

		// calculate the thermocouple t90 from thermal EMF per NIST polynomial
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
		TC_Temp_C += Temps[COLD_JCT_NDX]; // add in the cold junction temp

		// this trend and peak detection is affected by noisy ADC readings since there is no averaging of the readings
		// for speed reasons.
			
		if ( (TC_Temp_C - TREND_HYST) > Temps[curr_ch] )
		{
			Trend[curr_ch] = 1;		// Rising 
		}
		else if ((TC_Temp_C + TREND_HYST) < Temps[curr_ch] )
		{
			Trend[curr_ch] = 2;		// Falling 
		}
		else 
			Trend[curr_ch] = 0;		// neither 
			
		Temps[curr_ch] = TC_Temp_C;

		
	}

// end of acquiring TC readings  

  int8_t min_ndx =0, max_ndx=0;
  int n;
  
  if ( curr_ch ==  0)	// update min, max and alarm once around the loop
  {
	float min,max,temp;



	for (n = 1, min=9999, max =0; n <= MAX_CHANNEL; n++ )
	{
		temp = Temps[n];

		if (temp > max ) 
		{
			max = temp;
			max_ndx=n;
		}
			
		if (temp < min ) 
		{
			min = temp;
			min_ndx =n;
		}
	}
	  
	if (max > MAX_temp)
		MAX_temp = max;  // will get saved to SD card upon end of flight
			

	if (max > C_ALARM) 
		digitalWrite( LED1_PIN, 1);
	else
		digitalWrite( LED1_PIN, 0);
	
	if (max > C_WARN) 
		digitalWrite( LED2_PIN, 1);
	else
		digitalWrite( LED2_PIN, 0);
		
	if (digitalRead(RotaryKnob_Push) == 0 )
	{
		Options.bits.BAR_text = !Options.bits.BAR_text; // toggle display 
		tft.fillScreen(ST7735_BLACK);					// clear the screen 
		
		if (Options.bits.BAR_text)						// redisplay the grid
		{
			tft.setTextSize(1);
			DispGrid();
		}
		else
			tft.setTextSize(2);
		
		while (digitalRead(RotaryKnob_Push) == 0 )	// wait until button is released
		;
	}
	
	
  }

	// display text
	if (Options.bits.BAR_text )
	{
	 // display graph
		BarDisplay( curr_ch, Temps,Trend[curr_ch]);
	}
	else	// text display
	{
		if ( curr_ch == 0 ) // print the channel indicator and delta
		{	 // update delta and min max indicatioin once around all 6 CH 

			for ( n = 1; n <= MAX_CHANNEL; n++ )
			{
				tft.setCursor(TEXT_leftMargin,(n-1)*TEXT_lineHeight);
			
				if (n == max_ndx)
					tft.setTextColor( ST7735_RED,ST7735_BLACK );
				else if (n == min_ndx)
					tft.setTextColor( ST7735_BLUE,ST7735_BLACK );
				else
					tft.setTextColor( ST7735_WHITE,ST7735_BLACK );
				tft.print("CH");
				tft.print(n);
				tft.print(":");
				
				
			}
			// the delta in F
			if (loopcnt > 0)
			{
				delta = CtoF(Temps[max_ndx]) - CtoF(Temps[min_ndx]);
				tft.setTextColor( ST7735_YELLOW,ST7735_BLACK );
				tft.setCursor(TEXT_leftMargin,(n-1)*TEXT_lineHeight);
				tft.print("Delta: ");
				tft.print( delta,0);
				tft.print("  "); // erase tail
			}
		}
		else // print the channel values 
		{
			int line = (curr_ch-1) * TEXT_lineHeight;
			// using setTextColor with BG set to black erases 
			tft.setTextColor( ST7735_WHITE,ST7735_BLACK );
			tft.setCursor(TEXT_colVAL,line);
			tft.print(CtoF(Temps[curr_ch]),0);
			tft.print("   "); // erasing tail if it goes from 3 to 2 digits
				
			if (loopcnt > 1)
			{
				tft.fillRect(TEXT_trendCOL,line,ARROW_WIDTH,TEXT_lineHeight,ST7735_BLACK);
				if ( Trend[curr_ch] == 1 )
					tft.drawBitmap(TEXT_trendCOL,line,arrowUp,ARROW_WIDTH,ARROW_HEIGHT,ST7735_RED);
				else if (Trend[curr_ch] == 2 )
					tft.drawBitmap(TEXT_trendCOL,line,arrowDown,ARROW_WIDTH,ARROW_HEIGHT,ST7735_GREEN);

			}
	
		}

	}


	
#ifdef WITH_SD_CARD	

	if (file.isOpen() && loopcnt%5 == 0 && curr_ch ==0 ) 
	{
		
		unsigned short secs = time_ms/1000;
		if (secs/60 <10 )
			file.print("0");
		file.print(secs/60 );file.print(":");		// minutes and seconds since start
		if (secs%60 <10 )
			file.print("0");
		file.print(secs%60 );file.print(",");	
		file.print(CtoF(Temps[1]),0);file.print(",");
		file.print(CtoF(Temps[2]),0);file.print(",");
		file.print(CtoF(Temps[3]),0);file.print(",");
		file.print(CtoF(Temps[4]),0);file.print(",");
		file.print(CtoF(Temps[5]),0);file.print(",");
		file.print(CtoF(Temps[6]),0);file.print(",");
		file.println(CtoF(delta),0);
		file.sync();

		loopcnt++;

    }
#endif	
			
	curr_ch = next_ch;  // for next time around


}
