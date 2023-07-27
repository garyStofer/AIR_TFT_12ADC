// The pin definitions are as per obfuscated Arduino pin defines -- see aka for ATMEL pin names as found on the MEGA328P spec sheet

// Using the HW SPI interface of ATMEL 328P -- Note Atmega pin "SS" (Wiring opin 10)  must be configured as OUTPUT even though it might not be used as SlaveSelect. 
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

//#define RotaryKnob_A 2	//aka D2 PD2, Int0
//#define RotaryKnob_B 14	// aka A0,D14,PC0
//#define RotaryKnob_Push 3	// aka D3,PD3,Int1// ADC 5 used for Slave Select on SD card SPI interface

#define Enc_A_PIN 2     // This generates the interrupt, providing the click  aka PD2 (Int0)
#define Enc_B_PIN 14    // This is providing the direction aka PC0,A0
#define Enc_PRESS_PIN 3 // aka PD3 ((Int1)


#define SD_CARD_CS    5   // aka D5,PD5   SD chip select pin.
#define VBUS_ADC 7			// ADC7 on atmega 328P
 




