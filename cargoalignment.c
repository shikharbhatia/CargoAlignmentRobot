/*
 *
 * Team Id: eYRC#2172-CA
 * Author List: Shikhar Bhatia, Shivam Singh, Sanjay Singh, Vishal Singh
 * Filename: cargoalignment.c
 * Theme: Cargo alignment 

 * Functions: spi_pin_config (void), lcd_port_config (void), left_encoder_pin_config (void), right_encoder_pin_config (void), adc_pin_config (void), spi_init(void),
               buzzer_pin_config (void), left_position_encoder_interrupt_init (void), right_position_encoder_interrupt_init (void), ISR(INT5_vect), ISR(INT4_vect), 
			   motion_pin_config (void),  buzzer_on (void),  buzzer_off (void), servo1_pin_config (void), servo2_pin_config (void), Sharp_GP2D12_estimation(unsigned char adc_reading),
			    servo_1(unsigned char degrees), servo_2(unsigned char degrees), servo_1_free (void), servo_2_free (void), free_1(), free_2(), white_sensing(), port_init(),
				 timer5_init(), timer1_init(), adc_init(), ADC_Conversion(unsigned char Ch), sensor(), print_sensor(char row, char coloumn,unsigned char channel),
				 velocity (unsigned char left_motor, unsigned char right_motor), motion_set (unsigned char Direction), forward(void), back(void), left(void), right(void),
				 soft_left(void), soft_right(void), soft_left_2(void), soft_right_2(void), back_mm(unsigned int DistanceInMM), left_degrees(unsigned int Degrees),
				 right_degrees(unsigned int Degrees), soft_left_degrees(unsigned int Degrees), soft_right_degrees(unsigned int Degrees), soft_right_degrees_2(unsigned int Degrees),
				 soft_left_degrees_2(unsigned int Degrees), stop (void), angle_rotate(unsigned int Degrees), linear_distance_mm(unsigned int DistanceInMM),
				 spi_master_tx_and_rx (unsigned char data), forward_mm(unsigned int DistanceInMM), init_devices (void), sharp(), turn(), pathC(), pathC1(), Rsearch(), Lsearch(), R1search(),
				 L1search(), recursive(), wallfollower(), white_left(), white_right(), D1exit(), D1Start(), D2start(), main().

 * Global Variables: ShaftCountLeft, ShaftCountRight, Degrees, data_received[2], ADC_Value, sharp_front, sharp_side, distance, value_front, value_side, Left_white_line, Center_white_line,
					Right_white_line, node, count, Rflag, Lflag, false1, false2
 *
  */

// clock frequency
#define F_CPU 14745600
//header files
#include <avr/io.h>
#include <avr/interrupt.h> //included to support interrupt functions
#include <util/delay.h> // included to support _dealy_ms() function

#include <math.h> //included to support power function
#include "lcd.h" //included to support LCD functions


// global function declaration
void port_init(); 
void timer5_init();
void timer1_init();
void velocity(unsigned char, unsigned char);
void motors_delay();
unsigned int func(unsigned int);
unsigned char spi_master_tx_and_rx (unsigned char data);

//global variable declaration
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

unsigned char data_received [2];

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value,sharp_front,sharp_side,distance;
unsigned int value_front = 0;
unsigned int value_side = 0;

unsigned char Left_white_line = 0; // to keep track of left white line sensor values
unsigned char Center_white_line = 0; // to keep track of center white line sensor values
unsigned char Right_white_line = 0; // to keep track of right white line sensor values
unsigned int node = 0;
unsigned int count = 0;
unsigned int Rflag = 0;
unsigned int Lflag = 0;
unsigned int false1 = 0;
unsigned int false2 = 0;

/*
* Function Name: spi_pin_config (void)
* Input: None
* Output: None
* Logic: Configure SPI pin
* Example Call: spi_pin_config (void);

*/
void spi_pin_config (void)
{
	DDRB = DDRB | 0x07;
	PORTB = PORTB | 0x07;
}
/*
* Function Name: lcd_port_config (void)
* Input: None
* Output: None
* Logic: Initialize LCD port
* Example Call: lcd_port_config (void);

*/
void lcd_port_config (void)
{
	//all the LCD pin's direction set as output
	DDRC = DDRC | 0xF7; 
	
	// all the LCD pins are set to logic 0 except PORTC 7
	PORTC = PORTC & 0x80; 
}

/*
* Function Name: left_encoder_pin_config (void)
* Input: None
* Output: None
* Logic: Function to configure INT5 (PORTE 4) pin as input for the left position encoder
* Example Call: left_encoder_pin_config (void);

*/
void left_encoder_pin_config (void)
{
	//Set the direction of the PORTE 4 pin as input
	DDRE  = DDRE & 0xEF;  
	
	//Enable internal pull-up for PORTE 4 pin
	PORTE = PORTE | 0x10; 
}

/*
* Function Name: right_encoder_pin_config (void)
* Input: None
* Output: None
* Logic: Function to configure INT5 (PORTE 5) pin as input for the right position encoder
* Example Call: right_encoder_pin_config (void);
*/
void right_encoder_pin_config (void)
{
	 //Set the direction of the PORTE 4 pin as input
	DDRE  = DDRE & 0xDF; 
	
	//Enable internal pull-up for PORTE 4 pin
	PORTE = PORTE | 0x20; 
}

/*
* Function Name: adc_pin_config (void)
* Input: None
* Output: None
* Logic:  function for ADC pin configuration
* Example Call: adc_pin_config (void);

*/
void adc_pin_config (void)
{
	//set PORTF direction as input
	DDRF = 0x00;
	
	//set PORTF pins floating
	PORTF = 0x00;
	
	//set PORTK direction as input
	DDRK = 0x00;
	//set PORTK pins floating
	PORTK = 0x00;
}
/*
* Function Name: spi_init(void)
* Input: None 
* Output: None
* Logic: Function to Initialize spi
* Example Call: spi_init(void);

*/
void spi_init(void)
{
	SPCR = 0x53; //setup SPI
	SPSR = 0x00; //setup SPI
	SPDR = 0x00;
}

/*
* Function Name: buzzer_pin_config (void)
* Input: None
* Output: None
* Logic: Function to initialize buzzer by changing the logic at PORTC 3
* Example Call: buzzer_pin_config (void);

*/
void buzzer_pin_config (void)
{
	//Setting PORTC 3 as outpt
	DDRC = DDRC | 0x08;
	
	//Setting PORTC 3 logic low to turnoff buzzer
	PORTC = PORTC & 0xF7;		
}

/*
* Function Name: left_position_encoder_interrupt_init (void)
* Input: none
* Output: none
* Logic: Function to initialize the left position encoder interrupt by enabling the interrupt 4
* Example Call: left_position_encoder_interrupt_init (void);

*/
void left_position_encoder_interrupt_init (void) // interrupt 4 enable
{
	//Clears the global interrupt
	cli(); 
	
	// INT4 is set to trigger with falling edge
	EICRB = EICRB | 0x02; 
	
	// Enable Interrupt INT4 for left position encoder
	EIMSK = EIMSK | 0x10; 
	
	// Enables the global interrupt
	sei();   
}

/*
* Function Name: right_position_encoder_interrupt_init (void)
* Input: none
* Output: none
* Logic: Function to initialize the right position encoder interrupt by enabling the interrupt 5
* Example Call: right_position_encoder_interrupt_init (void);

*/
void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	//Clears the global interrupt
	cli(); 
	
	// INT5 is set to trigger with falling edge
	EICRB = EICRB | 0x08; 
	
	// Enable Interrupt INT5 for right position encoder
	EIMSK = EIMSK | 0x20; 
	
	// Enables the global interrupt
	sei();   
}

/*
* Function Name: ISR(INT5_vect)
* Input: none
* Output: value of right shaft count
* Logic: Interrupt Service ROutine for right position encoder. Keeps track of value of right shaft count.
* Example Call: ISR(INT5_vect);

*/
ISR(INT5_vect)
{
	//increment right shaft position count
	ShaftCountRight++;  
}

/*
* Function Name: ISR(INT4_vect)
* Input: none
* Output: Value of left shaft count
* Logic: Interrupt Service Routine for Left Position Encoder. Keeps track of value of left shaft count.
* Example Call: ISR(INT4_vect);

*/
ISR(INT4_vect)
{
	//increment left shaft position count
	ShaftCountLeft++;  
}


/*
* Function Name:  motion_pin_config (void)
* Input: none
* Output: none
* Logic: Function to configure ports to enable robot's motion
* Example Call: motion_pin_config (void);

*/
void motion_pin_config (void)
{
	//set direction of the PORTA 3 to PORTA 0 pins as output
	DDRA = DDRA | 0x0F;
	
	 // set initial value of the PORTA 3 to PORTA 0 pins to logic 0
	PORTA = PORTA & 0xF0;
	
	//Setting PL3 and PL4 pins as output for PWM generation
	DDRL = DDRL | 0x18;   
	
	//PL3 and PL4 pins are for velocity control using PWM.
	PORTL = PORTL | 0x18;
}

/*
* Function Name: buzzer_on(void)
* Input: none
* Output: turns the buzzer ON
* Logic: giving the logic high to the buzzer pin
* Example Call: buzzer_on(void);

*/
void buzzer_on (void)
{
	// dummy variable initialization to 0
	unsigned char port_restore = 0;
	
	// storing the value of PINC in port_restore
	port_restore = PINC;
	
	// changing the value of port_restore
	port_restore = port_restore | 0x08;
	
	// storing the changed value of port_restore in PORTC
	PORTC = port_restore;
}

/*
* Function Name: servo1_pin_config (void)
* Input: none
* Output: none 
* Logic: Configuring PORTB 5 pin for servo motor 1 ,(i.e.,the one used for closing the arm), operation
* Example Call: servo1_pin_config (void);

*/
void servo1_pin_config (void)
{
	 //making PORTB 5 pin output
	DDRB  = DDRB | 0x20; 
	
	//setting PORTB 5 pin to logic 1
	PORTB = PORTB | 0x20; 
}

/*
* Function Name: servo2_pin_config (void)
* Input: none
* Output: none
* Logic: Configure PORTB 6 pin for servo motor 2,(i.e, the one used for rotating the arm), operation
* Example Call:servo2_pin_config (void);

*/
void servo2_pin_config (void)
{
	 //making PORTB 6 pin output
	DDRB  = DDRB | 0x40; 
	
	//setting PORTB 6 pin to logic 1
	PORTB = PORTB | 0x40; 
}
/*
* Function Name: buzzer_off(void)
* Input: none
* Output: turns the buzzer OFF
* Logic: giving the logic low to the buzzer pin
* Example Call: buzzer_off(void);

*/
void buzzer_off (void)
{
	// dummy variable initialization to 0
	unsigned char port_restore = 0;
	
	// storing the value of PINC in port_restore
	port_restore = PINC;
	
	// changing the value of port_restore
	port_restore = port_restore & 0xF7;
	
	// storing the changed value of port_restore in PORTC
	PORTC = port_restore;
}

/*
* Function Name: Sharp_GP2D12_estimation(unsigned char adc_reading)
* Input: adc reading from Sharp Sensors
* Output: distance in mm
* Logic: This Function calculates the actual distance in millimeters(mm) from the input analog value of Sharp Sensor
* Example Call: Sharp_GP2D12_estimation(40);

*/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	//variable declaration
	float distance;
	unsigned int distanceInt;
	
	//distance calculation in mm
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	
	//conversion of value of distance obtained into integers
	distanceInt = (int)distance;
	
	//setting the maximum value of distanceInt to 800
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	
	// returning the integer value of distance calculated
	return distanceInt;
}

/*
* Function Name: servo_1(unsigned char degrees)
* Input: degrees
* Output: rotates the servo 1,,(i.e.,the one used for closing the arm), by the specified angle in multiples of 1.86 degrees 
* Logic: turns the servo motor 1 ,(i.e.,the one used for closing the arm), by a specified angle in multiples of 1.86 degrees
* Example Call: servo_1(69);

*/
void servo_1(unsigned char degrees)
{
	// initializing the PostionPanServo variable with zero 
	float PositionPanServo = 0;
	
	//calculation of value of PostionPanServo corresponding to the specified value
	PositionPanServo = ((float)degrees / 1.86) + 35.0;
	
	//initializing with 0x00
	OCR1AH = 0x00;
	
	//updating the value of OCR1AL with calculated value of PositionPanServo
	OCR1AL = (unsigned char) PositionPanServo;
}


/*
* Function Name: servo_2(unsigned char degrees)
* Input: degrees
* Output: rotates the servo 2,,(i.e.,the one used for rotating the arm), by the specified angle in multiples of 1.86 degrees
* Logic: turns the servo motor 2 ,(i.e.,the one used for rotating the arm), by a specified angle in multiples of 1.86 degrees
* Example Call: servo_2(69);

*///Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
	// initializing the PostionTiltServo variable with zero
	float PositionTiltServo = 0;
	
	//calculation of value of PostionTiltServo corresponding to the specified value
	PositionTiltServo = ((float)degrees / 1.86) + 35.0;
    
	//initializing with 0x00
	OCR1BH = 0x00;
	
	//updating the value of OCR1BL with calculated value of PositionTiltServo
	OCR1BL = (unsigned char) PositionTiltServo;
}
/*
* Function Name: servo_1_free (void)
* Input: none
* Output: none
* Logic: makes the servo 1 free rotating
* Example Call: servo_1_free (void);

*/

void servo_1_free (void)
{
	OCR1AH = 0x03;
	
	//Servo 1 off
	OCR1AL = 0xFF; 
}

/*
* Function Name: servo_2_free (void)
* Input: none
* Output: none
* Logic: makes the servo 2 free rotating
* Example Call: servo_2_free (void);

*/
void servo_2_free (void) 
{
	OCR1BH = 0x03;
	
	//Servo 2 off
	OCR1BL = 0xFF; 
}
/*
* Function Name: white_sensing()
* Input: none
* Output: displays the reading of extra white line sensors on LCD
* Logic: receives the value from extra white line sensor and storing it in data_received[]
* Example Call: white_sensing();

*/
void white_sensing()
{
	//read the value of extra white line sensor connected to channel 1 of servo pod (atmega 8)
	data_received[0] = spi_master_tx_and_rx(1);
	
	//read the value of extra white line sensor connected to channel 3 of servo pod (atmega 8)
	data_received[1] = spi_master_tx_and_rx(3);
	
	//displays the value read on LCD 
	lcd_print(2,1,data_received[0],3);
	lcd_print(2,5,data_received[1],3);
}


/*
* Function Name:port_init()
* Input: none
* Output: none
* Logic: Function to initialize ports
* Example Call: port_init();

*/
void port_init()
{

	//initialize all the functions needed and all of them are already described above
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config();
	right_encoder_pin_config();
	buzzer_pin_config();
	spi_pin_config();
	servo1_pin_config();
	servo2_pin_config();
}

/*
* Function Name:  timer5_init()
* Input: none
* Output: none
* Logic: Timer 5 initialized in PWM mode for velocity control
		Prescale:256
		PWM 8bit fast, TOP=0x00FF
		Timer Frequency:225.000Hz
* Example Call: timer5_init();
*/

void timer5_init()
{
	//Stop
	TCCR5B = 0x00;	
	
	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5H = 0xFF;	
	
	//Counter lower 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	
	
	//Output compare register high value for Left Motor
	OCR5AH = 0x00;
	
	//Output compare register low value for Left Motor
	OCR5AL = 0xFF;	
	
	//Output compare register high value for Right Motor
	OCR5BH = 0x00;	
	
	//Output compare register low value for Right Motor
	OCR5BL = 0xFF;	
	
	//Output compare register high value for Motor C1
	OCR5CH = 0x00;	
	
	//Output compare register low value for Motor C1
	OCR5CL = 0xFF;	
	TCCR5A = 0xA9;
	
	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
	TCCR5B = 0x0B;	
}

/*
* Function Name: timer1_init(void)
* Input: none
* Output: none 
* Logic: TIMER1 initialization in 10 bit fast PWM mode
		  prescale:256
		WGM: 7) PWM 10bit fast, TOP=0x03FF
		actual value: 52.25Hz
* Example Call: timer1_init(void);

*/
void timer1_init(void)
{
 //stop
 TCCR1B = 0x00; 
 
 //Counter high value to which OCR1xH value is to be compared with
 TCNT1H = 0xFC; 
 
 //Counter low value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	
 
 //Output compare Register high value for servo 1
 OCR1AH = 0x03;	
 
 //Output Compare Register low Value For servo 1
 OCR1AL = 0xFF;
 
 //Output compare Register high value for servo 2
 OCR1BH = 0x03;	
 
 //Output Compare Register low Value For servo 2
 OCR1BL = 0xFF;
 
 //Output compare Register high value for servo 3
 OCR1CH = 0x03;	
 
 //Output Compare Register low Value For servo 3
 OCR1CL = 0xFF;	
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 
 //{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
//For Overriding normal port functionality to OCRnA outputs.
//{WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1A = 0xAB;
 TCCR1C = 0x00;
 
 //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
 TCCR1B = 0x0C; 
}

/*
* Function Name: adc_init()
* Input: none
* Output: none
* Logic: function to initialize the adc pins
* Example Call: adc_init();

*/
void adc_init()
{
	ADCSRA = 0x00;
	//MUX5 = 0
	ADCSRB = 0x00;			
//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ADMUX = 0x20;		
	ACSR = 0x80;
	
	//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
	ADCSRA = 0x86;		
	
}

/*
* Function Name: ADC_Conversion(unsigned char Ch)
* Input: channel number
* Output: corresponding analog value
* Logic: This Function accepts the Channel Number and returns the corresponding Analog Value
* Example Call: ADC_Conversion(5);

*/
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	
	//Set start conversion bit
	ADCSRA = ADCSRA | 0x40;		
	
	//Wait for conversion to complete
	while((ADCSRA&0x10)==0);	
	a=ADCH;
	
	//clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRA = ADCSRA|0x10; 
	ADCSRB = 0x00;
	return a;
}

/*
* Function Name: sensor()
* Input: 
* Output: displays reading of white line sensors provided in Firebird V on LCD
* Logic: read the value at white line sensors and then convert them into corresponding analog value using ADC_conversion() function and then print on lcd.
* Example Call: sensor();

*/
void sensor()
{
	//Getting data of Left White Line Sensor
	Left_white_line = ADC_Conversion(3);	
	
	//Getting data of Center White Line Sensor
	Center_white_line = ADC_Conversion(2);	
	
	//Getting data of Right White Line Sensor
	Right_white_line = ADC_Conversion(1); 
	
	//Prints value of White Line Sensor1
	print_sensor(1,1,3);	
	
	//Prints Value of White Line Sensor2
	print_sensor(1,5,2);	
	
	//Prints Value of White Line Sensor3
	print_sensor(1,9,1);	
	

}


/*
* Function Name: print_sensor(char row, char coloumn,unsigned char channel)
* Input: row, column, value
* Output: prints the value on LCD at desired row and column location
* Logic: receives the desired row and column location from user and then prints the corresponding value at that location
* Example Call: print_sensor(1,9,1);

*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	//storing the value to be printed
	ADC_Value = ADC_Conversion(channel);
	
	//printing the value at specified row and column location
	lcd_print(row, coloumn, ADC_Value, 3);
}

/*
* Function Name: velocity (unsigned char, unsigned char)
* Input: receives two unsigned value (between 0 and 255)
* Output: speed of left and right dc motors according to given values
* Logic: receives the speed for left and right dc motors and controls the speed accordingly.
* Example Call: velocity(200,200);

*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	//controlling the speed of left motor
	OCR5AL = (unsigned char)left_motor;
	
	//controlling the speed of right motor
	OCR5BL = (unsigned char)right_motor;
}

/*
* Function Name: motion_set (unsigned char Direction)
* Input: direction in form of Hexadecimal value
* Output: moves the robot accordingly
* Logic: receives the hexadecimal value corresponding to the desired direction and then sets the motion of left and right motors accordingly.
* Example Call:

*/
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	// removing upper nibble for the protection
	Direction &= 0x0F; 		
	
	// reading the PORTA original status
	PortARestore = PORTA; 		
	
	// making lower direction nibble to 0
	PortARestore &= 0xF0; 		
	
	// adding lower nibble for forward command and restoring the PORTA status
	PortARestore |= Direction; 
	
	// executing the command
	PORTA = PortARestore; 		
	
}

/*
* Function Name: forward(void)
* Input: none
* Output: moves the robot in forward direction
* Logic: sets both the wheels of the robot to move forward
* Example Call: forward(void);

*/
void forward (void)
{
	//both wheels forward
	motion_set (0x06);
}

/*
* Function Name: back (void)
* Input: none
* Output: moves the robot in backward direction
* Logic: sets both the wheels of the robot to move backward
* Example Call: back(void);

*/
void back (void) 
{
	//both wheels backward
	motion_set(0x09);
}

/*
* Function Name: left(void)
* Input: none
* Output: moves the robot in left direction
* Logic: sets the left wheel of the robot to move backward and right wheel of the robot to move forward
* Example Call: left(void);

*/
void left (void) 
{
	//Left wheel backward, Right wheel forward
	motion_set(0x05);
}

/*
* Function Name: right(void)
* Input: none
* Output: moves the robot in right direction
* Logic: sets the right wheel of the robot to move backward and left wheel of the robot to move forward
* Example Call: right(void);

*/
void right (void) 
{
	//Left wheel forward, Right wheel backward
	motion_set(0x0A);
}

/*
* Function Name: soft_left(void)
* Input: none
* Output: moves the robot in left direction softly
* Logic: sets the left wheel of the robot to stationary and right wheel of the robot to move forward
* Example Call: soft_left (void);

*/
void soft_left (void) 
{
	//Left wheel stationary, Right wheel forward
	motion_set(0x04);
}

/*
* Function Name: soft_right(void)
* Input: none
* Output: moves the robot in right direction softly
* Logic: sets the right wheel of the robot to stationary and left wheel of the robot to move forward
* Example Call: soft_right (void);

*/
void soft_right (void)
{
	 //Left wheel forward, Right wheel is stationary
	motion_set(0x02);
}

/*
* Function Name: soft_left_2 (void)
* Input: none
* Output: moves the robot in reverse left direction softly
* Logic: sets the right wheel of the robot to stationary and left wheel of the robot to move backward
* Example Call: soft_left_2 (void);

*/
void soft_left_2 (void) 
{
	//Left wheel backward, right wheel stationary
	motion_set(0x01);
}

/*
* Function Name: soft_right_2(void)
* Input: none
* Output: moves the robot in reverse right direction softly
* Logic: sets the left wheel of the robot to stationary and right wheel of the robot to move backward
* Example Call: soft_right_2(void);

*/
void soft_right_2 (void) 
{
	//Left wheel stationary, Right wheel backward
	motion_set(0x08);
}

/*
* Function Name: back_mm(unsigned int)
* Input: distance in mm
* Output: moves the robot backwards by given distance
* Logic: receives the distance by which the robot has to move.Then the velocity and direction are given;
* Example Call: back_mm(40);

*/

void back_mm(unsigned int DistanceInMM)
{
	//giving the velocity
	velocity(200,200);
	
	//giving the direction of movement
	back();
	
	//giving the distance to move
	linear_distance_mm(DistanceInMM);
}

/*
* Function Name: left_degrees(unsigned int Degrees)
* Input: degrees
* Output: turns the robot left, accordingly
* Logic: receives the degree bye which robot has to rotate left. Then calls left() function for motion in left direction 
         and angle_rotate() function for rotating the specified degree.
* Example Call: left_degrees(90); 

*/
void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	//Turn left
	left(); 
	
	//rotate
	angle_rotate(Degrees);
}

/*
* Function Name: right_degrees(unsigned int Degrees)
* Input: degrees
* Output: turns the robot right, accordingly
* Logic: receives the degree bye which robot has to rotate right. Then calls right() function for motion in right direction 
         and angle_rotate() function for rotating the specified degree.
* Example Call: right_degrees(90); 

*/

void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	//Turn right
	right(); 
	
	//rotate
	angle_rotate(Degrees);
}

/*
* Function Name: soft_left_degrees(unsigned int Degrees)
* Input: degrees
* Output: turns the robot left, accordingly and softly
* Logic: receives the degree bye which robot has to rotate left. Then calls soft_left() function for motion in left direction softly 
         and angle_rotate() function for rotating the specified degree.
* Example Call: soft_left_degrees(90); 

*/

void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	//Turn soft left
	soft_left(); 
	Degrees=Degrees*2;
	
	//rotate
	angle_rotate(Degrees);
}

/*
* Function Name: soft_right_degrees(unsigned int Degrees)
* Input: degrees
* Output: turns the robot right, accordingly and softly
* Logic: receives the degree bye which robot has to rotate right. Then calls soft_right() function for motion in right direction softly 
         and angle_rotate() function for rotating the specified degree.
* Example Call: soft_right_degrees(90); 

*/

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	//Turn soft right
	soft_right();  
	 
	Degrees=Degrees*2;
	//rotate
	angle_rotate(Degrees);
}

/*
* Function Name: soft_left_2_degrees(unsigned int Degrees)
* Input: degrees
* Output: turns the robot in reverse left direction, accordingly and softly
* Logic: receives the degree bye which robot has to rotate in reverse left. Then calls soft_left_2() function for motion in reverse left direction softly 
         and angle_rotate() function for rotating the specified degree.
* Example Call: soft_left_2_degrees(10); 

*/

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	//Turn reverse soft left
	soft_left_2(); 
	Degrees=Degrees*2;
	
	//rotate
	angle_rotate(Degrees);
}

/*
* Function Name: soft_right_2_degrees(unsigned int Degrees)
* Input: degrees
* Output: turns the robot in reverse right direction, accordingly and softly
* Logic: receives the degree bye which robot has to rotate in reverse right. Then calls soft_right_2() function for motion in reverse right direction softly 
         and angle_rotate() function for rotating the specified degree.
* Example Call: soft_right_2_degrees(7); 

*/

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	//Turn reverse soft right
	soft_right_2();  
	Degrees=Degrees*2;
	
	//rotate
	angle_rotate(Degrees);
}

/*
* Function Name: stop(void)
* Input: none
* Output: stops the robot
* Logic: stops the left and right dc motors by providing 0x00 to the port
* Example Call: stop();

*/
void stop (void)
{
	//stop
	motion_set (0x00);
}

/*
* Function Name: angle_rotate(unsigned int Degrees)
* Input: degrees
* Output: rotates the robot accordingly
* Logic: first calculate the shaft count (ReqdShaftCount) by dividing degrees with 4.090 (resolution, i.e., for each shaft count, the robot turns 4.090 degress).
		then the robot is rotated until the shaft count becomes equal to the ReqdShaftCount. 
* Example Call: angle_rotate(24);

*/
void angle_rotate(unsigned int Degrees)
{
	//variable declaration
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	// calculation of ReqdShaftCount
	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	
	//conversion to integer form
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	
	
	ShaftCountRight = 0;
	ShaftCountLeft = 0;
 //compare ShaftCountLeft and ReqdShaftCount with ReqdShaftCountInt
	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		{
			break;
		}
		
	}
	
	//Stop robot
	stop(); 
}

/*
* Function Name: linear_distance_mm(unsigned int DistanceInMM)
* Input: distance in mm
* Output: move the robot linearly
* Logic: calculate the value of required shaft count with resolution equals to 5.338 and then move the robot linearly
* Example Call: linear_distance_mm(100)

*/
void linear_distance_mm(unsigned int DistanceInMM)
{
	//variable declaration
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	//calculation of ReqdShaftCount
	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	
	//comparision of ShaftCountLeft and ReqdShaftCountInt
 	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	//Stop robot
	stop(); 
}

/*
* Function Names: spi_master_tx_and_rx (unsigned char data)
* Input: none
* Output: none
* Logic: send byte to the slave microcontroller and get ADC channel data from the slave microcontroller
* Example Call: spi_master_tx_and_rx (2);

*/
unsigned char spi_master_tx_and_rx (unsigned char data)
{
	//variable declaration
	unsigned char rx_data = 0;

	// make SS pin low
	PORTB = PORTB & 0xFE; 
	SPDR = data;
	
	//wait for data transmission to complete
	while(!(SPSR & (1<<SPIF))); 

	//time for ADC conversion in the slave microcontroller
	_delay_ms(1); 
	
	// send dummy byte to read back data from the slave microcontroller
	SPDR = 0x50; 
	
	//wait for data reception to complete
	while(!(SPSR & (1<<SPIF))); 
	rx_data = SPDR;
	
	
	// make SS high
	PORTB = PORTB | 0x01; 
	return rx_data;
}

/*
* Function Name: forward_mm(unsigned int)
* Input: distance in mm
* Output: moves the robot forward by given distance
* Logic: receives the distance by which the robot has to move.Then the velocity and direction are given
* Example Call: forward_mm(40);

*/


void forward_mm(unsigned int DistanceInMM)
{
	
	//velocity with which robot has to move
	velocity(200,200);
	
	//direction
	forward();
	
	//distance
	linear_distance_mm(DistanceInMM);
}

/*
* Function Name: init_devices(void)
* Input: none
* Output: none
* Logic: initialiation of various other functions contained
* Example Call: init_devices(void);

*/
void init_devices (void)
{
	//Clears the global interrupts
	cli(); 
	
	//initialization
	port_init();
	spi_init();
	adc_init();
	timer1_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	
	//Enables the global interrupts
	sei();   
}

/*
* Function Name: sharp()
* Input: none
* Output:  none
* Logic: Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp" and print value of distance measured in mm by sharp sensor
* Example Call: sharp();

*/
void sharp()    
{
	//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	sharp_front = ADC_Conversion(11);						
	
	//Stores Distance calculated in a variable "value".
	value_front = Sharp_GP2D12_estimation(sharp_front);				
	
	//Prints Value Of Distance in MM measured by Sharp Sensor.
	lcd_print(2,2,value_front,3); 						
	
	//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	sharp_side = ADC_Conversion(9);						
	
	//Stores Distance calculated in a variable "value".
	value_side = Sharp_GP2D12_estimation(sharp_side);				
	
	//Prints Value Of Distance in MM measured by Sharp Sensor.
	lcd_print(2,5,value_side,3);
}

/*
* Function Name: turn()
* Input: none
* Output: none
* Logic: if all the white line sensors detect white surface , then keep turning the robot in both direction alternatively and increasing the angle of rotation gradually,
			 until any of the white line sensor detects black surface
* Example Call: turn();

*/
void turn() 
{
	unsigned int degree = 2;
	
	//loop that breaks only if any of the white line sensors detects the black surface
	while (1)
	{	

		//providing the velocity
		velocity(200,200);
		
		//turning right softly
		soft_right_degrees(degree);
		
		//stop the bot for 500 ms
		stop();
		_delay_ms(500);
		
		// white line sensors reading
		sensor();
		_delay_ms(50);
		
		//check whether any of the white line sensors detects the black surface
		if (Center_white_line > 0x20 ||Right_white_line > 0x20 || Left_white_line > 0x20)
		{
			break;
		}
		
		
		//providing the velocity
		velocity(200,200);
		
		//turning reverse left softly
		soft_left_2_degrees(degree);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		// white line sensors reading
		sensor();
		_delay_ms(50);
		
		//check whether any of the white line sensors detects the black surface
		if (Center_white_line > 0x20 ||Right_white_line > 0x20 || Left_white_line > 0x20)
		{
			break;
		}	
		
		//providing the velocity
		velocity(200,200);
		
		//turning left softly
		soft_left_degrees(degree);
		//stop for 500 ms
		stop();
		_delay_ms(500);
		//white line sensor reading
		sensor();
		_delay_ms(50);
		
		//check whether any of the white line sensors detects the black surface
		if (Center_white_line > 0x20 ||Right_white_line > 0x20 || Left_white_line > 0x20)
		{
			break;
		}

		//providing the velocity
		velocity(200,200);
		
		//turning revers right softly
		soft_right_2_degrees(degree);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//white line sensors reading
		sensor();
		_delay_ms(50);
		
		//check whether any of the white line sensors detects the black surface
		if (Center_white_line > 0x20 ||Right_white_line > 0x20 || Left_white_line > 0x20)
		{
			break;
		}
		//increasing the value of degree by 2
		degree = degree + 2;
	}
}


/*
* Function Name: pathC()
* Input: none
* Output: makes the robot to follow the black track
* Logic: take the reading from white line sensors( provided on Firebird V), and orient the robot such that the center white line sensor detects 
		black surface and move the robot keeping the center white line sensor on black track
* Example Call: pathC();

*/void pathC() // Line Follower
{
	//loop to ensure that the robot follows the black track
	while(1)
	{
		//white line sensors reading
		sensor();
		_delay_ms(20);
		
		//status of white line sensors
		if ((Center_white_line > 0x20) && (Right_white_line < 0x20) && (Left_white_line < 0x20))
		{
			//velocity of wheels
			velocity(235,230);
			
			//direction
			forward();
			_delay_ms(20);
			
		}
		//status of white line sensors
		if ((Center_white_line < 0x20) && (Left_white_line > 0x20) && (Right_white_line < 0x20))
		{
			
			//velocity of wheels
			velocity(120,200);
			
			//direction
			forward();
			_delay_ms(20);
			
		}

		//status of white line sensors
		if ((Center_white_line < 0x20) && (Right_white_line > 0x20) && (Left_white_line < 0x20))
		{
			//velocity of wheels
			velocity(200,120);
			
			//direction
			forward();
			_delay_ms(20);
		}
		//status of white line sensors
		if (Center_white_line <= 0x20 && Left_white_line <= 0x20 && Right_white_line <= 0x20)
		{
			//search for the black surface
			turn();
			_delay_ms(20);
		}
		
		//status of white line sensors
		if ((Center_white_line > 0x20 && Left_white_line > 0x20 && Right_white_line > 0x20)||(Center_white_line > 0x20 && Left_white_line > 0x20)||(Center_white_line > 0x20 && Right_white_line > 0x20))
		{
			//stop for 200 ms
			stop();
			_delay_ms(200);
			
			//velocity of wheels
			velocity(220,220);
			
			//move forward by 60 mm
			forward_mm(60);
			
			//stop for 200 ms
			stop();
			_delay_ms(200);
			
			//search for the black surface/track
			turn();
			_delay_ms(20);
			break;
		}
	}
}

/*
* Function Name: pathC1()
* Input: none
* Output: follow the black track while travering in row 1 in D1
* Logic: when the reading of all the white line sensors is white, don't  call the turn function while the robot is traversing in row 1 in D1  
* Example Call: pathC1();

*/
void pathC1()
{
	//loop for traversing the robot in row 1 of D1 
	while(1)
	{
		//white line sensors reading
		sensor();
		_delay_ms(20);
		
		//status of white line sensors
		if ((Center_white_line > 0x20) && (Right_white_line < 0x20) && (Left_white_line < 0x20))
		{
			//velocity of wheels
			velocity(235,230);
			
			//direction
			forward();
			_delay_ms(20);
			
		}

		//status of white line sensors
		if ((Center_white_line < 0x20) && (Left_white_line > 0x20) && (Right_white_line < 0x20))
		{
			
			//velocity of wheels
			velocity(120,200);
			
			//direction
			forward();
			_delay_ms(20);
			
		}

		//status of white line sensors
		if ((Center_white_line < 0x20) && (Right_white_line > 0x20) && (Left_white_line < 0x20))
		{
			
			//velocity of wheels
			velocity(200,120);
			
			//direction
			forward();
			_delay_ms(20);
		}
		
		//status of white line sensors
		if (Center_white_line <= 0x20 && Left_white_line <= 0x20 && Right_white_line <= 0x20)
		{
			//search for the black track/ surface
			turn();
			_delay_ms(20);
		}
		
		//status of white line sensors
		if ((Center_white_line > 0x20 && Left_white_line > 0x20 && Right_white_line > 0x20)||(Center_white_line > 0x20 && Left_white_line > 0x20)||(Center_white_line > 0x20 && Right_white_line > 0x20))
		{
			//stop for 200 ms
			stop();
			_delay_ms(200);
			
			//velocity of wheels
			velocity(220,220);
			
			//moving the robot in forwars direction by 60 mm
			forward_mm(60);
			
			//stop for 200 ms
			stop();
			_delay_ms(200);
			
			break;
		}
	}
}

/*
* Function Name: Rsearch()
* Input: none
* Output: checks whether block is present at nodes in right side of column C in D1
* Logic: using the value of front sharp sensor, first check the presence of block at node immediate right to column C. If block is absent, then check 
         for its presence at next right node in same row
* Example Call: Rsearch();

*/
void Rsearch() // right search of c path
{	
	//velocity of wheels
	velocity(230,210);			//starting Right
	
	//turn right by 90 degree
	right_degrees(90);
	
	//stop for 500ms
	stop();
	_delay_ms(500);
	
	//velocity of wheels
	velocity(230,210);
	
	//move backward by 20mm
	back_mm(20);
	
	//stop for 500 ms
	stop();
	_delay_ms(500);
	
	//search for black track
	turn();
	_delay_ms(20);
	
	//velocity of wheels
	velocity(230,210);
	
	//move forward by 20mm
	forward_mm(20);
	
	//stop for 500 ms
	stop();
	_delay_ms(500);
	
	//take the readings of sharp sensors
	sharp();
	_delay_ms(20);
	
	//if the value of front sharp sensor is less then 150 that means block is present and is present at immediate node after C column
	if (value_front < 150)
	{
		//buzzer ON for 500ms
		buzzer_on();
		_delay_ms(500);
		
		//buzzer off
		buzzer_off();
		
		//check whether block is properly oriented according to the given configuration
		white_right();
		_delay_ms(1000);
				
		//velocity of wheels
		velocity(230,210);
		
		//turn left by 90 degrees
		left_degrees(90);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		
		//velocity of wheels
		velocity(230,210);
		
		//move backwards by 20mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		
		//velocity of wheels
		velocity(230,210);
		
		//move forward by 40 mm
		forward_mm(40);
		_delay_ms(500);
	}
	// if there is no block at immediate node after C column
	else {
		//follow the black track
		pathC();
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//velocity of wheels
		velocity(230,210);
		
		//move forward by 20 mm
		forward_mm(20);
		_delay_ms(500);
		
		//take the reading of sharp sensors
		sharp();
		_delay_ms(20);
		
		//if the value of front sharp sensor is less than 150, block is present at second node after C column 
		if (value_front < 150)
		{
			//buzzer ON for 500 ms
			buzzer_on();
			_delay_ms(500);
			
			//buzzer off
			buzzer_off();
			
			//check whether block is properly oriented according to the given configuration
			white_right();
			_delay_ms(1000);
			
			
		}
		
		//velocity of wheels
		velocity(230,210);
		
		//turn right by 180 degrees
		right_degrees(180);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//velocity of wheels
		velocity(230,210);
		
		//move backward by 20 mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black surface
		turn();
		_delay_ms(20);
		
		//follow the black path/ track
		pathC();
		
		//velocity of wheels
		velocity(230,210);
		
		//turn right by 90 degrees
		right_degrees(90);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//velocity of wheels
		velocity(230,210);
		
		//move backwards by 20 mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//velocity of wheels
		velocity(230,210);
		
		//move forward by 40 mm
		forward_mm(40);
		
		//stop
		stop();
	}
}

/*
* Function Name: Lsearch()
* Input: none
* Output:checks whether block is present at nodes in left side of column C
* Logic: using the value of front sharp sensor, first check the presence of block at node immediate leftt to column C. If block is absent, then check
         for its presence at next leftt node in same row
* Example Call: Lsearch();

*/
void Lsearch()  // Left search of c path
{
	//velocity of wheels
	velocity(230,210);			//starting Right
	
	//turn left by 90 degrees
	left_degrees(90);
	
	//stop for 500 ms
	stop();
	_delay_ms(500);
	
	//velocity of wheels
	velocity(230,210);
	
	//move backward by 20 mm
	back_mm(20);
	
	//stop for 500 ms
	stop();
	_delay_ms(500);
	
	//search for black track
	turn();
	_delay_ms(20);
	
	//velocity of wheels
	velocity(230,210);
	
	//move forward by 20 mm
	forward_mm(20);
	
	//stop for 500 ms
	stop();
	_delay_ms(500);
	
	// take the readings of sharp sensors
	sharp();
	_delay_ms(20);
	
	//if reading of front sharp sensor is less than 150, block is present and is present at immediate node after C column
	if (value_front < 150)
	{
		//buzzer ON for 500 ms
		buzzer_on();
		_delay_ms(500);
		
		//buzzer off
		buzzer_off();
		
		//check whether block is properly oriented
		white_left();
		_delay_ms(1000);
		
		//velocity of wheels
		velocity(230,210);
		
		//turn right by 90 degrees
		right_degrees(90);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//velocity of wheels
		velocity(230,210);
		
		// move backward by 20 mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//velocity of wheels
		velocity(230,210);
		
		//move forward by 40 mm
		forward_mm(40);
		_delay_ms(500);
	}
	// if no block is present at immediate node after C column
	else {
		// follow the black track
		pathC();
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//velocity of wheels
		velocity(230,210);
	
		//move forward by 20 mm
		forward_mm(20);
		_delay_ms(500);
		
		//take the reading from sharp sensors
		sharp();
		_delay_ms(20);
		
		//if value of front sharp sensor less than 150, block is present at second node after C column
		if (value_front < 150)
		{
			//turn buzzer ON for 500 ms
			buzzer_on();
			_delay_ms(500);
			
			//turn buzzer OFF
			buzzer_off();
			
			//check the whether the block is properly oriented
			white_left();
			_delay_ms(1000);	
		}
		
		//velocity of wheels
		velocity(230,210);
		
		//turn left by 200 degree
		left_degrees(200);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//velocity of wheels
		velocity(230,210);
		
		//move backward by 20 mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//follow the black track
		pathC();
		
		//velocity of wheels
		velocity(230,210);
		
		//turn left by 100 degrees
		left_degrees(100);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//velocity of wheels
		velocity(230,210);
		
		//move backwards by 20 mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//velocity of wheels
		velocity(230,210);
		
		//move forward by 40 mm
		forward_mm(40);
		_delay_ms(500);
	}
}

/*
* Function Name: R1search()
* Input: none
* Output:checks whether block is present at nodes in right side of column C in row 1
* Logic: using the value of front sharp sensor, first check the presence of block at node immediate right to column C in row 1. If block is absent, then check
         for its presence at next right node in row 1.
* Example Call: R1search();

*/

void R1search() // right search of c path
{
	//velocity of wheels
	velocity(230,210);			//starting Right
	
	//turn right by 90 degrees
	right_degrees(90);
	
	//stop for 200 ms
	stop();
	_delay_ms(200);
	
	//velocity of wheels
	velocity(230,210);
	
	//move backward by 20 mm
	back_mm(20);
	
	//stop for 200,s
	stop();
	_delay_ms(200);
	
	
	//velocity of wheels
	velocity(230,210);
	
	//move forward by 20 mm
	forward_mm(20);
	
	//stop for 100 ms
	stop();
	_delay_ms(100);
	
	//search for black track
	turn();
	_delay_ms(20);
	
	//take the readings of sharp sensors
	sharp();
	_delay_ms(20);
	
	//if value of front sharp sensor less than 150, block is present at node B1 in D1
	if (value_front < 150)
	{
		//turn the buzzer on for 500 ms
		buzzer_on();
		_delay_ms(500);
		
		//turn the buzzer OFF
		buzzer_off();
		
		//check the orientation of block
		white_right();
		_delay_ms(1000);
		
		//velocity of wheels
		velocity(230,210);
		
		//turn left by 90 degrees
		left_degrees(90);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		

	}
	// no block found at B1 in D1
	else {
		// follow the  track
		pathC();
		
		//take the readings of sharp sensor
		sharp();
		_delay_ms(20);
		
		//if value of front sharp sensor is less than 150, block is present at A1 in D1
		if (value_front < 150)
		{
			//turn the buzzer ON for 500 ms
			buzzer_on();
			_delay_ms(500);
			
			//turn OFF the buzzer
			buzzer_off();
			
			//check the orientation of block
			white_left();
			_delay_ms(1000);
		}
		
		//velocity of wheels
		velocity(230,210);
		
		//turn right by 200 degrees
		right_degrees(200);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//velocity of wheels
		velocity(230,210);
		
		//move backward by 20 mm
		back_mm(20);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//follow the black track
		pathC();
		
		//velocity of wheels
		velocity(230,210);
		
		//turn right by 90 degrees
		right_degrees(90);
		
		//stop for 500 ms
		stop();
		_delay_ms(500);
	}
}

/*
* Function Name: L1search
* Input: none
* Output: checks whether block is present at nodes in left side of column C in row 1.
* Logic: using the value of front sharp sensor, first check the presence of block at node immediate left to column C in row 1. If block is absent, then check
         for its presence at next left node in row 1
* Example Call: L1search();

*/
void L1search()  // Left search of c path
{
	//velocity of wheels
	velocity(230,210);			//starting Right
	
	//turn left by 90 degrees
	left_degrees(90);
	
	//stop for 500 ms
	stop();
	_delay_ms(500);
	
	//velocity of wheels
	velocity(230,210);
	
	//backwards by 20 mm
	back_mm(20);
	
	//stop for 500ms
	stop();
	_delay_ms(500);
	
	//search for balck track
	turn();
	_delay_ms(20);
	
	//velocity of wheels
	velocity(230,210);
	
	//move forward by 20mm
	forward_mm(20);
	
	//stop for 100 ms
	stop();
	_delay_ms(100);

//take the readings of sharp sensors
	sharp();
	_delay_ms(20);

	//if value of front sharp sensor is less than 150, block is present at D1 in D1
	if (value_front < 150)
	{
		//buzzer ON for 500ms
		buzzer_on();
		_delay_ms(500);
		
		buzzer_off();
		
		//check the orientation of block
		white_left();
		_delay_ms(1000);
		
		velocity(230,210);
		
		//turn right by 90 degrees
		right_degrees(90);
		
		stop();
		_delay_ms(500);
		
		
		
		
	}
	else {
		//follow the black track
		pathC();
		
		//sharp sensors reading
		sharp();
		_delay_ms(20);
		
		//if value of front sharp sensor is less than 150, block is present at E1 in D1
		if (value_front < 150)
		{
			buzzer_on();
			_delay_ms(500);
			
			buzzer_off();
			
			//check the orientation of block
			white_left();
			_delay_ms(1000);
			
			
		}
		velocity(230,210);
		
		//turn left by 90 degrees
		left_degrees(200);
		
		stop();
		_delay_ms(500);
		
		
		velocity(230,210);
		
		//move backward by 20mm
		back_mm(20);
		
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(100);
		
		//follow the black track
		pathC();
		
		velocity(230,210);
		
		//turn left by 90 degrees
		left_degrees(90);
		
		stop();
		_delay_ms(500);
		
		
	}
}
/*
* Function Name: recursive()
* Input: none
* Output: keeps track of value of dummy variable count

* Logic: helps in exiting the D1 arena by checking the presence of blocks in vicinity of node E2.
* Example Call: recursive();

*/
void recursive()
{
	//follow the black track
	pathC();
	 velocity(230,210);
	right_degrees(90);
	stop();
	_delay_ms(500);
	velocity(230,210);
	back_mm(20);
	stop();
	_delay_ms(500);
	
	//search for black track
	turn();
	
	velocity(230,210);
	forward_mm(20);
	stop();
	
	//take the readings of sharp sensors 
	sharp();
	
	//if value of front sharp sensor is less than 150, block is present
	if (value_front < 150)
	{
		velocity(230,210);
		left_degrees(100);
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		velocity(230,210);
		back_mm(20);
		stop();
		_delay_ms(500);
		stop();
		
		//increase count by 1
		count+=1;
		
		//recursive call
		recursive();
	}
}

/*
* Function Name: wallfollower
* Input: none
* Output: lets the robot to follow the wall properly while entering in D2 from D1 arena.
* Logic: move straight until the value of side sharp sensor and proximity sensor 1 goes below certain value.
		if the side sharp sensor value goes below , turn the robot in left direction and if proximity sensor 1 value goes below, turn the robot in right direction.
		Do above until the value of side sensor becomes equal to 800.
* Example Call: wallfollower();

*/
void wallfollower()
{
	while(1)
	{
		
		//take the readings of sharp sensors
		sharp();
		_delay_ms(20);
		
		if (value_front < 200)
		{
			stop();
			_delay_ms(500);
			
			velocity(220,195);
			left_degrees(100);
			_delay_ms(500);
			
			break;
			
		}
		
		velocity(220,195);
		forward();
		_delay_ms(20);
		
		
		
		
		
	}
	
	while(1)
	{
		
		//reading from sharp sensors
		sharp();
		_delay_ms(20);
		
		// comparing the value of Side Sharp Sensor, mounted in right side of the robot
		if (value_side == 800)
		{
			stop();
			_delay_ms(500);
			
			velocity(220,195);
			forward_mm(60);
			_delay_ms(500);
			
			velocity(210,190);
			right_degrees(100);
			_delay_ms(500);
			
			break;
			
			
		}
		velocity(220,195);
		forward();
		_delay_ms(20);
	}
	
	
	while(1)
	{
		//take readings of white line sensors
		sensor();
		_delay_ms(20);
		
		//checking the status of white line sensors
		if ((Center_white_line > 0x20 && Right_white_line > 0x20 && Left_white_line > 0x20) || (Center_white_line > 0x20 && Left_white_line > 0x20) || (Center_white_line > 0x20 && Right_white_line > 0x20))
		{
			stop();
			_delay_ms(500);
			
			velocity(210,190);
			forward_mm(60);
			_delay_ms(500);
			
			velocity(210,190);
			left_degrees(100);
			_delay_ms(500);
			
			//search for black track
			turn();
			_delay_ms(20);			
			
			break;
		}
		
		velocity(220,195);
		forward();
		_delay_ms(20);
	}

}

/*
* Function Name: white_left();
* Input: none
* Output: tells about the orientation of blocks present at nodes in left direction of column C.
* Logic: if both left and right extra white line sensors sense white, the block is properly oriented.
		if both left and right extra white line sensors senses black, the block is improperly oriented.
* Example Call: white_left();

*/
void white_left()
{
	unsigned char i = 0;
	
		//taking the readings of extra white line sensors
		white_sensing();
		_delay_ms(20);
		
		//compare the values of extra white line sensor received to check the orientation of block
		if (data_received[0] < 100 || data_received[1] < 100) //black surface
		{
			back_mm(20);
			_delay_ms(500);
			
			//rotate servo 1 by 90 degrees
			for (i = 0; i < 90; i++)
			{
				servo_1(i);
				_delay_ms(30);
				
			}

			_delay_ms(500);
			
			//rotate servo 2 by 180 degrees
			for (i = 0; i < 180; i++)
			{
				servo_2(i);
				_delay_ms(30);
			}
			
			_delay_ms(1000);
			
			//free run the servo motors
			servo_1_free();
			servo_2_free();
			
			
			//bring the servo 1 to initial position
			servo_1(0);
			_delay_ms(500);
			
			velocity(230,210);
			back_mm(80);
			_delay_ms(500);
			
			//bring the servo 2 to initial position
			servo_2(0);
			_delay_ms(1000);
			
			//free run the servo motors
			servo_1_free();
			servo_2_free();
			
			velocity(230,210);
			forward_mm(120);
			_delay_ms(500);
			
		}
		
	
	
	
}
/*
* Function Name: white_right()
* Input: none
* Output:tells about the orientation of blocks present at nodes in right direction of column C.
* Logic: if both left and right extra white line sensors sense white, the block is improperly oriented.
         if both left and right extra white line sensors senses black, the block is properly oriented.
* Example Call: white_right();

*/
void white_right()
{
	unsigned char i = 0;
	
	//take the reading from extra white line sensors
	white_sensing();
	_delay_ms(20);
	
	//compare the values of extra white line sensor received to check the orientation of block
	if (data_received[0] > 100 || data_received[1] > 100) //black surface
	{
		back_mm(20);
		_delay_ms(500);
		
		//rotate servo 1 by 90 degrees
		for (i = 0; i < 90; i++)
		{
			servo_1(i);
			_delay_ms(30);
			
		}

		_delay_ms(500);
		
		//rotate servo 2 by 180 degrees
		for (i = 0; i < 180; i++)
		{
			servo_2(i);
			_delay_ms(30);
		}
		
		_delay_ms(1000);
		
		//free run the servos
		servo_1_free();
		servo_2_free();
		
		back_mm(40);
		_delay_ms(500);
		
		
	   //bring back the servo 1 to initial position
	    servo_1(0);
		_delay_ms(1000);
		
		velocity(230,210);
		back_mm(80);
		_delay_ms(500);
		
		//bring back the servo 2 to inital position
		servo_2(0);
		_delay_ms(1000);
		
		//free run the servos
		servo_1_free();
		servo_2_free();
		
		velocity(230,210);
		forward_mm(120);
		_delay_ms(500);
	}
	
}

/*
* Function Name: D1exit()
* Input: none
* Output: let the robot exit from D1 arena
* Logic: checks the presence of blocks at nodes in vicinity of E2 and accordingly helps the robot to exit the D1 arena.
* Example Call: D1exit();

*/
void D1exit()
{
	velocity(230,210);
	left_degrees(100);
	stop();
	_delay_ms(500);
	velocity(230,210);
	back_mm(20);
	stop();
	_delay_ms(500);
	
	//search the black track
	turn();
	velocity(230,210);
	forward_mm(20);
	stop();
	
	//take the reading of sharp sensors
	sharp();
	
	//if block is present
	if (value_front < 150)
	{
		velocity(230,210);
		left_degrees(100);
		stop();
		_delay_ms(500);
		velocity(230,210);
		back_mm(20);
		stop();
		_delay_ms(500);
		turn();
		stop();
		
		//call the recursive funtion
		recursive();
		
		//follow the black track
		pathC();
		
		//follow row 1 in D1
		pathC1();
		velocity(230,210);
		right_degrees(90);
		stop();
		_delay_ms(500);
		velocity(230,210);
		back_mm(20);
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		
		//comparing the value of dummy variable false1 with count
		for(false1=0;false1<count;false1++){
			//follow the track
			pathC();
		}
		velocity(230,210);
		left_degrees(100);
		stop();
		_delay_ms(500);
		velocity(230,210);
		back_mm(20);
		stop();
		_delay_ms(500);
		
		//search for black track
		turn();
		_delay_ms(20);
		velocity(230,210);
		forward_mm(20);
		stop();
		_delay_ms(1000);
		

		//search for black track
		turn();
		_delay_ms(500);		
	}else {
		//search for black track
		pathC();
		
		//take the reading of sharp sensors
		sharp();
		
		//if block is present
		if (value_front < 150)
		{
			velocity(230,210);
			left_degrees(100);
			stop();
			_delay_ms(500);
			velocity(230,210);
			back_mm(20);
			stop();
			_delay_ms(500);
			
			//search for black track
			turn();
			_delay_ms(20);
			
			velocity(230,210);
			forward_mm(20);
			stop();
			
			//take reading of sharp sensors
			sharp();
			
			
			//block is present
			if (value_front < 150)
			{
				velocity(230,210);
				left_degrees(100);
				stop();
				_delay_ms(500);
				velocity(230,210);
				back_mm(20);
				stop();
				_delay_ms(500);
				//search for black track
				turn();
				_delay_ms(20);
				
				stop();
				
				//follow the track
				pathC();
				velocity(230,210);
				right_degrees(90);
				stop();
				_delay_ms(500);
				velocity(230,210);
				back_mm(20);
				stop();
				_delay_ms(500);
				
				//search for black track
				turn();
				stop();
				recursive();
				
				//follow the black track
				pathC();
				
				//follow the row1 in D1 pathC1();
				velocity(230,210);
				right_degrees(90);
				stop();
				_delay_ms(500);
				velocity(230,210);
				back_mm(20);
				stop();
				_delay_ms(500);
				turn();
				stop();
				
				//compare the value of dummy variable false1 with count
				for(false1=0;false1<count;false1++){
					//follow the black track
					pathC();
				}
				velocity(230,210);
				left_degrees(100);
				stop();
				_delay_ms(500);
				velocity(230,210);
				back_mm(20);
				stop();
				_delay_ms(500);
				turn();
				_delay_ms(20);
				stop();
				_delay_ms(1000);
				
				//search for black track
				turn();
				_delay_ms(500);
			}
			//block is absent
			else{
				//follow the track
				pathC();
				 
				 velocity(230,210);
				right_degrees(90);
				stop();
				_delay_ms(500);
				velocity(230,210);
				back_mm(20);
				stop();
				_delay_ms(500);
				
				//search for black track
				turn();
				stop();
				
				//follow the black track
				pathC();
			}
		}
		else{
			
			pathC1();
			velocity(230,210);
			left_degrees(100);
			stop();
			_delay_ms(500);
			velocity(230,210);
			back_mm(20);
			stop();
			_delay_ms(500);
			
			//search the black track
			turn();
			stop();
			
			pathC();
			velocity(230,210);
			right_degrees(90);
			stop();
			_delay_ms(500);
			velocity(230,210);
			back_mm(20);
			stop();
			_delay_ms(500);
			turn();
			stop();
		}					
	}
}

/*
* Function Name: D1Start()
* Input: none
* Output: help the robot traverse in D1 arena
* Logic: follow the black track by keeping the center white line sensor on black track
* Example Call: D1Start();

*/

void D1Start()
{
	int i;
	
	while(1)
	{
		//take the reading of white line sensors
		sensor();
		_delay_ms(20);
		
		//check the status of white line sensors
		if ((Center_white_line > 0x20) && (Right_white_line < 0x20) && (Left_white_line < 0x20))
		{
			velocity(235,230);
			forward();
			_delay_ms(20);
			
		}

		//check the status of white line sensors
		if ((Center_white_line < 0x20) && (Left_white_line > 0x20) && (Right_white_line < 0x20))
		{
			
			velocity(120,200);
			forward();
			_delay_ms(20);
			
		}

		//check the status of white line sensors
		if ((Center_white_line < 0x20) && (Right_white_line > 0x20) && (Left_white_line < 0x20))
		{
			
			velocity(200,120);
			forward();
			_delay_ms(20);
		}
		
		//check the status of white line sensors
		if (Center_white_line <= 0x20 && Left_white_line <= 0x20 && Right_white_line <= 0x20)
		{
			//search the black track
			turn();
			_delay_ms(20);
		}
		
		//check the status of white line sensors
		if ((Center_white_line > 0x20 && Left_white_line > 0x20 && Right_white_line > 0x20)||(Center_white_line > 0x20 && Left_white_line > 0x20)||(Center_white_line > 0x20 && Right_white_line > 0x20))
		{
			stop();
			_delay_ms(200);
			velocity(220,220);
			forward_mm(60);
			stop();
			_delay_ms(200);
			
			turn();
			_delay_ms(20);
			break;
		}
	}
	
	//iteration for traveling in D1 arena

	for(i=1;i<=5;i++) {
		pathC();
		Rsearch();
		Lsearch();
	}
	//function calls
	pathC1();
	R1search();
	L1search();
	D1exit();
}

/*
* Function Name: D2Start()
* Input: none
* Output: help the robot traverse in D2 arena
* Logic: follow the black track by keeping the center white line sensor on black track
* Example Call: D2Start();

*/
 
void D2start()
{
	
		int i;
		velocity(210,190);
		left_degrees(200);
		Lsearch();
		//iteration for travelling in D2 arena
		for(i=1;i<=4;i++){
			pathC();
			Lsearch();
		}
		pathC1();
		velocity(210,190);
		left_degrees(90);
		_delay_ms(10);
		turn();
		_delay_ms(10);
		pathC();
		pathC();
		
		while(1)
		{
			//taking the reading of white line sensors
			sensor();
			_delay_ms(20);
			
			// check the status of white line sensors
			if ((Center_white_line > 0x20) && (Right_white_line < 0x20) && (Left_white_line < 0x20))
			{
				velocity(235,230);
				forward();
				_delay_ms(20);
				
			}

			// check the status of white line sensors
			if ((Center_white_line < 0x20) && (Left_white_line > 0x20) && (Right_white_line < 0x20))
			{
				
				velocity(120,200);
				forward();
				_delay_ms(20);
				
			}

			// check the status of white line sensors
			if ((Center_white_line < 0x20) && (Right_white_line > 0x20) && (Left_white_line < 0x20))
			{
				
				velocity(200,120);
				forward();
				_delay_ms(20);
			}
			// check the status of white line sensors
			if (Center_white_line <= 0x20 && Left_white_line <= 0x20 && Right_white_line <= 0x20)
			{
				//search for black track
				turn();
				_delay_ms(20);
			}
			
			// check the status of white line sensors
			if ((Center_white_line > 0x20 && Left_white_line > 0x20 && Right_white_line > 0x20)||(Center_white_line > 0x20 && Left_white_line > 0x20)||(Center_white_line > 0x20 && Right_white_line > 0x20))
			{
				velocity(0,0);
				stop();
				
				break;
				
			}
		}
}

/*
* Function Name: main()
* Input: none
* Output: int to inform the caller that the program exited correctly or incorrectly (C code standard)
* Logic: executes the entire program by function calls.
* Example Call: Called automatically by the Operating System

*/int main()
{
	//function calls
	init_devices();
	lcd_set_4bit();
	lcd_init();
	
	D1Start();
	wallfollower();
	D2start();
	
	//infite loop
	while(1)
	{
		buzzer_on();
		_delay_ms(500);
	}
}