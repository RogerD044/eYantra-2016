/*  
 * Team Id: 0971 
 * Author List: Soumen Das, Pooja Asati 
 * Filename: MAIN.c 
 * Theme: Transporter Bot 
 * Functions: ADC_Conversion(char),lcd_port_config(void), motion_pin_config (void),servo1_pin_config (void),servo2_pin_config (void),port_init()
			   timer1_init(void),timer5_init(),adc_init(),velocity(unsigned char, unsigned char)
			   encoder_pin_config (void),left_position_encoder_interrupt_init (void) ,right_position_encoder_interrupt_init (void),print_sensor(char ,char, char)
			   angle_rotate(unsigned int),linear_distance_mm(unsigned int) ,uart0_init(void),send()
			   forward(),back(),left (void),right (void)
			   soft_left (void),soft_right (void),soft_left_2 (void),soft_right_2 (void)
			   stop (void),init_devices (void),servo_1(unsigned char ), servo_2(unsigned char)
			   servo_1_free(),Pick(),Drop(),beep(),LINE_FOLLOW()
			   turn(int,int),turn_Fourty_Five(int,int),align(),set_to_middle(),Move(int),Traverse(int),Traverse_Last_Zones(int)
			   
 * Global Variables:  p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,count,loc,c[],deposition_complete,dir,last_flag,threshold,data,ADC_Value,
					  Left_white_line,Center_white_line,Right_white_line,ShaftCountLeft,ShaftCountRight,Degrees,Up_POSITION,Down_POSITION,
					  Down_POSITION_Drop,Grip_Angle,Loose_Angle_Drop,Loose_Angle,BATT_Voltage,Line_Align.
 */ 
#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"
#include <math.h> //included to support power function

#define SETBIT(ADDRESS, BIT)    (ADDRESS |= (1 << BIT))				// Sets the Pin at a given ADDRESS to 1
#define CLEARBIT(ADDRESS, BIT)  (ADDRESS &= ~(1 << BIT))			// Clears the Pin at a given ADDRESS to 0


int p1=1,p2=0,p3=3,p4=0,p5=5,p6=0,p7=7,p8=0,p9=9,p10=0,p11=0,p12=12;	// The Pickup Points from 1 to 12; 0 means no crate in this Numbered Point, Any other number corresponds to that crate position 
int count;																// To keep a count of the Number of Nodes it has to travel
int loc = 0;															// 0 : Normal Node;  1 : Central Node;	2 : Droping Node 
int c[13] = {99,13,99,13,99,12,99,11,99,11,99,99,13};					// Denotes the colour of the section that will face NORTH, 99: No crate in that point so no Movement; 11 : Red; 12 : Blue; 13 : Yellow; 14 : Green
int deposition_complete = 0;											// Special variable for zone 1,6 which indicates that a block has been deposited
int dir;																// Keep Track of the direction of firebird, LEFT OR RIGHT
int last_flag = 0;														
int threshold = 35;														// The input the threshold while black line following
unsigned char data;														// To store the data that needs to be send from the xbee
unsigned char ADC_Value;												// Stores the value after ADC Conversion	
unsigned char Left_white_line = 0;										// To store the Value of Left White Line Sensor
unsigned char Center_white_line = 0;									// To store the Value of Center White Line Sensor
unsigned char Right_white_line = 0;										// To store the Value of Right White Line Sensor
unsigned long int ShaftCountLeft = 0;									//to keep track of left position encoder
unsigned long int ShaftCountRight = 0;								    //to keep track of right position encoder
unsigned int Degrees; 													//to accept angle in degrees for turning
unsigned char Up_POSITION = 165;										// Servo Position to Move the Arm UP to INITIAL POSITION
unsigned char Down_POSITION = 65;										// Servo Position to Move the Arm DOWN during PICKING											
unsigned char Down_POSITION_Drop = 130;									// Servo Position to Move the Arm DOWN during DROPPING
unsigned char Grip_Angle = 155;											// Servo Position to GRIP the block
unsigned char Loose_Angle_Drop =138;									// Servo Position to LOOSEN the block while dropping
unsigned char Loose_Angle =40;											// Servo Position to completely open the GRIPPER
float BATT_Voltage;														// Stores the BATTERY VOLTAGE
int Line_Align;															// To control Line Following Motion, and move opposite to the previous motion when it looses track of line

unsigned char ADC_Conversion(unsigned char);
void Traverse(int);
void turn(int,int);
void Move(int);
void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

/*  
 * Function Name:		 lcd_port_config
 * Input:				 none
 * Output:				 No particular output; It configures the LCD PORT
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 lcd_port_config();			 
 * 
 */ 
void lcd_port_config(void)
{
	DDRC = DDRC | 0xF7;			//all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;		// all the LCD pins are set to logic 0 except PORTC 7
}


/*  
 * Function Name:		 adc_pin_config
 * Input:				 none
 * Output:				 No particular output; It configures the ADC PINS
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 adc_pin_config();			 
 * 
 */ 
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

/*  
 * Function Name:		 motion_pin_config
 * Input:				 none
 * Output:				 No particular output; Sets the Motor Pins and gets the motor ready for motion
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 motion_pin_config();			 
 * 
 */ 
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

/*  
 * Function Name:		 servo1_pin_config
 * Input:				 none
 * Output:				 No particular output; Sets the ARM Servo Pins
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 servo1_pin_config();			 
 * 
 */ 
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

/*  
 * Function Name:		 servo2_pin_config
 * Input:				 none
 * Output:				 No particular output; Sets the GRIPPER Servo Pins
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 servo2_pin_config();			 
 * 
 */ 
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

/*  
 * Function Name:		 port_init
 * Input:				 none
 * Output:				 No particular output; Sets the Ports active of the respective Function Inside
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 port_init();			 
 * 
 */
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
    servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
    servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation
}

/*  
 * Function Name:		 timer1_init
 * Input:				 none
 * Output:				 No particular output; TIMER1 initialization in 10 bit fast PWM mode
 * Logic:				 prescale:256;	WGM: 7) PWM 10bit fast, TOP=0x03FF;	actual value: 52.25Hz 
 * Example Call:		 timer1_init();			 
 * 
 */
void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


/*  
 * Function Name:		 timer5_init
 * Input:				 none
 * Output:				 No particular output; Timer 5 initialized in PWM mode for velocity control
 * Logic:				 prescale:256;	PWM 8bit fast, TOP=0x00FF;		Timer Frequency:225.000Hz
 * Example Call:		 timer5_init();			 
 * 
 */
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*  
 * Function Name:		 adc_init
 * Input:				 none
 * Output:				 No particular output; INITIALIZES ADC
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 adc_init();			 
 * 
 */
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*  
 * Function Name:		 ADC_Conversion
 * Input:				 Ch -> The channel Number whose ADC needs to be converted
 * Output:				 Returns the value after ADC conversion
 * Logic:				 Setting the Bits according to the Conversion required
 * Example Call:		 ADC_Conversion(3);			 
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
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/*  
 * Function Name:		 velocity
 * Input:				 left_motor -> Left Motor SPEED;		right_motor -> Right Motor SPEED;	
 * Output:				 Sets the speed of the motors according to the input
 * Logic:				 Setting the Bits according to PWM to be generated
 * Example Call:		 velocity(115,100);			 
 */
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*  
 * Function Name:		 encoder_pin_config
 * Input:				 none	
 * Output:				 Configures the Encoders Connected to the Motors
 * Logic:				 Setting the correponding bits HIGH
 * Example Call:		 encoder_pin_config();			 
 */
void encoder_pin_config (void)
{
	DDRE  = 0b11001111;  
	PORTE = 0b00110000;
}

/*  
 * Function Name:		 left_position_encoder_interrupt_init
 * Input:				 none	
 * Output:				 Enables the interrupt corresponding to the left motor
 * Logic:				 Clears all Global Interrupts-> Configures This Interrupt as Enable -> Enables all Global Interrupts
 * Example Call:		 left_position_encoder_interrupt_init();			 
 */
void left_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

/*  
 * Function Name:		 right_position_encoder_interrupt_init
 * Input:				 none	
 * Output:				 Enables the interrupt corresponding to the right motor
 * Logic:				 Clears all Global Interrupts-> Configures This Interrupt as Enable -> Enables all Global Interrupts
 * Example Call:		 right_position_encoder_interrupt_init();			 
 */
void right_position_encoder_interrupt_init (void) 
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

/*  
 * Function Name:		 print_sensor
 * Input:				 row-> Particular Row of LCD;	column-> Particular Column of LCD;	channel-> channel of sensor;		
 * Output:				 No such output;	Prints on the LCD the value after ADC conversion  of the channel number in specified row and column
 * Logic:				 Finds the value and the prints it
 * Example Call:		 print_sensor(1,1,1);			 
 */
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}


/*  
 * Function Name:		 angle_rotate
 * Input:				 Degrees-> The angle to be rotated in degrees 
 * Output:				 No such output;	Rotates the firebirf by the specified angle
 * Logic:				 Checks the encoder values and performs rotation as such
 * Example Call:		 angle_rotate(90);			 
 */
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
}

/*  
 * Function Name:		 linear_distance_mm
 * Input:				 DistanceInMM-> The distance in milimeters
 * Output:				 No such output;	Moves the firebirf by the specified distance
 * Logic:				 Checks the encoder values and performs movement as such
 * Example Call:		 linear_distance_mm(90);			 
 */
void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
	
	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
}

/*  
 * Function Name:		 uart0_init
 * Input:				 none
 * Output:				 No particular output; INITIALIZES UART REGISTER
 * Logic:				 No such logic; just use OR and AND to set bits
 * Example Call:		 uart0_init();			 
 * 
 */
void uart0_init(void)
{
	UCSR0B = 0x00; //disable while setting baud rate
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x5F; //set baud rate lo
	UBRR0H = 0x00; //set baud rate hi
	UCSR0B = 0x98;
}

/*  
 * Function Name:		 send
 * Input:				 data-> char which stores the data to be sent
 * Output:				 No particular output; Changes the UDR0 register with the value to be sent
 * Logic:				 No such logic; assigns the data value to the UDR0 register
						 DATA SENT ARE:
						 FOR THE BLENDER INTERFACE
						 0:RIGHT TURN	 1: LEFT TURN	 2:	MOVE FORWARD	 4: 180 DEGREE ROTATION		5: PICKIN			6: DROPPPING
						 7:	45DEG TURN	 
						 FOR ROTATING STRUCTURE
						 11: RED FACES NORTH 
						 12: BLUE FACES NORTH
						 13: YELLOW FACES NORTH
						 14: GREEN FACES NORTH
						 	
 * Example Call:		 send(5);			 
 * 
 */
unsigned send (data)
{
	UDR0=data;
}

/*  
 * Function Name:		 forward
 * Input:				 none
 * Output:				 No particular output; Sets both wheel forward
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 forward();			 
 * 
 */
void forward (void) //both wheels forward
{
  PORTA = 0x06;
}

/*  
 * Function Name:		 back
 * Input:				 none
 * Output:				 No particular output; Sets both wheel backward
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 backward();			 
 * 
 */
void back (void) //both wheels backward
{
  PORTA = 0x09;
}

/*  
 * Function Name:		 left
 * Input:				 none
 * Output:				 No particular output; Sets Left wheel backward, Right wheel forward
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 left();			 
 * 
 */
void left (void) //Left wheel backward, Right wheel forward
{
  PORTA = 0x05;
}

/*  
 * Function Name:		 right
 * Input:				 none
 * Output:				 No particular output; Sets Left wheel forward, Right wheel backward
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 right();			 
 * 
 */
void right (void) //Left wheel forward, Right wheel backward
{
  PORTA = 0x0A;
}

/*  
 * Function Name:		 soft_left
 * Input:				 none
 * Output:				 No particular output; Sets Left wheel stationary, Right wheel forward
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 soft_left();			 
 * 
 */
void soft_left (void) //Left wheel stationary, Right wheel forward
{
	PORTA = 0x04;
}

/*  
 * Function Name:		 soft_right
 * Input:				 none
 * Output:				 No particular output; Left wheel forward, Right wheel is stationary
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 soft_right();			 
 * 
 */
void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	PORTA = 0x02;
}

/*  
 * Function Name:		 soft_left_2
 * Input:				 none
 * Output:				 No particular output; Left wheel backward, right wheel stationary
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 soft_left_2();			 
 * 
 */
void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	PORTA = 0x01;
}

/*  
 * Function Name:		 soft_right_2
 * Input:				 none
 * Output:				 No particular output; Left wheel stationary, Right wheel backward
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 soft_right_2();			 
 * 
 */
void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	PORTA = 0x08;
}

/*  
 * Function Name:		 stop
 * Input:				 none
 * Output:				 No particular output; both wheels stop
 * Logic:				 No such logic; assigns particular value to PORTA 
 * Example Call:		 stop();			 
 * 
 */
void stop (void) //hard stop
{
  PORTA = 0x00;
}

/*  
 * Function Name:		 init_devices
 * Input:				 none
 * Output:				 No particular output; initializes all the Ports till know
 * Logic:				 No such logic; clears global interrupts initializes all the port and pin configurations and then enables all global interrupts 
 * Example Call:		 init_devices();			 
 * 
 */
void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	uart0_init();
	adc_init();
	timer5_init();
	timer1_init();
	encoder_pin_config ();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	
	sei();   //Enables the global interrupts
}

/*  
 * Function Name:		 servo_1
 * Input:				 degrees-> char which stores the angle to be rotated in SERVO 1
 * Output:				 No particular output; rotates the servo by the specified angle
 * Logic:				 rotates by a specified angle in the multiples of 1.86 degrees
 * Example Call:		 servo_1(20);			 
 * 
 */
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}

/*  
 * Function Name:		 servo_2
 * Input:				 degrees-> char which stores the angle to be rotated in SERVO 2
 * Output:				 No particular output; rotates the servo by the specified angle
 * Logic:				 rotates by a specified angle in the multiples of 1.86 degrees
 * Example Call:		 servo_2(20);			 
 * 
 */
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

/*  
 * Function Name:		 servo_1_free
 * Input:				 none
 * Output:				 No particular output; frees the servo
 * Logic:				 switches off servo 1
 * Example Call:		 servo_1_free();			 
 * 
 */
void servo_1_free (void) //makes servo 1 free rotating
{
	OCR1AH = 0x03;
	OCR1AL = 0xFF; //Servo 1 off
}

/*  
 * Function Name:		 Pick
 * Input:				 none
 * Output:				 No particular output; Picks the block from the pickup point
 * Logic:				 moves back 15 mm to accomodate properly
 *						 sends signal to blender interface
 *						 Arm servo comes down,	Gripper servo tightens,	Arm servo again goes up
 *						 Arm servo becomes free	
 * Example Call:		 Pick();			 
 * 
 */
void Pick()
{
	back();
	linear_distance_mm(15);
	stop();
	
	send(5);
	servo_1(Down_POSITION);
	_delay_ms(1000);
	servo_2(Grip_Angle);
	_delay_ms(1000);
	servo_1(Up_POSITION);
	_delay_ms(1000);
	servo_1_free();
}

/*  
 * Function Name:		 Drop
 * Input:				 none
 * Output:				 No particular output; Drops the block in deposition zone
 * Logic:				 Moves back 55 mm such that it does not collide with the rotating structure and then stops
 *						 sends signal to blender interface
 *						 Arm servo comes down,	Gripper servo loosens a little,	Arm servo again goes up, Gripper servo loosens completely
 *						 moves forward by 55 mm
 * Example Call:		 Drop();			 
 * 
 */
void Drop()
{
	back();
	linear_distance_mm(55);
	stop();
	
	send(6);
	servo_1(Down_POSITION_Drop);
	_delay_ms(1000);
	servo_2(Loose_Angle_Drop);
	_delay_ms(1000);
	servo_1(Up_POSITION);
	_delay_ms(1000);
	servo_2(Loose_Angle);
	_delay_ms(1000);
	
	forward();
	linear_distance_mm(55);
	
}

/*  
 * Function Name:		 beep
 * Input:				 none
 * Output:				 No particular output; gives a small beep from the buzzer
 * Logic:				 Buzzer turns on for 15 ms and then turns off
 * Example Call:		 beep();			 
 * 
 */
void beep()
{
	SETBIT(PORTC,PC3);
	_delay_ms(15);
	CLEARBIT(PORTC,PC3);

// 	PORTJ = 0xFF;
// 	_delay_ms(100);
// 	PORTJ = 0x00;
}

/*  
 * Function Name:		 LINE_FOLLOW
 * Input:				 none
 * Output:				 No particular output; Follows the Line
 * Logic:				 On the basis of the white line sensor's values it follows the line 
 * Example Call:		 LINE_FOLLOW();			 
 * 
 */
void LINE_FOLLOW()
{
		lcd_print(1,7,ShaftCountRight,3);				//Prints Right Encoder value
		lcd_print(1,3,ShaftCountLeft,3);				//Prints Right Encoder value
		print_sensor(2,2,3);							//Prints value of White Line Sensor1
		print_sensor(2,6,2);							//Prints Value of White Line Sensor2
		print_sensor(2,10,1);							//Prints Value of White Line Sensor2
		Left_white_line = ADC_Conversion(3);			//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);			//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);			//Getting data of Right WL Sensor		
		if((Center_white_line> threshold && Left_white_line> threshold) || (Center_white_line> threshold && Right_white_line> threshold) || Center_white_line> 120)		//Condition For Node Detection
		{
			if(ShaftCountRight > 20)		//Special Check to ensure that a node is not detected again
			{								//encoders value is used to check that next node is being detected a particular distance after detection of previous node
			ShaftCountRight= 0;				
			ShaftCountLeft= 0;
			forward();
			--count;
			beep();
			}
		}
		
		if((Center_white_line < threshold && Left_white_line < threshold && Right_white_line < threshold))
		{
			forward();
		}
		
		if(Center_white_line > threshold )			// If center line sensor is on black line
		{
			forward();
		}

		else if(Left_white_line> threshold)			// If left line sensor is on black line
		{
			soft_left();
			Line_Align = 0;							//if bot is turn left it uses this value to turn right when it comes on a white surface
		}

		else if((Right_white_line> threshold))		// If right line sensor is on black line
		{
			soft_right();
			Line_Align = 1;							//if bot is turn right it uses this value to turn left when it comes on a white surface
		}
		else {}
		lcd_print(1,16,count,1);
}

/*  
 * Function Name:		 turn
 * Input:				 val-> Integer denoting which turn to take,	n-> integer denoting number of lines
 * Output:				 No particular output; Turns according to the input
 * Logic:				 Sends a signal according to the variable 'var' to the blender
 *						 For val-> 0: right turn;	1: Left Turn
 *						 For n-> This integer show till how many lines will the firebird turn
 *						 if n equals 2, the firebird will turn until two black lines are being detected
 * Example Call:		 turn(0,2);			 
 * 
 */
void turn(int val,int n)
{
	send(val);			// sends the turn command to the blender
	unsigned char ch;
	if(val == 0)
	ch = 0x0A;			// assigns PORTA to turn RIGHT configuration if val equals 0
	else
	ch = 0x05;			// assigns PORTA to turn LEFT configuration if val equals 1
	int i;
	for(i=1;i<=n;i++)	//Loops till the bot turns through 'n' lines
	{	
		PORTA = ch;
		angle_rotate(35);	// turns by a small angles of 35 deg such that the center line senor comes to white surface assuming it to be on the black line previously
		while(ADC_Conversion(2) < threshold)	//turns till the center white line sensor is in white surface
		{	}
	}

	stop();
	beep();
}

/*  
 * Function Name:		 turn_Fourty_Five
 * Input:				 val-> Integer denoting which turn to take,	n-> integer denoting number of lines
 * Output:				 No particular output; Turns according to the input
 * Logic:				 Sends a signal to the blender DIFFERENT from the signal in the Function 'turn'
 *						 Sends a value 7 to indicate in the blender that a 45 deg turn is to be made
 *						 For val-> 0: right turn;	1: Left Turn
 *						 For n-> This integer show till how many lines will the firebird turn
 *						 In the main code n value is always given as 1 to ensure 45 deg turn
 *						 This extra method is used just to give a different signal to blender
 * Example Call:		 turn_Fourty_Five(0,1);			 
 * 
 */
void turn_Fourty_Five(int val,int n)
{
	send(7);
	unsigned char ch;
	if(val == 0)
	ch = 0x0A;
	else
	ch = 0x05;
	int i;
	for(i=1;i<=n;i++)
	{
		PORTA = ch;
		angle_rotate(25);
		while(ADC_Conversion(2) < threshold)
	{	}
	}

	stop();
	beep();
}

/*  
 * Function Name:		 align
 * Input:				 none
 * Output:				 No particular output; Alligns to the black line
 * Logic:				 Since the encoder readings are not fully reliable, so this align function is used after a 180 deg turn function to properly align with the line
 * Example Call:		 align();			 
 * 
 */
void align()
{
	while(ADC_Conversion(2) < threshold)  // continously rotates till the center line sensor encounters a black surface
	{	}
}

/*  
 * Function Name:		 Traverse
 * Input:				 x-> integer to denote the pick up point number
 * Output:				 No particular output; Traverses from the center to the designated pickup point and the drops it in the deposition zone and again comes back to the center
 * Logic:				 Works only if the particular pickup point value is not 0: if it would have been zero then the function would not have been executed and the bot would remain as it is.  
 *						 As the arena is divided into 6 zones with 2 points coming under 1 zone
 *						 Points 1,2 comes under zone 1, Points 3,4 comes under zone 2, and similarly points 11,12 fall under zone 6.
 *						 The value of pickup point helps in finding its particular zone and traverse accordingly 
 * 						 Calculates some information on the basis of the pickup point value 
 *						 and uses them to traverse to a pick up point and to the dropping point finally arriving at the center.
 *						 This Function is basically for PICKUP POINTS 3-10
 * Example Call:		 Traverse(p1), Traverse(1);			 
 * 
 */

/*  
 * Function Name:		 set_to_middle
 * Input:				 none
 * Output:				 No particular output; The bot goes to the center after clearing zone1 and directs to zone6 if there exists some pickup points; If zone1 does not containg blocks and zone6 does; Then this function turns the bot the zone6 direction
 * Logic:				 The deposition_complete flag shows that the bot is not at the center and it is in the bottom left deposition point;
 *						 To bring the bot to the center and get ready for the next zone this function is been written;
 *						 The bot first goes to zone1 if crates appear there and continues from there to zone6 only if crates are present in zone 6;
 *						 If there were no crates in points 11,12. this function would not have been executed.
 * Example Call:		 set_to_middle();			 
 * 
 */
void set_to_middle()
{
	if( (deposition_complete == 1) && (p11 == 11 || p12 == 12) )	// To check if the bot deposited any crate in zone 1 and if any crate in zone 6 exists.
	{
		right();
		send(4);
		angle_rotate(180);
		align();
		stop();
		
		Move(1);
		turn(0,1);
		turn_Fourty_Five(0,1);
		loc = 1;		// Approaching The Center
		Move(3);
		loc = 0;		// Resetting For Approaching Normal Node
		deposition_complete = 0;	// resetting the flag
	}
}

/*  
 * Function Name:		 Move
 * Input:				 value-> integer to denote how many nodes will the firebird move.
 * Output:				 No particular output; Just the block moves and stops after 'value' number of nodes has been detected.
 * Logic:				 Sends command to the blender interface to move forward. Calls the LINE_FOLLOW function to traverse and detect node.
 *						 Moves a particular distance afterward according to whether the last node is a NORMAL NODE, DROPPING NODE depending upon value of loc 
 * Example Call:		 Move(2);			 
 * 
 */
void Move(int value)
{
	send(2);
	count = value;
    while(count != 0)	// Follows line till the count variable is 0,or it has finished traverlling that number of nodes
    {
        LINE_FOLLOW();
    }
	
	if(loc == 0)
	{
		linear_distance_mm(45);
	}		
	else if (loc == 1)	// for Central Node sometimes it was necessary to travel a large distance to avoid getting confused with the 8 lines converging there.
	{
		forward();
		linear_distance_mm(45);	//This distance is variable and can be changed to see different results
// 		if(ADC_Conversion(2) < threshold)
// 		{
// 			if(Line_Align == 0)
// 			while (ADC_Conversion(2) < threshold)
// 			soft_right_2();
// 			else if(Line_Align == 1)
// 			while (ADC_Conversion(2) < threshold)
// 			soft_left_2();
// 		}
// 		PORTJ = 0x00;
	}		
    stop();
}

void Traverse(int x)
{
    if(x != 0)
    {
    int pp=x;
    double temp = pp/2.0;					//Divide the number by 2 to get the zone value (As there are 12 points and 6 zones, dividing by 2 gives its particular zone number)
    int zone = ceil(temp);                  // selects zone number
    int Node_pick = (zone + pp)%2;          // Stores how many nodes it has to travel to get to the pick up point after taking a turn at the 4 edges of the arena
											// To travel point 5, the Node_pick value will be 2 as it has to travel 2 Nodes from the Central Node connected the points 5,6,7,8
											// the Node_pick value 1 means that it will now travel 1 Node to get to the pickup point, value 0 or 2 means it has to travel 2 Nodes more
											//The value is Odd or even according to the sum of zone number and the pickup point number
											// The modulo of that sum gives us how many nodes the pickup point is from the central node connecting the 4 pickup points on any the 3 sides of the arena.
    dir = zone % 2;		                    //selects direction for turning :   0 for right  :   1 for left
    lcd_print(1,12,pp,1);
    lcd_print(1,13,zone,1);
    lcd_print(1,14,Node_pick,1);
    lcd_print(1,15,dir,1);
    
	if(zone == 2)							//Is the bot needs to traverse to zone 2,5, it has to take and extra turn at the center.
        turn(1,2);
    else if (zone == 5)
        turn(0,2);
    else {}

    Move(2);								// Indicates the bot will move 2 nodes
    if(Node_pick == 0)                      // for farther pick up points, Node_pick gets the value "0"
        Node_pick = 2;                      // Change Node_pick value to 2
	
	lcd_print(1,15,dir,1);
	
	turn(dir,1);
    Move(Node_pick);
    dir = dir^1;							// Toggles the direction: if dir was 0 now it will be 1-> showing that now it will take left turns instead of right
    
    lcd_print(1,15,dir,1);
    
	turn(dir,1);
    	
	send(c[x]);								//Depending on the value of the pickup point, determines which coloured section will point the NORTH direction and sends the command to the rotating structure
	Pick();									// Initializes PICKING
	
	lcd_print(1,15,dir,1);
	
	turn(dir,1);
    Move(Node_pick);
    
    lcd_print(1,15,dir,1);
    
	turn(dir,1);
    loc = 1;		// Approaching The Center
	Move(2);
    loc = 0;		// Resetting For Approaching Normal Node
	
	if(zone == 2 || zone == 5)
    {
        dir = dir^1;
        turn(dir,2);
    }
    
	loc = 2;		// Approaching The Dropping Point
    Move(2);
    loc = 0;		// Resetting For Approaching Normal Node
    
	Drop();			//Initializes DROPPING
	
    right();
	send(4);		// sends 4 for 180 degree turn
	angle_rotate(180);
	align();		// aligns with the black line
	stop();
	loc = 1;		// Approaching The Center
	Move(2);
	loc = 0;		// Resetting For Approaching Normal Node
	stop();
    }
}

/*  
 * Function Name:		 Traverse_Last_Zones
 * Input:				 x-> integer to denote the pick up point number
 * Output:				 No particular output; Traverses from the center to the designated pickup point and the drops it in the deposition zone and again comes back to the center
 * Logic:				 Works only if the particular pickup point value is not 0: if it would have been zero then the function would not have been executed and the bot would remain as it is.  
 *						 As the arena is divided into 6 zones with 2 points coming under 1 zone
 *						 Points 1,2 comes under zone 1, Points 3,4 comes under zone 2, and similarly points 11,12 fall under zone 6.
 *						 The value of pickup point helps in finding its particular zone and traverse accordingly 
 * 						 Calculates some information on the basis of the pickup point value 
 *						 and uses them to traverse to a pick up point and to the dropping point finally arriving at the center.
 *						 This Function is basically for PICKUP POINTS 1,2,11,12 which will be traversed at the end .
 *						 Due to different traversing technique for these blocks a separate function is written for the last zones.
 * Example Call:		 Traverse_Last_Zones(p11), Traverse_Last_Zones(11);			 
 * 
 */

void Traverse_Last_Zones(int x)
{
	if(x != 0)								// Function runs only if block is present,i.e: Value of x is not zero
	{
		if(deposition_complete == 1)		// for 2 blocks in one zone, this deposition_complete acts as a flag to indicate that one block is already been placed and its time to place the 2nd block;
											// A different condition is been given because the robot uses different traversal techniques for traversing 2 blocks in 1 zone. This basically shortens the path saving time.  
		{
			right();
			send(4);
			angle_rotate(180);
			align();
			stop();
			
			Move(1);
			dir = dir^1;
			turn(dir,1);
			Move(1);
			dir = dir^1;
			turn(dir,1);
			
			send(c[x]);						//Depending on the value of the pickup point, determines which coloured section will point the NORTH direction and sends the command to the rotating structure
			Pick();							// Picks up the block
		
			turn(dir,1);
			Move(1);
			turn(dir,1);
			
			loc = 2;		// Approaching The Dropping Point
			Move(1);
			loc = 0;		// Resetting For Approaching Normal Node
				
			Drop();							// Drops up the block
		}
		else
		{			
			int pp=x;
			double temp = pp/2.0;
			int zone = ceil(temp);                  // selects zone number
			int Node_pick = (zone + pp)%2;          // selects pickup point number
			dir = zone % 2;                         //selects direction for turning :   0 for right  :   1 for left
			lcd_print(1,12,pp,1);
			lcd_print(1,13,zone,1);
			lcd_print(1,14,Node_pick,1);
			lcd_print(1,15,dir,1);
			
			if (last_flag == 0)
			turn(dir,2);
			else
			turn_Fourty_Five(dir,1);			// Calls special function to turn 45 deg and send different data to the blender interface

			Move(2);
			turn(dir,1);
			
			if(Node_pick == 0)                      // for farther pick up points, Node_pick gets the value "0"
			Node_pick = 2;                      // Change Node_pick value to 2
			
			lcd_print(1,15,dir,1);
			
			
			Move(Node_pick);
			dir = dir^1;						// Toggles the direction
			
			lcd_print(1,15,dir,1);
			
			turn(dir,1);
			
			send(c[x]);							// sends data to the rotating structure
			Pick();								// Picks up the block
		
			dir = dir^1;						// Toggles the directioin
			lcd_print(1,15,dir,1);
			
			turn(dir,1);
			Move(3-Node_pick);
			
			lcd_print(1,15,dir,1);
			
			turn(dir,1);
			
			loc = 2;		// Approaching The Dropping Point
			Move(1);
			loc = 0;		// Resetting For Approaching Normal Node
			Drop();			// Picks up the block
			deposition_complete = 1;		// Shows the One block in that zone is deposited
			last_flag = 1;					
		}	
	}
}	


/*  
 * Function Name:		 main
 * Input:				 none
 * Output:				 int to inform the caller that the program exited correctly or Incorrectly
 * Logic:				 The bot traverses the arena as per problem statement.
						 All ports are initialized
						 Servo is Initialized to a Starting position
						 Delay is given for the XBEE's to connect
						 Rotates 180 as per given instruction
						 The Traverse Function is called and points are given in the order in which we want our bot to traverse the arena.
						 Depending on the value of picking points, the bot either traverses that point or it does't and moves to the next Traverse Function
						 After all Traversal is complete, the bot is stopped and the buzzer is kept in ON state 
 * Example Call:		 Called automatically by the Operating System 			 
 * 
 */
int main()
{
	
	init_devices();
	lcd_set_4bit();
	lcd_init();
	velocity(130,115);
	BATT_Voltage = ((ADC_Conversion(0)*100)*0.07902) + 0.7;	//Prints Battery Voltage Status
	lcd_print(1,1,BATT_Voltage,4);
	servo_1(Up_POSITION);
	_delay_ms(500);
	servo_2(Loose_Angle);
	_delay_ms(2000);
	_delay_ms(2000);
	_delay_ms(2000);
	_delay_ms(2000);
	send(4);
	right();
	angle_rotate(180);
	align();
	stop();
	Traverse(p5);
	Traverse(p6);
	Traverse(p7);
	Traverse(p8);
	Traverse(p3);
	Traverse(p4);
	Traverse(p9);
	Traverse(p10);
	Traverse_Last_Zones(p2);
	Traverse_Last_Zones(p1);
	set_to_middle();
	Traverse_Last_Zones(p11);
	Traverse_Last_Zones(p12);
	stop();
	SETBIT(PORTC,PC3);
	return 0;
}	