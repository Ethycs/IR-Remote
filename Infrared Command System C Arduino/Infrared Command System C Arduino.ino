/*
  Name:    Infrared_Command_System_C_Arduino.ino
  Created: 3/25/2019 12:55:29 PM
  Author:  Jasper H. Yao

  This code was written for the Husky Satellite Lab at The University of Washington.
  It's purpose is to control a rendezvous intercept clock over infrared

  Licensed Under GNU LGPL 1.21

 Summary of operation:
  This code is used to control transmission of infrared signals for an unknown model of countdown timer.
  A sequence of key_btn[16] is ordered by a command sequence of arbitrary length
  and in turn orders key pulses to an unsigned integer array.
  If key_btn[i] = 1, input the key pulses A[0], and A[1] in order,
	 key_btn[i] = 0, input the key pulses B[0], and B[1] in order
  At the end of each key sequence add C[0] to the raw sequence
  e.g. bool key0[16] =   { 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0 }
	-> Turns into  {600,1700,600,1700,600,1700, 600,600...}
  keymap stores an array of pointers to arrays in a convenient, well ordered way,
  refer to enum 'command'.
  Pulse parity is +-+-+-....+-, each key is given by {+x_1,-x_2},
  a pause C[0]needs to be inserted at the end of the key sequence.
  This would lead to a raw interpretation of {+x_1,-C[0]}

  For some sequence of keys, read each key in order.
  Then create a key pulse sequence depending on the sequence of each key.
  Finally put it all into a big array of unsigned integers.
  Sequence burst: 16 * 2 key pulses + 1 pause = 33 pulses
		  {shortp, longp}      +  pause
  Raw conversion length = 13 * (16 * (2 key pulses || x[1] + 1 pause)) = 416
  Raw conversion length = seq length * (key length * pulse length )
*/

/*Includes Infrared Remote Library: IRremote, by Ken Shirriff
  https://www.pjrc.com/teensy/td_libs_IRremote.html
  https://github.com/z3t0/Arduino-IRremote
  Licensed Under GNU LGPL
*/
#include <IRremote.h>

/*IRsend Object*/
IRsend irsend;

#define default_freq 38

/*
  Pin 13 LED Blink
  Toggles pin 13 HIGH, for time (milliseconds)
  Then toggles off.
*/
void basic_blink(int time) {
	digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(time);                       // wait for a second
	digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}

/* Define Key Pulses*/
#define key_pulse_length 2
//These values have been nicely rounded, but may need to be changed.
struct key_pulse {
	int shortp = 256, longp = 1835, pause = 20000,
		shortk = 810;
}pulse;


/*
  Encode Sequence Data
  The sequence list of key buttons sampled from the remote.
  Remark: I could use Gauss Jordan-elimination to find the span of this matrix
  and extract coordinate vectors?
  To save memory, first 8 bits are identical for all key
*/
#define  keylength 16 //Key Length
const bool key_power[16] =      { 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0 };
const bool key_reset[16] =      { 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0 };
const bool key_ok[16] =         { 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0 };
const bool key_leftarrow[16] =  { 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0, 0 };
const bool key_rightarrow[16] = { 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0 };
const bool key_down[16] =       { 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 0 };
const bool key_exit[16] =       { 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0 };
const bool key_set[16] =        { 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 };
const bool key0[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0 };
const bool key1[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const bool key2[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0 };
const bool key3[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
const bool key4[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0 };
const bool key5[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0 };
const bool key6[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0 };
const bool key7[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0 };
const bool key9[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0 };
const bool key8[16] =           { 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 0, 0 };
//Turns into  {600,1700,600,1700,600,1700, 600,600...}

/*Unused but included for completeness*/
//bool command_space[16] = {1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 0};
//bool command_dot[16] = {1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 0};
//bool command_brightup[16] = {1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0};
//bool commandn_brightdown[16] = {1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0};



/*
  Maps key names onto keymap array
  These names will come in handing to remember how to make control sequences
*/
enum command
{
	power = 11,
	reset = 12,
	ok = 13,
	left_arrow = 14,
	right_arrow = 15,
	down = 16,
	exitk = 17,
	set = 18
};




/*
   keymap[i][j]
   i - the index of the pointer to a boolean key array.
   j - the bit value of the ith key at the jth position.
*/
const bool *keymap[18] =
{         //INDEX  ASCII
  key0,                //0  0
  key1,        //1  1
  key2,        //2  2
  key3,        //3  3
  key4,        //4  4
  key5,        //5  5
  key6,        //6  6
  key7,        //7  7
  key8,        //8  8
  key9,        //9  9
  key_power,       //10 :
  key_reset,       //11 ;
  key_ok,        //12 <
  key_leftarrow,     //13 =
  key_rightarrow,    //14 >
  key_down,      //15 ?
  key_exit,      //16 @
  key_set        //17 A
};




// Provide a handy data format for when we take date input
struct date
{      //# Range
	int d_1; //1 [0, 9]
	int d_2; //2 [0, 9]
	int d_3; //3 [0, 9]
	int h_1; //4 [0, 2]
	int h_2; //5 [0, 9]
	int m_1; //6 [0, 6]
	int m_2; //7 [0, 9]
	int s_1; //8 [0, 5]
	int s_2; //9 [0, 9]
	int length = 9;
} input;





/*Control Sequence List*/

/*
  Control Sequence: Initiate Countdown;
  The order of commands
  DOWN, SET, XXX[0,999]XX[0-24]XX[0,59]XX[0-60] (9 Digits), SET, OK (Starts)
 (We can cut this) */
char seq_time[13] = {
  down,   //DOWN
  set,    //SET
  input.d_1,  //X[0-9]
  input.d_2,  //X[0-9]
  input.d_3,  //X[0-9]
  input.h_1,  //X[0-2]
  input.h_2,  //X[0-9]
  input.m_1,  //X[0-6]
  input.m_2,  //X[0-9]
  input.s_1,  //X[0-6]
  input.s_2,  //X[0-9]
  set,    //SET
  ok      //OK
};




/*
  Create IR Signal Sequence

  raw[] is the transmission
  raw_length is the length of raw[]

  sequence[] is the control sequence
  seq_length is the length of sequence[]
*/
unsigned int* make_raw(unsigned int raw[], int seq_length, char sequence[])
{
	int k = 0;
	for (int j = 0; j < seq_length; j++) {
		int seq_index = sequence[j];
		for (int i = 0; i < keylength; i++) {
			//      if *keymap[
			Serial.print(keymap[seq_index][i]); //Print out the generated sequence
			Serial.write("\n");
			//Populate the raw 
			raw[k] = pulse.shortp; //Assigns 1st pulse
			if (i == keylength - 1) // Add pause after each command 
				raw[k + 1] = pulse.pause;
			else {//Add the short pulse for the 1 bit
				if (keymap[sequence[j]][i]) {
					raw[k + 1] = pulse.longp;//*
				}
				else {//Add 0 pulses to buffer
					raw[k + 1] = pulse.shortk;
					Serial.write('@');
				}
			}
			k += 2;

		}

	}
	return raw;
}


//Send a single command
void single_command(char key)
{
	const int n = 32;
	unsigned int raw[n];
	*raw = *make_raw(raw, 1, &key); //I can put in looping here for rebroadcast of sequences.
	irsend.sendRaw(raw, n, default_freq);
}


//SETTIME:045;02;23;12
void set_time(struct date input) //Label my argument as struct date input in arduino IDE
{
	seq_time[2] = input.d_1; //Set Days
	seq_time[3] = input.d_2;
	seq_time[4] = input.d_3;
	seq_time[5] = input.h_1; //Set Hours
	seq_time[6] = input.h_2;
	seq_time[7] = input.m_1; //Set Minutes
	seq_time[8] = input.m_2;
	seq_time[9] = input.s_1; //Set Seconds
	seq_time[10] = input.s_2;
	const int n = 416; //Buffer length
	unsigned int raw[416];
	*raw = *make_raw(raw, 13, seq_time);
	irsend.sendRaw(raw, n, default_freq);

}


//Debug Stub
void writeString(String stringData) { // Used to serially push out a String with Serial.write()

	for (int i = 0; i < stringData.length(); i++)
	{
		Serial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
	}
	Serial.write("\n");

}// end writeString

/*
  Command interpreter checks if the serial buffer contains the appropriate
  command sequence. No malformed input is assumed but
  some integrity checks on the sequence to try to identify if it contains
  a command are performed anyways.

*/
void command_interpreter(char buffer[]) {
	String command_x = Serial.readStringUntil(':');
	command_x.trim();
	//DEBUG Print interpreted command
	//writeString(command_x);
	size_t  com_data_t = Serial.readBytesUntil('}', buffer, Serial.available());
	int com_data_length = com_data_t / sizeof(buffer[0]); //Conversion of size_t to int
	//Serial.print(com_data_length, DEC);
	//DEBUG Print buffer back
	//Serial.write(buffer, com_data_length);
	//If no chars were moved, print a message and return to main loop
	if (com_data_length < 2) {
		Serial.println("Command Argument Empty, Aborted");
		Serial.flush();
		return;
	}
	else {//Identify the command type
		if (buffer[0] == '{') {
			//Handle the single command instance (series transmission)
			if (command_x.equalsIgnoreCase("COMMAND"))
			{
				//key1,key2,key3,key4, s
				//-> 1,2,p,s,d,u 
				//-> reads the string 
				//-> extracts command characters 
				//-> transmits a sequence of commands
				/*  COMMAND:{1,2,p,1,l,r,u}
				-> Commands sent by single command
				-> read from a buffer
				-> parsed by comma
				-> build command n
				-> send command n
				->
				*/
				//Individual sequences should be even
				//odd is data
				for (int i = 1; i < com_data_length; i++)
				{
					//Take every odd buffer 
					//Convert ASCII byte to integer byte with " - '0'"
					if (buffer[i] - '0' <= 17 && i % 2 == 1)
					{
						single_command(buffer[i] - '0');
					}
					else //if the element is also not a ',' a
						if (buffer[i] != ',')
						{
							Serial.println("Invalid Command Format, inappropriate commas"); //Break?
						}
				}

			}
			else// Handle the settime command
				if (command_x.equalsIgnoreCase("SETTIME"))
				{
					if (com_data_length = 13) {
						input.d_1 = buffer[1];
						input.d_2 = buffer[2];
						input.d_3 = buffer[3];
						input.h_1 = buffer[5];
						input.h_2 = buffer[6];
						input.m_1 = buffer[8];
						input.m_2 = buffer[9];
						input.s_1 = buffer[11];
						input.s_2 = buffer[12];
						set_time(input);
					}
				}
				else {
					Serial.println("Invalid Command");
					return;
				}
		}
		else
			Serial.println("Invalid Command Sequence, format is command:{argument}");
	}

}


/*      Arduino template code    */

// put your setup code here, to run once:
void setup() {

	/* Encode Sequence Data*/
	Serial.begin(115200);
	while (!Serial) {
		basic_blink(500);
	}
	Serial.println("IR-Remote Connected");
}

//Command buffer for serial input
char buffer[64];


void loop() {//Note Serial buffer on the arduino is only 64 (63?) bytes long, a 256 char buffer is excessive. 
  //Check if the serial has a command waiting that is valid
	if (1 < Serial.available() && Serial.available() <= 64) {
		command_interpreter(buffer);
	}
	else
		if (Serial.available() > 64)
			Serial.println("Input longer than 64 chars, Ignoring!");

	basic_blink(1000);

}