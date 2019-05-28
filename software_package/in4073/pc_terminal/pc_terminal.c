/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include<stdbool.h>
#include <string.h>
#include <inttypes.h>
//#include "temp.h"
#include "temp.c"

#define HEADER 0b10100000
#define JS_DEV	"/dev/input/js0"

uint8_t mode =0;
int panic =0;

int inp[6];
int j_inp[6];
int k_inp[6];
int button[12];

bool diff_pckType;
char pckType;

//packet from pc to drone
struct packet{
	int8_t head;
	int8_t pckType;
	int8_t roll;
	int8_t pitch;
	int8_t yaw;
	int8_t lift;
	int16_t crc;
} pc_to_drone;

//packet from drone to pc
struct pc_pckt{
	uint8_t head;
	uint8_t pcktType;
	uint8_t dt1_1;
	uint8_t dt1_2;
	uint8_t dt2_1;
	uint8_t dt2_2;
	uint8_t dt3_1;
	uint8_t dt3_2;
	uint8_t dt4_1;
	uint8_t dt4_2;
} drone_to_pc;
queue recQu;

int16_t motor[4];

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
{
	putc(c,stderr);
}

int	term_getchar_nb()
{
        static unsigned char 	line [2];

        if (read(0,line,1)) // note: destructive read
        		return (int) line[0];

        return -1;
}

int	term_getchar()
{
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */
#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include "joystick.h"

int serial_device = 0;
int fd_RS232;

void rs232_open(void)
{
  	char 		*name;
  	int 		result;
  	struct termios	tty;

       	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

	assert(fd_RS232>=0);

  	result = isatty(fd_RS232);
  	assert(result == 1);

  	name = ttyname(fd_RS232);
  	assert(name != 0);

  	result = tcgetattr(fd_RS232, &tty);
	assert(result == 0);

	tty.c_iflag = IGNBRK; /* ignore break condition */
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

	cfsetospeed(&tty, B115200);
	cfsetispeed(&tty, B115200);

	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 1; // added timeout

	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}


void 	rs232_close(void)
{
  	int 	result;

  	result = close(fd_RS232);
  	assert (result==0);
}


int	rs232_getchar_nb()
{
	int 		result;
	unsigned char 	c;

	result = read(fd_RS232, &c, 1);

	if (result == 0)
		return -1;

	else
	{
		assert(result == 1);
		return (int) c;
	}
}


int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -1)
		;
	return c;
}


int 	rs232_putchar(char c)
{
	int result;

	do {
		result = (int) write(fd_RS232, &c, 1);
	} while (result == 0);

	assert(result == 1);
	return result;
}

/*----------------------------------------------------------------
* Compute CRC value from Data
* Internet
*-----------------------------------------------------------------
*/

uint16_t compute_crc(const uint8_t *pck_data, uint32_t size, const uint16_t *pck_crc)
{
	uint32_t i = 1;
	uint16_t temp = (pck_crc == NULL) ? 0xffff : *pck_crc;

	for(i=0;i<size;i++)
	{
		temp = (unsigned char)(temp >> 8) | (temp << 8);
		temp ^= pck_data[i];
		temp ^= (unsigned char)(temp & 0xff) >> 4;
		temp ^= (temp << 8) << 4;
		temp ^= ((temp & 0xff) <<4) <<1;
	}
	return temp;
}

/*----------------------------------------------------------------
* Packet PC to Drone : 1 byte (header & Mode); 1 byte (packet type);
* 4 bytes (data); 2 bytes (CRC) = 8 bytes
* Created by Diwakar
*-----------------------------------------------------------------
*/

//Setting Header, Data and CRC (after computing)

void set_header()
{
	pc_to_drone.head = (uint8_t) HEADER;
	pc_to_drone.head = pc_to_drone.head | mode;
}

void set_data(int *data, int size)
{
	pc_to_drone.pckType = (int8_t) 0;
	pc_to_drone.roll = (int8_t) (*data/250);
	data++;
	pc_to_drone.pitch = (int8_t) (*data/250);
	data++;
	pc_to_drone.yaw = (int8_t) (*data/250);
	data++;
	pc_to_drone.lift = (int8_t) (*data/250);
}

void set_crc()
{
	uint16_t temp = NULL;
	uint8_t pck[6] = {pc_to_drone.head, pc_to_drone.pckType, 
						pc_to_drone.roll,pc_to_drone.pitch,
						pc_to_drone.yaw, pc_to_drone.lift};
	uint8_t *pck_temp = pck;
	temp = compute_crc(pck_temp, 6, NULL);
	pc_to_drone.crc = (uint16_t) temp;
}

//Creating the packet

void pck_create()
{
	set_header();
	//todo //set_data(inp,6) // special features to be added
	switch(pckType)
	{
		case 'n':
		set_data(inp,6);
		break;
		case 'u':
		dec_value(10);
		break;
		case 'j':
		inc_value(10);
		break;
		case 'i':
		dec_value(20);
		break;
		case 'k':
		inc_value(20);
		break;
		case 'o':
		dec_value(30);
		break;
		case 'l':
		inc_value(30);
		break;
		default:
		set_data(inp, 6);
		break;
	}
	set_crc();
}

//Sending the packet

void pck_send()
{
	int flag;
	do
	{
		flag = (int) write(fd_RS232, &pc_to_drone, 8);
	} while (flag==8);
	
	
}

//send panic packet
void send_panicPckt()
{
	set_header();
	pc_to_drone.roll = pc_to_drone.head;
	pc_to_drone.pitch = pc_to_drone.head;
	pc_to_drone.yaw = pc_to_drone.head;
	pc_to_drone.lift = pc_to_drone.head;
	set_crc();
	pck_send();
}

/*----------------------------------------------------------------
* Packet Drone to PC : 1 byte (header & Mode); 1 byte (packet type);
* 8 bytes (data) = 10 bytes
* Created by Diwakar
*-----------------------------------------------------------------
*/
uint8_t timestamp;
uint32_t kp1, kp2, kp;



bool checkHeader(uint8_t temp)
{
	bool flag = false;
	uint8_t check = temp;
	check = check >> 4;
	if(check == 0b00001010)
	{
		flag = true;
	}
	return flag;
}

uint8_t read_pckt()
{
	bool headercheck = false;

	do{
		drone_to_pc.head = dequeue(&recQu);
		headercheck = checkHeader(drone_to_pc.head);
	}while(!headercheck && (recQu.count >9));

	if(!headercheck || (recQu.count <9))
	{
		return mode;
	}

	drone_to_pc.pcktType = dequeue(&recQu);
	drone_to_pc.dt1_1 = dequeue(&recQu);
	drone_to_pc.dt1_2 = dequeue(&recQu);
	drone_to_pc.dt2_1 = dequeue(&recQu);
	drone_to_pc.dt2_2 = dequeue(&recQu);
	drone_to_pc.dt3_1 = dequeue(&recQu);
	drone_to_pc.dt3_2 = dequeue(&recQu);
	drone_to_pc.dt4_1 = dequeue(&recQu);
	drone_to_pc.dt4_2 = dequeue(&recQu);

	uint8_t tempMode = (uint8_t) drone_to_pc.head;
	tempMode = tempMode << 4;
	tempMode = tempMode >> 4;

	if(drone_to_pc.pcktType == 'm')
	{
		motor[0] = (int16_t)(drone_to_pc.dt1_1 << 8) | drone_to_pc.dt1_2;
		motor[1] = (int16_t)(drone_to_pc.dt2_1 << 8) | drone_to_pc.dt2_2;
		motor[2] = (int16_t)(drone_to_pc.dt3_1 << 8) | drone_to_pc.dt3_2;
		motor[3] = (int16_t)(drone_to_pc.dt4_1 << 8) | drone_to_pc.dt4_2;
		if(mode != 5)
		{
			fprintf( stderr, "Mode: %d motor[0]: %d motor[1]: %d motor[2]: %d motor[3]: %d \n", tempMode, motor[0], motor[1], motor[2], motor[3]);
		}
	}
	else if(drone_to_pc.pcktType == 'p')
	{
		mode = tempMode;
		panic = false;
		pck_create();
		pck_send();
		fprintf(stderr, "%s %d\n", " Switching Mode", mode);
	}
	else if(drone_to_pc.pcktType == 'o')
	{
		mode = tempMode;
		panic = true;
		fprintf("%s %d \n","Panic Mode");
	}
	else if(drone_to_pc.pcktType == 'c')
	{
		mode = tempMode;
		fprintf("%s %d\n","Calibration mode");
	}
	else if(drone_to_pc.pcktType == 'k')
	{
		timestamp = drone_to_pc.dt2_1;
		kp = (uint8_t) drone_to_pc.dt3_1;
		kp1 = (uint8_t) drone_to_pc.dt3_2;
		kp2 = (uint8_t) drone_to_pc.dt4_1;
		uint16_t temp_var = (uint16_t) (drone_to_pc.dt1_1 << 8) | drone_to_pc.dt1_2;
		fprintf(stderr, "MODE: %d   motor[0]: %d motor[1]: %d motor[2]: %d motor[3]: %d", tempMode, motor[0], motor[1], motor[2], motor[3]);
		fprintf(stderr, " kp: %d, kp1: %d, kp2: %d \n", kp, kp1, kp2);
	}
}

//Keyboard mapping
int key_data(char c)
{
	switch (c)
	{
		case 27:
		if(mode != 0)
		{
			mode = 1;
			panic = 1;
		}
		else
		exit(0);
		break;
		case '0':
		mode = 0;
		
		break;
		case '1':
		if(mode != 0)
		{
			mode = 1;
			panic = 1;
		}
		break;
		case '2':
		if(mode == 0)
		{
			mode = 2;
		}
		break;
		case '3':
		if(mode == 0)
		{
			mode = 3;
		}
		break;
		case '4':
		if(mode == 0)
		{
			mode == 4;
		}
		break;
		case '5':
		if(mode == 0)
		{
			mode = 5;
		}
		break;
		case '6':
		if(mode == 0)
		{
			mode = 6;
		}
		break;
		case '7':
		mode = 7;
		break;
		case 'u':
		diff_pckType = true;
		pckType = 'u';
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'j':
		diff_pckType = true;
		pckType = 'j';
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'i':
		diff_pckType = true;
		pckType = 'i';
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'k':
		diff_pckType = true;
		pckType = 'k';
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'o':
		diff_pckType = true;
		pckType = 'o';
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'l':
		diff_pckType = true;
		pckType = 'l';
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'y':
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'h':
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 't':
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'g':
		pc_to_drone.pckType = (int8_t) 0;
		diff_pckType = false;
		break;
		case 'A':
		k_inp[1] -= 300;
		break;
		case 'B':
		k_inp[1] += 300;
		break;
		case 'C':
		k_inp[0] -= 300;
		break;
		case 'D':
		k_inp[0] += 300;
		break;
		case 'a':
		k_inp[3] -= 300;
		break;
		case 'z':
		k_inp[3] += 300;
		break;
		case 'q':
		k_inp[2] -= 300;
		break;
		case 'w':
		k_inp[2] += 300;
		break;
	}
}

void dec_value(int8_t v)
{
	set_header();
	pc_to_drone.pckType = (int8_t) v;
	pc_to_drone.pitch = (int8_t) 0;
	pc_to_drone.roll = (int8_t) 0;
	pc_to_drone.yaw = (int8_t) 0;
	pc_to_drone.lift = (int8_t) 0;
	set_crc();
}

void inc_value(int8_t v)
{
	set_header();
	pc_to_drone.pckType = (int8_t) v;
	pc_to_drone.pitch = (int8_t) 64;
	pc_to_drone.roll = (int8_t) 64;
	pc_to_drone.yaw = (int8_t) 64;
	pc_to_drone.lift = (int8_t) 64;
	set_crc();
}

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	int	c;
	#ifdef JOYSTICK_CONNECTED
	struct js_event jinp;

	if(open(JS_DEV, O_RDONLY) < 0)
	{
		perror("joysticktest");
		exit(1);
	}
#endif
	pckType = 'n';

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	fcntl(c, F_SETFL, O_RDONLY);

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	init_queue(&recQu);
	int count =0;
	/* send & receive
	 */
	for (;;)
	{
		// if ((c = term_getchar_nb()) != -1)
		// 	rs232_putchar(c);

		// if ((c = rs232_getchar_nb()) != -1)
		// 	term_putchar(c);
	rs232_getchar_nb();
		if(panic)
		{
			send_panicPckt();
			//panic packet create
		}
		else if(count > 50)
		{
			count =0;
			pck_create();
			pck_send();
			pckType = 'n';
		}
		count++;

		//input from Keyboard
		char keyinp = term_getchar_nb();
		char nxtinp;
		char arrowinp;
		if(keyinp == 27)
		{
			if(nxtinp == term_getchar_nb() != -1)
			{
				arrowinp = term_getchar_nb();
				//keyboard mapping
				key_data(arrowinp);
			}
			else
			{
				//keyboard mapping else
				key_data(27);
			}
			
		}
		else
		{
			//keyboardmapping - keyinp
			key_data(keyinp);
		}
		
		//input from joystick
		#ifdef JOYSTICK_CONNECTED
		while(read(c, &jinp, sizeof(struct js_event)) == sizeof(struct js_event))
		{
			fprintf(stderr, ".");
			switch(jinp.type & ~ JS_EVENT_INIT)
			{
				case JS_EVENT_BUTTON:
				button[jinp.number] = jinp.value;
				break;
				case JS_EVENT_AXIS:
				j_inp[jinp.number] = jinp.value;
				break;
			}
		}
#endif
		//key input + js input
		for(int i=0;i<4;i++)
		{
			if(j_inp[i] + k_inp[i] > 32767)
			{
				inp[i] = 32767;
			}
			else if(j_inp[i] + k_inp[i] < -32767)
			{
				inp[i] = -32767;
			}
			else
			{
				inp[i] = j_inp[i] + k_inp[i];
			}
		}
#ifdef JOYSTICK_DEBUG
printf("\n");
for(int i=0;i<6;i++)
{
	printf("%6d", j_inp[i]);
}
fprintf(" | ");
for(int i=0;i<12;i++)
{
	printf("%d", button[i]);
}
#endif
		//fire button
		if(button[0])
		{
			if(mode != 0) { mode =1; panic =1;}
			else break;
		}



	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}

