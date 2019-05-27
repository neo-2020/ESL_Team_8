/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include <stdio.h>
#include <string.h>
/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c)
{
	switch (c)
	{
		case 'q':
			ae[0] += 10;
			break;
		case 'a':
			ae[0] -= 10;
			if (ae[0] < 0) ae[0] = 0;
			break;
		case 'w':
			ae[1] += 10;
			break;
		case 's':
			ae[1] -= 10;
			if (ae[1] < 0) ae[1] = 0;
			break;
		case 'e':
			ae[2] += 10;
			break;
		case 'd':
			ae[2] -= 10;
			if (ae[2] < 0) ae[2] = 0;
			break;
		case 'r':
			ae[3] += 10;
			break;
		case 'f':
			ae[3] -= 10;
			if (ae[3] < 0) ae[3] = 0;
			break;
		case 27:
			demo_done = true;
			break;
		default:
			nrf_gpio_pin_toggle(RED);
	}
}
/*
void printInputValues(void)
{
	printf("%10ld | ", get_time_us());
	printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
	printf("%6d %6d %6d | ", phi, theta, psi);
	printf("%6d %6d %6d | ", sp, sq, sr);
	printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);
}
*/

 
/*------------------------------------------------------------------
 * main -- everything you need is here :)
 * 
 * Modify by Yuhao, some part refer to last yearś code
 * date 11/5/2019
 *------------------------------------------------------------------
 */
int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);	
	baro_init();
	spi_flash_init();
	//ble_init();

	uint32_t time_diff;
	uint32_t counter = 0;
	uint32_t no_packet = 0;
	
	demo_done = false;

	switch_mode(0);

	//pckType = 'v';

	//flushQueue(&rx_queue);
	//flushQueue(&tx_queue);

	timestamp = get_time_us();
	uint32_t start_time_msg = get_time_us();

	while (!demo_done)
	{
		if (rx_queue.count>7){
			if (pre_mode != read_packet()){
				switch_mode(mode);
			}
			no_packet =0;
		}
		else
		{
			no_packet ++;
		}

		if (no_packet ==250)
		{
			if(mode!=0)
			{
				lost_connectFlag = 1;
				mode = 1;
				p_Flag = 1;
			}
		}
		/*		
		#ifdef BATTERYCHECK
		batteryMonitor();
		if(!batteryFlag)
		{
			if(mode!=0) {printf("\nLow Battery! Panic Mode\n"); mode=1; panicFlag=1;}
			else {printf("\nLow Battery! Aborting ...\n"); demo_done=true;}
		}
		#endif
		*/
		if (p_Flag)
		{
			panic_mode();
		}
		mode_function();
		
		//Copy from last yearś code
		if (check_timer_flag()) 
		{
			if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

			adc_request_sample();
			read_baro();
						
			//logData();
			//printInputValues();

			clear_timer_flag();
		}
		
		//Writen by Yuhao Xuan
		//date 11/5/2019
		if (check_sensor_int_flag() && !rawFlag) 
		{	
			get_dmp_data();
		} else {
			//imu_init(false, 256);
		}

		
		time_diff = get_time_us() - start_time_msg;

		if (time_diff>600)
		{
			send_packet(pckType);
			start_time_msg = get_time_us();
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
