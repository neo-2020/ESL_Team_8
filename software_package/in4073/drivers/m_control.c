/**
* Written by Yuhao Xuan
* Date: 7/5/2019
* Mode control and the pointer the the right function is here
*
**/
#include "in4073.h"
// Function for safe mode (mode0)
void safe_mode(void){
    p_Flag = 0;
    ae[0] = 0;
    ae[1] = 0;
    ae[2] = 0;
    ae[3] = 0;
    update_motors();
}


// Function for panic mode (mode1)
void panic_mode(void){
    ae[0] = 150;
    ae[1] = 150;
    ae[2] = 150;
    ae[3] = 150;
    update_motors();
    //Time Delay
    for(int i=0;i<=400;i++) send_packet('o');

    //b=b+b;
    p_Flag = false;

    if(!lost_connectFlag){
        //TODO Function name not certain
        send_mode_change();
    }
    else
    {
        switch_mode(0);
    }
    
    safe_mode();
    
}


// Function for manual mode (mode2)
void manual_mode(void){
    //TODO Function name not certain
    manualModePacket();
	calculateMotorRPM();
	update_motors();
}


// Function for calibration mode (mode3)
void cali_mode(void){
    //TODO Function name not certain
    mode = 0;
    mode_change_acknowledged = false;
    send_mode_change();
}


// Function for yaw control mode (mode4)
void yaw_mode(void){
    	if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
    //TODO Function name not certain
	yaw_cal();
	calculateMotorRPM();
	update_motors();
}


// Function for full control mode (mode5)
void full_mode(void){
    	if (check_sensor_int_flag()) 
		{
			get_dmp_data();
		}
    //TODO Function name not certain
	roll_cal();
	calculateMotorRPM();
	update_motors();
}


// Function for raw mode (mode 6)
void raw_mode(void){

	get_raw_sensor_data();
	filter_function();

	raw_ctr();
	calculateMotorRPM();
	update_motors();
}


// Function for height control (mode 7)
void height_mode(void){
	read_baro();
	height_ctr();
	calculateMotorRPM();
	update_motors();
}

//State Machine Function
void switch_mode(int mode_input){
    switch(mode_input)
    {
        case 0:
			type_pck = 'm';
			mode_function = &safe_mode;
			break;
		case 1:
			type_pck = 'o';
			rawFlag=0;		
			prevAcknowledgeMode = 1;
			mode_function = &panic_mode;
			break;
		case 2:
			type_pck = 'm';
			mode_function = &manual_mode;
			break;
		case 3:
			type_pck = 'c';
			rawFlag=0;				
			prevAcknowledgeMode = 3;
			//buffer_fill_index = 0;
			mode_function = &cali_mode;
			break;
		case 4:
			type_pck = 'm';
			rawFlag=0;		
			mode_function = &yaw_mode;
			break;

		case 5:
			type_pck = 'k';
			timestamp = 0;
			rawFlag=0;		

			mode_function = &full_mode;
			break;
		
		case 6:
			rawFlag=0;

			rawFlag=1;
			mode_function = &raw_mode;
			break;

		case 7:

			rawFlag=0;
			mode_function = &height_mode;
    }

	pre_mode = mode_input;
}