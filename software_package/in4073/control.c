/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>


#define shift_lift 1
//p
#define RATE_SHIFT_YAW 0
#define RATE_GAIN_SHIFT_YAW 0

//p1
#define shift_ang 0
#define shift_ang_gain 0

//p2
#define shift_rate 0
#define shift_rate_gain 0

//height_control
#define shift_rate_hgt 0
#define shift_rate_gain_hgt 0


//SPEED CONTROL
#define MAX_SPEED 450

void update_motors(void)
{	
    for(int i=0; i<4; i++)
	{
		if(ae[i]>MAX_SPEED)
		{
			motor[i] = MAX_SPEED;
		}
		else
		{
			motor[i] = ae[i];
		}		
	}	
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters

	// ae[0] = xxx, ae[1] = yyy etc etc
	update_motors();
}



/*#####################  Yaw Control  ###################*/
//Written by Yuhao

void yaw_cal(){

	if(kp<1) kp=1;
	lift = (int32_t)-1*(pc_drone.yaw*256);
	roll = (int32_t)pc_drone.roll*256;
	pitch = (int32_t)pc_drone.pitch*256;

	err_yaw = (((int32_t)pc_drone.yaw*256)>>RATE_SHIFT_YAW)+((sr-cali_r)>>RATE_SHIFT_YAW);
	yaw = (kp*err_yaw)>>RATE_GAIN_SHIFT_YAW;
}




/*#################### Full Control ####################*/
//Written by Yuhao
void roll_cal(){
	lift = (((int32_t)-1 * (pc_drone.lift -127)*256)>>shift_lift);

	err_roll = ((((int32_t)pc_drone.roll*256)/4)-((phi-cali_phi)>>shift_ang));
	roll = ((kp1*err_roll)>>shift_ang_gain)-(kp2*((sq-cali_q)>>shift_rate)>>shift_rate_gain);

	err_pitch = ((((int32_t)pc_drone.pitch*256)/4)-((theta-cali_theta)>>shift_ang));
	pitch = ((kp1*err_pitch)>>shift_ang_gain) + (kp2*((sq-cali_q)>>shift_rate)>>shift_rate_gain);

	err_yaw = (((int32_t)pc_drone.yaw*256)/4)-((sr-cali_r)>>RATE_SHIFT_YAW);
	yaw = (kp*err_yaw)>>RATE_GAIN_SHIFT_YAW;
}



/*#################### Raw Control #####################*/
//Written by Yuhao
void raw_ctr(){
	lift = (int32_t)-1*(pc_drone.lift-127)*256;

	err_roll = ((((int32_t)pc_drone.roll*256)/4)-((phi_est-cali_phi)>>shift_ang));
	roll = ((err_roll*kp1)>>shift_ang_gain)-((((p_est-cali_p)>>shift_rate)*kp2)>>shift_rate_gain);

	err_pitch = ((((int32_t)pc_drone.pitch*256)/4)-((theta_est-cali_theta)>>shift_ang));
	pitch = ((err_pitch*kp1)>>shift_ang_gain)-((((q_est-cali_q)>>shift_rate)*kp2)>>shift_rate_gain);

	err_yaw = (((int32_t)pc_drone.yaw*256)/4)-((r_butter-cali_r)>>RATE_SHIFT_YAW);
	yaw = ((err_yaw*kp)>>RATE_GAIN_SHIFT_YAW);
}


/*#################### Height Control ##################*/
//Written by Yuhao
void height_ctr(){

	err_lift = (((int32_t)-1*(pc_drone.lift-127)*256)>>shift_rate_hgt)-((pressure-cali_pressure)>>shift_rate_hgt);
	lift = kp*err_lift>>shift_rate_gain_hgt;
	roll = (int32_t)pc_drone.roll*256;
	pitch = (int32_t)pc_drone.pitch*256;
	yaw = (int32_t)pc_drone.yaw*256;
	}


// written by Sushant
// BUFFER_SIZE size can be more. 600(say).


void calibration(void)
{
    uint8_t i;
    
    static int16_t c_phi[BUFFER_SIZE], 
    c_theta[BUFFER_SIZE], c_sax[BUFFER_SIZE], 
    c_say[BUFFER_SIZE], c_sp[BUFFER_SIZE],
    c_sq[BUFFER_SIZE], c_sr[BUFFER_SIZE];
	
	int32_t sum_phi = 0, sum_theta = 0, sum_sax = 0, sum_say = 0, sum_sp = 0, sum_sq = 0, sum_sr = 0;

	// collecting the values by shifting them by one. 
    for (i = 0; i < BUFFER_SIZE - 1; i++)
    {
        c_phi[i] = c_phi[i+1];
        c_theta[i] = c_theta[i+1];
        c_sax[i] = c_sax[i+1];
        c_say[i] = c_say[i+1];
        c_sp[i] = c_sp[i+1];
        c_sq[i] = c_sq[i+1];
        c_sr[i] = c_sr[i+1];
		
	}

// pushing new value
c_phi[BUFFER_SIZE-1] = phi; 
c_theta[BUFFER_SIZE-1] = theta;
c_sax[BUFFER_SIZE-1] = sax;
c_say[BUFFER_SIZE-1] = say;
c_sp[BUFFER_SIZE-1] = sp;
c_sq[BUFFER_SIZE-1] = sq;
c_sr[BUFFER_SIZE-1] = sr;

// summing the input data
for (i = 0; i < BUFFER_SIZE; i++)
    {
        sum_phi += c_phi[i]; sum_theta += c_theta[i]; sum_sax += c_sax[i]; 
        sum_say += c_say[i]; sum_sp += c_sp[i]; sum_sq += c_sq[i]; sum_sr += c_sr[i];

    }


/*################### Rotor Control #####################
Written by Yuhao*/
void calculateMoterRPM(){
	int32_t v0,v1,v2,v3;

	int32_t a = 1;
	int32_t b = 1;

	int mul_fac=6;
	int minMV = 150;
	int maxMV = 900;
	

	v0 = (lift / a + 2 * pitch / a - yaw / b) / 4;
	v1 = (lift / a - 2 * roll / a + yaw / b) / 4;
	v2 = (lift / a - 2 * pitch / a - yaw / b) / 4;
	v3 = (lift / a + 2 * roll / a + yaw / b) / 4;


	if (v0<0) v0 = 0;
	if (v1<0) v1 = 0;
	if (v2<0) v2 = 0;
	if (v3<0) v3 = 0;

	if (((minMV/mul_fac)*(minMV/mul_fac)*4*b)<lift){
		ae[0] = (mul_fac*sqrt(v0));
		if(ae[0]<minMV) ae[0] = minMV;
		else if (ae[0]>maxMV) ae[0] = maxMV;

		ae[1] = (mul_fac*sqrt(v1));
		if(ae[1]<minMV) ae[1] = minMV;
		else if (ae[1]>maxMV) ae[1] = maxMV;

		ae[2] = (mul_fac*sqrt(v2));
		if(ae[2]<minMV) ae[2] = minMV;
		else if (ae[2]>maxMV) ae[2] = maxMV;

		ae[3] = (mul_fac*sqrt(v3));
		if(ae[3]<minMV) ae[3] = minMV;
		else if (ae[3]>maxMV) ae[3] = maxMV;
		
	}
	else
	{
		ae[0] = 0;
		ae[1] = 0;
		ae[2] = 0;
		ae[3] = 0;
	}
	
	
}