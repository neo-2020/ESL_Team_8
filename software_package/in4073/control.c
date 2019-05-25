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

//p
#define RATE_SHIFT_YAW 0
#define RATE_GAIN_SHIFT_YAW 0

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

void calculate_yaw_control(){

	if(kp<1) kp=1;
	lift = (int32_t)-1*(values_Packet.yaw*256);
	roll = (int32_t)values_Packet.roll*256;
	pitch = (int32_t)values_Packet.pitch*256;

	err_yaw = (((int32_t)values_Packet.yaw*256)>>RATE_SHIFT_YAW)+((sr-cr)>>RATE_SHIFT_YAW);
	yaw = (kp*err_yaw)>>RATE_GAIN_SHIFT_YAW;
}




/*#################### Full Control ####################*/
//Written by Yuhao
void calculate_roll_control(){
	lift = (int32_t)-1*(values_Packet.lift-127)*256;

	err_roll = ((((int32_t)values_Packet.roll*256)/4)-((phi-cphi)>>ANGLE_SHIFT));
	roll = ((kp1*err_roll)>>ANGLE_GAIN_SHIFT)-(kp2*(sq-cq)>>RATE_SHIFT)>>RATE_GAIN_SHIFT;

	err_pitch = ((((int32_t)values_Packet.pitch*256)/4)-((theta-ctheta)>>ANGLE_SHIFT));
	pitch = ((kp1*err_pitch)>>ANGLE_GAIN_SHIFT) + (kp2*((sq-cq)>>RATE_SHIFT)>>RATE_GAIN_SHIFT);

	err_yaw = (((int32_t)values_Packet.yaw*256)/4)-((sr-cr)>>RATE_SHIFT_YAW);
	yaw = (kp*err_yaw)>>RATE_GAIN_SHIFT_YAW;
}



/*#################### Raw Control #####################*/
//Written by Yuhao
void rawControl(){
	lift = (int32_t)-1*(values_Packet.lift-127)*256;

	err_roll = ((((int32_t)values_Packet.roll*256)/4)-((phi_est-cphi)>>ANGLE_SHIFT));
	roll = ((err_roll*kp1)>>ANGLE_GAIN_SHIFT)-((((p_est-cp)>>RATE_SHIFT)*kp2)>>RATE_GAIN_SHIFT);

	err_pitch = ((((int32_t)values_Packet.pitch*256)/4)-((theta_est-ctheta)>>ANGLE_SHIFT));
	pitch = ((err_pitch*kp1)>>ANGLE_GAIN_SHIFT)-((((q_est-cq)>>RATE_SHIFT)*kp2)>>RATE_GAIN_SHIFT);

	err_yaw = (((int32_t)values_Packet.yaw*256)/4)-((r_butter-cr)>>RATE_SHIFT_YAW);
	yaw = ((err_yaw*kp)>>RATE_GAIN_SHIFT_YAW);
}


/*#################### Height Control ##################*/
//Written by Yuhao
void heightControl(){

	err_lift = (((int32_t)-1*(values_Packet.lift-127)*256)>>RATE_SHIFT_PRESS)-((pressure-cpressure)>>RATE_SHIFT_PRESS);
	lift = kp*err_lift>>RATE_GAIN_SHIFT_PRESS;
	roll = (int32_t)values_Packet.roll*256;
	pitch = (int32_t)values_Packet.pitch*256;
	yaw = (int32_t)values_Packet.yaw*256;
	}


/*################### Rotor Control #####################
Written by Yuhao*/
void calculateMoterRPM(){
	int32_t v0,v1,v2,v3;

	int32_t b =1;
	int32_t d = 1;

	int mul_fac=6;
	int minMV = 150;
	int maxMV = 900;
	

	v0 = (lift / b + 2 * pitch / b - yaw / d) / 4;
	v1 = (lift / b - 2 * roll / b + yaw / d) / 4;
	v2 = (lift / b - 2 * pitch / b - yaw / d) / 4;
	v3 = (lift / b + 2 * roll / b + yaw / d) / 4;


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