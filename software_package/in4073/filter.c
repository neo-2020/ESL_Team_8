//Authored by Diwakar Babu
//filter.c - Filters sr, sp and sq sensor values for yaw, roll and pitch respectively

#include "in4073.h"

static int32_t pb=0;
static int32_t bq=0;
static float y[2];
static float x[2];
static int32_t  ephi; 
static int32_t temp_estimated_phi;
static int32_t temp_estimated_theta;
static int32_t etheta;

 //-------------------fixed point implementation
    
    int32_t float2fix2(double x2)
    {
        uint32_t y2;
        y2=x2*(1<<4);
        return y2;
    }

    int32_t fix2float2(int32_t x2)
    {
        double y2;
        y2=((double) x2)/(1<<4);
        return round(y2);
    }

    int32_t fixadd2(int32_t a,int32_t b)
    {
        return a+b;
    }

    int32_t fixmul2(int32_t x2, int32_t x3)
    {
        int64_t temp=x2 * x3;
        return ((int32_t) temp >> 4);
    }

    int32_t fixsub2(int32_t x2,int32_t x3)
    {
        int32_t y3;
        y3=x2-x3;
        return y3;
    }

    int32_t fixdiv2(int32_t x2, int32_t x3)
    {
        int64_t temp=(int64_t) x2<<4;
        return ((int32_t)temp/x3);
    }


void filter_function()
{
   
    //variables
    int32_t a[2];
    int32_t b[2];
    //float_t p2phi=0;
    //float_t s2theta=0;
    //int16_t c1;
    //int16_t c2;
   // int16_t sp_0=0;
   // int16_t sq_0=0;

    //Butterworth Filtering for Yaw
    a[0]= float2fix2(1);
    a[1]=float2fix2(1);
    b[0]=float2fix2(0.7804076597);
    b[1]=float2fix2(1);

    //printf("before filter sr: %4d \n",sr);

    //Filtering Process
    x[0]=sr; 
    
    int32_t x0=float2fix2(x[0]);   
    int32_t x1=float2fix2(x[1]); 
    int32_t y0=float2fix2(y[0]);
    int32_t y1=float2fix2(y[1]);

   
    y0 = fixsub2( fixadd2( fixmul2(x0,a[0]), fixmul2(x1,a[1])), fixmul2(y1,b[1]));
    y0 = fixdiv2(y0,b[0]);
    y[0] = fix2float2(y0);
    r_butter = y[0];
    y[1]=y[0];
    x[1]=x[0]; 
    


    //Kalman Filtering for Roll and Pitch :

    /*****************************************************************************/
	//kalman variable 
	//*****************************************************************************/
    int32_t c1phi = float2fix2(128);
	int32_t c1theta = float2fix2(128);
	int32_t c2phi = float2fix2(1000000.0);
	int32_t p2phi = float2fix2(0.023);
    int32_t s2phi = float2fix2(0.023);
    //c2phi = c1phi + 1000;
	int16_t c2theta = float2fix2(1000000.0);
	//estimated_p = 0;
	//estimated_q = 0;


	int32_t temp_estimated_p;
    int32_t temp_estimated_q;
    int32_t temp_sp;
    int32_t temp_sq;
    int32_t temp_sax;
    int32_t temp_say;
	

    //int p2phi= 177 >> 4;
    //s2theta=1.2;
    //c1=1000;
    //c2=10;
    //sp and say is multiplied by 256, so that we do not need to scale down pb.
    temp_sp = float2fix2(sp);
    temp_say = float2fix2(say);
    temp_estimated_p = (temp_sp - pb);
    temp_estimated_phi = temp_estimated_phi + (fixmul2(temp_estimated_p , p2phi));
    ephi = temp_estimated_phi - temp_say;
    temp_estimated_phi = temp_estimated_phi - (fixdiv2(ephi,c1phi));
   // phi = phi - (phi - sphi)/c1;
    pb = pb + (fixdiv2(ephi,c2phi));
   	phi_est = fix2float2(temp_estimated_phi); //scale down estimated_phi to the original value
	p_est = fix2float2(temp_estimated_p); //scale down estimated_p to the original value


  //sq and sax is multiplied by 256, so that we do not need to scale down bq.
	temp_sq = float2fix2(sq);
	temp_sax = float2fix2(sax);
	temp_estimated_q = (temp_sq-bq);
	temp_estimated_theta = temp_estimated_theta + (fixmul2(temp_estimated_q,s2phi));
	etheta = temp_estimated_theta - temp_sax;
	temp_estimated_theta = temp_estimated_theta - (fixdiv2(etheta,c1theta));
	bq = bq + (fixdiv2(ephi,c2theta));
	theta_est = float2fix2(temp_estimated_theta); //scale down estimated_theta to the original value
	q_est = float2fix2(temp_estimated_q); //scale down estimated_q to the original value
    
}    
    
  