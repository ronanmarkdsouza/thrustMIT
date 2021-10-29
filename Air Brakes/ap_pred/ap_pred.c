#include <stdio.h>
#include <math.h>

float k1,k2,k3,k4,L1,L2,L3,L4;       // rk4 variables k for finding x and L for finding v
float pred_vel;                  //need to check if i can reduce array size its 50 coz pred is run for 5 sec and dt is 0.01 so 50 value
float pred_h;                 
float timer;
float cur_v = 129.35,cur_h = 171.49; //velocity and height in m/s and m resp
float thre_h = 100; //minimum height required for parachute deployment in case of failure in meters
float setp = 500; //required apogee in meters
int counter=0; //the counter used in the pred arrays
float dt= 0.01; // in  s
float rho = 1.21;
int cur_state = 0;
float A[3] = {0.0049,1,2};
float dry_mass = 4.45;
float cd = 0.47;


int main(void)
{
    //bla bla bla
    pred_vel=cur_v;
    pred_h=cur_h;
 for(counter=0;pred_vel>0;counter+=1) // runs rk till v goes 0 or -ve  //check once pred or current
   {
    k1 = pred_vel;
    L1 = (((-cd)*rho*pow(pred_vel,2)*A[cur_state])/(2*dry_mass))-9.81;
    k2 = pred_vel;
    L2 = (((-cd)*rho*pow(pred_vel+(0.5*dt*k1),2)*A[cur_state]))/(2*dry_mass)- 9.81;
    k3 = pred_vel;
    L3 = (((-cd)*rho*pow(pred_vel+(0.5*dt*k2),2)*A[cur_state]))/(2*dry_mass)- 9.81;
    k4 = pred_vel;
    L4 = (((-cd)*rho*pow(pred_vel+(dt*k3),2)*A[cur_state]))/(2*dry_mass)- 9.81;
    pred_vel=pred_vel+(dt*(L1+(2*L2)+(2*L3)+L4))/6;
    pred_h=pred_h+(dt*(k1+(2*k2)+(2*k3)+k4))/6;
    timer = timer + dt;
    printf("%f,%f,%f\n", pred_h,pred_vel,timer);
   }
}