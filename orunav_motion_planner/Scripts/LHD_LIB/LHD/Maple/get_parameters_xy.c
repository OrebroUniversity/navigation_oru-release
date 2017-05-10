/* drdv: generated using codegen (2012-06-02, 00:31:09) */
#include <math.h>
void get_parameters_xy(a0_x,a0_y,aT_x,aT_y,k_0,k_T,x_0,y_0,theta_0,x_T,y_T,
theta_T,all)
double a0_x;
double a0_y;
double aT_x;
double aT_y;
double k_0;
double k_T;
double x_0;
double y_0;
double theta_0;
double x_T;
double y_T;
double theta_T;
double all[12];
{
  double t10;
  double t15;
  double t22;
  double t23;
  double t25;
  double t26;
  double t28;
  double t29;
  double t3;
  double t34;
  double t4;
  double t6;
  double t7;
  double t9;
  {
    t3 = cos(theta_0);
    t4 = k_0*t3;
    t6 = cos(theta_T);
    t7 = k_T*t6;
    t9 = a0_x/2.0;
    t10 = aT_x/2.0;
    all[0] = -6.0*x_0+6.0*x_T-3.0*t4-3.0*t7-t9+t10;
    t15 = 3.0/2.0*a0_x;
    all[1] = 15.0*x_0-15.0*x_T+8.0*t4+7.0*t7+t15-aT_x;
    all[2] = -10.0*x_0+10.0*x_T-6.0*t4-4.0*t7-t15+t10;
    all[3] = t9;
    all[4] = t4;
    all[5] = x_0;
    t22 = sin(theta_0);
    t23 = k_0*t22;
    t25 = sin(theta_T);
    t26 = k_T*t25;
    t28 = a0_y/2.0;
    t29 = aT_y/2.0;
    all[6] = -6.0*y_0+6.0*y_T-3.0*t23-3.0*t26-t28+t29;
    t34 = 3.0/2.0*a0_y;
    all[7] = 15.0*y_0-15.0*y_T+8.0*t23+7.0*t26+t34-aT_y;
    all[8] = -10.0*y_0+10.0*y_T-6.0*t23-4.0*t26-t34+t29;
    all[9] = t28;
    all[10] = t23;
    all[11] = y_0;
    return;
  }
}

