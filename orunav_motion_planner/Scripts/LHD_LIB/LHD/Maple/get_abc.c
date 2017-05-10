/* drdv: generated using codegen (2012-06-26, 23:37:29) */
#include <math.h>
void get_abc(L1,L2,a0_x,a0_y,aT_x,aT_y,k_0,k_T,x_0,y_0,theta_0,x_T,y_T,theta_T,
t,A)
double L1;
double L2;
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
double t;
double A[3];
{
  double t11;
  double t12;
  double t13;
  double t14;
  double t21;
  double t22;
  double t23;
  double t30;
  double t34;
  double t35;
  double t38;
  double t39;
  double t4;
  double t41;
  double t42;
  double t45;
  double t46;
  double t5;
  double t53;
  double t54;
  double t61;
  double t65;
  double t66;
  double t67;
  double t68;
  double t69;
  double t7;
  double t8;
  double t89;
  {
    t4 = cos(theta_0);
    t5 = k_0*t4;
    t7 = cos(theta_T);
    t8 = k_T*t7;
    t11 = aT_x/2.0;
    t12 = -6.0*x_0+6.0*x_T-3.0*t5-3.0*t8-a0_x/2.0+t11;
    t13 = t*t;
    t14 = t13*t13;
    t21 = 3.0/2.0*a0_x;
    t22 = 15.0*x_0-15.0*x_T+8.0*t5+7.0*t8+t21-aT_x;
    t23 = t13*t;
    t30 = -10.0*x_0+10.0*x_T-6.0*t5-4.0*t8-t21+t11;
    t34 = 5.0*t12*t14+4.0*t22*t23+3.0*t30*t13+a0_x*t+t5;
    t35 = t34*t34;
    t38 = sin(theta_0);
    t39 = k_0*t38;
    t41 = sin(theta_T);
    t42 = k_T*t41;
    t45 = aT_y/2.0;
    t46 = -6.0*y_0+6.0*y_T-3.0*t39-3.0*t42-a0_y/2.0+t45;
    t53 = 3.0/2.0*a0_y;
    t54 = 15.0*y_0-15.0*y_T+8.0*t39+7.0*t42+t53-aT_y;
    t61 = -10.0*y_0+10.0*y_T-6.0*t39-4.0*t42-t53+t45;
    t65 = 5.0*t46*t14+4.0*t54*t23+3.0*t61*t13+a0_y*t+t39;
    t66 = t65*t65;
    t67 = t35+t66;
    t68 = sqrt(t67);
    t69 = 1/L2;
    A[0] = -t68*t69;
    t89 = ((20.0*t46*t23+12.0*t54*t13+6.0*t61*t+a0_y)*t34-(20.0*t12*t23+12.0*
t22*t13+6.0*t30*t+a0_x)*t65)/t67;
    A[1] = -t89*L1*t69;
    A[2] = -t89;
    return;
  }
}

