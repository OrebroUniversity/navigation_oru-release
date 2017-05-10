/** \brief Forms the ODE for one stage in a maneuver (used for optimization)

    \param[in] t TIME
    \param[in] a0_x stage parameter
    \param[in] a0_y stage parameter
    \param[in] aT_x stage parameter
    \param[in] aT_y stage parameter
    \param[in] k_0 stage parameter
    \param[in] k_T stage parameter
    \param[in] x_0 initial x via parameter 
    \param[in] y_0 initial y via parameter 
    \param[in] theta_0 initial theta via parameter 
    \param[in] x_T final x via parameter 
    \param[in] y_T final y via parameter 
    \param[in] theta_T final theta via parameter 
    \param[in] L1 Distance from the front axis to the joint
    \param[in] L2 Distance from the rear axis to the joint
    \param[out] a coefficient of the ODE
    \param[out] b coefficient of the ODE
    \param[out] c coefficient of the ODE
    \return void
*/
void ODE_OneStage_opt( ACADO::TIME &t,
		       ACADO::IntermediateState a0_x,
		       ACADO::IntermediateState a0_y,
		       ACADO::IntermediateState aT_x,
		       ACADO::IntermediateState aT_y,
		       ACADO::IntermediateState k_0,
		       ACADO::IntermediateState k_T,
		       ACADO::IntermediateState x_0,
		       ACADO::IntermediateState y_0,
		       ACADO::IntermediateState theta_0,
		       ACADO::IntermediateState x_T,
		       ACADO::IntermediateState y_T,
		       ACADO::IntermediateState theta_T,
		       double L1, 
		       double L2,
		       ACADO::IntermediateState &a,
		       ACADO::IntermediateState &b,
		       ACADO::IntermediateState &c )
{
  ACADO::IntermediateState t11;
  ACADO::IntermediateState t12;
  ACADO::IntermediateState t13;
  ACADO::IntermediateState t14;
  ACADO::IntermediateState t21;
  ACADO::IntermediateState t22;
  ACADO::IntermediateState t23;
  ACADO::IntermediateState t30;
  ACADO::IntermediateState t34;
  ACADO::IntermediateState t35;
  ACADO::IntermediateState t38;
  ACADO::IntermediateState t39;
  ACADO::IntermediateState t4;
  ACADO::IntermediateState t41;
  ACADO::IntermediateState t42;
  ACADO::IntermediateState t45;
  ACADO::IntermediateState t46;
  ACADO::IntermediateState t5;
  ACADO::IntermediateState t53;
  ACADO::IntermediateState t54;
  ACADO::IntermediateState t61;
  ACADO::IntermediateState t65;
  ACADO::IntermediateState t66;
  ACADO::IntermediateState t67;
  ACADO::IntermediateState t68;
  ACADO::IntermediateState t69;
  ACADO::IntermediateState t7;
  ACADO::IntermediateState t8;
  ACADO::IntermediateState t89;

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
  a = -t68*t69;
  t89 = ((20.0*t46*t23+12.0*t54*t13+6.0*t61*t+a0_y)*t34-(20.0*t12*t23+12.0*t22*t13+6.0*t30*t+a0_x)*t65)/t67;
  b = -t89*L1*t69;
  c = -t89;
}
