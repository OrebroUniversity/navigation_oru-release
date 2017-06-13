

a = 5.690;
b = 4.720;
c = 4.765;
C = acos((a*a+b*b-c*c)/(2*a*b))

refl_0_x = 0
refl_0_y = cos(C)*5.69 + 2.65

refl_1_x = sin(C)*5.69
refl_1_y = 2.65

refl_2_x = refl_1_x
refl_2_y = 2.65+4.72

refl_3_x = refl_1_x + 4.77
refl_3_y = 2.65 - (4.765-4.72)/2

refl_4_x = refl_3_x
refl_4_y = refl_3_y + 4.765


