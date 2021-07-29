% probelma ottimo

%funzione da minimizzare
f  = beta^2 + dr^2 + df^2;  %beta sliping angle, se zero veicolo orientato lungo la curva, se pari a dr e df allora traslazione

%sistema
beta = atan( ( lf*tan(dr) + lr*tan(df) )/(lf+lr) );
v = (vf*cos(df) + vr*cos(dr))/2*cos(beta);

x_d = v*cos(phi+beta);
y_d = v*sin(phi+beta);
phi_d = v*cos(beta)*(tan(df) - tan(dr))/(lf+lr); %qui c'è il meno

%steraing angle
df_d = wd;
dr_d = wr;

%limitazioni sugli angoli
abs(dr) < di_max;
abs(df) < di_max;
abs(wr) < wi_max;
abs(wf) < wi_max;
abs(vf) < v_max;
vr = vf;


s0 = [ 0 0 0 0 0] %punto iniziale
s1 = [ 2 2 pi/6 0 0]; %punto finale