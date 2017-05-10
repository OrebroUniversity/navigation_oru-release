
clear;clc

q_i = 1;
q_f = 2;
dq_c = 1.1;
t_f = 1;
Ts = 0.01;

[T,q,dq,ddq,err] = trapez(q_i,q_f,dq_c,t_f,Ts);

if err
  disp('err')
  return
end

figure(1);
subplot(3,1,1);hold on
plot(T,q,'b');
grid on; title('q')

subplot(3,1,2);hold on
plot(T,dq,'b');
grid on; title('dq')

subplot(3,1,3);hold on
plot(T,ddq,'b');
grid on; title('ddq')