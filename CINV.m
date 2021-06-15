%% CINEMATICA INVERSA


function[q1,q2,q3]=CINV(x,y,z,L2,L3);

%% q1
q1 =(z/0.8)*360;

%% q3

q3=acos((x.^2 + y.^2 - L2.^2 -L3.^2 )/(2*L2*L3))*180/pi;

%% q2

q2 = atan2(y,x) - atan2(L3*sin(q3),L2 + L3*cos(q3))*180/pi;

end