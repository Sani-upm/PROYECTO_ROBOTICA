%% Funcion cinematica inversa para el robot pastelero

function[q1,q2,q3]=cinematicaINV(x,y,z,L1,L2,L3)

%% q1
q1 = z; %%calcular exactamente n vueltas para esa altura

%% q3

q3 = acos(((x-L1).^2 + y.^2 - L2.^2 - L3.^2)/(2*L2*L3)) * 180/pi;


%% q2

q2 = (atan2(y,x-L1) - atan2(L3*sin(q3),L2 + L3*cos(q3))) * 180/pi;
