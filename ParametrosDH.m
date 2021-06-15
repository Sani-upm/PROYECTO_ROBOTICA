%%FICHERO QUE CONTIENE LOS PARAMETROS DH DEL ROBOT PARA EL PROYECTO

clear all,clc

startup_rvc

%%%%%%%%%%%%%%%%
L1=10;
L2=20;
L3=10;
L4=10;

q1=0;
q2=0;
q3=0;
q4=0;

%% tipo de articulación teta/d/a/alfa/
L(1)=Link([pi/2   0   L1  0              1]); 
L(1).qlim=[0 L1];
L(2)=Link([-1     0   L2  0              0]);
L(3)=Link([-1     0   0  pi/2            0]);
L(4)=Link([0      L3  0   0              0]);

mirobot=SerialLink(L);
mirobot.name='CREMITA';
%L(4).qlim=[0 20];
mirobot.plot([0 0 0 0],'workspace',[-40 40 -40 40 -40 40]);
mirobot.teach();