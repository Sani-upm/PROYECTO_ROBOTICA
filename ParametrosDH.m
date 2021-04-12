%%FICHERO QUE CONTIENE LOS PARAMETROS DH DEL ROBOT PARA EL PROYECTO

clear all,clc

startup_rvc


L1=3;
L2=10;
L3=10;
L4=10;

%% tipo de articulación teta/d/a/alfa/
L(1)=Link([0 0 L2 0 0]); 
L(2)=Link([0 L1 L3 0 0]);
L(3)=Link([0 0 0 pi 0]);
L(4)=Link([0 L4 0 0 1]);

Ej2=SerialLink(L);
Ej2.name='cilindrico';

L(4).qlim=[0 20];

Ej2.plot([0 0 0 0],'workspace',[-30 30 -30 30 -20 30]);
Ej2.teach();