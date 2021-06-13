%limpieza de valores

%Inicio programa
startup_rvc

l1=17.6;
l2=13.3;

offset=9.3;
offset_codo=2.8;
offset_gripper=3.8;

       %  teta /d           /a /alfa/tipo de articulación
L(1)=Link([90   0            0   0    1]);
L(2)=Link([-1  offset        l1  0    0]);
L(3)=Link([-1 -offset_codo   l2  pi   0]);
L(4)=Link([ 0  offset_gripper 0  0    0]);


Ej2=SerialLink(L);
Ej2.name='cilindrico';

L(1).qlim=[0 8.5];


Ej2.plot([0 0 0 0],'workspace',[-40 40 -40 40 0 40]);
Ej2.teach();