%% OBTINER LAS MATRICES DE TRANSFORMACIÓN A PARTIR DEL DH

q1=90;
q2=90;
q3=90;
q4=90;
L1=120;
L2=100;
L3=100;
L4=100;

%% 1 MATRIZ SISTEMA 1 - 0
% TETHA  - ROTACION EN Z
% D      - TRASLACIÓN EN Z
% A      - TRASLACIÓN EN X
% ALFA   - ROTACION EN X

A10 = [ cos(q1),  -sin(q1),  0,  L1;
        sin(q1),   cos(q1),  0,   0;
              0,         0,  1,   0;
              0,         0,  0,   1 ]


%% 2  MATRIZ SISTEMA 2 - 1
% TETHA  - ROTACION EN Z
% D      - TRASLACIÓN EN Z
% A      - TRASLACIÓN EN X
% ALFA   - ROTACION EN X

 A21 = [ cos(q2),  -sin(q2),  0,  L2;
         sin(q2),   cos(q2),  0,   0;
              0,         0,   1,  L3;
              0,         0,   0,   1 ]

%% 3  MATRIZ SISTEMA 3 - 2
% TETHA  - ROTACION EN Z
% D      - TRASLACIÓN EN Z
% A      - TRASLACIÓN EN X
% ALFA   - ROTACION EN X

  
 A32 = [ cos(q3),  -sin(q3),  0,   0;
         sin(q3),   cos(q3),  0,   0;
              0,         0,   1,   0;
              0,         0,   0,   1 ]

%% 4  MATRIZ SISTEMA 4 - 3
% TETHA  - ROTACION EN Z
% D      - TRASLACIÓN EN Z
% A      - TRASLACIÓN EN X
% ALFA   - ROTACION EN X
%A43 = [   1     1  0  0
%          1     1  0  0
%          0     0  1  L4
  %        0     0  0  1 ];
 A43 = [       1,        0,    0,    0;
               0,        1,    0,    0;
               0,        0,    1,   L4;
               0,        0,    0,    1 ]
      
%% MATRIZ DE TRANSFORMACION

T = A10 * A21 * A32 * A43
