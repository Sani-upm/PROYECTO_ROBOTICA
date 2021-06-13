%% PARA LA CINEMATICA INVERSA USAREMOS EL METODO DE LAS MATRICES HOMOGENEAS
%% PARTIENDO DE
% T = A1*A2*A3*A4
%% SUSTITUIMOS EN T EL VECTOR V POR [0,0,0,0] Y OBTENEMOS T PARA LA POSICION 0
T
% (A1^-1)T = A2*A3*A4

%% (A1^-1) ES TAN FACIL COMO TRASPORNER LA MATRZI DE ROTACION Y MULTIPLICAR ESCALARMENTE EL VECTOR DE POSICION CON EL VECTOR DE ROTACION POR MENOS.
INV_A1= [     0, 1, 0, L1
             -1, 0, 0, 0        
              0, 0, 1, v1  
              0, 0, 0, 1]

%% (A2^-1)
INV_A2=     [ cos(v2),-sin(v2), 0,  -L2*cos(v2)^2
              sin(v2), cos(v2), 0,  -L2*sin(v2)^2
                    0,       0, 1,        0
                    0,       0, 0,        1]
             
             
             
%% (A3^-1) 
INV_A3=     [ cos(v3 - pi/2), -sin(v3 - pi/2),               0, 0
                           0,               0,               1, 0
             -sin(v3 - pi/2), -cos(v3 - pi/2),               0, 0
                           0,               0,               0, 1]


%% A2*A3*A4
A2A3A4 = A2 * A3 * A4
A3A4 = A3 * A4

%INV_A1*T 
%INV_A2*INV_A1*T
%INV_A2*INV_A1*INV_A3*T