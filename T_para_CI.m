%% QUEREMOS OBTENER LA MATRIZ T PERO EN LA POSICION 0,0,0,0,


%% GUARDAMOS EL VALOR DE A_sdr en T
T = A_sdr;

%% sustituimos el valor de las v
v1 = 0;
v2 = 0;
v3 = 0;
v4 = 0; 

%% ponemos las longitudes

L1 = 1;
L2 = 1;
L3 = 1;


%% CALCULAMOS T
T = [cos(v4)*(cos(v2)*sin(v3 - pi/2) + cos(v3 - pi/2)*sin(v2)), -sin(v4)*(cos(v2)*sin(v3 - pi/2) + cos(v3 - pi/2)*sin(v2)),   cos(v2)*cos(v3 - pi/2) - sin(v2)*sin(v3 - pi/2),      L3*(cos(v2)*cos(v3 - pi/2) - sin(v2)*sin(v3 - pi/2)) + L2*sin(v2)
     cos(v4)*(cos(v2)*cos(v3 - pi/2) - sin(v2)*sin(v3 - pi/2)), -sin(v4)*(cos(v2)*cos(v3 - pi/2) - sin(v2)*sin(v3 - pi/2)), - cos(v2)*sin(v3 - pi/2) - cos(v3 - pi/2)*sin(v2), L1 - L3*(cos(v2)*sin(v3 - pi/2) + cos(v3 - pi/2)*sin(v2)) + L2*cos(v2)
                                                       sin(v4),                                                    cos(v4),                                                 0,                                                                     v1
                                                             0,                                                          0,                                                 0,                                                                      1]
 